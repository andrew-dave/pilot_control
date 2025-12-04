#!/usr/bin/env python3
"""
MPC-Based Autonomous Control Node
==================================
Subscribes to tilt-corrected odometry and wheel encoder measurements to perform
Model Predictive Control (MPC) for autonomous navigation.

Features:
- Subscribes to /Odometry_tilt_corrected_diff for localization
- Subscribes to wheel encoder measurements (/left/controller_status, /right/controller_status)
- Online wheel slip estimation based on past 1 second of Fast-LIO timesteps
- MPC optimization step for control computation
- Local waypoint feeder based on next global waypoint and current location

Usage:
  ros2 run pilot_control mpc_autonomous_controller.py

Topics:
  Subscribed:
    - /Odometry_tilt_corrected_diff (nav_msgs/Odometry) - Tilt-corrected odometry
    - /left/controller_status (odrive_can/ControllerStatus) - Left wheel encoder
    - /right/controller_status (odrive_can/ControllerStatus) - Right wheel encoder
    - /set_target_pose (std_msgs/Float64MultiArray) - [x, y, z, yaw(rad)]
  Published:
    - /left/control_message (odrive_can/ControlMessage) - Left wheel velocity
    - /right/control_message (odrive_can/ControlMessage) - Right wheel velocity
"""

from pickle import TRUE
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float64, String, Bool
from geometry_msgs.msg import Twist
from odrive_can.msg import ControlMessage, ControllerStatus
from odrive_can.srv import AxisState
from std_srvs.srv import Trigger, Empty
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import math
from typing import Tuple, Optional, List, Dict
from collections import deque
import time
import os
import signal
import csv

# Optional dependencies for MPC
try:
    import osqp
    OSQP_AVAILABLE = True
except ImportError:
    OSQP_AVAILABLE = False
    import warnings
    warnings.warn("OSQP not available. Install with: pip install osqp")

try:
    from scipy import sparse
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    import warnings
    warnings.warn("scipy not available. Install with: pip install scipy")


class SlipAwareMPC:
    """
    Linear Time-Varying (LTV) MPC for differential drive robot with slip compensation.
    
    State: Error vector x = [xe, ye, θe]^T (Error in Body Frame)
    Input: Wheel velocities u = [ωL, ωR]^T (rad/s)
    
    Dynamics: x_{k+1} = A_k*x_k + B_k*u_kta
    - A_k depends on reference trajectory (v_ref, omega_ref)
    - B_k depends on slip ratios (λ_L, λ_R)
    """
    
    def __init__(self, N, Ts, r, L, w_min, w_max, Q_xe, Q_ye, Q_yaw, R_delta, logger=None, 
                 weight_increase_xe=0.0, weight_increase_ye=0.0, weight_increase_yaw=0.0,
                 v_min=None, v_max=None, omega_min=None, omega_max=None):
        """
        Initialize MPC optimizer.
        
        Args:
            N: Prediction horizon
            Ts: Time step (s)
            r: Wheel radius (m)
            L: Wheel base (m)
            w_min: Minimum wheel velocity (rad/s)
            w_max: Maximum wheel velocity (rad/s)
            Q_xe: Cost weight for position error x (base weight)
            Q_ye: Cost weight for position error y (base weight)
            Q_yaw: Cost weight for yaw error (base weight)
            R_delta: Cost weight for control input change (delta u)
            logger: Optional logger for debug messages
            weight_increase_xe: Linear weight increase factor per time step for xe error.
                               Weight at step k = base_weight * (1 + weight_increase_xe * k)
            weight_increase_ye: Linear weight increase factor per time step for ye error.
                               Weight at step k = base_weight * (1 + weight_increase_ye * k)
            weight_increase_yaw: Linear weight increase factor per time step for yaw error.
                                Weight at step k = base_weight * (1 + weight_increase_yaw * k)
            v_min, v_max: Optional bounds on linear velocity v of the robot [m/s].
                          If not provided, they are derived from wheel limits.
            omega_min, omega_max: Optional bounds on angular velocity ω of the robot [rad/s].
                                  If not provided, they are derived from wheel limits.
        """
        self.N = N
        self.Ts = Ts
        self.r = r
        self.L = L
        # Wheel limits (rad/s)
        self.w_min = w_min
        self.w_max = w_max
        self.logger = logger
        self.weight_increase_xe = weight_increase_xe
        self.weight_increase_ye = weight_increase_ye
        self.weight_increase_yaw = weight_increase_yaw
        
        # State and control dimensions
        self.nx = 3  # [xe, ye, θe]
        self.nu = 2  # [ωL, ωR]
        self.nz = N * (self.nu + self.nx)  # Decision vector size

        # ------------------------------------------------------------
        # Robot-level velocity limits (v and ω) for constraints
        # ------------------------------------------------------------
        # If not explicitly provided, derive conservative values from wheel limits.
        # v = (r/2)*(ωL + ωR)  ⇒ |v| ≤ r * |w_max|
        # ω = (r/L)*(ωR - ωL)  ⇒ |ω| ≤ 2*r*|w_max|/L
        if v_max is None:
            v_max = abs(self.r * self.w_max)
        if v_min is None:
            v_min = -v_max
        if omega_max is None:
            omega_max = abs(2.0 * self.r * self.w_max / self.L) if self.L != 0.0 else 0.0
        if omega_min is None:
            omega_min = -omega_max

        self.v_min = float(v_min)
        self.v_max = float(v_max)
        self.omega_min = float(omega_min)
        self.omega_max = float(omega_max)

        # Store base values so we can apply dynamic scaling to v-bounds
        self._v_min_base = self.v_min
        self._v_max_base = self.v_max
        
        # Base cost matrix Q (for states) - will be scaled per time step in setup()
        self.Q_xe_base = Q_xe
        self.Q_ye_base = Q_ye
        self.Q_yaw_base = Q_yaw
        self.Q = sparse.diags([Q_xe, Q_ye, Q_yaw])  # Keep for backward compatibility, but will use scaled versions
        
        # Cost matrix R_delta (for control input changes: delta u = u_k - u_{k-1})
        self.R_delta = sparse.diags([R_delta, R_delta])
        
        # OSQP solver
        self.solver = None
        self.initialized = False
        
        # Sparse matrices (created once, updated each cycle)
        self.P = None  # Cost matrix
        self.A_constr = None  # Constraint matrix
        self.l_constr = None  # Lower bounds
        self.u_constr = None  # Upper bounds
        
        # Pre-computed indices for efficient sparse matrix updates
        self.A_indices = {}  # {step_k: [indices]} for A matrix updates
        self.B_indices = {}  # {step_k: [indices]} for B matrix updates
        
        # Previous solution for warm starting
        self.prev_solution = None

    def set_velocity_bound_scale(self, scale: float) -> None:
        """
        Scale the robot linear velocity bounds by the given factor in [0, 1].
        scale = 1.0 -> full v_min/v_max
        scale = 0.0 -> v constrained to (approximately) zero.
        """
        try:
            s = float(scale)
        except (TypeError, ValueError):
            s = 1.0
        s = max(0.0, min(1.0, s))
        self.v_min = self._v_min_base * s
        self.v_max = self._v_max_base * s
    
    def setup(self):
        """
        Initialize OSQP solver with sparse matrix structure.
        Called once during initialization.
        
        Creates:
        - P matrix: Quadratic cost matrix (sparse, block-diagonal with Q)
        - A_constr matrix: Constraint matrix (sparse) with dynamics and input bounds
        - Pre-computes indices for efficient matrix updates
        - Sets up OSQP solver
        """
        if not OSQP_AVAILABLE or not SCIPY_AVAILABLE:
            if self.logger:
                self.logger.error("OSQP and scipy required for MPC")
            return False
        
        N = self.N
        nx = self.nx
        nu = self.nu
        nz = self.nz
        r = self.r
        L = self.L
        
        # ============================================================
        # STEP 1: BUILD COST MATRIX P
        # ============================================================
        # Cost function: J = sum(x_k^T Q x_k) + sum((u_k - u_{k-1})^T R_delta (u_k - u_{k-1}))
        # P is nz x nz with:
        #   - Q blocks for state positions (x1, x2, ..., xN)
        #   - R_delta blocks for control input changes (u1-u0, u2-u1, ..., u_{N-1}-u_{N-2})
        # Decision vector: [u0, x1, u1, x2, u2, ..., u_{N-1}, xN]
        
        # Build P matrix: sparse matrix with Q blocks for states and R_delta for control changes
        P_data = []
        P_row = []
        P_col = []
        
        # For each state x_k (k=0 to N-1, but indexed as 1 to N in decision vector)
        # Actually k here is the step index: 0-based in the horizon
        for k in range(N):
            # Position of x_k in decision vector: k * (nu + nx) + nu
            x_k_start = k * (nu + nx) + nu
            
            # Scale Q for this time step: weight_k = base_weight * (1 + alpha * k)
            # Use separate scaling factors for each error term
            weight_scale_xe = 1.0 + self.weight_increase_xe * k
            weight_scale_ye = 1.0 + self.weight_increase_ye * k
            weight_scale_yaw = 1.0 + self.weight_increase_yaw * k
            
            # Create scaled Q matrix for this time step (diagonal matrix)
            # Since Q is diagonal, we can directly add diagonal entries
            Q_xe_scaled = self.Q_xe_base * weight_scale_xe
            Q_ye_scaled = self.Q_ye_base * weight_scale_ye
            Q_yaw_scaled = self.Q_yaw_base * weight_scale_yaw
            
            # Add Q matrix diagonal entries for this state
            # State vector: [xe, ye, θe]
            P_row.append(x_k_start + 0)  # xe
            P_col.append(x_k_start + 0)
            P_data.append(Q_xe_scaled)
            
            P_row.append(x_k_start + 1)  # ye
            P_col.append(x_k_start + 1)
            P_data.append(Q_ye_scaled)
            
            P_row.append(x_k_start + 2)  # yaw
            P_col.append(x_k_start + 2)
            P_data.append(Q_yaw_scaled)
        
        # Add cost for control input changes: (u_k - u_{k-1})^T R_delta (u_k - u_{k-1})
        # This expands to: u_k^T R_delta u_k - u_k^T R_delta u_{k-1} - u_{k-1}^T R_delta u_k + u_{k-1}^T R_delta u_{k-1}
        # Since R_delta is symmetric: = u_k^T R_delta u_k - 2*u_k^T R_delta u_{k-1} + u_{k-1}^T R_delta u_{k-1}
        # But in the Hessian form [u_{k-1}, u_k]^T H [u_{k-1}, u_k], we need H = [[r, -r], [-r, r]]
        # So the cross term coefficient in the Hessian is -r (not -2*r)
        # For k >= 1: penalize (u_k - u_{k-1})
        # Since R_delta is diagonal, we can simplify
        R_delta_diag = self.R_delta.diagonal()  # Get diagonal values [r, r]
        
        # Also add a small cost on u_0 to ensure positive definiteness
        for i in range(nu):
            r_val = R_delta_diag[i]
            u_0_idx = i  # u_0 is at the start of decision vector
            P_row.append(u_0_idx)
            P_col.append(u_0_idx)
            P_data.append(r_val * 0.5)  # Small cost on initial control
        
        for k in range(1, N):  # k from 1 to N-1
            # Position of u_{k-1} and u_k in decision vector
            u_km1_start = (k - 1) * (nu + nx)  # u_{k-1}
            u_k_start = k * (nu + nx)  # u_k
            
            # For each control input dimension
            for i in range(nu):
                r_val = R_delta_diag[i]
                
                # u_k[i]^2 term: coefficient = r
                P_row.append(u_k_start + i)
                P_col.append(u_k_start + i)
                P_data.append(r_val)
                
                # u_{k-1}[i]^2 term: coefficient = r
                P_row.append(u_km1_start + i)
                P_col.append(u_km1_start + i)
                P_data.append(r_val)
                
                # Cross term: -r*u_k[i]*u_{k-1}[i] (symmetric)
                # The Hessian has -r in both off-diagonal positions
                # Only add upper triangle, symmetry will be enforced
                P_row.append(u_k_start + i)
                P_col.append(u_km1_start + i)
                P_data.append(-r_val)
        
        # Create sparse P matrix (nz x nz)
        # First create COO matrix (duplicate entries will be summed automatically)
        P_coo = sparse.coo_matrix(
            (P_data, (P_row, P_col)),
            shape=(nz, nz)
        )
        
        # Ensure symmetry: P = (P + P^T) / 2
        # This ensures the matrix is symmetric (required for QP)
        P_sym = (P_coo + P_coo.T) / 2.0
        
        # Convert to CSC format
        self.P = P_sym.tocsc()
        
        # Add small regularization to ensure positive definiteness
        # This prevents numerical issues with OSQP
        regularization = 1e-6
        self.P = self.P + sparse.eye(nz, format='csc') * regularization
        
        # Verify P is positive semi-definite
        try:
            from scipy.sparse.linalg import eigsh
            eigenvals = eigsh(self.P, k=min(3, nz-1), which='SA', return_eigenvectors=False)
            min_eigenval = eigenvals[0]
            if min_eigenval < -1e-6:
                if self.logger:
                    self.logger.error(f"P matrix is not positive semi-definite! Min eigenvalue: {min_eigenval:.2e}")
                # Apply even stronger regularization
                regularization = abs(min_eigenval) + 1e-3
                self.P = self.P + sparse.eye(nz, format='csc') * regularization
                if self.logger:
                    self.logger.warning(f"Applied stronger regularization: {regularization:.2e}")
            elif self.logger:
                self.logger.debug(f"P matrix is positive semi-definite. Min eigenvalue: {min_eigenval:.2e}")
        except Exception as e:
            if self.logger:
                self.logger.warning(f"Could not verify P matrix eigenvalues: {e}")
        
        # ============================================================
        # STEP 2: BUILD CONSTRAINT MATRIX A_constr
        # ============================================================
        # Constraints:
        # 1. Dynamics: -B_k*u_k + x_{k+1} - A_k*x_k = 0 (equality, bounds = 0)
        # 2. Input bounds: w_min <= u_k <= w_max
        
        # Number of constraints
        n_dynamics = N * nx  # Dynamics constraints
        # For inputs we now constrain robot linear velocity v and angular velocity ω
        # instead of individual wheel speeds. Each step contributes:
        #   - 1 constraint for v
        #   - 1 constraint for ω
        n_input_bounds = N * 2
        n_constraints = n_dynamics + n_input_bounds
        
        # Build constraint matrix structure using COO format
        row_indices = []
        col_indices = []
        data_values = []
        
        # Pre-compute indices for efficient updates
        self.A_indices = {}
        self.B_indices = {}
        
        # Dynamics constraints: x_{k+1} = A_k*x_k + B_k*u_k
        # Rearranged: -B_k*u_k + x_{k+1} - A_k*x_k = 0
        constraint_row = 0
        for k in range(N):
            # Position in decision vector z = [u0, x1, u1, x2, ..., u_{N-1}, xN]
            # For step k (0-based):
            #   - u_k is always at index:      u_k_idx    = k * (nu + nx)
            #   - x_{k+1} is always at index:  x_kp1_idx  = k * (nu + nx) + nu
            #   - x_k (for k > 0) is at index: x_k_idx    = (k - 1) * (nu + nx) + nu
            u_k_idx = k * (nu + nx)
            x_k_idx = (k - 1) * (nu + nx) + nu if k > 0 else None  # x_k (for k>0)
            x_kp1_idx = k * (nu + nx) + nu  # x_{k+1}
            
            # Store indices for A_k and B_k updates
            self.A_indices[k] = []
            self.B_indices[k] = []
            
            # For each state component in x_{k+1}
            for i in range(nx):
                # x_{k+1}[i] term (coefficient = 1.0)
                row_indices.append(constraint_row + i)
                col_indices.append(x_kp1_idx + i)
                data_values.append(1.0)
                
                # -A_k[i,:]*x_k terms (if k > 0, x_k exists)
                if k > 0:
                    for j in range(nx):
                        idx = len(data_values)
                        row_indices.append(constraint_row + i)
                        col_indices.append(x_k_idx + j)
                        data_values.append(0.0)  # Placeholder, will be updated with -A_k
                        self.A_indices[k].append(idx)
                
                # -B_k[i,:]*u_k terms
                for j in range(nu):
                    idx = len(data_values)  # Index before appending
                    row_indices.append(constraint_row + i)
                    col_indices.append(u_k_idx + j)
                    data_values.append(0.0)  # Placeholder, will be updated
                    self.B_indices[k].append(idx)  # Store index of this element
            
            constraint_row += nx
        
        # Input bound constraints: bounds on robot v and ω
        #
        # v = (r/2) * (ωL + ωR)
        # ω = (r/L) * (ωR - ωL)
        #
        # We encode these as linear inequalities in terms of wheel speeds.
        for k in range(N):
            u_k_idx = k * (nu + nx)

            # --- Linear velocity constraint: v_min <= v <= v_max ---
            # Row for v: coefficients on [ωL, ωR] -> [r/2, r/2]
            row_indices.append(constraint_row)
            col_indices.append(u_k_idx + 0)  # ωL
            data_values.append(r / 2.0)

            row_indices.append(constraint_row)
            col_indices.append(u_k_idx + 1)  # ωR
            data_values.append(r / 2.0)
            constraint_row += 1

            # --- Angular velocity constraint: omega_min <= ω <= omega_max ---
            # Row for ω: coefficients on [ωL, ωR] -> [-r/L, +r/L]
            # Guard against L == 0 just in case (should not happen in practice).
            if L != 0.0:
                coeff_omega = r / L
            else:
                coeff_omega = 0.0

            row_indices.append(constraint_row)
            col_indices.append(u_k_idx + 0)  # ωL
            data_values.append(-coeff_omega)

            row_indices.append(constraint_row)
            col_indices.append(u_k_idx + 1)  # ωR
            data_values.append(coeff_omega)
            constraint_row += 1
        
        # Create sparse constraint matrix
        # Store COO format first to preserve index mapping
        A_constr_coo = sparse.coo_matrix(
            (data_values, (row_indices, col_indices)),
            shape=(n_constraints, nz)
        )
        
        # Convert to CSC for OSQP (but keep COO for index mapping)
        self.A_constr = A_constr_coo.tocsc()
        
        # Build mapping from (row, col) to data index in CSC format
        # This is needed because CSC format may reorder data
        self.row_col_to_data_idx = {}
        
        # CSC format attributes: indices (row indices), indptr (column pointers), data
        csc_indices = self.A_constr.indices  # Row indices
        csc_indptr = self.A_constr.indptr    # Column pointers
        csc_data = self.A_constr.data
        
        # Build mapping: for each column, find all row indices and map (row, col) to data index
        for col in range(nz):
            col_start = csc_indptr[col]
            col_end = csc_indptr[col + 1]
            for csc_idx in range(col_start, col_end):
                row = csc_indices[csc_idx]
                self.row_col_to_data_idx[(row, col)] = csc_idx
        
        # Now update stored indices to point to CSC data array
        for k in range(N):
            if k in self.A_indices:
                # Rebuild A_indices using row/col mapping
                new_A_indices = []
                if k > 0:
                    # x_k is at index (k-1)*(nu + nx) + nu in the decision vector
                    x_k_idx = (k - 1) * (nu + nx) + nu
                    for i in range(nx):
                        for j in range(nx):
                            row = k * nx + i
                            col = x_k_idx + j
                            if (row, col) in self.row_col_to_data_idx:
                                new_A_indices.append(self.row_col_to_data_idx[(row, col)])
                self.A_indices[k] = new_A_indices
            
            if k in self.B_indices:
                # Rebuild B_indices using row/col mapping
                new_B_indices = []
                u_k_idx = k * (nu + nx)
                for i in range(nx):
                    for j in range(nu):
                        row = k * nx + i
                        col = u_k_idx + j
                        if (row, col) in self.row_col_to_data_idx:
                            new_B_indices.append(self.row_col_to_data_idx[(row, col)])
                self.B_indices[k] = new_B_indices
        
        # Verify dimensions
        if self.logger:
            self.logger.info(f'MPC matrix dimensions: P={self.P.shape}, A={self.A_constr.shape}, nz={nz}, n_constraints={n_constraints}, data_size={len(self.A_constr.data)}')
        
        # ============================================================
        # STEP 3: BUILD CONSTRAINT BOUNDS
        # ============================================================
        self.l_constr = np.zeros(n_constraints)
        self.u_constr = np.zeros(n_constraints)
        
        # Dynamics constraints: equality (bounds = 0, will be updated for initial state)
        # Already zeros, no change needed
        
        # Input bounds: robot-level v and ω limits per step
        for k in range(N):
            # Index for v constraint at step k
            v_idx = n_dynamics + 2 * k
            # Index for ω constraint at step k
            omega_idx = n_dynamics + 2 * k + 1

            self.l_constr[v_idx] = self.v_min
            self.u_constr[v_idx] = self.v_max

            self.l_constr[omega_idx] = self.omega_min
            self.u_constr[omega_idx] = self.omega_max
        
        # ============================================================
        # STEP 4: SETUP OSQP SOLVER
        # ============================================================
        # Verify all dimensions match
        assert self.P.shape == (nz, nz), f"P matrix wrong shape: {self.P.shape}, expected ({nz}, {nz})"
        assert self.A_constr.shape == (n_constraints, nz), f"A matrix wrong shape: {self.A_constr.shape}, expected ({n_constraints}, {nz})"
        assert len(self.l_constr) == n_constraints, f"l_constr wrong length: {len(self.l_constr)}, expected {n_constraints}"
        assert len(self.u_constr) == n_constraints, f"u_constr wrong length: {len(self.u_constr)}, expected {n_constraints}"
        
        self.solver = osqp.OSQP()
        self.solver.setup(
            P=self.P,
            q=None,  # Linear cost term (zero)
            A=self.A_constr,
            l=self.l_constr,
            u=self.u_constr,
            verbose=False,
            warm_start=True,
            polish=True
        )
        
        self.initialized = True
        if self.logger:
            self.logger.info('✓ MPC optimizer initialized')
        return True

    def update_v_bounds(self, v_scale: float) -> None:
        """
        Update the linear velocity bounds for all steps by a scale factor.

        Args:
            v_scale: Scale factor in [0, 1] applied to |v_max|.
                     v_max_k = v_scale * self.v_max, v_min_k = -v_max_k.
        """
        if not self.initialized:
            return

        # Clamp scale to [0, 1]
        v_scale = max(0.0, min(1.0, float(v_scale)))

        N = self.N
        nx = self.nx
        n_dynamics = N * nx

        scaled_v_max = self.v_max * v_scale
        scaled_v_min = -scaled_v_max

        for k in range(N):
            v_idx = n_dynamics + 2 * k
            self.l_constr[v_idx] = scaled_v_min
            self.u_constr[v_idx] = scaled_v_max
    
    def update_matrices(self, ref_traj, slip_left, slip_right):
        """
        Update A and B matrices based on reference trajectory and slip ratios.
        
        Args:
            ref_traj: Reference trajectory list of waypoints [x, y, yaw, vx, vy, vyaw]
            slip_left: Left wheel slip ratio
            slip_right: Right wheel slip ratio
        """
        if not self.initialized:
            return
        
        N = self.N
        Ts = self.Ts
        r = self.r
        L = self.L
        nx = self.nx
        nu = self.nu
        
        # Get data array for efficient updates
        A_constr_data = self.A_constr.data.copy()
        
        # Verify data array size matches expected
        expected_size = len(self.A_constr.data)
        if len(A_constr_data) != expected_size:
            if self.logger:
                self.logger.error(f'A_constr data size mismatch: {len(A_constr_data)} != {expected_size}')
            return
        
        # Slip efficiency factors
        eta_L = 1.0 - slip_left
        eta_R = 1.0 - slip_right
        
        # Update A and B matrices for each step
        for k in range(N):
            # Get reference velocities from trajectory
            if k < len(ref_traj):
                ref_waypoint = ref_traj[k]
                v_ref = math.sqrt(ref_waypoint[3]**2 + ref_waypoint[4]**2)
                omega_ref = ref_waypoint[5]
            else:
                # Use last waypoint if not enough waypoints
                if len(ref_traj) > 0:
                    ref_waypoint = ref_traj[-1]
                    v_ref = math.sqrt(ref_waypoint[3]**2 + ref_waypoint[4]**2)
                    omega_ref = ref_waypoint[5]
                else:
                    v_ref = 0.0
                    omega_ref = 0.0
            
            # Compute A_k matrix (based on reference trajectory)
            # A_k = [[1, -ω_ref*Ts,      0],
            #        [ω_ref*Ts,    1,  v_ref*Ts],
            #        [0,          0,      1]]
            #
            # Lateral error dynamics (with our error definitions, matching PoseController):
            #   - Error state: x = [xe, ye, θe]^T in BODY frame
            #   - θe = yaw_ref - yaw_current  (same as PoseController d_yaw)
            #   - ye = e_lat = -sin(yaw)*dx + cos(yaw)*dy
            #
            # For a straight path with v_ref > 0 and small errors, using the standard
            # Kanayama-style error dynamics, we have approximately:
            #   ye_dot ≈  v_ref * θe
            # so the discrete-time term is A_k[1,2] =  v_ref * Ts.
            A_k = np.array([
                [1.0, -omega_ref * Ts, 0.0],
                [omega_ref * Ts, 1.0,  v_ref * Ts],
                [0.0, 0.0, 1.0]
            ])
            
            # Compute B_k matrix (based on slip ratios)
            # Differential drive forward kinematics:
            #   v = (r/2) * (ωL + ωR)  [forward velocity]
            #   ω = (r/L) * (ωR - ωL)  [angular velocity]
            #
            # Error state dynamics (linearized around small errors, Kanayama-style error):
            #   xe_dot ≈ v_ref - v
            #   ye_dot ≈ -v_ref * θe          [lateral drift, cannot be directly controlled]
            #   θe     = yaw_ref - yaw_current
            #   yaw_dot = ω = (r/L)*(ωR - ωL)
            #   ⇒ θe_dot = ω_ref - yaw_dot ≈ ω_ref - (r/L)*(ωR - ωL)
            #
            # So the continuous-time Jacobian J = ∂[xe_dot, ye_dot, θe_dot]/∂[ωL, ωR] is:
            #   Row 0 (xe):  ∂xe_dot/∂ωL = -r/2,   ∂xe_dot/∂ωR = -r/2
            #   Row 1 (ye):  ≈ 0, 0  (no direct control of lateral error)
            #   Row 2 (θe):  ∂θe_dot/∂ωL = +r/L,   ∂θe_dot/∂ωR = -r/L   (because θe_dot = ω_ref - ω)
            #
            # Discrete-time B_k is simply Ts * J. The constraint matrix stores -B_k, but the
            # underlying dynamics model uses B_k directly in x_{k+1} = A_k x_k + B_k u_k.
            B_k = Ts * np.array([
                [-r/2.0 * eta_L, -r/2.0 * eta_R],  # xe: correct sign for xe_dot = v_ref - v
                [0.0, 0.0],                         # ye: cannot be directly controlled
                [ r/L * eta_L, -r/L * eta_R]        # θe: correct sign for θe_dot = ω_ref - ω
            ])
            
            # Update sparse matrix data
            # Update -A_k terms (if k > 0, x_k exists)
            if k > 0 and k in self.A_indices:
                A_neg = -A_k
                for idx, matrix_idx in enumerate(self.A_indices[k]):
                    if matrix_idx >= len(A_constr_data):
                        if self.logger:
                            self.logger.error(f'A index out of bounds: k={k}, idx={idx}, matrix_idx={matrix_idx}, data_size={len(A_constr_data)}')
                        continue
                    i = idx // nx
                    j = idx % nx
                    if i < nx and j < nx:
                        A_constr_data[matrix_idx] = A_neg[i, j]
            
            # Update -B_k terms
            if k in self.B_indices:
                B_neg = -B_k
                for idx, matrix_idx in enumerate(self.B_indices[k]):
                    if matrix_idx >= len(A_constr_data):
                        if self.logger:
                            self.logger.error(f'B index out of bounds: k={k}, idx={idx}, matrix_idx={matrix_idx}, data_size={len(A_constr_data)}')
                        continue
                    i = idx // nu
                    j = idx % nu
                    if i < nx and j < nu:
                        A_constr_data[matrix_idx] = B_neg[i, j]
        
        # Update solver with new matrix data
        self.A_constr.data = A_constr_data
        self.solver.update(Ax=A_constr_data)
    
    def solve(self, current_error, ref_traj, slip_left, slip_right):
        """
        Solve MPC optimization problem.

        Args:
            current_error: Current error state [xe, ye, θe]^T (in body frame)
            ref_traj: Reference trajectory list
            slip_left: Left wheel slip ratio
            slip_right: Right wheel slip ratio

        Returns:
            Tuple (u0_optimal, solve_time_ms, solution):
                u0_optimal: [ωL, ωR]^T in rad/s
                solve_time_ms: Solve time in milliseconds
                solution: Full MPC solution vector
        """
        if not self.initialized:
            if not self.setup():
                return np.array([0.0, 0.0]), 0.0, None
        
        import time
        solve_start = time.time()
        
        # Update dynamics matrices
        self.update_matrices(ref_traj, slip_left, slip_right)
        
        # Update initial state constraint
        # For k=0: x_1 = A_0*current_error + B_0*u_0
        # Constraint: -B_0*u_0 + x_1 = A_0*current_error
        if len(ref_traj) > 0:
            ref_waypoint = ref_traj[0]
            v_ref = math.sqrt(ref_waypoint[3]**2 + ref_waypoint[4]**2)
            omega_ref = ref_waypoint[5]
        else:
            v_ref = 0.0
            omega_ref = 0.0
        
        A_0 = np.array([
            [1.0, -omega_ref * self.Ts, 0.0],
            [omega_ref * self.Ts, 1.0,  v_ref * self.Ts],
            [0.0, 0.0, 1.0]
        ])
        
        # Compute A_0*current_error (RHS of constraint)
        rhs_0 = A_0 @ current_error
        
        # Update bounds for first dynamics constraint (equality with RHS)
        for i in range(self.nx):
            self.l_constr[i] = rhs_0[i]
            self.u_constr[i] = rhs_0[i]

        # Also refresh robot-level v bounds (v_min/v_max may have been scaled dynamically)
        n_dynamics = self.N * self.nx
        for k in range(self.N):
            v_idx = n_dynamics + 2 * k
            self.l_constr[v_idx] = self.v_min
            self.u_constr[v_idx] = self.v_max
        
        # Warm start
        if self.prev_solution is not None:
            # Shift previous solution: [u0, x1, u1, x2, ...] -> [u1, x2, u2, x3, ...]
            prev_sol = self.prev_solution
            warm_start = np.zeros(self.nz)
            
            if len(prev_sol) >= (self.nu + self.nx):
                warm_start[:-self.nu] = prev_sol[self.nu:]
                warm_start[-self.nu:] = prev_sol[-self.nu:]
            
            self.solver.warm_start(x=warm_start)
        
        # Debug: Log solver inputs
        if self.logger and hasattr(self, 'solver_debug_enabled') and self.solver_debug_enabled:
            try:
                from scipy.sparse.linalg import norm as sparse_norm
                P_nnz = self.P.nnz
                P_norm = sparse_norm(self.P)
                A_nnz = self.A_constr.nnz
                A_norm = sparse_norm(self.A_constr)
                
                # Verify dynamics constraints are equalities
                n_dynamics = self.N * self.nx
                dynamics_eq_check = np.abs(self.l_constr[:n_dynamics] - self.u_constr[:n_dynamics]).max()
                
                self.logger.info(f'Solver Inputs: P matrix: nnz={P_nnz}, norm={P_norm:.6f}')
                self.logger.info(f'Solver Inputs: A matrix: nnz={A_nnz}, norm={A_norm:.6f}')
                self.logger.info(f'Solver Inputs: l bounds: min={self.l_constr.min():.6f}, max={self.l_constr.max():.6f}')
                self.logger.info(f'Solver Inputs: u bounds: min={self.u_constr.min():.6f}, max={self.u_constr.max():.6f}')
                self.logger.info(f'Solver Inputs: Dynamics equality check: max|l-u|={dynamics_eq_check:.10e} (should be <1e-9)')
                self.logger.info(f'Solver Inputs: Initial state constraint: [{self.l_constr[0]:.6f}, {self.l_constr[1]:.6f}, {self.l_constr[2]:.6f}]')
                self.logger.info(f'Solver Inputs: Current error state: [{current_error[0]:.6f}, {current_error[1]:.6f}, {current_error[2]:.6f}]')
            except Exception as e:
                if self.logger:
                    self.logger.warn(f'Error logging solver inputs: {e}')

        # Update solver bounds
        self.solver.update(l=self.l_constr, u=self.u_constr)

        # Solve
        result = self.solver.solve()

        if result.info.status != 'solved':
            if self.logger:
                self.logger.warn(f'MPC solve failed: {result.info.status}')
            return np.array([0.0, 0.0]), 0.0, None

        # Store result for debug access
        self._last_result = result

        # Extract control
        solution = result.x
        u0_optimal = solution[:self.nu]  # First control input [ωL, ωR]

        # Debug: Enhanced solver diagnostics
        if self.logger and hasattr(self, 'solver_debug_enabled') and self.solver_debug_enabled:
            try:
                # OSQP result info
                obj_val = result.info.obj_val if hasattr(result.info, 'obj_val') else 0.0
                iterations = result.info.iter if hasattr(result.info, 'iter') else 0
                run_time_ms = result.info.run_time * 1000.0 if hasattr(result.info, 'run_time') else 0.0
                
                self.logger.info(f'Solver Result: status={result.info.status}, '
                               f'obj_val={obj_val:.6f}, iter={iterations}, run_time={run_time_ms:.3f}ms')

                # Compute actual cost
                actual_cost = self.compute_solution_cost(solution, current_error, ref_traj)
                self.logger.info(f'Solver Result: computed_cost={actual_cost:.6f}, OSQP_obj_val={obj_val:.6f}')

                # Solution statistics
                solution_norm = np.linalg.norm(solution)
                self.logger.info(f'Solver Result: solution norm={solution_norm:.6f}')
                self.logger.info(f'Solver Result: u0_optimal=[{u0_optimal[0]:.6f}, {u0_optimal[1]:.6f}]')

                # Constraint satisfaction check
                A_solution = self.A_constr @ solution
                violations = np.zeros(len(A_solution))
                
                # Check each constraint properly
                for i in range(len(A_solution)):
                    if abs(self.l_constr[i] - self.u_constr[i]) < 1e-9:  # Equality constraint
                        # For equality: check if A_solution equals the bound value
                        violations[i] = abs(A_solution[i] - self.l_constr[i])
                    else:  # Inequality constraint
                        # For inequality: check if within [l_constr, u_constr]
                        if A_solution[i] < self.l_constr[i]:
                            violations[i] = self.l_constr[i] - A_solution[i]  # Violation: below lower bound
                        elif A_solution[i] > self.u_constr[i]:
                            violations[i] = A_solution[i] - self.u_constr[i]  # Violation: above upper bound
                        else:
                            violations[i] = 0.0  # Satisfied
                
                max_violation = violations.max()
                max_violation_idx = violations.argmax()
                
                self.logger.info(f'Solver Result: max_constraint_violation={max_violation:.10e} at constraint {max_violation_idx}')
                
                # Log details of the most violated constraint
                if max_violation > 1e-6:
                    self.logger.warn(f'  Constraint {max_violation_idx}: A_solution={A_solution[max_violation_idx]:.6f}, '
                                   f'l={self.l_constr[max_violation_idx]:.6f}, u={self.u_constr[max_violation_idx]:.6f}')
                    
                    # Count how many constraints are violated
                    n_violated = np.sum(violations > 1e-6)
                    n_equality = np.sum(np.abs(self.l_constr - self.u_constr) < 1e-9)
                    n_dynamics = self.N * self.nx
                    
                    # Check dynamics constraints specifically
                    dynamics_violations = violations[:n_dynamics]
                    input_violations = violations[n_dynamics:]
                    max_dynamics_violation = dynamics_violations.max() if len(dynamics_violations) > 0 else 0.0
                    max_input_violation = input_violations.max() if len(input_violations) > 0 else 0.0
                    
                    self.logger.warn(f'  Violated constraints: {n_violated}/{len(violations)} '
                                   f'(Equality constraints: {n_equality}, Dynamics: {n_dynamics})')
                    self.logger.warn(f'  Max violations - Dynamics: {max_dynamics_violation:.10e}, '
                                   f'Input bounds: {max_input_violation:.10e}')
                    
                    # Check if initial state constraint is violated
                    if max_violation_idx < self.nx:
                        self.logger.warn(f'  Initial state constraint violated! Constraint {max_violation_idx} of first {self.nx}')
                        self.logger.warn(f'    Expected: {self.l_constr[max_violation_idx]:.6f}, '
                                       f'Got: {A_solution[max_violation_idx]:.6f}')
            except Exception as e:
                if self.logger:
                    self.logger.warn(f'Error logging solver results: {e}')

        # Store solution for warm start next time
        self.prev_solution = solution

        solve_time_ms = (time.time() - solve_start) * 1000.0
        return u0_optimal, solve_time_ms, solution
    
    def compute_solution_cost(self, solution, current_error, ref_traj):
        """
        Compute the actual cost value for the MPC solution.

        Cost = sum(x_k^T Q x_k) + sum((u_k - u_{k-1})^T R_delta (u_k - u_{k-1}))

        Solution vector format: [u0, x1, u1, x2, u2, ..., u_{N-1}, xN]

        Args:
            solution: Full MPC solution vector
            current_error: Current error state (unused but kept for API consistency)
            ref_traj: Reference trajectory (unused but kept for API consistency)

        Returns:
            Computed cost value (float)
        """
        if solution is None or len(solution) < self.nz:
            return 0.0

        cost = 0.0
        Q_dense = self.Q.toarray()
        R_delta_diag = self.R_delta.diagonal()

        # State costs: sum(x_k^T Q x_k) for k=1 to N
        # x_k is at position: k * (nu + nx) - nx + nu = k * (nu + nx) + nu - nx (simplified)
        for k in range(1, self.N + 1):
            # x_k position in solution: [u0, x1, u1, x2, ...] -> x_k at k*(nu+nx) - nx + nu
            x_idx = k * (self.nu + self.nx) - self.nx  # Actual: k*(nu+nx) + nu - nx, but simplified
            # Correct indexing: x1 is at nu, x2 is at nu+nx+nu, etc.
            # Actually: x_k is at (k-1)*(nu+nx) + nu
            x_idx = (k - 1) * (self.nu + self.nx) + self.nu
            if x_idx + self.nx <= len(solution):
                x_k = solution[x_idx:x_idx + self.nx]
                cost += x_k @ Q_dense @ x_k

        # Control change costs: sum((u_k - u_{k-1})^T R_delta (u_k - u_{k-1}))
        # u_0 is at 0, u_1 is at nu+nx, u_2 is at 2*(nu+nx), etc.
        for k in range(1, self.N):
            u_km1_idx = (k - 1) * (self.nu + self.nx)  # u_{k-1}
            u_k_idx = k * (self.nu + self.nx)  # u_k
            if u_k_idx + self.nu <= len(solution) and u_km1_idx + self.nu <= len(solution):
                u_km1 = solution[u_km1_idx:u_km1_idx + self.nu]
                u_k = solution[u_k_idx:u_k_idx + self.nu]
                delta_u = u_k - u_km1
                cost += delta_u[0]**2 * R_delta_diag[0] + delta_u[1]**2 * R_delta_diag[1]

        return cost


class MPCAutonomousController(Node):
    """
    MPC-based autonomous controller for differential drive robot.
    """
    
    def __init__(self):
        super().__init__('mpc_autonomous_controller')
        
        # ============================================================
        # PARAMETER DECLARATION
        # ============================================================
        
        # Robot kinematics parameters
        self.declare_parameter('wheel_radius', 0.09)      # m
        self.declare_parameter('wheel_base', 0.32)         # m (track width)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', True)
        
        # Control parameters
        self.declare_parameter('control_frequency', 10.0)  # Hz
        self.declare_parameter('max_linear_velocity', 0.5) # m/s
        self.declare_parameter('max_angular_velocity', 1.0) # rad/s
        
        # MPC parameters
        self.declare_parameter('mpc_horizon', 50)           # Prediction horizon steps
        self.declare_parameter('mpc_dt', 0.1)             # Time step for MPC (s)
        
        # Slip estimation parameters
        self.declare_parameter('slip_history_length', 100)  # Buffer size (should be large enough for 1 second)
        self.declare_parameter('slip_estimation_window', 1.0)  # Time window for slip estimation (seconds)
        
        # Waypoint parameters
        self.declare_parameter('lookahead_distance', 0.5)  # m
        # Optional CSV file containing a sequence of waypoints for fully autonomous runs
        self.declare_parameter('waypoints_csv_path', '')   # Path to CSV with waypoints [x,y] per row
        
        # Stopping criteria
        self.declare_parameter('target_reached_threshold', 0.01)  # Stop when within threshold of target (m)
        
        # Topic names
        self.declare_parameter('odometry_topic', '/Odometry_tilt_corrected_diff')
        self.declare_parameter('left_control_topic', '/left/control_message')
        self.declare_parameter('right_control_topic', '/right/control_message')
        self.declare_parameter('left_encoder_topic', '/left/controller_status')
        self.declare_parameter('right_encoder_topic', '/right/controller_status')
        
        # ============================================================
        # PARAMETER RETRIEVAL
        # ============================================================
        
        # Robot kinematics
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value
        
        # Control parameters
        self.control_freq = self.get_parameter('control_frequency').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        
        # MPC parameters
        self.mpc_horizon = self.get_parameter('mpc_horizon').value
        self.mpc_dt = self.get_parameter('mpc_dt').value
        
        # Slip estimation parameters
        self.slip_history_length = self.get_parameter('slip_history_length').value
        self.slip_estimation_window = self.get_parameter('slip_estimation_window').value
        
        # Waypoint parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.waypoints_csv_path = self.get_parameter('waypoints_csv_path').value
        
        # Stopping criteria
        self.target_reached_threshold = self.get_parameter('target_reached_threshold').value
        
        # Autonomy and behavior flags (can be overridden per-launch)
        self.declare_parameter('mpc_autonomy_enabled_default', False)
        self.declare_parameter('enable_yaw_gating', True)  # If True, gate v_ref/v_bounds based on yaw error near start of segment
        self.autonomy_enabled: bool = bool(self.get_parameter('mpc_autonomy_enabled_default').value)
        self.enable_yaw_gating: bool = bool(self.get_parameter('enable_yaw_gating').value)
        
        # Topic names
        odometry_topic = self.get_parameter('odometry_topic').value
        left_control_topic = self.get_parameter('left_control_topic').value
        right_control_topic = self.get_parameter('right_control_topic').value
        left_encoder_topic = self.get_parameter('left_encoder_topic').value
        right_encoder_topic = self.get_parameter('right_encoder_topic').value
        
        # ============================================================
        # STATE VARIABLES INITIALIZATION
        # ============================================================
        
        # Current pose from Fast-LIO2 (odometry)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vyaw = 0.0
        self.pose_initialized = False
        self.last_odom_time = self.get_clock().now()
        
        # Previous pose for velocity estimation
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.0
        self.prev_odom_time = None
        
        # Wheel encoder measurements
        self.left_position = 0.0      # Position in turns
        self.left_velocity = 0.0     # Velocity in rev/s (from encoder)
        self.right_position = 0.0    # Position in turns
        self.right_velocity = 0.0    # Velocity in rev/s (from encoder)
        self.left_encoder_initialized = False
        self.right_encoder_initialized = False
        
        # History buffers for slip estimation
        # Store past Fast-LIO positions: [x, y, yaw, timestamp] (velocities estimated from positions)
        self.odom_history = deque(maxlen=self.slip_history_length)
        # Store past encoder measurements: [left_vel, right_vel, timestamp] (velocities from encoder messages)
        self.encoder_history = deque(maxlen=self.slip_history_length)
        
        # Estimated slip parameters (updated by slip estimation function)
        self.left_slip_ratio = 0.0    # Slip ratio for left wheel
        self.right_slip_ratio = 0.0   # Slip ratio for right wheel
        
        # ============================================================
        # MPC OPTIMIZER INITIALIZATION
        # ============================================================
        
        # MPC cost weights (penalize xe, ye more than yaw_e)
        # Note: Q_ye is higher because lateral errors must be corrected through rotation (harder to correct)
        self.declare_parameter('mpc_Q_xe', 15.0)  # Weight for position error x
        self.declare_parameter('mpc_Q_ye', 20.0)  # Weight for position error y (much higher for lateral correction)
        self.declare_parameter('mpc_Q_yaw',5.0)  # Weight for yaw error (higher to help lateral correction)
        self.declare_parameter('mpc_R_delta', 0.00010)  # Weight for control input change
        
        # Weight scaling: increase weights linearly into the future (separate factors for each error term)
        # Weight at step k = base_weight * (1 + weight_increase * k)
        # Example: 0.1 means 10% increase per step (step 0: 1.0x, step 1: 1.1x, step 2: 1.2x, ...)
        self.declare_parameter('mpc_weight_increase_xe', 0.0)  # Weight increase factor for xe error
        self.declare_parameter('mpc_weight_increase_ye', 0.1)  # Weight increase factor for ye error
        self.declare_parameter('mpc_weight_increase_yaw', 0.0)  # Weight increase factor for yaw error
        
        # Solver debug parameter
        self.declare_parameter('solver_debug_enabled', False)  # Enable detailed solver debugging
        
        # Get cost weights
        mpc_Q_xe = self.get_parameter('mpc_Q_xe').value
        mpc_Q_ye = self.get_parameter('mpc_Q_ye').value
        mpc_Q_yaw = self.get_parameter('mpc_Q_yaw').value
        mpc_R_delta = self.get_parameter('mpc_R_delta').value
        mpc_weight_increase_xe = self.get_parameter('mpc_weight_increase_xe').value
        mpc_weight_increase_ye = self.get_parameter('mpc_weight_increase_ye').value
        mpc_weight_increase_yaw = self.get_parameter('mpc_weight_increase_yaw').value
        
        # Get solver debug setting
        self.solver_debug_enabled = self.get_parameter('solver_debug_enabled').value
        
        # Compute wheel velocity limits (rad/s) from max linear velocity.
        # These are used as physical wheel caps; MPC constraints are applied
        # on robot linear and angular velocity (v, ω), not directly on wheels.
        max_wheel_vel_rev_per_s = self.max_linear_vel / (self.wheel_radius * 2.0 * math.pi)
        w_max = max_wheel_vel_rev_per_s * 2.0 * math.pi  # rad/s
        w_min = -w_max
        
        # Initialize MPC optimizer
        self.mpc_optimizer = SlipAwareMPC(
            N=self.mpc_horizon,
            Ts=self.mpc_dt,
            r=self.wheel_radius,
            L=self.wheel_base,
            w_min=w_min,
            w_max=w_max,
            Q_xe=mpc_Q_xe,
            Q_ye=mpc_Q_ye,
            Q_yaw=mpc_Q_yaw,
            R_delta=mpc_R_delta,
            logger=self.get_logger(),
            weight_increase_xe=mpc_weight_increase_xe,
            weight_increase_ye=mpc_weight_increase_ye,
            weight_increase_yaw=mpc_weight_increase_yaw,
            # Robot-level velocity limits (used for constraints inside MPC)
            v_min=-self.max_linear_vel,
            v_max=self.max_linear_vel,
            omega_min=-self.max_angular_vel,
            omega_max=self.max_angular_vel
        )
        
        # Set solver debug flag on optimizer
        self.mpc_optimizer.solver_debug_enabled = self.solver_debug_enabled
        
        # Target pose
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.has_target = False
        self.target_reached = False  # Flag to track if target has been reached
        
        # Path planning state for current single target
        self.path_start_x = 0.0  # Initial position when target was set
        self.path_start_y = 0.0
        self.path_start_yaw = 0.0
        self.path_initialized = False  # Whether path has been initialized
        
        # Waypoint sequence state (for CSV-based navigation)
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_index: int = 0
        self.waypoint_navigation_active: bool = False
        self.previous_waypoint: Optional[Tuple[float, float]] = None
        
        # Pending waypoints from F2C GUI (stored until "Start Navigation" is pressed)
        self.pending_waypoints: List[Tuple[float, float]] = []
        
        # Control outputs
        self.left_wheel_velocity = 0.0   # rad/s (motor turns/s)
        self.right_wheel_velocity = 0.0  # rad/s (motor turns/s)
        
        # ============================================================
        # ROS SUBSCRIBERS
        # ============================================================
        
        # Subscriber to tilt-corrected odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            odometry_topic,
            self.odometry_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(f'Subscribed to odometry: {odometry_topic}')
        
        # Subscribers to wheel encoder measurements
        self.left_encoder_sub = self.create_subscription(
            ControllerStatus,
            left_encoder_topic,
            self.left_encoder_callback,
            10
        )
        self.get_logger().info(f'Subscribed to left encoder: {left_encoder_topic}')
        
        self.right_encoder_sub = self.create_subscription(
            ControllerStatus,
            right_encoder_topic,
            self.right_encoder_callback,
            10
        )
        self.get_logger().info(f'Subscribed to right encoder: {right_encoder_topic}')
        
        # Subscriber to set target pose
        self.set_target_sub = self.create_subscription(
            Float64MultiArray,
            '/set_target_pose',
            self.set_target_callback,
            10
        )
        self.get_logger().info('Subscribed to target pose: /set_target_pose')
        
        # Subscriber to start waypoint navigation from CSV (similar to pose_controller)
        # Receives std_msgs/String with CSV file path.
        self.waypoint_nav_sub = self.create_subscription(
            String,
            '/start_waypoint_navigation',
            self.start_waypoint_navigation_callback,
            10
        )
        self.get_logger().info('Subscribed to waypoint CSV topic: /start_waypoint_navigation')
        
        # Subscriber for direct waypoint arrays from F2C GUI
        # Format (same as pose_controller): [x1,y1,z1,yaw1, x2,y2,z2,yaw2, ...]
        # or [0.0] as a potential start signal (ignored here, we start on full list).
        self.f2c_waypoint_array_sub = self.create_subscription(
            Float64MultiArray,
            '/f2c_waypoints',
            self.f2c_waypoint_array_callback,
            10
        )
        self.get_logger().info('Subscribed to F2C waypoints: /f2c_waypoints')
        
        # Subscriber to enable/disable MPC autonomy (e.g. from a key-watcher node).
        # When autonomy is disabled, this node will idle and publish zero velocity
        # even if waypoints/targets have been received.
        self.autonomy_enable_sub = self.create_subscription(
            Bool,
            '/mpc_autonomy_enable',
            self.autonomy_enable_callback,
            10
        )
        self.get_logger().info('Subscribed to autonomy enable topic: /mpc_autonomy_enable')
        
        # ============================================================
        # ROS PUBLISHERS
        # ============================================================
        
        # Publishers for wheel velocity commands
        self.left_pub = self.create_publisher(
            ControlMessage,
            left_control_topic,
            10
        )
        self.get_logger().info(f'Publishing to left control: {left_control_topic}')
        
        self.right_pub = self.create_publisher(
            ControlMessage,
            right_control_topic,
            10
        )
        self.get_logger().info(f'Publishing to right control: {right_control_topic}')
        
        # Diagnostic publishers
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mpc_control/cmd_twist',
            10
        )

        self.slip_pub = self.create_publisher(
            Float64MultiArray,
            '/mpc_control/slip_ratios',
            10
        )

        # Additional debugging publishers
        self.mpc_controls_pub = self.create_publisher(
            Float64MultiArray,
            '/mpc_control/control_sequence',
            10
        )

        self.error_state_pub = self.create_publisher(
            Float64MultiArray,
            '/mpc_control/error_state',
            10
        )

        self.ref_trajectory_pub = self.create_publisher(
            Float64MultiArray,
            '/mpc_control/reference_trajectory',
            10
        )

        self.solver_diagnostics_pub = self.create_publisher(
            Float64MultiArray,
            '/mpc_control/solver_diagnostics',
            10
        )

        self.pose_velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/mpc_control/pose_velocity',
            10
        )

        self.mpc_bounds_pub = self.create_publisher(
            Float64MultiArray,
            '/mpc_control/constraint_bounds',
            10
        )

        # Solver debug publishers
        self.solver_inputs_pub = self.create_publisher(
            Float64MultiArray,
            '/mpc_control/solver_inputs',
            10
        )

        self.solver_cost_pub = self.create_publisher(
            Float64MultiArray,
            '/mpc_control/solver_cost',
            10
        )
        
        # ============================================================
        # SERVICES
        # ============================================================
        
        # Service to trigger graceful shutdown (zero velocity → disarm → stop nodes)
        self.soft_shutdown_srv = self.create_service(
            Trigger,
            '/mpc_autonomous_controller/soft_shutdown',
            self._soft_shutdown_service
        )
        
        # ODrive axis state clients for arming/disarming
        self.left_axis_client = self.create_client(
            AxisState,
            '/left/request_axis_state'
        )
        self.right_axis_client = self.create_client(
            AxisState,
            '/right/request_axis_state'
        )
        
        # ODrive clear error clients
        self.left_clear_client = self.create_client(
            Empty,
            '/left/clear_errors'
        )
        self.right_clear_client = self.create_client(
            Empty,
            '/right/clear_errors'
        )
        
        # Shutdown service client (for calling shutdown_service node)
        self.shutdown_client = self.create_client(
            Trigger,
            '/shutdown_mapping'
        )
        
        # Motor arming state
        self._arm_attempts = 0
        self._arm_max_attempts = 5
        self._arm_timer = self.create_timer(1.0, self._attempt_arm_motors)
        self._last_log_time_ns = {}  # For throttled logging
        
        # Control loop timing diagnostics
        self._last_control_time_ns: Optional[int] = None
        self._control_freq_ema: Optional[float] = None
        self._control_dt_ema: Optional[float] = None
        self._last_control_freq_log_time_ns: int = 0
        
        # ============================================================
        # CONTROL LOOP TIMER
        # ============================================================
        
        control_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        # ============================================================
        # STARTUP LOGGING
        # ============================================================
        
        self.get_logger().info('='*70)
        self.get_logger().info('MPC Autonomous Controller Node Started')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Robot Parameters:')
        self.get_logger().info(f'  Wheel radius: {self.wheel_radius:.3f} m')
        self.get_logger().info(f'  Wheel base: {self.wheel_base:.3f} m')
        self.get_logger().info(f'  Gear ratio: {self.gear_ratio:.2f}')
        self.get_logger().info(f'  Invert left/right: {self.invert_left}/{self.invert_right}')
        self.get_logger().info(f'')
        self.get_logger().info(f'Control Parameters:')
        self.get_logger().info(f'  Control frequency: {self.control_freq:.1f} Hz')
        self.get_logger().info(f'  Max linear velocity: {self.max_linear_vel:.2f} m/s')
        self.get_logger().info(f'  Max angular velocity: {self.max_angular_vel:.2f} rad/s')
        self.get_logger().info(f'')
        self.get_logger().info(f'MPC Parameters:')
        self.get_logger().info(f'  Horizon: {self.mpc_horizon} steps')
        self.get_logger().info(f'  Time step: {self.mpc_dt:.3f} s')
        self.get_logger().info(f'')
        self.get_logger().info(f'Slip Estimation:')
        self.get_logger().info(f'  History buffer size: {self.slip_history_length} samples')
        self.get_logger().info(f'  Estimation window: {self.slip_estimation_window:.1f} seconds')
        self.get_logger().info(f'')
        self.get_logger().info(f'Dependencies:')
        self.get_logger().info(f'  OSQP available: {OSQP_AVAILABLE}')
        self.get_logger().info(f'  scipy available: {SCIPY_AVAILABLE}')
        self.get_logger().info(f'')
        self.get_logger().info(f'Diagnostic Topics:')
        self.get_logger().info(f'  /mpc_control/cmd_twist - Current commanded twist')
        self.get_logger().info(f'  /mpc_control/slip_ratios - Wheel slip ratios [left, right]')
        self.get_logger().info(f'  /mpc_control/control_sequence - MPC control sequence over horizon')
        self.get_logger().info(f'  /mpc_control/error_state - Current error state [xe, ye, θe]')
        self.get_logger().info(f'  /mpc_control/reference_trajectory - Current reference waypoint')
        self.get_logger().info(f'  /mpc_control/solver_diagnostics - Solver info [solve_time, obj_val, iter, osqp_time]')
        self.get_logger().info(f'  /mpc_control/pose_velocity - Current pose and velocity')
        self.get_logger().info(f'  /mpc_control/constraint_bounds - MPC constraint bounds (sample)')
        if self.solver_debug_enabled:
            self.get_logger().info(f'  /mpc_control/solver_inputs - Solver inputs [P_nnz, A_nnz, l_min, l_max, u_min, u_max, xe, ye, θe]')
            self.get_logger().info(f'  /mpc_control/solver_cost - Solver cost [computed_cost, OSQP_obj_val, iter, solve_time_ms]')
        self.get_logger().info('='*70)
        
        # ============================================================
        # OPTIONAL WAYPOINT CSV INITIALIZATION
        # ============================================================
        if isinstance(self.waypoints_csv_path, str) and self.waypoints_csv_path.strip():
            loaded = self._load_waypoints_from_csv(self.waypoints_csv_path.strip())
            if loaded:
                self.waypoints = loaded
                self.waypoint_navigation_active = True
                self.current_waypoint_index = 0
                self.previous_waypoint = None
                self.get_logger().info(
                    f'Waypoint CSV loaded: {len(self.waypoints)} waypoints from '
                    f'"{self.waypoints_csv_path}"'
                )
            else:
                self.get_logger().warn(
                    f'Waypoint CSV path set but no waypoints loaded: "{self.waypoints_csv_path}"'
                )
    
    # ============================================================
    # CALLBACK: ODOMETRY
    # ============================================================
    
    def odometry_callback(self, msg: Odometry):
        """
        Callback for tilt-corrected odometry messages.
        Extracts pose, estimates velocities from position differences, and stores in history.
        """
        # Extract pose
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        q_local = (
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )
        self.current_yaw = self.quaternion_to_yaw(q_local[0], q_local[1], q_local[2], q_local[3])
        
        # Estimate velocities from position differences
        current_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        
        if self.prev_odom_time is not None:
            # Compute time difference
            dt = current_time - self.prev_odom_time
            
            if dt > 1e-6:  # Avoid division by zero
                # Position differences in global frame
                dx_global = self.current_x - self.prev_x
                dy_global = self.current_y - self.prev_y
                
                # Velocity components in global frame (keep in global frame)
                self.current_vx = dx_global / dt
                self.current_vy = dy_global / dt
                
                # Angular velocity: yaw change / time
                dyaw = self.normalize_angle(self.current_yaw - self.prev_yaw)
                self.current_vyaw = dyaw / dt
            else:
                # Time difference too small, keep previous velocities
                pass
        else:
            # First measurement, velocities are zero
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_vyaw = 0.0
        
        # Store only positions in history for slip estimation (velocities will be estimated)
        timestamp = current_time
        self.odom_history.append({
            'x': self.current_x,
            'y': self.current_y,
            'yaw': self.current_yaw,
            'timestamp': timestamp
        })
        
        # Update previous pose for next velocity estimation
        self.prev_x = self.current_x
        self.prev_y = self.current_y
        self.prev_yaw = self.current_yaw
        self.prev_odom_time = current_time
        
        # Mark pose as initialized
        if not self.pose_initialized:
            self.pose_initialized = True
            self.get_logger().info(
                f'✓ Odometry initialized: x={self.current_x:.3f}, '
                f'y={self.current_y:.3f}, yaw={math.degrees(self.current_yaw):.1f}°'
            )
        
        # Update last odometry time
        self.last_odom_time = self.get_clock().now()
    
    # ============================================================
    # CALLBACK: WHEEL ENCODERS
    # ============================================================
    
    def left_encoder_callback(self, msg: ControllerStatus):
        """
        Callback for left wheel encoder measurements.
        Stores position and velocity from encoder messages.
        """
        try:
            self.left_position = float(msg.pos_estimate)  # Position in turns
            self.left_velocity = float(msg.vel_estimate)  # Velocity in rev/s
            
            # Store in history for slip estimation (velocities from encoder)
            timestamp = self.get_clock().now().nanoseconds / 1e9
            self.encoder_history.append({
                'left_vel': self.left_velocity,
                'right_vel': self.right_velocity,  # Use last known right velocity
                'timestamp': timestamp
            })
            
            if not self.left_encoder_initialized:
                self.left_encoder_initialized = True
                self.get_logger().info('✓ Left encoder initialized')
        except Exception as e:
            self.get_logger().warn(f'Error in left encoder callback: {e}')
    
    def right_encoder_callback(self, msg: ControllerStatus):
        """
        Callback for right wheel encoder measurements.
        Stores position and velocity from encoder messages.
        """
        try:
            self.right_position = float(msg.pos_estimate)  # Position in turns
            self.right_velocity = float(msg.vel_estimate)  # Velocity in rev/s
            
            # Update the most recent encoder history entry if it exists
            if len(self.encoder_history) > 0:
                self.encoder_history[-1]['right_vel'] = self.right_velocity
            
            if not self.right_encoder_initialized:
                self.right_encoder_initialized = True
                self.get_logger().info('✓ Right encoder initialized')
        except Exception as e:
            self.get_logger().warn(f'Error in right encoder callback: {e}')
    
    # ============================================================
    # CALLBACK: SET TARGET POSE
    # ============================================================
    
    def set_target_callback(self, msg: Float64MultiArray):
        """
        Topic callback to set a new target pose via Float64MultiArray [x, y, z, yaw(rad)].
        Initializes the straight-line path from current position to target.
        """
        try:
            data = list(msg.data)
            if len(data) < 4:
                self.get_logger().warning('set_target_pose requires 4 elements: [x,y,z,yaw]')
                return
            
            # Only set path start if pose is initialized
            if not self.pose_initialized:
                self.get_logger().warning('Cannot set target: odometry not initialized yet')
                return
            
            # Store target
            self.target_x = float(data[0])
            self.target_y = float(data[1])
            self.target_yaw = float(data[3])
            
            # Initialize path start point (current position when target is set)
            self.path_start_x = self.current_x
            self.path_start_y = self.current_y
            self.path_start_yaw = self.current_yaw
            self.path_initialized = True
            self.has_target = True
            self.target_reached = False  # Reset target reached flag for new target
            
            self.get_logger().info(
                f'🎯 New target set: x={self.target_x:.2f}m, y={self.target_y:.2f}m, '
                f'yaw={math.degrees(self.target_yaw):.1f}°'
            )
            self.get_logger().info(
                f'   Path start: x={self.path_start_x:.2f}m, y={self.path_start_y:.2f}m'
            )
        except Exception as e:
            self.get_logger().warning(f'Invalid set_target_pose payload: {e}')
    
    # ============================================================
    # WAYPOINT GENERATION
    # ============================================================
    
    def generate_local_waypoints(self) -> List[np.ndarray]:
        """
        Generate local waypoints for MPC based on straight-line path.

        Approach:
        1. Compute straight-line path from initial position to target global waypoint
        2. Find closest point on the line to current position
        3. Generate 'prediction horizon' number of waypoints along the line
        4. Use fixed spacing at cruising speed (0.4 m/s) for most cases
        5. Only adjust spacing when very close to target to ensure accurate final approach

        Returns:
            List of waypoint states [x, y, yaw, vx, vy, vyaw] for MPC horizon
        """
        if not self.path_initialized or not self.has_target:
            return []

        # Compute straight-line path vector
        path_dx = self.target_x - self.path_start_x
        path_dy = self.target_y - self.path_start_y
        path_length = math.sqrt(path_dx*path_dx + path_dy*path_dy)

        # Debug logging (reduced verbosity)
        if self.get_logger().get_effective_level() <= 10:  # Only if DEBUG level
            self.get_logger().debug(
                f'Path: start=({self.path_start_x:.3f}, {self.path_start_y:.3f}), '
                f'target=({self.target_x:.3f}, {self.target_y:.3f}), length={path_length:.3f}m'
            )
        
        if path_length < 1e-6:
            # Already at target, return target as all waypoints
            waypoints = []
            for _ in range(self.mpc_horizon):
                waypoint = np.array([
                    self.target_x,
                    self.target_y,
                    self.target_yaw,
                    0.0,  # vx
                    0.0,  # vy
                    0.0   # vyaw
                ])
                waypoints.append(waypoint)
            return waypoints
        
        # Normalize path direction
        path_dir_x = path_dx / path_length
        path_dir_y = path_dy / path_length
        
        # Find closest point on the line to current position
        # Vector from path start to current position
        to_current_x = self.current_x - self.path_start_x
        to_current_y = self.current_y - self.path_start_y

        # Project current position onto path line
        # Distance along path (parameter t: 0 = start, 1 = end)
        t_closest = (to_current_x * path_dir_x + to_current_y * path_dir_y) / path_length

        # Clamp to path segment [0, 1]
        t_closest = np.clip(t_closest, 0.0, 1.0)

        # Closest point on path
        closest_x = self.path_start_x + t_closest * path_dx
        closest_y = self.path_start_y + t_closest * path_dy

        # Compute waypoint spacing: distance traveled at cruising speed for one time step
        cruising_speed = 0.5 # Fixed cruising speed as requested
        waypoint_spacing = cruising_speed * self.mpc_dt  # m (should be 0.04m)

        # Compute distance from closest point to target along path
        distance_to_target_along_path = (1.0 - t_closest) * path_length
        
        # Generate waypoints along the path
        # Method:
        #   - Far from the target: fixed spacing based on cruising speed.
        #   - Near the target (within proximity_threshold): compress horizon into the
        #     remaining distance, but enforce a minimum spacing between waypoints.
        #     Once the target is reached, repeat the target waypoint.
        waypoints = []
        waypoint_positions = []
        
        proximity_threshold = 0.5  # [m] distance at which to start compressing waypoints
        min_spacing = 0.04         # [m] minimum distance between consecutive waypoints
        
        if distance_to_target_along_path > proximity_threshold:
            # FAR REGION: use fixed cruising spacing along the path
            adjusted_spacing = waypoint_spacing
            
            for k in range(self.mpc_horizon):
                # Distance along path from closest point
                distance_along_path = adjusted_spacing * (k + 1)
                
                # Parameter along path (0 = start, 1 = end)
                t_waypoint = t_closest + (distance_along_path / path_length)
                t_waypoint = np.clip(t_waypoint, 0.0, 1.0)
                
                waypoint_x = self.path_start_x + t_waypoint * path_dx
                waypoint_y = self.path_start_y + t_waypoint * path_dy
                
                heading_to_target = math.atan2(path_dy, path_dx)
                waypoint_yaw = self.normalize_angle(heading_to_target)
                
                waypoint_positions.append((waypoint_x, waypoint_y, waypoint_yaw))
        else:
            # NEAR REGION: compress horizon inside remaining distance, enforcing a
            # minimum spacing. Once we reach the target, repeat the target point.
            for k in range(self.mpc_horizon):
                desired_distance = min_spacing * (k + 1)
                
                if desired_distance < distance_to_target_along_path:
                    # Place waypoint along the path before the target
                    dist_from_start = t_closest * path_length + desired_distance
                    t_waypoint = dist_from_start / path_length
                else:
                    # We've reached (or passed) the target: clamp to target
                    t_waypoint = 1.0
                
                t_waypoint = np.clip(t_waypoint, 0.0, 1.0)
                
                waypoint_x = self.path_start_x + t_waypoint * path_dx
                waypoint_y = self.path_start_y + t_waypoint * path_dy
                
                heading_to_target = math.atan2(path_dy, path_dx)
                waypoint_yaw = self.normalize_angle(heading_to_target)
                
                waypoint_positions.append((waypoint_x, waypoint_y, waypoint_yaw))
        
        # Compute velocities for each waypoint.
        # For a straight line path: v_ref is along path direction. We can optionally
        # shape v_ref near the start of the segment based on distance along the line,
        # and (optionally) also gate based on yaw error if enable_yaw_gating is True.
        cruising_speed = 0.5  # m/s along path (maximum desired forward speed)
        path_heading = math.atan2(path_dy, path_dx)  # Heading along path

        # Gating region: fixed distance from start of line where we may shape v_ref / bounds
        d_gate = 0.10  # [m] gate region length from path start

        # Distance from start to closest point along the path
        s_closest = t_closest * path_length

        # Compute scalar scales in [0,1]:
        #   - v_scale_bounds: used to gate MPC v-constraints (how much forward speed is allowed)
        #   - v_scale_ref:    used to gate v_ref in the reference trajectory (how much we "ask for")
        if self.enable_yaw_gating:
            # -------- Yaw + distance-based gating (feature enabled) --------
            # Yaw error between robot and path heading
            yaw_err = self.normalize_angle(path_heading - self.current_yaw)
            abs_yaw_err = abs(yaw_err)

            # Yaw thresholds:
            # - |yaw_err| >= yaw_stop: v_ref ~ 0 (prefer pure rotation)
            # - |yaw_err| <= yaw_full: v_ref ~ cruising_speed
            # - In between: ramp 0 -> cruising_speed
            yaw_stop = math.radians(5.0)  # tighter thresholds
            yaw_full = math.radians(2.0)

            if s_closest <= d_gate:
                # Yaw-based scaling near start
                if abs_yaw_err >= yaw_stop:
                    yaw_scale = 0.0
                elif abs_yaw_err <= yaw_full:
                    yaw_scale = 1.0
                else:
                    ratio_yaw = (abs_yaw_err - yaw_full) / (yaw_stop - yaw_full)
                    ratio_yaw = max(0.0, min(1.0, ratio_yaw))
                    yaw_scale = 1.0 - ratio_yaw

                # Distance-based shaping (0 -> 1 over first d_gate meters)
                ratio_t = max(0.0, min(1.0, s_closest / d_gate))
                p_shape = 2.0
                f_t = ratio_t ** p_shape

                # Bounds: use yaw only (optionally slightly tightened)
                v_scale_bounds = yaw_scale
                # Reference: combine yaw and distance shaping
                v_scale_ref = yaw_scale * f_t
            else:
                v_scale_bounds = 1.0
                v_scale_ref = 1.0
        else:
            # -------- Distance-only shaping (yaw gating disabled) --------
            if s_closest <= d_gate:
                ratio_t = max(0.0, min(1.0, s_closest / d_gate))  # in [0,1]
                p_shape = 2.0  # shape exponent; >1 makes behavior sharper near start
                f_t = ratio_t ** p_shape

                # Bounds: keep full velocity bounds (no gating on constraints)
                v_scale_bounds = 1.0
                # Reference: distance-based shaping (small v_ref very close to start)
                v_scale_ref = f_t
            else:
                v_scale_bounds = 1.0
                v_scale_ref = 1.0

        # Store v_scale for use in MPC constraints (velocity bounds gating)
        self._current_v_scale = float(v_scale_bounds)

        for k in range(self.mpc_horizon):
            waypoint_x, waypoint_y, waypoint_yaw = waypoint_positions[k]

            # Use v_scale_ref for v_ref
            v_ref = cruising_speed * v_scale_ref

            # Velocity components in global frame: v_ref along path direction
            vx_ref = v_ref * math.cos(path_heading)
            vy_ref = v_ref * math.sin(path_heading)

            # Angular velocity should be zero for straight line path
            vyaw_ref = 0.0

            waypoint = np.array([
                waypoint_x,   # x
                waypoint_y,   # y
                waypoint_yaw, # yaw
                vx_ref,
                vy_ref,
                vyaw_ref
            ])
            waypoints.append(waypoint)
        
        return waypoints
    
    # ============================================================
    # WAYPOINT SEQUENCE (CSV) SUPPORT
    # ============================================================
    
    def _load_waypoints_from_csv(self, csv_file_path: str) -> List[Tuple[float, float]]:
        """
        Load a sequence of waypoints from a CSV file.
        
        Expected format: each row contains at least two columns [x, y, ...].
        A header row is allowed and will be skipped if it contains typical
        keywords such as 'x', 'y', 'waypoint', or 'point'.
        """
        waypoints: List[Tuple[float, float]] = []
        try:
            with open(csv_file_path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                for row_num, row in enumerate(reader):
                    if not row:
                        continue
                    try:
                        x = float(row[0].strip())
                        y = float(row[1].strip())
                        waypoints.append((x, y))
                    except (ValueError, IndexError):
                        # Check if this might be a header row
                        if row_num == 0 and any(
                            keyword in ' '.join(row).lower()
                            for keyword in ['x', 'y', 'waypoint', 'point']
                        ):
                            continue  # Skip header
                        else:
                            self.get_logger().warn(
                                f'Skipping invalid row {row_num + 1} in waypoint CSV: {row}'
                            )
                            continue
        except Exception as e:
            self.get_logger().error(
                f'Error parsing waypoint CSV file "{csv_file_path}": {e}'
            )
            return []
        
        if waypoints:
            self.get_logger().info(
                f'Parsed {len(waypoints)} waypoints from "{csv_file_path}"'
            )
        return waypoints
    
    def _set_next_waypoint_target(self) -> None:
        """
        Set the next waypoint in the CSV sequence as the current MPC target.
        
        Behavior:
        - target_x, target_y: current waypoint coordinates.
        - target_yaw: heading from previous waypoint to current waypoint
          (or current yaw for the very first waypoint).
        - Path start is set to the robot's current pose so we generate a
        straight-line path segment from the current position to the waypoint.
        """
        if not self.waypoint_navigation_active:
            return
        if self.current_waypoint_index >= len(self.waypoints):
            # No more waypoints
            self.has_target = False
            return
        
        wp_x, wp_y = self.waypoints[self.current_waypoint_index]
        self.target_x = float(wp_x)
        self.target_y = float(wp_y)
        
        # Compute target yaw based on previous waypoint if available
        if self.previous_waypoint is not None:
            dx = self.target_x - self.previous_waypoint[0]
            dy = self.target_y - self.previous_waypoint[1]
            self.target_yaw = math.atan2(dy, dx)
        else:
            # First waypoint: use current yaw as target yaw
            self.target_yaw = self.current_yaw
        
        # Initialize path start at current pose for straight-line control
        self.path_start_x = self.current_x
        self.path_start_y = self.current_y
        self.path_start_yaw = self.current_yaw
        self.path_initialized = True
        
        self.has_target = True
        self.target_reached = False
        
        self.get_logger().info(
            f'🎯 New waypoint target {self.current_waypoint_index + 1}/'
            f'{len(self.waypoints)}: x={self.target_x:.2f}, y={self.target_y:.2f}, '
            f'yaw_target={math.degrees(self.target_yaw):.1f}°'
        )
    
    def start_waypoint_navigation_callback(self, msg: String) -> None:
        """
        Topic callback to start waypoint navigation with CSV file.
        Receives std_msgs/String message containing CSV file path.
        """
        try:
            csv_file_path = msg.data.strip()
            if not csv_file_path:
                self.get_logger().error('Received empty CSV file path for waypoint navigation')
                return
            
            if not os.path.exists(csv_file_path):
                self.get_logger().error(f'CSV file not found: {csv_file_path}')
                return
            
            # Parse waypoints from CSV
            waypoints = self._load_waypoints_from_csv(csv_file_path)
            if len(waypoints) < 1:
                self.get_logger().error(f'No valid waypoints found in CSV: {csv_file_path}')
                return
            
            # Ensure odometry is initialized before starting navigation
            if not self.pose_initialized:
                self.get_logger().error(
                    'Odometry not initialized yet - cannot start waypoint navigation'
                )
                return
            
            # Initialize waypoint navigation
            self.waypoints = waypoints
            self.current_waypoint_index = 0
            self.waypoint_navigation_active = True
            
            # For first waypoint, use current position as previous waypoint so
            # the target yaw is along the line from current pose to first waypoint
            self.previous_waypoint = (self.current_x, self.current_y)
            
            # Start navigation to first waypoint immediately
            self._set_next_waypoint_target()
            
            self.get_logger().info(
                f'✓ Waypoint navigation started: {len(waypoints)} waypoints '
                f'loaded from "{csv_file_path}"'
            )
        except Exception as e:
            self.get_logger().error(f'Error starting waypoint navigation: {e}')
    
    def f2c_waypoint_array_callback(self, msg: Float64MultiArray) -> None:
        """
        Callback for waypoint arrays from F2C GUI.
        Format: [x1, y1, x2, y2, ...] (pairs of x,y coordinates)
        
        Behavior:
          - When receiving waypoint list: Store in pending_waypoints (don't start yet)
          - When receiving [0.0] signal: Start navigation with pending waypoints
        """
        try:
            data = list(msg.data)
            if len(data) == 0:
                self.get_logger().warn('Received empty /f2c_waypoints message')
                return

            # [0.0] is the "Start Navigation" signal from F2C GUI
            if len(data) == 1 and data[0] == 0.0:
                self.get_logger().info('Received /f2c_waypoints start signal [0.0]')
                self._start_pending_navigation()
                return

            if len(data) % 2 != 0:
                self.get_logger().error(
                    f'/f2c_waypoints length {len(data)} not divisible by 2; expected [x1,y1, x2,y2, ...]'
                )
                return

            num_waypoints = len(data) // 2
            waypoints_xy: List[Tuple[float, float]] = []
            for i in range(0, len(data), 2):
                x, y = data[i], data[i+1]
                waypoints_xy.append((float(x), float(y)))

            if not waypoints_xy:
                self.get_logger().error('Parsed zero waypoints from /f2c_waypoints')
                return

            # Store waypoints as pending (don't start navigation yet)
            self.pending_waypoints = waypoints_xy
            
            self.get_logger().info('')
            self.get_logger().info('╔═══════════════════════════════════════════════════════╗')
            self.get_logger().info(f'║  📡 RECEIVED {num_waypoints} WAYPOINTS FROM F2C GUI')
            self.get_logger().info('║  ⏳ Waiting for "Start Navigation" button...         ║')
            self.get_logger().info('║  (Or press X to enable MPC, then click Start)        ║')
            self.get_logger().info('╚═══════════════════════════════════════════════════════╝')
            self.get_logger().info('')
            
            # Log first few waypoints for verification
            for idx, (x, y) in enumerate(waypoints_xy[:5]):
                self.get_logger().info(f'  Waypoint {idx+1}: x={x:.2f}, y={y:.2f}')
            if num_waypoints > 5:
                self.get_logger().info(f'  ... and {num_waypoints - 5} more waypoints')
                
        except Exception as e:
            self.get_logger().error(f'Error handling /f2c_waypoints: {e}')
    
    def _start_pending_navigation(self) -> None:
        """
        Start navigation with previously received pending waypoints.
        Called when "Start Navigation" button is pressed in F2C GUI (sends [0.0] signal).
        """
        if not self.pending_waypoints:
            self.get_logger().warn('⚠️  No pending waypoints to navigate - publish waypoints first!')
            return
        
        if not self.pose_initialized:
            self.get_logger().error(
                '❌ Odometry not initialized yet - cannot start navigation'
            )
            return
        
        # Transfer pending waypoints to active navigation
        self.waypoints = self.pending_waypoints.copy()
        self.pending_waypoints = []  # Clear pending
        self.current_waypoint_index = 0
        self.waypoint_navigation_active = True
        self.previous_waypoint = (self.current_x, self.current_y)
        
        # Start navigation to first waypoint
        self._set_next_waypoint_target()
        
        self.get_logger().info('')
        self.get_logger().info('╔═══════════════════════════════════════════════════════╗')
        self.get_logger().info(f'║  🚀 NAVIGATION STARTED: {len(self.waypoints)} waypoints')
        if self.autonomy_enabled:
            self.get_logger().info('║  ✅ MPC autonomy is ENABLED - robot will move         ║')
        else:
            self.get_logger().info('║  ⚠️  MPC autonomy DISABLED - press X to enable!       ║')
        self.get_logger().info('╚═══════════════════════════════════════════════════════╝')
        self.get_logger().info('')
    
    def autonomy_enable_callback(self, msg: Bool) -> None:
        """
        Enable or disable MPC autonomy.
        When disabled, the controller will publish zero velocity even if targets
        or waypoints are available. This allows manual driving first, then
        enabling autonomy (e.g. via a keyboard-triggered node).
        """
        self.autonomy_enabled = bool(msg.data)
        state = 'ENABLED' if self.autonomy_enabled else 'DISABLED'
        self.get_logger().info(f'MPC autonomy {state} via /mpc_autonomy_enable')
    
    # ============================================================
    # ERROR STATE COMPUTATION
    # ============================================================
    
    def compute_error_state(self, ref_waypoint):
        """
        Compute error state in body frame: [xe, ye, θe]^T
        
        Args:
            ref_waypoint: Reference waypoint [x, y, yaw, vx, vy, vyaw]
        
        Returns:
            Error state [xe, ye, θe]^T in body frame
        """
        # Position error in global frame
        dx_global = ref_waypoint[0] - self.current_x
        dy_global = ref_waypoint[1] - self.current_y
        
        # Rotate to body frame
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        
        xe = cos_yaw * dx_global + sin_yaw * dy_global
        ye = -sin_yaw * dx_global + cos_yaw * dy_global
        
        # Yaw error
        yaw_error = self.normalize_angle(ref_waypoint[2] - self.current_yaw)
        
        return np.array([xe, ye, yaw_error])
    
    # ============================================================
    # SOFT SHUTDOWN SERVICE
    # ============================================================
    
    def _soft_shutdown_service(self, request, response):
        """
        Service callback for graceful shutdown.
        Performs: zero velocity → disarm ODrives → call shutdown service → signal parent launch.
        """
        del request
        self.get_logger().warning('🛑 Soft shutdown service called → zero velocity, disarm, stop nodes')
        try:
            self._send_zero_velocity()
            self._disarm_odrives()
            self._call_shutdown_service()
            # Signal parent launch to stop
            try:
                os.kill(os.getppid(), signal.SIGINT)
            except Exception:
                pass
            response.success = True
            response.message = 'Shutdown initiated'
        except Exception as e:
            response.success = False
            response.message = f'Error during shutdown: {e}'
        return response
    
    def _send_zero_velocity(self):
        """
        Send zero velocity commands multiple times to ensure delivery.
        """
        for _ in range(3):
            self.publish_zero_velocity()
            time.sleep(0.02)
    
    def _disarm_odrives(self):
        """
        Request IDLE state for both ODrive axes to disarm motors.
        """
        try:
            req_idle = AxisState.Request()
            req_idle.axis_requested_state = 1  # IDLE
            
            # Call both services
            if self.left_axis_client.service_is_ready():
                self.left_axis_client.call_async(req_idle)
            if self.right_axis_client.service_is_ready():
                self.right_axis_client.call_async(req_idle)
            
            time.sleep(0.2)
        except Exception as e:
            self.get_logger().warn(f'Error disarming ODrives: {e}')
    
    def _call_shutdown_service(self):
        """
        Call the shutdown service to stop mapping nodes.
        """
        try:
            if self.shutdown_client.service_is_ready():
                req = Trigger.Request()
                self.shutdown_client.call_async(req)
            else:
                # Try to wait briefly
                self.shutdown_client.wait_for_service(timeout_sec=0.5)
                if self.shutdown_client.service_is_ready():
                    req = Trigger.Request()
                    self.shutdown_client.call_async(req)
        except Exception as e:
            self.get_logger().warn(f'Error calling shutdown service: {e}')
    
    def _attempt_arm_motors(self):
        """
        Attempt to arm ODrive motors by requesting CLOSED_LOOP_CONTROL state.
        Retries up to _arm_max_attempts times.
        """
        if self._arm_attempts >= self._arm_max_attempts:
            self._arm_timer.cancel()
            return
        self._arm_attempts += 1
        
        # Ensure service availability
        if not (self.left_axis_client.service_is_ready() and self.right_axis_client.service_is_ready() and
                self.left_clear_client.service_is_ready() and self.right_clear_client.service_is_ready()):
            self.warn_throttled('odrive_services', 'Waiting for ODrive CAN services to be ready...', 5.0)
            return
        
        try:
            # Clear errors first
            self.left_clear_client.call_async(Empty.Request())
            self.right_clear_client.call_async(Empty.Request())
            time.sleep(0.1)
            
            # Request CLOSED_LOOP_CONTROL (state 8) for both axes
            req_left = AxisState.Request()
            req_left.axis_requested_state = 8  # CLOSED_LOOP_CONTROL
            req_right = AxisState.Request()
            req_right.axis_requested_state = 8  # CLOSED_LOOP_CONTROL
            
            self.left_axis_client.call_async(req_left)
            self.right_axis_client.call_async(req_right)
            self.get_logger().info('Arming ODrive axes (CLOSED_LOOP_CONTROL requested)')
            # Stop timer after successful dispatch
            self._arm_timer.cancel()
        except Exception as e:
            self.get_logger().warn(f'Arm attempt failed: {e}')
    
    def warn_throttled(self, key: str, message: str, period_sec: float) -> None:
        """
        Log a warning message, but throttle it to avoid spam.
        Same implementation as pose_controller.
        """
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self._last_log_time_ns.get(key, 0)
        if last_ns == 0 or (now_ns - last_ns) >= int(period_sec * 1e9):
            self._last_log_time_ns[key] = now_ns
            self.get_logger().warning(message)
    
    # ============================================================
    # ONLINE WHEEL SLIP ESTIMATION
    # ============================================================
    
    def estimate_wheel_slip(self) -> Tuple[float, float]:
        """
        Estimate wheel slip ratios based on localization and wheel encoder values
        for the past 1 second.
        
        Slip estimation approach:
        1. Filter history to past 1 second
        2. Estimate Fast-LIO body linear velocity (v) and angular velocity (omega) from position differences
        3. Compute expected wheel velocities using differential drive forward kinematics:
           - v_left_expected = v - (omega * wheel_base / 2)
           - v_right_expected = v + (omega * wheel_base / 2)
        4. Calculate average wheel velocities from encoder readings (R*ω)
        5. Estimate slip ratios for each wheel: slip_ratio = (R*ω - v_expected) / (R*ω)
        
        Returns:
            Tuple[left_slip_ratio, right_slip_ratio]:
                - Slip ratios where negative = wheel spinning faster than expected, positive = wheel slipping
        """
        #disbbled for now
        return 0.0, 0.0

        current_time = self.get_clock().now().nanoseconds / 1e9  # Current time in seconds
        
        # Filter history to past 1 second
        odom_data = [d for d in self.odom_history 
                     if (current_time - d['timestamp']) <= self.slip_estimation_window]
        encoder_data = [d for d in self.encoder_history 
                        if (current_time - d['timestamp']) <= self.slip_estimation_window]
        
        if len(odom_data) < 2 or len(encoder_data) < 1:
            # Not enough data yet, return no slip
            return 0.0, 0.0
        
        # Step 1: Estimate Fast-LIO body linear velocity (v) and angular velocity (omega) from position differences
        # Compute velocities from position differences over the time window
        v_estimates = []  # Linear velocity estimates (m/s)
        omega_estimates = []  # Angular velocity estimates (rad/s)
        
        for i in range(1, len(odom_data)):
            dt = odom_data[i]['timestamp'] - odom_data[i-1]['timestamp']
            if dt <= 0:
                continue
            
            # Linear velocity: distance / time
            dx = odom_data[i]['x'] - odom_data[i-1]['x']
            dy = odom_data[i]['y'] - odom_data[i-1]['y']
            distance = math.sqrt(dx*dx + dy*dy)
            v = distance / dt
            v_estimates.append(v)
            
            # Angular velocity: yaw change / time
            dyaw = self.normalize_angle(odom_data[i]['yaw'] - odom_data[i-1]['yaw'])
            omega = dyaw / dt
            omega_estimates.append(omega)
        
        if len(v_estimates) == 0 or len(omega_estimates) == 0:
            return 0.0, 0.0
        
        # Average body velocities over the time window
        avg_v = np.mean(v_estimates)  # m/s (body linear velocity)
        avg_omega = np.mean(omega_estimates)  # rad/s (body angular velocity)
        
        # Step 2: Compute expected wheel velocities using differential drive forward kinematics
        # Differential drive forward kinematics:
        # v_left = v - (omega * wheel_base / 2)
        # v_right = v + (omega * wheel_base / 2)
        v_left_expected = avg_v - (avg_omega * self.wheel_base / 2.0)  # m/s
        v_right_expected = avg_v + (avg_omega * self.wheel_base / 2.0)  # m/s
        
        # Step 3: Calculate average wheel velocities from encoder readings
        # Use velocities directly from encoder messages (already in rev/s)
        omega_wheel_left_encoder_list = [d['left_vel'] for d in encoder_data if 'left_vel' in d]
        omega_wheel_right_encoder_list = [d['right_vel'] for d in encoder_data if 'right_vel' in d]
        
        if len(omega_wheel_left_encoder_list) == 0 or len(omega_wheel_right_encoder_list) == 0:
            return 0.0, 0.0
        
        # Average encoder wheel angular velocities (in rev/s)
        avg_omega_wheel_left_encoder_rev = np.mean(omega_wheel_left_encoder_list)  # rev/s
        avg_omega_wheel_right_encoder_rev = np.mean(omega_wheel_right_encoder_list)  # rev/s
        
        # Convert encoder angular velocities to rad/s
        # omega (rev/s) -> omega (rad/s) = omega_rev * 2*pi
        omega_wheel_left_encoder_rad = avg_omega_wheel_left_encoder_rev * 2.0 * math.pi  # rad/s
        omega_wheel_right_encoder_rad = avg_omega_wheel_right_encoder_rev * 2.0 * math.pi  # rad/s
        
        # Compute R*ω (wheel linear velocity) for each wheel
        Rw_left = self.wheel_radius * omega_wheel_left_encoder_rad  # m/s
        Rw_right = self.wheel_radius * omega_wheel_right_encoder_rad  # m/s
        
        # Step 4: Estimate slip ratios for each wheel: slip_ratio = (R*ω - v_expected) / (R*ω)
        # If R*ω is zero, then slip_ratio = 0
        if abs(Rw_left) > 0.1:
            left_slip = (Rw_left - v_left_expected) / Rw_left
        else:
            left_slip = 0.0
        
        if abs(Rw_right) > 0.1:
            right_slip = (Rw_right - v_right_expected) / Rw_right
        else:
            right_slip = 0.0
        
        # Clamp slip ratios to reasonable range [-1, 1]
        left_slip = np.clip(left_slip, 0.0, 1.0)
        right_slip = np.clip(right_slip, 0.0, 1.0)
        
        # Store slip ratios
        self.left_slip_ratio = left_slip
        self.right_slip_ratio = right_slip
        
        return left_slip, right_slip
    
    # ============================================================
    # CONTROL LOOP
    # ============================================================
    
    def control_loop(self):
        """
        Main control loop - runs at specified frequency.
        """
        # If autonomy is not enabled, do nothing at all.
        # Let diff_drive_controller handle motor control via teleop.
        # No logging, no timing diagnostics - completely silent.
        if not self.autonomy_enabled:
            return
        
        # --- Control loop timing diagnostics (only when autonomy enabled) ---
        now_ns = self.get_clock().now().nanoseconds
        if self._last_control_time_ns is not None:
            dt = (now_ns - self._last_control_time_ns) / 1e9
            if dt > 1e-6:
                inst_freq = 1.0 / dt
                if self._control_freq_ema is None:
                    self._control_freq_ema = inst_freq
                else:
                    alpha = 0.1  # smoothing factor for EMA
                    self._control_freq_ema = (1.0 - alpha) * self._control_freq_ema + alpha * inst_freq
                
                # Track an EMA of dt as well
                if self._control_dt_ema is None:
                    self._control_dt_ema = dt
                else:
                    alpha = 0.1
                    self._control_dt_ema = (1.0 - alpha) * self._control_dt_ema + alpha * dt
                
                # Optional per-cycle debug of dt (only visible at DEBUG log level)
                self.get_logger().debug(f'MPC control loop dt: {dt*1000.0:.3f} ms')
                
                # Log the EMA timing at most every 5 seconds
                if now_ns - self._last_control_freq_log_time_ns >= int(5.0 * 1e9):
                    self._last_control_freq_log_time_ns = now_ns
                    if self._control_freq_ema is not None and self._control_dt_ema is not None:
                        self.get_logger().info(
                            f'Control loop timing (EMA): dt={self._control_dt_ema*1000.0:.2f} ms, '
                            f'freq={self._control_freq_ema:.2f} Hz '
                            f'(requested: {self.control_freq:.2f} Hz)'
                        )
        self._last_control_time_ns = now_ns
        # Check if pose and encoders are initialized
        if not self.pose_initialized:
            self.publish_zero_velocity()
            return
        
        if not (self.left_encoder_initialized and self.right_encoder_initialized):
            self.publish_zero_velocity()
            return
        
        # If using CSV-based waypoint navigation and we don't currently have a target,
        # set the next waypoint as the current target (once pose is initialized).
        if self.waypoint_navigation_active and not self.has_target:
            if self.current_waypoint_index < len(self.waypoints):
                self._set_next_waypoint_target()
            else:
                # No more waypoints → stop
                self.publish_zero_velocity()
                return
        
        # Check if target is set (either via /set_target_pose or waypoint CSV)
        if not self.has_target:
            self.publish_zero_velocity()
            return
        
        # Check if target has been reached (stopping criteria)
        if not self.target_reached:
            # Compute distance to target ALONG THE PATH LINE, not Euclidean distance
            # and not along the robot's current heading. This uses the straight-line
            # segment from path_start -> target and measures how much distance
            # remains along that segment, ignoring lateral offset.
            dx_path = self.target_x - self.path_start_x
            dy_path = self.target_y - self.path_start_y
            path_length = math.sqrt(dx_path*dx_path + dy_path*dy_path)
            
            if path_length > 1e-6:
                path_dir_x = dx_path / path_length
                path_dir_y = dy_path / path_length
                
                to_current_x = self.current_x - self.path_start_x
                to_current_y = self.current_y - self.path_start_y
                
                # Signed distance from start along the path line
                s_current = to_current_x * path_dir_x + to_current_y * path_dir_y
                # Clamp to [0, path_length]
                if s_current < 0.0:
                    s_current = 0.0
                elif s_current > path_length:
                    s_current = path_length
                
                distance_along_line_remaining = path_length - s_current
            else:
                # Degenerate path: fall back to simple Euclidean distance
                dx_to_target = self.target_x - self.current_x
                dy_to_target = self.target_y - self.current_y
                distance_along_line_remaining = math.sqrt(dx_to_target**2 + dy_to_target**2)
            
            if distance_along_line_remaining <= self.target_reached_threshold:
                self.target_reached = True
                
                # If we are following a CSV waypoint sequence, move to the next waypoint
                if self.waypoint_navigation_active and self.current_waypoint_index < len(self.waypoints) - 1:
                    self.get_logger().info(
                        f'✅ Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} '
                        f'reached at x={self.current_x:.3f}m, y={self.current_y:.3f}m'
                    )
                    # Update previous waypoint and advance index
                    self.previous_waypoint = (self.target_x, self.target_y)
                    self.current_waypoint_index += 1
                    # Clear current target; next loop will call _set_next_waypoint_target()
                    self.has_target = False
                    # Optionally send a brief stop at the waypoint
                    self.publish_zero_velocity()
                    return
                else:
                    # Final target reached (single target or last waypoint)
                    self.has_target = False
                    self.publish_zero_velocity()
                    self.get_logger().info(
                        f'✅ Target reached! Distance along path line: '
                        f'{distance_along_line_remaining:.3f} m '
                        f'(threshold: {self.target_reached_threshold:.3f} m)'
                    )
                    self.get_logger().info(
                        f'   Final position: x={self.current_x:.3f}m, y={self.current_y:.3f}m, '
                        f'target: x={self.target_x:.3f}m, y={self.target_y:.3f}m'
                    )
                    return
        
        # If target reached, stop
        if self.target_reached:
            self.publish_zero_velocity()
            return
        
        # Estimate wheel slip
        left_slip, right_slip = self.estimate_wheel_slip()
        
        # Publish slip ratios for diagnostics
        try:
            slip_msg = Float64MultiArray()
            slip_msg.data = [float(left_slip), float(right_slip)]
            self.slip_pub.publish(slip_msg)
        except Exception:
            pass
        
        # Generate local waypoints for MPC
        reference_trajectory = self.generate_local_waypoints()

        if len(reference_trajectory) == 0:
            self.publish_zero_velocity()
            return
        
        # Compute error state for MPC (use first waypoint as reference for forward progress)
        # The error state should be zero when robot reaches the first waypoint
        # This ensures forward progress along the path
        if len(reference_trajectory) > 0:
            ref_waypoint = reference_trajectory[0]  # First waypoint (ahead on path)
            current_error = self.compute_error_state(ref_waypoint)
        else:
            # Fallback: compute relative to closest point if no waypoints
            path_dx = self.target_x - self.path_start_x
            path_dy = self.target_y - self.path_start_y
            path_length = math.sqrt(path_dx*path_dx + path_dy*path_dy)
            if path_length > 1e-6:
                path_dir_x = path_dx / path_length
                path_dir_y = path_dy / path_length
                to_current_x = self.current_x - self.path_start_x
                to_current_y = self.current_y - self.path_start_y
                t_closest = (to_current_x * path_dir_x + to_current_y * path_dir_y) / path_length
                t_closest = np.clip(t_closest, 0.0, 1.0)
                
                closest_x = self.path_start_x + t_closest * path_dx
                closest_y = self.path_start_y + t_closest * path_dy
                path_heading = math.atan2(path_dy, path_dx)
                
                closest_waypoint = np.array([
                    closest_x, closest_y, path_heading,
                    0.4 * math.cos(path_heading), 0.4 * math.sin(path_heading), 0.0
                ])
                current_error = self.compute_error_state(closest_waypoint)
            else:
                current_error = np.array([0.0, 0.0, 0.0])
                self.get_logger().warn('No waypoints and path length too small, using zero error')
        
        # Debug: Log waypoint and error state information (only if solver debug is enabled)
        if self.solver_debug_enabled and len(reference_trajectory) > 0:
            self.get_logger().debug(
                f'MPC Current Error: xe={current_error[0]:.3f}, ye={current_error[1]:.3f}, '
                f'θe={current_error[2]:.3f} (relative to first waypoint, should be zero when reached)'
            )
            # Log first few waypoints for verification
            for k in range(min(3, len(reference_trajectory))):
                wp = reference_trajectory[k]
                self.get_logger().debug(
                    f'MPC Waypoint {k}: x={wp[0]:.3f}, y={wp[1]:.3f}, yaw={wp[2]:.3f}, '
                    f'v={math.sqrt(wp[3]**2 + wp[4]**2):.3f}, w={wp[5]:.3f}'
                )
        
        # Solve MPC optimization (apply dynamic v-bound gating if available)
        if hasattr(self, '_current_v_scale'):
            try:
                self.mpc_optimizer.set_velocity_bound_scale(self._current_v_scale)
            except Exception:
                pass

        u0_optimal, solve_time_ms, solution = self.mpc_optimizer.solve(
            current_error,
            reference_trajectory,
            left_slip,
            right_slip
        )
        
        # Extract wheel velocities (rad/s)
        omega_L = u0_optimal[0]
        omega_R = u0_optimal[1]
        
        # Convert to motor rev/s (accounting for gear ratio)
        left_rps = omega_L / (2.0 * math.pi * self.gear_ratio)
        right_rps = omega_R / (2.0 * math.pi * self.gear_ratio)
        
        # Publish wheel velocities
        self.publish_wheel_velocities(left_rps, right_rps)
        
        # Publish diagnostic messages
        try:
            # Convert to linear and angular velocities for diagnostics
            linear_vel = (omega_L + omega_R) * self.wheel_radius / 2.0
            angular_vel = (omega_R - omega_L) * self.wheel_radius / self.wheel_base

            twist_msg = Twist()
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = angular_vel
            self.cmd_pub.publish(twist_msg)

            # Only publish MPC-specific diagnostics if solution exists
            if solution is not None and hasattr(self.mpc_optimizer, 'l_constr') and self.mpc_optimizer.l_constr is not None:
                # Publish MPC control sequence over horizon [u0_L, u0_R, u1_L, u1_R, ..., u_{N-1}_L, u_{N-1}_R]
                control_seq_msg = Float64MultiArray()
                control_sequence = []
                expected_length = self.mpc_horizon * (self.mpc_optimizer.nu + self.mpc_optimizer.nx)
                if len(solution) >= expected_length:
                    for k in range(self.mpc_horizon):
                        u_idx = k * (self.mpc_optimizer.nu + self.mpc_optimizer.nx)  # Start of u_k
                        control_sequence.extend([float(solution[u_idx]), float(solution[u_idx + 1])])  # u_k_L, u_k_R
                    control_seq_msg.data = control_sequence
                    self.mpc_controls_pub.publish(control_seq_msg)

            # Publish current error state
            error_state_msg = Float64MultiArray()
            error_state_msg.data = [float(current_error[0]), float(current_error[1]), float(current_error[2])]
            self.error_state_pub.publish(error_state_msg)

            # Publish reference trajectory (first waypoint)
            if len(reference_trajectory) > 0:
                ref_wp = reference_trajectory[0]
                ref_traj_msg = Float64MultiArray()
                ref_traj_msg.data = [float(ref_wp[0]), float(ref_wp[1]), float(ref_wp[2]),
                                   float(ref_wp[3]), float(ref_wp[4]), float(ref_wp[5])]
                self.ref_trajectory_pub.publish(ref_traj_msg)

            # Publish solver diagnostics and constraint bounds (only when MPC solved successfully)
            if solution is not None:
                # Enhanced solver diagnostics with more info from OSQP result
                solver_diag_msg = Float64MultiArray()
                obj_val = 0.0
                iterations = 0
                osqp_time = 0.0
                try:
                    # Try to get result info (stored in optimizer after solve)
                    if hasattr(self.mpc_optimizer, '_last_result') and self.mpc_optimizer._last_result is not None:
                        result = self.mpc_optimizer._last_result
                        if hasattr(result, 'info'):
                            obj_val = result.info.obj_val if hasattr(result.info, 'obj_val') else 0.0
                            iterations = result.info.iter if hasattr(result.info, 'iter') else 0
                            osqp_time = result.info.run_time * 1000.0 if hasattr(result.info, 'run_time') else 0.0
                except Exception:
                    pass
                
                solver_diag_msg.data = [
                    float(solve_time_ms),  # Total solve time (wall clock)
                    float(obj_val),  # OSQP objective value
                    float(iterations),  # Number of iterations
                    float(osqp_time)  # OSQP internal solve time (ms)
                ]
                self.solver_diagnostics_pub.publish(solver_diag_msg)

                # Publish solver inputs summary
                if self.solver_debug_enabled:
                    try:
                        inputs_msg = Float64MultiArray()
                        P_nnz = self.mpc_optimizer.P.nnz
                        A_nnz = self.mpc_optimizer.A_constr.nnz
                        inputs_msg.data = [
                            float(P_nnz),  # P matrix non-zeros
                            float(A_nnz),  # A matrix non-zeros
                            float(self.mpc_optimizer.l_constr.min()),  # Min lower bound
                            float(self.mpc_optimizer.l_constr.max()),  # Max lower bound
                            float(self.mpc_optimizer.u_constr.min()),  # Min upper bound
                            float(self.mpc_optimizer.u_constr.max()),  # Max upper bound
                            float(current_error[0]),  # Error xe
                            float(current_error[1]),  # Error ye
                            float(current_error[2])   # Error θe
                        ]
                        self.solver_inputs_pub.publish(inputs_msg)

                        # Publish solver cost
                        cost_msg = Float64MultiArray()
                        actual_cost = self.mpc_optimizer.compute_solution_cost(solution, current_error, reference_trajectory)
                        cost_msg.data = [
                            float(actual_cost),  # Computed cost
                            float(obj_val),  # OSQP objective value
                            float(iterations),  # Iterations
                            float(osqp_time)  # Solve time (ms)
                        ]
                        self.solver_cost_pub.publish(cost_msg)
                    except Exception as e:
                        self.get_logger().warn(f'Error publishing solver debug info: {e}')

                # Publish MPC constraint bounds (first few bounds for debugging)
                bounds_msg = Float64MultiArray()
                nx = self.mpc_optimizer.nx
                if len(self.mpc_optimizer.l_constr) > 2*nx and len(self.mpc_optimizer.u_constr) > 2*nx:
                    bounds_msg.data = [float(self.mpc_optimizer.l_constr[0]), float(self.mpc_optimizer.u_constr[0]),  # First dynamics constraint
                                     float(self.mpc_optimizer.l_constr[nx]), float(self.mpc_optimizer.u_constr[nx]),  # Second dynamics constraint
                                     float(self.mpc_optimizer.l_constr[2*nx]), float(self.mpc_optimizer.u_constr[2*nx])]  # Third dynamics constraint
                    self.mpc_bounds_pub.publish(bounds_msg)

            # Publish current pose and velocity (always available)
            pose_vel_msg = Float64MultiArray()
            pose_vel_msg.data = [float(self.current_x), float(self.current_y), float(self.current_yaw),
                               float(self.current_vx), float(self.current_vy), float(self.current_vyaw)]
            self.pose_velocity_pub.publish(pose_vel_msg)

        except Exception as e:
            self.get_logger().warn(f'Error publishing diagnostics: {e}')
    
    # ============================================================
    # PUBLISHING
    # ============================================================
    
    def publish_wheel_velocities(self, left_rps: float, right_rps: float):
        """
        Publish velocity commands to left and right ODrive motors.
        
        Args:
            left_rps: Left wheel velocity (motor turns/s)
            right_rps: Right wheel velocity (motor turns/s)
        """
        # Apply motor inversion
        left_cmd = -left_rps if self.invert_left else left_rps
        right_cmd = -right_rps if self.invert_right else right_rps
        
        # Create left motor command
        left_msg = ControlMessage()
        left_msg.control_mode = 2  # VELOCITY_CONTROL
        left_msg.input_mode = 1    # PASSTHROUGH
        left_msg.input_vel = float(left_cmd)
        left_msg.input_torque = 0.0
        left_msg.input_pos = 0.0
        
        # Create right motor command
        right_msg = ControlMessage()
        right_msg.control_mode = 2  # VELOCITY_CONTROL
        right_msg.input_mode = 1    # PASSTHROUGH
        right_msg.input_vel = float(right_cmd)
        right_msg.input_torque = 0.0
        right_msg.input_pos = 0.0
        
        # Publish
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
    
    def publish_zero_velocity(self):
        """
        Send zero velocity commands.
        """
        self.publish_wheel_velocities(0.0, 0.0)
    
    # ============================================================
    # UTILITY FUNCTIONS
    # ============================================================
    
    @staticmethod
    def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        """
        Convert quaternion to yaw angle (rotation around Z-axis).
        
        Args:
            qx, qy, qz, qw: Quaternion components
        
        Returns:
            yaw: Yaw angle in radians (-π to π)
        """
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-π, π] range.
        
        Args:
            angle: Angle in radians
        
        Returns:
            Normalized angle in radians
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    """
    Main entry point for the node.
    """
    rclpy.init(args=args)
    node = MPCAutonomousController()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  Interrupted by user')
    finally:
        node.publish_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

