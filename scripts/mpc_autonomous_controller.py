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

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float64
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
    
    Dynamics: x_{k+1} = A_k*x_k + B_k*u_k
    - A_k depends on reference trajectory (v_ref, omega_ref)
    - B_k depends on slip ratios (λ_L, λ_R)
    """
    
    def __init__(self, N, Ts, r, L, w_min, w_max, Q_xe, Q_ye, Q_yaw, R_delta, logger=None):
        """
        Initialize MPC optimizer.
        
        Args:
            N: Prediction horizon
            Ts: Time step (s)
            r: Wheel radius (m)
            L: Wheel base (m)
            w_min: Minimum wheel velocity (rad/s)
            w_max: Maximum wheel velocity (rad/s)
            Q_xe: Cost weight for position error x
            Q_ye: Cost weight for position error y
            Q_yaw: Cost weight for yaw error
            R_delta: Cost weight for control input change (delta u)
            logger: Optional logger for debug messages
        """
        self.N = N
        self.Ts = Ts
        self.r = r
        self.L = L
        self.w_min = w_min
        self.w_max = w_max
        self.logger = logger
        
        # State and control dimensions
        self.nx = 3  # [xe, ye, θe]
        self.nu = 2  # [ωL, ωR]
        self.nz = N * (self.nu + self.nx)  # Decision vector size
        
        # Cost matrix Q (for states)
        self.Q = sparse.diags([Q_xe, Q_ye, Q_yaw])
        
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
        
        # For each state x_k (k=1 to N)
        for k in range(N):
            # Position of x_k in decision vector: k * (nu + nx) + nu
            x_k_start = k * (nu + nx) + nu
            
            # Add Q matrix entries for this state
            Q_dense = self.Q.toarray()
            for i in range(nx):
                for j in range(nx):
                    if abs(Q_dense[i, j]) > 1e-10:  # Only non-zero entries
                        P_row.append(x_k_start + i)
                        P_col.append(x_k_start + j)
                        P_data.append(Q_dense[i, j])
        
        # Add cost for control input changes: (u_k - u_{k-1})^T R_delta (u_k - u_{k-1})
        # This expands to: u_k^T R_delta u_k - 2*u_k^T R_delta u_{k-1} + u_{k-1}^T R_delta u_{k-1}
        # For k >= 1: penalize (u_k - u_{k-1})
        # Since R_delta is diagonal, we can simplify
        R_delta_diag = self.R_delta.diagonal()  # Get diagonal values [r, r]
        
        for k in range(1, N):  # k from 1 to N-1
            # Position of u_{k-1} and u_k in decision vector
            u_km1_start = (k - 1) * (nu + nx)  # u_{k-1}
            u_k_start = k * (nu + nx)  # u_k
            
            # Add R_delta terms for u_k^T R_delta u_k (diagonal only)
            for i in range(nu):
                P_row.append(u_k_start + i)
                P_col.append(u_k_start + i)
                P_data.append(R_delta_diag[i])
            
            # Add R_delta terms for u_{k-1}^T R_delta u_{k-1} (diagonal only)
            for i in range(nu):
                P_row.append(u_km1_start + i)
                P_col.append(u_km1_start + i)
                P_data.append(R_delta_diag[i])
            
            # Add cross terms: -2*u_k^T R_delta u_{k-1} (symmetric, diagonal R_delta)
            for i in range(nu):
                # u_k[i] * u_{k-1}[i] term (off-diagonal, symmetric)
                P_row.append(u_k_start + i)
                P_col.append(u_km1_start + i)
                P_data.append(-2.0 * R_delta_diag[i])
                # Symmetric term
                P_row.append(u_km1_start + i)
                P_col.append(u_k_start + i)
                P_data.append(-2.0 * R_delta_diag[i])
        
        # Create sparse P matrix (nz x nz)
        self.P = sparse.coo_matrix(
            (P_data, (P_row, P_col)),
            shape=(nz, nz)
        ).tocsc()
        
        # ============================================================
        # STEP 2: BUILD CONSTRAINT MATRIX A_constr
        # ============================================================
        # Constraints:
        # 1. Dynamics: -B_k*u_k + x_{k+1} - A_k*x_k = 0 (equality, bounds = 0)
        # 2. Input bounds: w_min <= u_k <= w_max
        
        # Number of constraints
        n_dynamics = N * nx  # Dynamics constraints
        n_input_bounds = N * nu  # Input bound constraints
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
            u_k_idx = k * (nu + nx)
            x_k_idx = k * (nu + nx) + nu if k > 0 else None  # x_k (for k>0)
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
                        data_values.append(0.0)  # Placeholder, will be updated
                        self.A_indices[k].append(idx)
                
                # -B_k[i,:]*u_k terms
                for j in range(nu):
                    idx = len(data_values)  # Index before appending
                    row_indices.append(constraint_row + i)
                    col_indices.append(u_k_idx + j)
                    data_values.append(0.0)  # Placeholder, will be updated
                    self.B_indices[k].append(idx)  # Store index of this element
            
            constraint_row += nx
        
        # Input bound constraints: w_min <= u_k <= w_max
        for k in range(N):
            u_k_idx = k * (nu + nx)
            for j in range(nu):
                row_indices.append(constraint_row)
                col_indices.append(u_k_idx + j)
                data_values.append(1.0)  # Identity for input bounds
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
                    x_k_idx = k * (nu + nx) + nu
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
        
        # Input bounds: w_min <= u_k <= w_max
        for k in range(N):
            for j in range(nu):
                bound_idx = n_dynamics + k * nu + j
                self.l_constr[bound_idx] = self.w_min
                self.u_constr[bound_idx] = self.w_max
        
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
            # A_k = [[1, -ω_ref*Ts, 0],
            #        [ω_ref*Ts, 1, 0],
            #        [0, 0, 1]]
            A_k = np.array([
                [1.0, -omega_ref * Ts, 0.0],
                [omega_ref * Ts, 1.0, v_ref * Ts],
                [0.0, 0.0, 1.0]
            ])
            
            # Compute B_k matrix (based on slip ratios)
            # B_k = Ts * [[-r/2*(1-λ_L), -r/2*(1-λ_R)],
            #             [0, 0],
            #             [r/L*(1-λ_L), -r/L*(1-λ_R)]]
            B_k = Ts * np.array([
                [-r/2.0 * eta_L, -r/2.0 * eta_R],
                [0.0, 0.0],
                [r/L * eta_L, -r/L * eta_R]
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
            Tuple (u0_optimal, solve_time_ms):
                u0_optimal: [ωL, ωR]^T in rad/s
                solve_time_ms: Solve time in milliseconds
        """
        if not self.initialized:
            if not self.setup():
                return np.array([0.0, 0.0]), 0.0
        
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
            [omega_ref * self.Ts, 1.0, v_ref * self.Ts],
            [0.0, 0.0, 1.0]
        ])
        
        # Compute A_0*current_error (RHS of constraint)
        rhs_0 = A_0 @ current_error
        
        # Update bounds for first dynamics constraint (equality with RHS)
        for i in range(self.nx):
            self.l_constr[i] = rhs_0[i]
            self.u_constr[i] = rhs_0[i]
        
        # Warm start
        if self.prev_solution is not None:
            # Shift previous solution: [u0, x1, u1, x2, ...] -> [u1, x2, u2, x3, ...]
            prev_sol = self.prev_solution
            warm_start = np.zeros(self.nz)
            
            if len(prev_sol) >= (self.nu + self.nx):
                warm_start[:-self.nu] = prev_sol[self.nu:]
                warm_start[-self.nu:] = prev_sol[-self.nu:]
            
            self.solver.warm_start(x=warm_start)
        
        # Update solver bounds
        self.solver.update(l=self.l_constr, u=self.u_constr)
        
        # Solve
        result = self.solver.solve()
        
        if result.info.status != 'solved':
            if self.logger:
                self.logger.warn(f'MPC solve failed: {result.info.status}')
            return np.array([0.0, 0.0]), 0.0
        
        # Extract control
        solution = result.x
        u0_optimal = solution[:self.nu]  # First control input [ωL, ωR]
        
        # Store solution for warm start next time
        self.prev_solution = solution
        
        solve_time_ms = (time.time() - solve_start) * 1000.0
        return u0_optimal, solve_time_ms


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
        self.declare_parameter('max_linear_velocity', 0.4) # m/s
        self.declare_parameter('max_angular_velocity', 4.0) # rad/s
        
        # MPC parameters
        self.declare_parameter('mpc_horizon', 5)           # Prediction horizon steps
        self.declare_parameter('mpc_dt', 0.1)             # Time step for MPC (s)
        
        # Slip estimation parameters
        self.declare_parameter('slip_history_length', 100)  # Buffer size (should be large enough for 1 second)
        self.declare_parameter('slip_estimation_window', 1.0)  # Time window for slip estimation (seconds)
        
        # Waypoint parameters
        self.declare_parameter('lookahead_distance', 0.5)  # m
        
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
        self.declare_parameter('mpc_Q_xe', 10.0)  # Weight for position error x
        self.declare_parameter('mpc_Q_ye', 10.0)  # Weight for position error y
        self.declare_parameter('mpc_Q_yaw', 1.0)  # Weight for yaw error (lower)
        self.declare_parameter('mpc_R_delta', 0.1)  # Weight for control input change (delta u)
        
        # Get cost weights
        mpc_Q_xe = self.get_parameter('mpc_Q_xe').value
        mpc_Q_ye = self.get_parameter('mpc_Q_ye').value
        mpc_Q_yaw = self.get_parameter('mpc_Q_yaw').value
        mpc_R_delta = self.get_parameter('mpc_R_delta').value
        
        # Compute wheel velocity limits (rad/s)
        # Convert max linear velocity to max wheel angular velocity
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
            logger=self.get_logger()
        )
        
        # Target pose
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.has_target = False
        
        # Path planning state
        self.path_start_x = 0.0  # Initial position when target was set
        self.path_start_y = 0.0
        self.path_start_yaw = 0.0
        self.path_initialized = False  # Whether path has been initialized
        
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
        self.get_logger().info('='*70)
    
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
        4. Waypoints are spaced at distances corresponding to travel at max speed for time_step duration
        
        Returns:
            List of waypoint states [x, y, yaw, vx, vy, vyaw] for MPC horizon
        """
        if not self.path_initialized or not self.has_target:
            return []
        
        # Compute straight-line path vector
        path_dx = self.target_x - self.path_start_x
        path_dy = self.target_y - self.path_start_y
        path_length = math.sqrt(path_dx*path_dx + path_dy*path_dy)
        
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
        
        # Compute waypoint spacing: distance traveled at max speed for one time step
        waypoint_spacing = self.max_linear_vel * self.mpc_dt  # m
        
        # Compute distance from closest point to target along path
        distance_to_target_along_path = (1.0 - t_closest) * path_length
        
        # Check if last waypoint would go beyond target
        max_distance_needed = waypoint_spacing * self.mpc_horizon
        
        # Adjust spacing if needed to stop exactly at target
        if max_distance_needed > distance_to_target_along_path:
            # Adjust spacing so last waypoint is at target
            if self.mpc_horizon > 0:
                adjusted_spacing = distance_to_target_along_path / self.mpc_horizon
            else:
                adjusted_spacing = waypoint_spacing
        else:
            adjusted_spacing = waypoint_spacing
        
        # Generate waypoints along the path
        waypoints = []
        waypoint_positions = []  # Store positions for velocity computation
        
        for k in range(self.mpc_horizon):
            # Distance along path from closest point
            distance_along_path = adjusted_spacing * (k + 1)  # k+1 to start ahead
            
            # Parameter along path (0 = start, 1 = end)
            t_waypoint = t_closest + (distance_along_path / path_length)
            
            # Clamp to path segment [0, 1]
            t_waypoint = np.clip(t_waypoint, 0.0, 1.0)
            
            # Compute waypoint position
            waypoint_x = self.path_start_x + t_waypoint * path_dx
            waypoint_y = self.path_start_y + t_waypoint * path_dy
            
            # Interpolate yaw between start and target
            # Blend between heading along path and target yaw
            heading_to_target = math.atan2(path_dy, path_dx)
            yaw_blend = (1.0 - t_waypoint) * self.path_start_yaw + t_waypoint * self.target_yaw
            # Also consider heading along path
            waypoint_yaw = 0.7 * heading_to_target + 0.3 * yaw_blend
            waypoint_yaw = self.normalize_angle(waypoint_yaw)
            
            waypoint_positions.append((waypoint_x, waypoint_y, waypoint_yaw))
        
        # Compute velocities for each waypoint based on next waypoint
        # For the last waypoint, compute an extra waypoint into the future
        for k in range(self.mpc_horizon):
            current_pos = waypoint_positions[k]
            
            # Get next waypoint position
            if k < self.mpc_horizon - 1:
                # Use next waypoint in the window
                next_pos = waypoint_positions[k + 1]
            else:
                # Last waypoint: compute extra waypoint into the future
                # Distance for next step
                next_distance = adjusted_spacing
                t_next = t_closest + (adjusted_spacing * (self.mpc_horizon + 1) / path_length)
                t_next = np.clip(t_next, 0.0, 1.0)
                
                next_x = self.path_start_x + t_next * path_dx
                next_y = self.path_start_y + t_next * path_dy
                
                # Interpolate yaw
                heading_to_target = math.atan2(path_dy, path_dx)
                yaw_blend = (1.0 - t_next) * self.path_start_yaw + t_next * self.target_yaw
                next_yaw = 0.7 * heading_to_target + 0.3 * yaw_blend
                next_yaw = self.normalize_angle(next_yaw)
                
                next_pos = (next_x, next_y, next_yaw)
            
            # Compute velocity from current to next waypoint
            dx = next_pos[0] - current_pos[0]
            dy = next_pos[1] - current_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Linear velocity components
            if self.mpc_dt > 1e-6:
                v_magnitude = distance / self.mpc_dt
                # Limit to max velocity
                v_magnitude = min(v_magnitude, self.max_linear_vel)
                
                if distance > 1e-6:
                    vx_ref = v_magnitude * (dx / distance)
                    vy_ref = v_magnitude * (dy / distance)
                else:
                    vx_ref = 0.0
                    vy_ref = 0.0
            else:
                vx_ref = 0.0
                vy_ref = 0.0
            
            # Angular velocity: yaw change / time
            dyaw = self.normalize_angle(next_pos[2] - current_pos[2])
            if self.mpc_dt > 1e-6:
                vyaw_ref = dyaw / self.mpc_dt
                # Limit to max angular velocity
                vyaw_ref = np.clip(vyaw_ref, -self.max_angular_vel, self.max_angular_vel)
            else:
                vyaw_ref = 0.0
            
            waypoint = np.array([
                current_pos[0],  # x
                current_pos[1],  # y
                current_pos[2],  # yaw
                vx_ref,
                vy_ref,
                vyaw_ref
            ])
            waypoints.append(waypoint)
        
        return waypoints
    
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
        if abs(Rw_left) > 1e-6:
            left_slip = (Rw_left - v_left_expected) / Rw_left
        else:
            left_slip = 0.0
        
        if abs(Rw_right) > 1e-6:
            right_slip = (Rw_right - v_right_expected) / Rw_right
        else:
            right_slip = 0.0
        
        # Clamp slip ratios to reasonable range [-1, 1]
        left_slip = np.clip(left_slip, -1.0, 1.0)
        right_slip = np.clip(right_slip, -1.0, 1.0)
        
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
        # Check if pose and encoders are initialized
        if not self.pose_initialized:
            self.publish_zero_velocity()
            return
        
        if not (self.left_encoder_initialized and self.right_encoder_initialized):
            self.publish_zero_velocity()
            return
        
        # Check if target is set
        if not self.has_target:
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
        
        # Compute error state for MPC (use first waypoint as reference)
        ref_waypoint = reference_trajectory[0]
        current_error = self.compute_error_state(ref_waypoint)
        
        # Solve MPC optimization
        u0_optimal, solve_time_ms = self.mpc_optimizer.solve(
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
        
        # Publish diagnostic twist message
        try:
            # Convert to linear and angular velocities for diagnostics
            linear_vel = (omega_L + omega_R) * self.wheel_radius / 2.0
            angular_vel = (omega_R - omega_L) * self.wheel_radius / self.wheel_base
            
            twist_msg = Twist()
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = angular_vel
            self.cmd_pub.publish(twist_msg)
        except Exception:
            pass
    
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

