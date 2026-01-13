#!/usr/bin/env python3
"""
Acceleration-based MPC Controller
=================================

This node implements a linear time-varying (LTV) MPC for a differential drive
robot where:

- **State**: augmented tracking error and input
      x = [xe, ye, θe, v, ω]^T
  where (xe, ye, θe) are pose errors in the body frame, and (v, ω) are the
  robot's linear and angular velocities.

- **Control input**: change in robot velocities (delta input)
      u = Δu = [Δv, Δω]^T

The discrete-time dynamics are:
  x_{k+1} = A_aug_k x_k + B_aug Δu_k

with:
  [xe, ye, θe] dynamics linearized around reference (v_ref, ω_ref)
  [v, ω]     updated by integrating Δu:
      v_{k+1} = v_k + Δv_k
      ω_{k+1} = ω_k + Δω_k

The controller tracks a straight-line reference segment between the current
pose and a user-specified target pose.

This script is intentionally simpler than the slip-aware MPC controller:
- No slip estimation
- No CSV waypoint sequences
- Minimal diagnostics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String, Bool
from geometry_msgs.msg import Twist
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
from std_srvs.srv import Trigger, Empty

import numpy as np
import math
from typing import List, Tuple, Optional
import time
import os
import signal
import csv

# Optional dependencies for MPC
try:
    import osqp
    OSQP_AVAILABLE = True
except ImportError:  # pragma: no cover - runtime check
    OSQP_AVAILABLE = False
    import warnings
    warnings.warn("OSQP not available. Install with: pip install osqp")

try:
    from scipy import sparse
    SCIPY_AVAILABLE = True
except ImportError:  # pragma: no cover - runtime check
    SCIPY_AVAILABLE = False
    import warnings
    warnings.warn("scipy not available. Install with: pip install scipy")


class WheelRampCompensator:
    """
    Compensates for actuator ramp dynamics at the wheel level.
    
    When using VEL_RAMP mode on ODrive, the velocity doesn't change instantly.
    This class computes an effective wheel velocity command such that the
    total wheel displacement over the MPC timestep matches what the MPC expects.
    
    Key features:
    - Tracks actual wheel velocity at end of each cycle (not just commanded)
    - Handles all ramp directions: speed up, slow down, sign changes
    - Accounts for pure transport delay (CAN latency, etc.)
    - Works independently for left and right wheels
    
    Math:
        With ramp rate R, starting from ω_prev, commanding ω_target:
        - Ramp time: t_r = |ω_target - ω_prev| / R
        - Displacement with ramp: θ = ω_prev*Ts + Δω*Ts - Δω²/(2R)  [if t_r < Ts]
        - We solve for ω_eff such that displacement equals ω_target * Ts
    """
    
    def __init__(
        self,
        ramp_rate: float = 20.0,      # Velocity ramp rate (turn/s² or consistent units)
        delay_time: float = 0.0,        # Pure transport delay (seconds)
        cycle_time: float = 0.1,        # MPC cycle time Ts (seconds)
        logger=None,
    ):
        """
        Initialize the wheel ramp compensator.
        
        Args:
            ramp_rate: ODrive vel_ramp_rate (turn/s² if wheel velocities are in turn/s)
            delay_time: Pure transport delay - time between command sent and 
                       actuator starting to respond (seconds)
            cycle_time: MPC control cycle period Ts (seconds)
            logger: Optional ROS logger for debug output
        """
        self.ramp_rate = float(ramp_rate)
        self.delay_time = float(delay_time)
        self.cycle_time = float(cycle_time)
        self.logger = logger
        
        # Track the actual velocity at end of previous cycle for each wheel
        # This is what the wheel velocity will be at the START of next cycle
        self._left_vel_at_cycle_end: float = 0.0
        self._right_vel_at_cycle_end: float = 0.0
        
        # Track what we commanded last cycle (for debugging/logging)
        self._left_cmd_prev: float = 0.0
        self._right_cmd_prev: float = 0.0
        
    def reset(self) -> None:
        """Reset internal state (call when stopping or reinitializing)."""
        self._left_vel_at_cycle_end = 0.0
        self._right_vel_at_cycle_end = 0.0
        self._left_cmd_prev = 0.0
        self._right_cmd_prev = 0.0
        
    def set_parameters(
        self,
        ramp_rate: Optional[float] = None,
        delay_time: Optional[float] = None,
        cycle_time: Optional[float] = None,
    ) -> None:
        """Update parameters at runtime."""
        if ramp_rate is not None:
            self.ramp_rate = float(ramp_rate)
        if delay_time is not None:
            self.delay_time = float(delay_time)
        if cycle_time is not None:
            self.cycle_time = float(cycle_time)
    
    def _compute_compensated_velocity(
        self,
        omega_prev: float,
        omega_target: float,
    ) -> Tuple[float, float]:
        """
        Compute compensated wheel velocity for a single wheel.
        
        Given:
        - omega_prev: Wheel velocity at START of this cycle (= end of previous cycle)
        - omega_target: Desired wheel velocity (what MPC wants to achieve)
        
        Returns:
        - omega_eff: Velocity to COMMAND to the wheel
        - omega_end: Predicted velocity at END of this cycle (for tracking)
        
        The goal is that total wheel displacement over Ts equals:
            θ_desired = omega_target * Ts
        
        With delay τ_d and VEL_RAMP mode, the actual displacement is:
            θ_actual = ω_prev × τ_d  +  [ramp displacement over Ts_eff]
                     = ω_prev × τ_d  +  ω_prev × Ts_eff + Δω_eff × Ts_eff - Δω_eff²/(2R)
                     = ω_prev × Ts   +  Δω_eff × Ts_eff - Δω_eff²/(2R)
        
        Setting θ_actual = θ_desired:
            ω_prev × Ts + Δω_eff × Ts_eff - Δω_eff²/(2R) = ω_target × Ts
            Δω_eff × Ts_eff - Δω_eff²/(2R) = Δω_target × Ts
        
        This is a quadratic in Δω_eff with solution:
            Δω_eff = R×Ts_eff - √((R×Ts_eff)² - 2×R×Ts×Δω_target)
        """
        R = self.ramp_rate
        Ts = self.cycle_time
        tau_d = self.delay_time
        
        # Effective time available for ramping (accounting for delay)
        Ts_eff = max(Ts - tau_d, 1e-6)
        
        # Change requested by MPC
        delta_target = omega_target - omega_prev
        
        # Handle near-zero change (avoid numerical issues)
        if abs(delta_target) < 1e-9:
            return omega_target, omega_target
        
        # Maximum change that can be fully compensated
        # From discriminant >= 0: (R×Ts_eff)² >= 2×R×Ts×|Δω_target|
        # => |Δω_target| <= R × Ts_eff² / (2 × Ts)
        max_compensatable_delta = R * Ts_eff * Ts_eff / (2.0 * Ts)
        
        # Direction of velocity change
        sign_delta = 1.0 if delta_target > 0 else -1.0
        abs_delta_target = abs(delta_target)
        
        if abs_delta_target <= max_compensatable_delta:
            # --- CASE 1: Can fully compensate ---
            # Solve: Δω_eff × Ts_eff - Δω_eff²/(2R) = Δω_target × Ts
            # Quadratic: Δω_eff² - 2×R×Ts_eff×Δω_eff + 2×R×Ts×|Δω_target| = 0
            # Solution: Δω_eff = R×Ts_eff - √((R×Ts_eff)² - 2×R×Ts×|Δω_target|)
            
            discriminant = (R * Ts_eff) ** 2 - 2.0 * R * Ts * abs_delta_target
            
            # Numerical protection
            if discriminant < 0:
                discriminant = 0.0
                
            delta_eff = R * Ts_eff - math.sqrt(discriminant)
            delta_eff *= sign_delta  # Apply direction
            
            omega_eff = omega_prev + delta_eff
            
            # Ramp time to reach omega_eff
            t_ramp = abs(delta_eff) / R if R > 1e-9 else 0.0
            
            if t_ramp <= Ts_eff:
                # Ramp completes within cycle - end velocity equals commanded
                omega_end = omega_eff
            else:
                # Shouldn't happen in this branch, but handle gracefully
                omega_end = omega_prev + sign_delta * R * Ts_eff
                
        else:
            # --- CASE 2: Cannot fully compensate ---
            # The requested change is too large to achieve in one cycle.
            # Command the maximum possible ramp; MPC will correct on next iteration.
            
            # Maximum velocity change achievable in Ts_eff
            delta_max = R * Ts_eff * sign_delta
            omega_eff = omega_prev + delta_max
            
            # The wheel will ramp for the full Ts_eff
            omega_end = omega_eff
            
            if self.logger:
                self.logger.debug(
                    f"Ramp compensation saturated: requested Δω={delta_target:.4f}, "
                    f"max={max_compensatable_delta:.4f}, commanding Δω={delta_max:.4f}"
                )
        
        return omega_eff, omega_end
    
    def compensate(
        self,
        left_target: float,
        right_target: float,
    ) -> Tuple[float, float]:
        """
        Compute ramp-compensated velocities for both wheels.
        
        Args:
            left_target: Desired left wheel velocity (from MPC → differential drive conversion)
            right_target: Desired right wheel velocity (from MPC → differential drive conversion)
            
        Returns:
            (left_eff, right_eff): Compensated velocities to command to wheels
            
        Note: Call this once per MPC cycle. It updates internal state tracking.
        """
        # Compute compensated velocities using previous cycle's end velocity as starting point
        left_eff, left_end = self._compute_compensated_velocity(
            self._left_vel_at_cycle_end, left_target
        )
        right_eff, right_end = self._compute_compensated_velocity(
            self._right_vel_at_cycle_end, right_target
        )
        
        # Update state for next cycle
        self._left_vel_at_cycle_end = left_end
        self._right_vel_at_cycle_end = right_end
        self._left_cmd_prev = left_eff
        self._right_cmd_prev = right_eff
        
        return left_eff, right_eff
    
    def get_current_wheel_velocities(self) -> Tuple[float, float]:
        """
        Get the tracked wheel velocities at end of previous cycle.
        
        Returns:
            (left_vel, right_vel): Estimated actual wheel velocities
        """
        return self._left_vel_at_cycle_end, self._right_vel_at_cycle_end
    
    def set_current_wheel_velocities(self, left_vel: float, right_vel: float) -> None:
        """
        Manually set the current wheel velocities.
        
        Use this to sync with actual encoder feedback if available,
        or to initialize from a known state.
        
        Args:
            left_vel: Current left wheel velocity
            right_vel: Current right wheel velocity
        """
        self._left_vel_at_cycle_end = float(left_vel)
        self._right_vel_at_cycle_end = float(right_vel)


class AccelMPC:
    """
    LTV MPC with augmented state and delta-input control.

    State (nx = 5):
        x = [xe, ye, θe, v, ω]^T

    Control (nu = 2):
        u = Δu = [Δv, Δω]^T

    Dynamics:
        x_{k+1} = A_aug_k x_k + B_aug Δu_k

    where:
      A_aug_k = [[A_err_k, B_err_k],
                 [0_2x3,   I_2   ]]

      B_aug   = [[0_3x2],
                 [I_2  ]]

    Error dynamics (Kanayama-style, small-angle linearization):
        xe_dot ≈ v_ref - v
        ye_dot ≈  v_ref * θe
        θe_dot ≈ ω_ref - ω
    """

    def __init__(
        self,
        N: int,
        Ts: float,
        v_min: float,
        v_max: float,
        omega_min: float,
        omega_max: float,
        Q_xe: float,
        Q_ye: float,
        Q_yaw: float,
        R_delta_v: float,
        R_delta_omega: float,
        dv_max: float,
        domega_max: float,
        logger=None,
        weight_increase_xe: float = 0.0,
        weight_increase_ye: float = 0.0,
        weight_increase_yaw: float = 0.0,
        solver_debug_enabled: bool = False,
    ) -> None:
        self.N = int(N)
        self.Ts = float(Ts)
        self.logger = logger

        # State and control dimensions
        self.nx = 5  # [xe, ye, θe, v, ω]
        self.nu = 2  # [Δv, Δω]
        # Decision vector: [Δu_0, x_1, Δu_1, x_2, ..., Δu_{N-1}, x_N]
        self.nz = self.N * (self.nu + self.nx)

        # Robot-level velocity limits (for v, ω)
        self.v_min = float(v_min)
        self.v_max = float(v_max)
        self.omega_min = float(omega_min)
        self.omega_max = float(omega_max)
        self._v_min_base = self.v_min
        self._v_max_base = self.v_max

        # Cost weights
        self.Q_xe_base = float(Q_xe)
        self.Q_ye_base = float(Q_ye)
        self.Q_yaw_base = float(Q_yaw)
        # Separate Δ-costs for linear and angular velocity
        self.R_delta_v = float(R_delta_v)
        self.R_delta_omega = float(R_delta_omega)

        # Rate limits on Δv and Δω per step (symmetric bounds)
        self.dv_max = float(abs(dv_max))
        self.domega_max = float(abs(domega_max))

        self.weight_increase_xe = float(weight_increase_xe)
        self.weight_increase_ye = float(weight_increase_ye)
        self.weight_increase_yaw = float(weight_increase_yaw)

        # Sparse matrices and solver
        self.P = None
        self.A_constr = None
        self.l_constr = None
        self.u_constr = None
        self.solver: Optional[osqp.OSQP] = None

        # Indices for fast updates
        self.A_indices = {}
        self.B_indices = {}
        self.row_col_to_data_idx = {}

        # For warm-starting
        self.prev_solution: Optional[np.ndarray] = None

        # Debug result storage
        self._last_result = None

        # Solver debug / verbosity
        self.solver_debug_enabled = bool(solver_debug_enabled)

        # Flag
        self.initialized = False

    def set_velocity_bound_scale(self, scale: float) -> None:
        """Scale the v bounds by a factor in [0, 1]."""
        try:
            s = float(scale)
        except (TypeError, ValueError):
            s = 1.0
        s = max(0.0, min(1.0, s))
        self.v_min = self._v_min_base * s
        self.v_max = self._v_max_base * s

    def setup(self) -> bool:
        """Pre-build QP matrices and set up the OSQP solver."""
        if not OSQP_AVAILABLE or not SCIPY_AVAILABLE:
            if self.logger:
                self.logger.error("OSQP and scipy required for AccelMPC")
            return False

        N = self.N
        nx = self.nx
        nu = self.nu
        nz = self.nz

        # ============================
        # 1) Cost matrix P
        # ============================
        P_data: List[float] = []
        P_row: List[int] = []
        P_col: List[int] = []

        # State cost: only on (xe, ye, θe), not on (v, ω)
        for k in range(N):
            x_k_start = k * (nu + nx) + nu

            # Time-varying weights
            w_xe = self.Q_xe_base * (1.0 + self.weight_increase_xe * k)
            w_ye = self.Q_ye_base * (1.0 + self.weight_increase_ye * k)
            w_yaw = self.Q_yaw_base * (1.0 + self.weight_increase_yaw * k)

            # xe
            P_row.append(x_k_start + 0)
            P_col.append(x_k_start + 0)
            P_data.append(w_xe)

            # ye
            P_row.append(x_k_start + 1)
            P_col.append(x_k_start + 1)
            P_data.append(w_ye)

            # yaw error
            P_row.append(x_k_start + 2)
            P_col.append(x_k_start + 2)
            P_data.append(w_yaw)

        # Control cost: ||Δu_k||^2_R for each step (no cross-terms between steps)
        # Allow different penalties for Δv and Δω.
        for k in range(N):
            u_k_start = k * (nu + nx)
            # Δv
            P_row.append(u_k_start + 0)
            P_col.append(u_k_start + 0)
            P_data.append(self.R_delta_v)
            # Δω
            P_row.append(u_k_start + 1)
            P_col.append(u_k_start + 1)
            P_data.append(self.R_delta_omega)

        P_coo = sparse.coo_matrix((P_data, (P_row, P_col)), shape=(nz, nz))
        P_sym = (P_coo + P_coo.T) / 2.0
        # Small regularization
        regularization = 1e-6
        P_sym = P_sym + sparse.eye(nz, format='csc') * regularization

        self.P = P_sym.tocsc()

        # ============================
        # 2) Constraint matrix A
        # ============================
        # Constraints:
        #   - N * nx dynamics rows
        #   - N * 2 velocity bounds rows (v and ω)
        #   - N * 2 rate bounds rows (Δv and Δω)
        n_dynamics = N * nx
        n_input_bounds = N * 2
        n_rate_bounds = N * 2
        n_constraints = n_dynamics + n_input_bounds + n_rate_bounds

        row_indices: List[int] = []
        col_indices: List[int] = []
        data_values: List[float] = []

        self.A_indices = {}
        self.B_indices = {}

        constraint_row = 0
        # Dynamics: -B_k Δu_k + x_{k+1} - A_k x_k = 0
        for k in range(N):
            u_k_idx = k * (nu + nx)
            x_k_idx = (k - 1) * (nu + nx) + nu if k > 0 else None
            x_kp1_idx = k * (nu + nx) + nu

            self.A_indices[k] = []
            self.B_indices[k] = []

            for i in range(nx):
                # x_{k+1}[i]
                row_indices.append(constraint_row + i)
                col_indices.append(x_kp1_idx + i)
                data_values.append(1.0)

                # -A_k x_k (for k>0, x_k is a decision variable)
                if k > 0:
                    for j in range(nx):
                        idx = len(data_values)
                        row_indices.append(constraint_row + i)
                        col_indices.append(x_k_idx + j)
                        data_values.append(0.0)  # placeholder for -A_aug_k[i,j]
                        self.A_indices[k].append(idx)

                # -B_k Δu_k
                for j in range(nu):
                    idx = len(data_values)
                    row_indices.append(constraint_row + i)
                    col_indices.append(u_k_idx + j)
                    data_values.append(0.0)  # placeholder for -B_aug[i,j]
                    self.B_indices[k].append(idx)

            constraint_row += nx

        # Input bounds: v and ω limits on x_{k+1}[3], x_{k+1}[4]
        for k in range(N):
            x_kp1_idx = k * (nu + nx) + nu

            # v_k+1 row
            row_indices.append(constraint_row)
            col_indices.append(x_kp1_idx + 3)  # v index in state
            data_values.append(1.0)
            constraint_row += 1

            # ω_k+1 row
            row_indices.append(constraint_row)
            col_indices.append(x_kp1_idx + 4)  # ω index in state
            data_values.append(1.0)
            constraint_row += 1

        # Rate bounds: Δv and Δω limits on the control inputs
        for k in range(N):
            u_k_idx = k * (nu + nx)

            # Δv_k row
            row_indices.append(constraint_row)
            col_indices.append(u_k_idx + 0)  # Δv index in decision vector
            data_values.append(1.0)
            constraint_row += 1

            # Δω_k row
            row_indices.append(constraint_row)
            col_indices.append(u_k_idx + 1)  # Δω index in decision vector
            data_values.append(1.0)
            constraint_row += 1

        A_coo = sparse.coo_matrix(
            (data_values, (row_indices, col_indices)),
            shape=(n_constraints, nz),
        )
        self.A_constr = A_coo.tocsc()

        # Build row/col -> data index mapping
        self.row_col_to_data_idx = {}
        csc_indices = self.A_constr.indices
        csc_indptr = self.A_constr.indptr

        for col in range(nz):
            col_start = csc_indptr[col]
            col_end = csc_indptr[col + 1]
            for csc_idx in range(col_start, col_end):
                row = csc_indices[csc_idx]
                self.row_col_to_data_idx[(row, col)] = csc_idx

        # Convert stored A_indices/B_indices to CSC indices
        for k in range(N):
            if k in self.A_indices:
                new_A_indices = []
                if k > 0:
                    x_k_idx = (k - 1) * (nu + nx) + nu
                    for i in range(nx):
                        for j in range(nx):
                            row = k * nx + i
                            col = x_k_idx + j
                            idx = self.row_col_to_data_idx.get((row, col))
                            if idx is not None:
                                new_A_indices.append(idx)
                self.A_indices[k] = new_A_indices

            if k in self.B_indices:
                new_B_indices = []
                u_k_idx = k * (nu + nx)
                for i in range(nx):
                    for j in range(nu):
                        row = k * nx + i
                        col = u_k_idx + j
                        idx = self.row_col_to_data_idx.get((row, col))
                        if idx is not None:
                            new_B_indices.append(idx)
                self.B_indices[k] = new_B_indices

        # ============================
        # 3) Bounds
        # ============================
        self.l_constr = np.zeros(n_constraints)
        self.u_constr = np.zeros(n_constraints)

        # Dynamics are equalities: l = u = RHS (set later in solve())
        # For now, leave at zero.

        # Velocity bounds (v, ω) and rate bounds (Δv, Δω)
        for k in range(N):
            # v, ω bounds indices
            v_idx = n_dynamics + 2 * k
            omega_idx = n_dynamics + 2 * k + 1

            self.l_constr[v_idx] = self.v_min
            self.u_constr[v_idx] = self.v_max

            self.l_constr[omega_idx] = self.omega_min
            self.u_constr[omega_idx] = self.omega_max

            # Rate bounds indices
            dv_idx = n_dynamics + n_input_bounds + 2 * k
            domega_idx = n_dynamics + n_input_bounds + 2 * k + 1

            self.l_constr[dv_idx] = -self.dv_max
            self.u_constr[dv_idx] = self.dv_max

            self.l_constr[domega_idx] = -self.domega_max
            self.u_constr[domega_idx] = self.domega_max

        # ============================
        # 4) Solver setup
        # ============================
        assert self.P.shape == (nz, nz)
        assert self.A_constr.shape == (n_constraints, nz)
        assert len(self.l_constr) == n_constraints
        assert len(self.u_constr) == n_constraints

        self.solver = osqp.OSQP()
        self.solver.setup(
            P=self.P,
            q=None,
            A=self.A_constr,
            l=self.l_constr,
            u=self.u_constr,
            verbose=self.solver_debug_enabled,
            warm_start=True,
            polish=True,
        )
        self.initialized = True
        if self.logger:
            self.logger.info("✓ AccelMPC optimizer initialized")
        return True

    def update_v_bounds(self, v_scale: float) -> None:
        """Update v bounds (used for optional gating)."""
        if not self.initialized:
            return
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

    def _build_A_aug_and_B_aug(
        self,
        v_ref: float,
        omega_ref: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Build augmented A and B for a given reference velocity.

        Error dynamics (discrete, sample time Ts):
            A_err_k = [[1, -ω_ref*Ts,   0],
                       [ω_ref*Ts,   1,  v_ref*Ts],
                       [0,          0,  1]]

            xe_dot ≈ v_ref - v
            ye_dot ≈ v_ref * θe
            θe_dot ≈ ω_ref - ω

        B_err_cont = [[-1,  0],
                      [ 0,  0],
                      [ 0, -1]]
        B_err      = Ts * B_err_cont
        """
        Ts = self.Ts

        A_err = np.array(
            [
                [1.0, -omega_ref * Ts, 0.0],
                [omega_ref * Ts, 1.0, v_ref * Ts],
                [0.0, 0.0, 1.0],
            ]
        )

        B_err = Ts * np.array(
            [
                [-1.0, 0.0],
                [0.0, 0.0],
                [0.0, -1.0],
            ]
        )

        # Augmented A (5x5)
        A_aug = np.eye(self.nx)
        A_aug[0:3, 0:3] = A_err
        A_aug[0:3, 3:5] = B_err
        # Bottom-right already identity for [v, ω]

        # Augmented B (5x2): Δv, Δω affect both error states (via B_err) and [v, ω]
        # This models "immediate effect" discretization where Δu takes effect at
        # the start of the interval, giving the solver more flexibility.
        B_aug = np.zeros((self.nx, self.nu))
        B_aug[0:3, :] = B_err        # Immediate effect on [xe, ye, θe]
        B_aug[3, 0] = 1.0            # v_{k+1} = v_k + Δv_k
        B_aug[4, 1] = 1.0            # ω_{k+1} = ω_k + Δω_k

        return A_aug, B_aug

    def update_matrices(self, ref_traj: List[np.ndarray]) -> None:
        """Update dynamic matrices (A_aug_k, B_aug) in constraint matrix."""
        if not self.initialized:
            return

        N = self.N
        nx = self.nx
        nu = self.nu

        A_data = self.A_constr.data.copy()
        if len(A_data) != len(self.A_constr.data):
            if self.logger:
                self.logger.error("A_constr data size mismatch during update")
            return

        for k in range(N):
            if k < len(ref_traj):
                wp = ref_traj[k]
            elif len(ref_traj) > 0:
                wp = ref_traj[-1]
            else:
                wp = np.zeros(6)

            # wp[5] (vyaw_ref) now carries the SIGNED body-frame velocity reference
            # This is crucial for correct error dynamics in reverse mode
            v_ref = float(wp[5])  # Signed v_ref (negative when reversing)
            omega_ref = 0.0  # Angular velocity reference (typically 0 for straight lines)

            A_aug, B_aug = self._build_A_aug_and_B_aug(v_ref, omega_ref)

            # -A_aug x_k terms (k>0)
            if k > 0 and k in self.A_indices:
                A_neg = -A_aug
                for idx, matrix_idx in enumerate(self.A_indices[k]):
                    if matrix_idx >= len(A_data):
                        continue
                    i = idx // nx
                    j = idx % nx
                    if i < nx and j < nx:
                        A_data[matrix_idx] = A_neg[i, j]

            # -B_aug Δu_k terms
            if k in self.B_indices:
                B_neg = -B_aug
                for idx, matrix_idx in enumerate(self.B_indices[k]):
                    if matrix_idx >= len(A_data):
                        continue
                    i = idx // nu
                    j = idx % nu
                    if i < nx and j < nu:
                        A_data[matrix_idx] = B_neg[i, j]

        self.A_constr.data = A_data
        self.solver.update(Ax=A_data)

    def solve(
        self,
        current_error: np.ndarray,
        current_v: float,
        current_omega: float,
        ref_traj: List[np.ndarray],
    ) -> Tuple[np.ndarray, float, Optional[np.ndarray]]:
        """
        Solve the MPC problem.

        Args:
            current_error: [xe, ye, θe]^T
            current_v:     current linear velocity (v)
            current_omega: current angular velocity (ω)
            ref_traj:      list of waypoints [x, y, yaw, vx, vy, vyaw]

        Returns:
            (Δu0_opt, solve_time_ms, solution)
        """
        if not self.initialized:
            if not self.setup():
                return np.zeros(2), 0.0, None

        import time as _time  # local import to avoid name clash

        solve_start = _time.time()

        # Update dynamics matrices based on new reference
        self.update_matrices(ref_traj)

        # Initial augmented state x0 = [xe, ye, θe, v, ω]
        # Clamp initial velocities to be within bounds to prevent infeasibility
        # when measured velocity exceeds limits (e.g., due to noise or disturbance)
        x0 = np.zeros(self.nx)
        x0[0:3] = current_error.reshape(3,)
        x0[3] = float(np.clip(current_v, self.v_min, self.v_max))
        x0[4] = float(np.clip(current_omega, self.omega_min, self.omega_max))

        # Build A_aug_0 for initial constraint RHS
        if len(ref_traj) > 0:
            wp0 = ref_traj[0]
            # wp0[5] now carries the SIGNED body-frame velocity reference
            v_ref0 = float(wp0[5])
            omega_ref0 = 0.0
        else:
            v_ref0 = 0.0
            omega_ref0 = 0.0

        A_aug0, _ = self._build_A_aug_and_B_aug(v_ref0, omega_ref0)
        rhs0 = A_aug0 @ x0

        # First nx dynamics rows (k=0): equality constraint to rhs0
        for i in range(self.nx):
            self.l_constr[i] = rhs0[i]
            self.u_constr[i] = rhs0[i]

        # For k>0 dynamics rows, RHS = 0 (homogeneous equality constraints)
        N = self.N
        nx = self.nx
        for k in range(1, N):
            for i in range(nx):
                row_idx = k * nx + i
                self.l_constr[row_idx] = 0.0
                self.u_constr[row_idx] = 0.0

        # Refresh v and ω bounds
        n_dynamics = N * nx
        n_input_bounds = N * 2
        for k in range(N):
            v_idx = n_dynamics + 2 * k
            omega_idx = n_dynamics + 2 * k + 1
            self.l_constr[v_idx] = self.v_min
            self.u_constr[v_idx] = self.v_max
            self.l_constr[omega_idx] = self.omega_min
            self.u_constr[omega_idx] = self.omega_max

            # Refresh rate bounds (Δv, Δω)
            dv_idx = n_dynamics + n_input_bounds + 2 * k
            domega_idx = n_dynamics + n_input_bounds + 2 * k + 1
            self.l_constr[dv_idx] = -self.dv_max
            self.u_constr[dv_idx] = self.dv_max
            self.l_constr[domega_idx] = -self.domega_max
            self.u_constr[domega_idx] = self.domega_max

        # Warm start
        if self.prev_solution is not None and len(self.prev_solution) == self.nz:
            prev = self.prev_solution
            warm = np.zeros_like(prev)
            # Shift by one stage: [Δu_0, x_1, Δu_1, ...] -> [Δu_1, x_2, ..., Δu_{N-1}, x_N]
            block = self.nu + self.nx
            if len(prev) >= block:
                warm[:-block] = prev[block:]
                warm[-block:] = prev[-block:]
            self.solver.warm_start(x=warm)

        # Update solver bounds
        self.solver.update(l=self.l_constr, u=self.u_constr)

        result = self.solver.solve()
        if result.info.status != "solved":
            if self.logger:
                self.logger.warn(f"AccelMPC solve failed: {result.info.status}")
            return np.zeros(2), 0.0, None

        self._last_result = result
        solution = result.x
        du0 = solution[0 : self.nu]

        self.prev_solution = solution
        solve_time_ms = (solve_start - _time.time()) * -1000.0
        return du0, solve_time_ms, solution

    def compute_solution_cost(
        self,
        solution: Optional[np.ndarray],
        ref_traj: List[np.ndarray],
    ) -> float:
        """Compute cost from solution for diagnostics."""
        if solution is None or len(solution) != self.nz:
            return 0.0

        cost = 0.0
        nx = self.nx
        nu = self.nu

        for k in range(self.N):
            x_idx = k * (nu + nx) + nu
            xk = solution[x_idx : x_idx + nx]

            w_xe = self.Q_xe_base * (1.0 + self.weight_increase_xe * k)
            w_ye = self.Q_ye_base * (1.0 + self.weight_increase_ye * k)
            w_yaw = self.Q_yaw_base * (1.0 + self.weight_increase_yaw * k)

            cost += w_xe * xk[0] ** 2 + w_ye * xk[1] ** 2 + w_yaw * xk[2] ** 2

        for k in range(self.N):
            u_idx = k * (nu + nx)
            uk = solution[u_idx : u_idx + nu]
            cost += (
                self.R_delta_v * float(uk[0] ** 2)
                + self.R_delta_omega * float(uk[1] ** 2)
            )

        return float(cost)


class MPCAccelController(Node):
    """
    ROS2 node that wraps AccelMPC and interfaces with ODrive CAN wheels.

    - Subscribes:
        /Odometry_tilt_corrected_diff  (nav_msgs/Odometry)
        /set_target_pose               (std_msgs/Float64MultiArray [x,y,z,yaw])

    - Publishes:
        /left/control_message          (odrive_can/ControlMessage)
        /right/control_message         (odrive_can/ControlMessage)
        /mpc_accel/cmd_twist           (geometry_msgs/Twist)
    """

    def __init__(self) -> None:
        super().__init__("mpc_accel_controller")

        # Parameters
        self.declare_parameter("wheel_radius", 0.09)
        self.declare_parameter("wheel_base", 0.32)
        self.declare_parameter("gear_ratio", 1.0)
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", True)

        self.declare_parameter("control_frequency", 10.0)
        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 1.0)

        self.declare_parameter("mpc_horizon", 40)
        self.declare_parameter("mpc_dt", 0.1)

        self.declare_parameter("mpc_Q_xe", 20.0)
        self.declare_parameter("mpc_Q_ye", 15.0)
        self.declare_parameter("mpc_Q_yaw", 2.0)  # Lower = less aggressive heading correction
        # Separate Δ-costs for linear and angular velocity
        # Higher R values = smoother motion, less oscillation
        self.declare_parameter("mpc_R_delta_v", 0.1)
        self.declare_parameter("mpc_R_delta_omega", 0.8)  # High value to prevent squiggly motion
        # Rate limits on Δv and Δω per step
        self.declare_parameter("mpc_dv_max", 0.05)      # m/s per control step
        self.declare_parameter("mpc_domega_max", 0.08)  # rad/s per control step
        # Optional time-varying weight scaling (set to 0 for stability)
        self.declare_parameter("mpc_weight_increase_xe", 0.0)
        self.declare_parameter("mpc_weight_increase_ye", 0.0)
        self.declare_parameter("mpc_weight_increase_yaw", 0.0)
        # Compatibility parameters with MPCAutonomousController (even if unused)
        self.declare_parameter("slip_history_length", 100)
        self.declare_parameter("slip_estimation_window", 1.0)
        self.declare_parameter("lookahead_distance", 0.5)
        self.declare_parameter("waypoints_csv_path", "")

        # Autonomy / behavior flags
        self.declare_parameter("mpc_autonomy_enabled_default", False)
        self.declare_parameter("enable_yaw_gating", False)

        # Topic parameters
        self.declare_parameter("odometry_topic", "/Odometry_tilt_corrected_diff")
        self.declare_parameter("left_control_topic", "/left/control_message")
        self.declare_parameter("right_control_topic", "/right/control_message")
        # Encoder topics are unused here but declared for launch compatibility
        self.declare_parameter("left_encoder_topic", "/left/controller_status")
        self.declare_parameter("right_encoder_topic", "/right/controller_status")

        # Solver debug flag (wired into AccelMPC)
        self.declare_parameter("solver_debug_enabled", False)

        # Stopping criterion (same semantics as MPCAutonomousController)
        self.declare_parameter("target_reached_threshold", 0.01)

        # Wheel ramp compensation parameters
        # ramp_rate: ODrive vel_ramp_rate in turn/s² (must match ODrive config)
        # delay_time: Pure transport delay (CAN latency + processing) in seconds
        # Set ramp_compensation_enabled=True and input_mode to VEL_RAMP for best results
        self.declare_parameter("ramp_compensation_enabled", False)
        self.declare_parameter("wheel_ramp_rate", 20.0)  # turn/s² (from ODrive config)
        self.declare_parameter("wheel_delay_time", 0.01)  # seconds (~10ms typical CAN delay)

        # Get parameters
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.gear_ratio = float(self.get_parameter("gear_ratio").value)
        self.invert_left = bool(self.get_parameter("invert_left").value)
        self.invert_right = bool(self.get_parameter("invert_right").value)

        self.control_freq = float(self.get_parameter("control_frequency").value)
        self.max_linear_vel = float(self.get_parameter("max_linear_velocity").value)
        self.max_angular_vel = float(self.get_parameter("max_angular_velocity").value)

        self.mpc_horizon = int(self.get_parameter("mpc_horizon").value)
        self.mpc_dt = float(self.get_parameter("mpc_dt").value)

        mpc_Q_xe = float(self.get_parameter("mpc_Q_xe").value)
        mpc_Q_ye = float(self.get_parameter("mpc_Q_ye").value)
        mpc_Q_yaw = float(self.get_parameter("mpc_Q_yaw").value)
        mpc_R_delta_v = float(self.get_parameter("mpc_R_delta_v").value)
        mpc_R_delta_omega = float(self.get_parameter("mpc_R_delta_omega").value)
        mpc_dv_max = float(self.get_parameter("mpc_dv_max").value)
        mpc_domega_max = float(self.get_parameter("mpc_domega_max").value)
        mpc_weight_increase_xe = float(self.get_parameter("mpc_weight_increase_xe").value)
        mpc_weight_increase_ye = float(self.get_parameter("mpc_weight_increase_ye").value)
        mpc_weight_increase_yaw = float(self.get_parameter("mpc_weight_increase_yaw").value)

        # Misc / compatibility params (some unused but kept for drop-in replacement)
        self.slip_history_length = int(self.get_parameter("slip_history_length").value)
        self.slip_estimation_window = float(
            self.get_parameter("slip_estimation_window").value
        )
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.waypoints_csv_path = str(self.get_parameter("waypoints_csv_path").value)

        # Autonomy flags
        self.autonomy_enabled: bool = bool(
            self.get_parameter("mpc_autonomy_enabled_default").value
        )
        self.enable_yaw_gating: bool = bool(
            self.get_parameter("enable_yaw_gating").value
        )

        # Topic names
        odom_topic = str(self.get_parameter("odometry_topic").value)
        left_ctrl_topic = str(self.get_parameter("left_control_topic").value)
        right_ctrl_topic = str(self.get_parameter("right_control_topic").value)
        # encoder topics retrieved but unused (for compatibility)
        self.left_encoder_topic = str(self.get_parameter("left_encoder_topic").value)
        self.right_encoder_topic = str(self.get_parameter("right_encoder_topic").value)

        # Solver debug
        self.solver_debug_enabled: bool = bool(
            self.get_parameter("solver_debug_enabled").value
        )

        self.target_reached_threshold = float(
            self.get_parameter("target_reached_threshold").value
        )

        # Wheel ramp compensation
        self.ramp_compensation_enabled = bool(
            self.get_parameter("ramp_compensation_enabled").value
        )
        self.wheel_ramp_rate = float(self.get_parameter("wheel_ramp_rate").value)
        self.wheel_delay_time = float(self.get_parameter("wheel_delay_time").value)

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.pose_initialized = False

        # Note: Velocity feedback is NOT used. Instead, we use commanded velocity
        # (v_cmd, omega_cmd) as the MPC state. This avoids noisy velocity estimates.

        # Commanded velocities (v, ω) that we integrate Δu into
        self.v_cmd = 0.0
        self.omega_cmd = 0.0

        # Target pose and path state
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.has_target = False
        self.target_reached = False
        # Straight-line path start (same semantics as mpc_autonomous_controller)
        self.path_start_x = 0.0
        self.path_start_y = 0.0
        self.path_start_yaw = 0.0
        self.path_initialized = False
        
        # Reverse mode flag: True if robot should drive backwards to reach target
        # (chosen when reversing requires less yaw change than going forward)
        self.reverse_mode = False

        # Waypoint sequence state (CSV / F2C waypoints)
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_index: int = 0
        self.waypoint_navigation_active: bool = False
        self.previous_waypoint: Optional[Tuple[float, float]] = None

        # Pending waypoints received from F2C GUI (until "Start Navigation" pressed)
        self.pending_waypoints: List[Tuple[float, float]] = []

        # Parameters for shaping reference behavior near the start of a line
        self.declare_parameter("error_ref_ahead_min_scale", 0.02)
        self.declare_parameter("error_ref_gate_distance", 0.20)
        self.error_ref_ahead_min_scale = float(
            self.get_parameter("error_ref_ahead_min_scale").value
        )
        self.error_ref_gate_distance = float(
            self.get_parameter("error_ref_gate_distance").value
        )

        # MPC optimizer
        self.mpc = AccelMPC(
            N=self.mpc_horizon,
            Ts=self.mpc_dt,
            v_min=-self.max_linear_vel,
            v_max=self.max_linear_vel,
            omega_min=-self.max_angular_vel,
            omega_max=self.max_angular_vel,
            Q_xe=mpc_Q_xe,
            Q_ye=mpc_Q_ye,
            Q_yaw=mpc_Q_yaw,
            R_delta_v=mpc_R_delta_v,
            R_delta_omega=mpc_R_delta_omega,
            dv_max=mpc_dv_max,
            domega_max=mpc_domega_max,
            logger=self.get_logger(),
            weight_increase_xe=mpc_weight_increase_xe,
            weight_increase_ye=mpc_weight_increase_ye,
            weight_increase_yaw=mpc_weight_increase_yaw,
            solver_debug_enabled=self.solver_debug_enabled,
        )

        # Wheel ramp compensator (for VEL_RAMP mode actuator dynamics)
        self.wheel_compensator = WheelRampCompensator(
            ramp_rate=self.wheel_ramp_rate,
            delay_time=self.wheel_delay_time,
            cycle_time=self.mpc_dt,
            logger=self.get_logger() if self.solver_debug_enabled else None,
        )

        if self.ramp_compensation_enabled:
            self.get_logger().info(
                f"✓ Wheel ramp compensation ENABLED: "
                f"ramp_rate={self.wheel_ramp_rate:.1f} turn/s², "
                f"delay={self.wheel_delay_time*1000:.1f} ms"
            )
        else:
            self.get_logger().info(
                "Wheel ramp compensation DISABLED (using direct velocity commands)"
            )

        # Subscribers
        self.create_subscription(
            Odometry,
            odom_topic,
            self.odometry_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Float64MultiArray,
            "/set_target_pose",
            self.set_target_callback,
            10,
        )

        # Subscribe to waypoint CSV trigger (topic name matches MPCAutonomousController)
        self.waypoint_nav_sub = self.create_subscription(
            String,
            "/start_waypoint_navigation",
            self.start_waypoint_navigation_callback,
            10,
        )

        # Subscriber for direct waypoint arrays from F2C GUI
        # Format: [x1, y1, x2, y2, ...] (pairs of x,y) or [0.0] as "start" signal
        self.f2c_waypoint_array_sub = self.create_subscription(
            Float64MultiArray,
            "/f2c_waypoints",
            self.f2c_waypoint_array_callback,
            10,
        )

        # Autonomy enable / disable (same topic as MPCAutonomousController)
        self.autonomy_enable_sub = self.create_subscription(
            Bool,
            "/mpc_autonomy_enable",
            self.autonomy_enable_callback,
            10,
        )

        # Publishers
        self.left_pub = self.create_publisher(ControlMessage, left_ctrl_topic, 10)
        self.right_pub = self.create_publisher(ControlMessage, right_ctrl_topic, 10)
        # Diagnostics
        self.cmd_pub = self.create_publisher(Twist, "/mpc_accel/cmd_twist", 10)
        self.error_state_pub = self.create_publisher(
            Float64MultiArray, "/mpc_accel/error_state", 10
        )
        self.ref_traj_pub = self.create_publisher(
            Float64MultiArray, "/mpc_accel/reference_trajectory", 10
        )
        self.delta_cmd_pub = self.create_publisher(
            Float64MultiArray, "/mpc_accel/delta_cmd", 10
        )

        # ODrive axis state / clear error clients for arming/disarming
        self.left_axis_client = self.create_client(
            AxisState, "/left/request_axis_state"
        )
        self.right_axis_client = self.create_client(
            AxisState, "/right/request_axis_state"
        )

        self.left_clear_client = self.create_client(Empty, "/left/clear_errors")
        self.right_clear_client = self.create_client(Empty, "/right/clear_errors")

        # Optional shutdown mapping client (same as main MPC controller)
        self.shutdown_client = self.create_client(Trigger, "/shutdown_mapping")

        # Motor arming state
        self._arm_attempts = 0
        self._arm_max_attempts = 5
        self._arm_timer = self.create_timer(1.0, self._attempt_arm_motors)

        # Soft shutdown service for this accel controller
        self.soft_shutdown_srv = self.create_service(
            Trigger,
            "/mpc_accel_autonomous_controller/soft_shutdown",
            self._soft_shutdown_service,
        )

        # Alias soft-shutdown service name for drop-in replacement with MPCAutonomousController
        self.soft_shutdown_alias_srv = self.create_service(
            Trigger,
            "/mpc_autonomous_controller/soft_shutdown",
            self._soft_shutdown_service,
        )

        # Control loop timer
        period = 1.0 / self.control_freq if self.control_freq > 0.0 else 0.1
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Acceleration-based MPC Controller started")
        self.get_logger().info(f"Horizon: {self.mpc_horizon}, dt: {self.mpc_dt:.3f} s")
        self.get_logger().info(f"Max v: {self.max_linear_vel:.2f} m/s, Max ω: {self.max_angular_vel:.2f} rad/s")
        self.get_logger().info("=" * 60)

    # -----------------------------
    # Callbacks
    # -----------------------------
    def odometry_callback(self, msg: Odometry) -> None:
        """
        Process odometry for POSITION only.
        Velocity is not needed - we use commanded velocity (v_cmd, omega_cmd) as the MPC state.
        """
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)
        self.current_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)

        if not self.pose_initialized:
            self.pose_initialized = True
            self.get_logger().info(
                f"✓ Odometry initialized at x={self.current_x:.3f}, "
                f"y={self.current_y:.3f}, yaw={math.degrees(self.current_yaw):.1f}°"
            )

    def set_target_callback(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 4:
            self.get_logger().warning("set_target_pose requires [x,y,z,yaw]")
            return
        if not self.pose_initialized:
            self.get_logger().warning("Cannot set target: odometry not initialized")
            return

        # Manual target pose clears any active waypoint navigation
        self.waypoint_navigation_active = False
        self.pending_waypoints = []

        self.target_x = float(data[0])
        self.target_y = float(data[1])
        
        # Compute heading from current position to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        forward_heading = math.atan2(dy, dx)
        
        # Compute reverse heading (180° offset)
        reverse_heading = self.normalize_angle(forward_heading + math.pi)
        
        # Choose the heading that requires minimum yaw change from current yaw
        forward_yaw_error = abs(self.normalize_angle(forward_heading - self.current_yaw))
        reverse_yaw_error = abs(self.normalize_angle(reverse_heading - self.current_yaw))
        
        if reverse_yaw_error < forward_yaw_error:
            # Reversing requires less turning - use reverse mode
            self.target_yaw = reverse_heading
            self.reverse_mode = True
        else:
            # Forward motion requires less or equal turning
            self.target_yaw = forward_heading
            self.reverse_mode = False
        
        self.has_target = True

        # Initialize straight-line path from current pose to target
        self.path_start_x = self.current_x
        self.path_start_y = self.current_y
        self.path_start_yaw = self.current_yaw
        self.path_initialized = True

        mode_str = "REVERSE" if self.reverse_mode else "FORWARD"
        self.get_logger().info(
            f"🎯 New MPC acceleration target: x={self.target_x:.2f}, "
            f"y={self.target_y:.2f}, yaw={math.degrees(self.target_yaw):.1f}° [{mode_str}]"
        )

    # -----------------------------
    # MPC utilities
    # -----------------------------
    def generate_reference_trajectory(self) -> List[np.ndarray]:
        """
        Generate local waypoints along the straight-line path from path_start to target,
        with spacing based on distance traveled along the line (mirrors the behavior
        of MPCAutonomousController.generate_local_waypoints).

        Returns:
            List of waypoints [x, y, yaw, vx, vy, vyaw] for the MPC horizon.
        """
        if (
            not self.path_initialized
            or not self.has_target
            or not self.pose_initialized
        ):
            return []

        # Path vector from start (when target was set) to target
        path_dx = self.target_x - self.path_start_x
        path_dy = self.target_y - self.path_start_y
        path_length = math.sqrt(path_dx * path_dx + path_dy * path_dy)

        if path_length < 1e-6:
            # Degenerate path: hold target pose
            waypoints: List[np.ndarray] = []
            for _ in range(self.mpc_horizon):
                waypoints.append(
                    np.array(
                        [self.target_x, self.target_y, self.target_yaw, 0.0, 0.0, 0.0],
                        dtype=float,
                    )
                )
            return waypoints

        # Unit direction along path
        path_dir_x = path_dx / path_length
        path_dir_y = path_dy / path_length

        # Project current position onto path line to get "travel distance" along the line
        to_current_x = self.current_x - self.path_start_x
        to_current_y = self.current_y - self.path_start_y
        t_closest = (to_current_x * path_dir_x + to_current_y * path_dir_y) / path_length
        t_closest = np.clip(t_closest, 0.0, 1.0)

        distance_to_target_along_path = (1.0 - t_closest) * path_length

        # Nominal spacing based on cruising speed and MPC time step
        cruising_speed = min(0.5, self.max_linear_vel)
        waypoint_spacing = cruising_speed * self.mpc_dt

        waypoints: List[np.ndarray] = []
        waypoint_positions: List[Tuple[float, float, float]] = []

        proximity_threshold = 0.5  # [m]
        min_spacing = 0.04  # [m]

        # Compute heading: forward or reverse based on reverse_mode flag
        forward_heading = math.atan2(path_dy, path_dx)
        if self.reverse_mode:
            # In reverse mode, the robot faces opposite to the path direction
            heading_to_target = self.normalize_angle(forward_heading + math.pi)
        else:
            heading_to_target = forward_heading

        if distance_to_target_along_path > proximity_threshold:
            # FAR region: fixed waypoint spacing along the path
            adjusted_spacing = waypoint_spacing
            for k in range(self.mpc_horizon):
                distance_along = adjusted_spacing * (k + 1)
                t_waypoint = t_closest + distance_along / path_length
                t_waypoint = float(np.clip(t_waypoint, 0.0, 1.0))

                wx = self.path_start_x + t_waypoint * path_dx
                wy = self.path_start_y + t_waypoint * path_dy
                wyaw = heading_to_target
                waypoint_positions.append((wx, wy, wyaw))
        else:
            # NEAR region: compress horizon within remaining distance, enforcing min spacing
            for k in range(self.mpc_horizon):
                desired_distance = min_spacing * (k + 1)
                if desired_distance < distance_to_target_along_path:
                    dist_from_start = t_closest * path_length + desired_distance
                    t_waypoint = dist_from_start / path_length
                else:
                    t_waypoint = 1.0
                t_waypoint = float(np.clip(t_waypoint, 0.0, 1.0))

                wx = self.path_start_x + t_waypoint * path_dx
                wy = self.path_start_y + t_waypoint * path_dy
                wyaw = heading_to_target
                waypoint_positions.append((wx, wy, wyaw))

        # v_ref shaping near the start of the line (same gating idea as original MPC)
        # Use forward_heading for velocity direction (path direction in world frame)
        d_gate_v = float(self.error_ref_gate_distance)
        s_closest = float(t_closest * path_length)

        if d_gate_v > 1e-6 and s_closest <= d_gate_v:
            start_scale = max(0.0, min(1.0, float(self.error_ref_ahead_min_scale)))
            ratio_s = s_closest / d_gate_v
            v_scale_ref = start_scale + (1.0 - start_scale) * ratio_s
        else:
            v_scale_ref = 1.0

        # Velocity reference: positive for forward, negative for reverse
        # This sign is crucial for the MPC error dynamics (ye_dot = v_ref * θe)
        v_ref = cruising_speed * v_scale_ref
        if self.reverse_mode:
            v_ref = -v_ref  # Negative body-frame velocity when reversing

        for k in range(self.mpc_horizon):
            wx, wy, wyaw = waypoint_positions[k]
            # Store velocity reference in the direction the robot is FACING (wyaw)
            # This encodes the signed body-frame velocity for MPC dynamics
            vx_ref = abs(v_ref) * math.cos(wyaw)  # World-frame velocity component
            vy_ref = abs(v_ref) * math.sin(wyaw)  # World-frame velocity component
            # Store the SIGNED v_ref in a way the MPC can extract it
            # We use the sign of the dot product with robot heading
            vyaw_ref = v_ref  # Hijack vyaw_ref to pass signed v_ref to MPC
            waypoints.append(
                np.array([wx, wy, wyaw, vx_ref, vy_ref, vyaw_ref], dtype=float)
            )

        return waypoints

    def compute_error_state(self, ref_wp: np.ndarray) -> np.ndarray:
        """
        Compute [xe, ye, θe]^T in the body frame relative to ref_wp.
        """
        dx = float(ref_wp[0]) - self.current_x
        dy = float(ref_wp[1]) - self.current_y

        cy = math.cos(self.current_yaw)
        sy = math.sin(self.current_yaw)

        xe = cy * dx + sy * dy
        ye = -sy * dx + cy * dy
        yaw_err = self.normalize_angle(float(ref_wp[2]) - self.current_yaw)

        return np.array([xe, ye, yaw_err], dtype=float)

    # -----------------------------
    # Waypoint helpers (CSV + F2C)
    # -----------------------------

    def _load_waypoints_from_csv(self, csv_file_path: str) -> List[Tuple[float, float]]:
        """
        Load a list of (x, y) waypoints from a CSV file.
        CSV format: x,y per row (header row is allowed and skipped).
        """
        waypoints: List[Tuple[float, float]] = []
        try:
            with open(csv_file_path, "r", newline="") as csvfile:
                reader = csv.reader(csvfile)
                for row_num, row in enumerate(reader):
                    if not row:
                        continue
                    try:
                        x = float(row[0].strip())
                        y = float(row[1].strip())
                        waypoints.append((x, y))
                    except (ValueError, IndexError):
                        if row_num == 0 and any(
                            keyword in " ".join(row).lower()
                            for keyword in ["x", "y", "waypoint", "point"]
                        ):
                            # Likely a header row
                            continue
                        else:
                            self.get_logger().warn(
                                f"Skipping invalid row {row_num + 1} in waypoint CSV: {row}"
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
        Set the next waypoint in the sequence as the current MPC target.

        Behavior:
        - target_x, target_y: current waypoint coordinates.
        - target_yaw: heading chosen to minimize yaw change from current heading.
          This allows the robot to reverse if it's more convenient than turning around.
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

        # Compute heading from current position to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        forward_heading = math.atan2(dy, dx)
        
        # Compute reverse heading (180° offset)
        reverse_heading = self.normalize_angle(forward_heading + math.pi)
        
        # Choose the heading that requires minimum yaw change from current yaw
        forward_yaw_error = abs(self.normalize_angle(forward_heading - self.current_yaw))
        reverse_yaw_error = abs(self.normalize_angle(reverse_heading - self.current_yaw))
        
        if reverse_yaw_error < forward_yaw_error:
            # Reversing requires less turning - use reverse mode
            self.target_yaw = reverse_heading
            self.reverse_mode = True
        else:
            # Forward motion requires less or equal turning
            self.target_yaw = forward_heading
            self.reverse_mode = False

        # Initialize path start at current pose for straight-line control
        self.path_start_x = self.current_x
        self.path_start_y = self.current_y
        self.path_start_yaw = self.current_yaw
        self.path_initialized = True

        self.has_target = True
        self.target_reached = False

        mode_str = "REVERSE" if self.reverse_mode else "FORWARD"
        self.get_logger().info(
            f"🎯 New waypoint target {self.current_waypoint_index + 1}/"
            f"{len(self.waypoints)}: x={self.target_x:.2f}, y={self.target_y:.2f}, "
            f"yaw_target={math.degrees(self.target_yaw):.1f}° [{mode_str}]"
        )

    def start_waypoint_navigation_callback(self, msg: String) -> None:
        """
        Topic callback to start waypoint navigation with CSV file.
        Receives std_msgs/String message containing CSV file path.
        """
        try:
            csv_file_path = msg.data.strip()
            if not csv_file_path:
                self.get_logger().error(
                    "Received empty CSV file path for waypoint navigation"
                )
                return

            if not os.path.exists(csv_file_path):
                self.get_logger().error(f"CSV file not found: {csv_file_path}")
                return

            # Parse waypoints from CSV
            waypoints = self._load_waypoints_from_csv(csv_file_path)
            if len(waypoints) < 1:
                self.get_logger().error(
                    f"No valid waypoints found in CSV: {csv_file_path}"
                )
                return

            # Ensure odometry is initialized before starting navigation
            if not self.pose_initialized:
                self.get_logger().error(
                    "Odometry not initialized yet - cannot start waypoint navigation"
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
                f"✓ Waypoint navigation started: {len(waypoints)} waypoints "
                f'loaded from "{csv_file_path}"'
            )
        except Exception as e:
            self.get_logger().error(f"Error starting waypoint navigation: {e}")

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
                self.get_logger().warn("Received empty /f2c_waypoints message")
                return

            # [0.0] is the "Start Navigation" signal from F2C GUI
            if len(data) == 1 and data[0] == 0.0:
                self.get_logger().info("Received /f2c_waypoints start signal [0.0]")
                self._start_pending_navigation()
                return

            if len(data) % 2 != 0:
                self.get_logger().error(
                    f"/f2c_waypoints length {len(data)} not divisible by 2; "
                    f"expected [x1,y1, x2,y2, ...]"
                )
                return

            num_waypoints = len(data) // 2
            waypoints_xy: List[Tuple[float, float]] = []
            for i in range(0, len(data), 2):
                x, y = data[i], data[i + 1]
                waypoints_xy.append((float(x), float(y)))

            if not waypoints_xy:
                self.get_logger().error("Parsed zero waypoints from /f2c_waypoints")
                return

            # Store waypoints as pending (don't start navigation yet)
            self.pending_waypoints = waypoints_xy

            self.get_logger().info("")
            self.get_logger().info(
                "╔═══════════════════════════════════════════════════════╗"
            )
            self.get_logger().info(
                f"║  📡 RECEIVED {num_waypoints} WAYPOINTS FROM F2C GUI"
            )
            self.get_logger().info(
                '║  ⏳ Waiting for "Start Navigation" button...         ║'
            )
            self.get_logger().info(
                "║  (Or press X to enable MPC, then click Start)        ║"
            )
            self.get_logger().info(
                "╚═══════════════════════════════════════════════════════╝"
            )
            self.get_logger().info("")

            # Log first few waypoints for verification
            for idx, (x, y) in enumerate(waypoints_xy[:5]):
                self.get_logger().info(f"  Waypoint {idx+1}: x={x:.2f}, y={y:.2f}")
            if num_waypoints > 5:
                self.get_logger().info(
                    f"  ... and {num_waypoints - 5} more waypoints"
                )

        except Exception as e:
            self.get_logger().error(f"Error handling /f2c_waypoints: {e}")

    def _start_pending_navigation(self) -> None:
        """
        Start navigation with previously received pending waypoints.
        Called when "Start Navigation" button is pressed in F2C GUI (sends [0.0] signal).
        """
        if not self.pending_waypoints:
            self.get_logger().warn(
                "⚠️  No pending waypoints to navigate - publish waypoints first!"
            )
            return

        if not self.pose_initialized:
            self.get_logger().error(
                "❌ Odometry not initialized yet - cannot start navigation"
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

        self.get_logger().info("")
        self.get_logger().info(
            "╔═══════════════════════════════════════════════════════╗"
        )
        self.get_logger().info(
            f"║  🚀 NAVIGATION STARTED: {len(self.waypoints)} waypoints"
        )
        if self.autonomy_enabled:
            self.get_logger().info(
                "║  ✅ MPC autonomy is ENABLED - robot will move         ║"
            )
        else:
            self.get_logger().info(
                "║  ⚠️  MPC autonomy DISABLED - press X to enable!       ║"
            )
        self.get_logger().info(
            "╚═══════════════════════════════════════════════════════╝"
        )
        self.get_logger().info("")

    def autonomy_enable_callback(self, msg: Bool) -> None:
        """
        Enable or disable MPC autonomy.
        When disabled, the controller will not publish ODrive commands even if
        targets or waypoints are available (teleop can control the robot).
        """
        self.autonomy_enabled = bool(msg.data)
        state = "ENABLED" if self.autonomy_enabled else "DISABLED"
        self.get_logger().info(f"MPC autonomy {state} via /mpc_autonomy_enable")

    # -----------------------------
    # Control loop
    # -----------------------------
    def control_loop(self) -> None:
        # When autonomy is disabled, remain completely silent (teleop controls motors)
        if not self.autonomy_enabled:
            return

        if not self.pose_initialized:
            self.send_zero_velocity()
            return

        # If using CSV/F2C waypoint navigation and we don't currently have a target,
        # set the next waypoint as the current target (once pose is initialized).
        if self.waypoint_navigation_active and not self.has_target:
            if self.current_waypoint_index < len(self.waypoints):
                self._set_next_waypoint_target()
            else:
                # No more waypoints → stop
                self.send_zero_velocity()
                return

        if not self.has_target:
            self.send_zero_velocity()
            return

        # --- Stopping logic based on distance traveled along the path line ---
        path_dx = self.target_x - self.path_start_x
        path_dy = self.target_y - self.path_start_y
        path_length = math.sqrt(path_dx * path_dx + path_dy * path_dy)

        if path_length > 1e-6:
            path_dir_x = path_dx / path_length
            path_dir_y = path_dy / path_length

            to_current_x = self.current_x - self.path_start_x
            to_current_y = self.current_y - self.path_start_y

            # Signed distance from start along the path line
            s_current = to_current_x * path_dir_x + to_current_y * path_dir_y
            if s_current < 0.0:
                s_current = 0.0
            elif s_current > path_length:
                s_current = path_length

            distance_along_line_remaining = path_length - s_current
        else:
            dx_to_target = self.target_x - self.current_x
            dy_to_target = self.target_y - self.current_y
            distance_along_line_remaining = math.sqrt(
                dx_to_target ** 2 + dy_to_target ** 2
            )

        if distance_along_line_remaining <= self.target_reached_threshold:
            # Target reached along current line segment
            self.target_reached = True

            # If we are following a waypoint sequence and more waypoints remain,
            # advance to the next waypoint (one per control step).
            if (
                self.waypoint_navigation_active
                and self.current_waypoint_index < len(self.waypoints) - 1
            ):
                self.get_logger().info(
                    f"✅ Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} "
                    f"reached at x={self.current_x:.3f}m, y={self.current_y:.3f}m"
                )
                # Update previous waypoint and advance index
                self.previous_waypoint = (self.target_x, self.target_y)
                self.current_waypoint_index += 1
                # Clear current target; next loop will call _set_next_waypoint_target()
                self.has_target = False
                # Optionally send a brief stop at the waypoint
                self.send_zero_velocity()
                return
            else:
                # Final target reached (single target or last waypoint)
                self.has_target = False
                self.waypoint_navigation_active = False
                self.send_zero_velocity()
                self.get_logger().info(
                    f"✅ MPC accel target reached along line: "
                    f"{distance_along_line_remaining:.3f} m "
                    f"(threshold: {self.target_reached_threshold:.3f} m)"
                )
                return

        # Build reference trajectory
        ref_traj = self.generate_reference_trajectory()
        if len(ref_traj) == 0:
            self.send_zero_velocity()
            return

        # Error reference: synthetic point near start of line or first waypoint
        if path_length > 1e-6:
            path_dir_x = path_dx / path_length
            path_dir_y = path_dy / path_length

            to_current_x = self.current_x - self.path_start_x
            to_current_y = self.current_y - self.path_start_y
            t_closest = (to_current_x * path_dir_x + to_current_y * path_dir_y) / path_length
            t_closest = np.clip(t_closest, 0.0, 1.0)
            s_closest = float(t_closest * path_length)

            d_gate = float(self.error_ref_gate_distance)
            if d_gate > 1e-6 and s_closest <= d_gate:
                # Near start of line: use closer synthetic point ahead on the path
                nominal_ahead = 0.04  # [m], same as in MPCAutonomousController
                start_scale = max(0.0, min(1.0, float(self.error_ref_ahead_min_scale)))
                ratio_s = s_closest / d_gate
                ahead_scale = start_scale + (1.0 - start_scale) * ratio_s
                ahead_dist = ahead_scale * nominal_ahead

                s_ref = min(s_closest + ahead_dist, path_length)
                t_ref = s_ref / path_length

                ref_x = self.path_start_x + t_ref * path_dx
                ref_y = self.path_start_y + t_ref * path_dy
                
                # Use appropriate heading based on reverse mode
                forward_path_heading = math.atan2(path_dy, path_dx)
                if self.reverse_mode:
                    ref_heading = self.normalize_angle(forward_path_heading + math.pi)
                else:
                    ref_heading = forward_path_heading

                cruising_speed = min(0.5, self.max_linear_vel)
                # Signed velocity reference (negative when reversing)
                signed_v_ref = -cruising_speed if self.reverse_mode else cruising_speed
                
                ref_waypoint = np.array(
                    [
                        ref_x,
                        ref_y,
                        ref_heading,
                        abs(cruising_speed) * math.cos(ref_heading),  # vx in world frame
                        abs(cruising_speed) * math.sin(ref_heading),  # vy in world frame
                        signed_v_ref,  # Signed body-frame v_ref for MPC dynamics
                    ]
                )
                err = self.compute_error_state(ref_waypoint)
            else:
                # Use first waypoint in horizon
                ref_wp0 = ref_traj[0]
                err = self.compute_error_state(ref_wp0)
        else:
            # Degenerate path: fall back to first waypoint
            ref_wp0 = ref_traj[0]
            err = self.compute_error_state(ref_wp0)

        # Use COMMANDED velocity as the MPC state instead of measured velocity.
        # This assumes the low-level ODrive velocity controller tracks well (which it does).
        # Benefits:
        #   - No noisy velocity measurements needed
        #   - Commanded velocity is inherently smooth (output of MPC itself)
        #   - Position errors are still corrected via error states (xe, ye, θe)
        # This is "open-loop on velocity, closed-loop on position" - common in cascaded control.
        v_body = self.v_cmd
        omega_body = self.omega_cmd

        # Solve MPC for Δu
        du0, solve_ms, solution = self.mpc.solve(err, v_body, omega_body, ref_traj)

        # Integrate Δu into command velocities (MPC's internal state)
        dv = float(du0[0])
        domega = float(du0[1])
        self.v_cmd = np.clip(self.v_cmd + dv, -self.max_linear_vel, self.max_linear_vel)
        self.omega_cmd = np.clip(self.omega_cmd + domega, -self.max_angular_vel, self.max_angular_vel)

        # Convert MPC's (v_cmd, ω_cmd) to TARGET wheel velocities (rad/s)
        # These are what the MPC wants the wheels to achieve
        # v = (r/2)(ωL + ωR), ω = (r/L)(ωR - ωL)
        # Solving for ωL, ωR:
        #   ωL = v/r - ω*L/(2r)
        #   ωR = v/r + ω*L/(2r)
        if abs(self.wheel_radius) < 1e-6 or abs(self.wheel_base) < 1e-6:
            omega_L_target = 0.0
            omega_R_target = 0.0
        else:
            omega_L_target = (self.v_cmd / self.wheel_radius) - (self.omega_cmd * self.wheel_base / (2.0 * self.wheel_radius))
            omega_R_target = (self.v_cmd / self.wheel_radius) + (self.omega_cmd * self.wheel_base / (2.0 * self.wheel_radius))

        # Convert target wheel angular velocities (rad/s) to motor rev/s
        # This is the unit that ODrive expects and that the compensator works in
        left_rps_target = omega_L_target / (2.0 * math.pi * self.gear_ratio)
        right_rps_target = omega_R_target / (2.0 * math.pi * self.gear_ratio)

        # Apply wheel-level ramp compensation if enabled
        if self.ramp_compensation_enabled:
            # Compensate for ramp dynamics at each wheel independently
            # This computes effective velocities to command such that actual
            # wheel displacement matches what MPC expects over the cycle time
            left_rps_eff, right_rps_eff = self.wheel_compensator.compensate(
                left_rps_target, right_rps_target
            )
        else:
            # Direct command (legacy behavior)
            left_rps_eff = left_rps_target
            right_rps_eff = right_rps_target
        
        self.get_logger().info(
            f"[{'REV' if self.reverse_mode else 'FWD'}] "
            f"err=[{err[0]:.3f}, {err[1]:.3f}, {math.degrees(err[2]):.1f}°] "
            f"v_cmd={self.v_cmd:.3f} ω_cmd={self.omega_cmd:.3f} "
            f"Δv={dv:.4f} Δω={domega:.4f} "
            f"wheels=[{left_rps_eff:.3f}, {right_rps_eff:.3f}]"
        )

        self.publish_wheel_velocities(left_rps_eff, right_rps_eff)

        # For diagnostics: compute effective v and ω from compensated wheel velocities
        omega_L_eff = left_rps_eff * 2.0 * math.pi * self.gear_ratio
        omega_R_eff = right_rps_eff * 2.0 * math.pi * self.gear_ratio
        v_eff = self.wheel_radius * (omega_L_eff + omega_R_eff) / 2.0
        w_eff = self.wheel_radius * (omega_R_eff - omega_L_eff) / self.wheel_base if abs(self.wheel_base) > 1e-6 else 0.0

        # Publish diagnostics
        try:
            # Command Twist (shows effective/compensated velocities sent to wheels)
            twist = Twist()
            twist.linear.x = float(v_eff)
            twist.angular.z = float(w_eff)
            self.cmd_pub.publish(twist)

            # Delta command [Δv, Δω]
            delta_msg = Float64MultiArray()
            delta_msg.data = [dv, domega]
            self.delta_cmd_pub.publish(delta_msg)

            # Error state [xe, ye, θe]
            err_msg = Float64MultiArray()
            err_msg.data = [float(err[0]), float(err[1]), float(err[2])]
            self.error_state_pub.publish(err_msg)

            # Reference trajectory (first waypoint for quick visualization)
            if len(ref_traj) > 0:
                wp0 = ref_traj[0]
                ref_msg = Float64MultiArray()
                ref_msg.data = [
                    float(wp0[0]),
                    float(wp0[1]),
                    float(wp0[2]),
                    float(wp0[3]),
                    float(wp0[4]),
                    float(wp0[5]),
                ]
                self.ref_traj_pub.publish(ref_msg)
        except Exception as e:
            self.get_logger().warn(f"Error publishing MPC accel diagnostics: {e}")

        # Optional log for debugging
        if solve_ms is not None:
            self.get_logger().debug(f"AccelMPC solve time: {solve_ms:.3f} ms")

    # -----------------------------
    # Publishing helpers
    # -----------------------------
    def publish_wheel_velocities(self, left_rps: float, right_rps: float) -> None:
        left_cmd = -left_rps if self.invert_left else left_rps
        right_cmd = -right_rps if self.invert_right else right_rps

        # Use VEL_RAMP (2) when compensation is enabled, PASSTHROUGH (1) otherwise
        # The ramp compensation math assumes VEL_RAMP mode on ODrive
        input_mode = 2 if self.ramp_compensation_enabled else 1

        left_msg = ControlMessage()
        left_msg.control_mode = 2  # VELOCITY_CONTROL
        left_msg.input_mode = input_mode
        left_msg.input_vel = float(left_cmd)
        left_msg.input_torque = 0.0
        left_msg.input_pos = 0.0

        right_msg = ControlMessage()
        right_msg.control_mode = 2
        right_msg.input_mode = input_mode
        right_msg.input_vel = float(right_cmd)
        right_msg.input_torque = 0.0
        right_msg.input_pos = 0.0

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    def send_zero_velocity(self) -> None:
        """Send zero velocity and reset compensator state."""
        self.publish_wheel_velocities(0.0, 0.0)
        # Reset compensator to zero state (wheels are stopped)
        self.wheel_compensator.reset()
        # Also reset MPC's internal velocity state
        self.v_cmd = 0.0
        self.omega_cmd = 0.0

    # -----------------------------
    # Motor arming / soft shutdown
    # -----------------------------
    def _soft_shutdown_service(self, request, response):
        """
        Service callback for graceful shutdown of the accel controller.
        Performs: zero velocity → disarm ODrives → optional shutdown signal.
        """
        del request
        self.get_logger().warning(
            "🛑 Accel MPC soft shutdown requested → zero velocity, disarm ODrives"
        )
        try:
            self._send_zero_velocity()
            self._disarm_odrives()
            self._call_shutdown_service()
            # Optionally signal parent launch to stop
            try:
                os.kill(os.getppid(), signal.SIGINT)
            except Exception:
                pass
            response.success = True
            response.message = "Accel MPC shutdown initiated"
        except Exception as e:
            response.success = False
            response.message = f"Error during accel MPC shutdown: {e}"
        return response

    def _send_zero_velocity(self) -> None:
        """
        Send zero-velocity commands multiple times to ensure delivery.
        """
        for _ in range(3):
            self.send_zero_velocity()
            time.sleep(0.02)

    def _disarm_odrives(self) -> None:
        """
        Request IDLE state for both ODrive axes to disarm motors.
        """
        try:
            req_idle = AxisState.Request()
            req_idle.axis_requested_state = 1  # IDLE

            if self.left_axis_client.service_is_ready():
                self.left_axis_client.call_async(req_idle)
            if self.right_axis_client.service_is_ready():
                self.right_axis_client.call_async(req_idle)

            time.sleep(0.2)
        except Exception as e:
            self.get_logger().warn(f"Error disarming ODrives (accel controller): {e}")

    def _call_shutdown_service(self) -> None:
        """
        Call the shutdown service (if available) to stop mapping nodes.
        """
        try:
            if self.shutdown_client.service_is_ready():
                req = Trigger.Request()
                self.shutdown_client.call_async(req)
        except Exception as e:
            self.get_logger().warn(
                f"Error calling external shutdown service (accel controller): {e}"
            )

    def _attempt_arm_motors(self) -> None:
        """
        Attempt to arm ODrive motors by requesting CLOSED_LOOP_CONTROL state.
        Retries up to _arm_max_attempts times.
        """
        if self._arm_attempts >= self._arm_max_attempts:
            self._arm_timer.cancel()
            return
        self._arm_attempts += 1

        # Ensure service availability
        if not (
            self.left_axis_client.service_is_ready()
            and self.right_axis_client.service_is_ready()
            and self.left_clear_client.service_is_ready()
            and self.right_clear_client.service_is_ready()
        ):
            self.get_logger().warn(
                "Waiting for ODrive CAN services to be ready (accel controller)..."
            )
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
            self.get_logger().info(
                "Arming ODrive axes (CLOSED_LOOP_CONTROL requested by accel controller)"
            )
            # Stop timer after successful dispatch
            self._arm_timer.cancel()
        except Exception as e:
            self.get_logger().warn(f"Arm attempt failed (accel controller): {e}")

    # -----------------------------
    # Utility functions
    # -----------------------------
    @staticmethod
    def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MPCAccelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.send_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

