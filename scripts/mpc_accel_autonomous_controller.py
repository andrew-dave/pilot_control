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
from std_msgs.msg import Float64MultiArray
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
        logger=None,
        weight_increase_xe: float = 0.0,
        weight_increase_ye: float = 0.0,
        weight_increase_yaw: float = 0.0,
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
        n_dynamics = N * nx
        n_input_bounds = N * 2
        n_constraints = n_dynamics + n_input_bounds

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

        # Velocity bounds
        for k in range(N):
            v_idx = n_dynamics + 2 * k
            omega_idx = n_dynamics + 2 * k + 1

            self.l_constr[v_idx] = self.v_min
            self.u_constr[v_idx] = self.v_max

            self.l_constr[omega_idx] = self.omega_min
            self.u_constr[omega_idx] = self.omega_max

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
            verbose=False,
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

        # Augmented B (5x2): Δv, Δω directly affect v, ω
        B_aug = np.zeros((self.nx, self.nu))
        B_aug[3, 0] = 1.0  # v_{k+1} = v_k + Δv_k
        B_aug[4, 1] = 1.0  # ω_{k+1} = ω_k + Δω_k

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

            v_ref = math.sqrt(float(wp[3]) ** 2 + float(wp[4]) ** 2)
            omega_ref = float(wp[5])

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
        x0 = np.zeros(self.nx)
        x0[0:3] = current_error.reshape(3,)
        x0[3] = float(current_v)
        x0[4] = float(current_omega)

        # Build A_aug_0 for initial constraint RHS
        if len(ref_traj) > 0:
            wp0 = ref_traj[0]
            v_ref0 = math.sqrt(float(wp0[3]) ** 2 + float(wp0[4]) ** 2)
            omega_ref0 = float(wp0[5])
        else:
            v_ref0 = 0.0
            omega_ref0 = 0.0

        A_aug0, _ = self._build_A_aug_and_B_aug(v_ref0, omega_ref0)
        rhs0 = A_aug0 @ x0

        # First nx dynamics rows: equality constraint to rhs0
        for i in range(self.nx):
            self.l_constr[i] = rhs0[i]
            self.u_constr[i] = rhs0[i]

        # Refresh v bounds in case they were scaled
        N = self.N
        n_dynamics = N * self.nx
        for k in range(N):
            v_idx = n_dynamics + 2 * k
            omega_idx = n_dynamics + 2 * k + 1
            self.l_constr[v_idx] = self.v_min
            self.u_constr[v_idx] = self.v_max
            self.l_constr[omega_idx] = self.omega_min
            self.u_constr[omega_idx] = self.omega_max

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

        self.declare_parameter("mpc_Q_xe", 15.0)
        self.declare_parameter("mpc_Q_ye", 20.0)
        self.declare_parameter("mpc_Q_yaw", 5.0)
        # Separate Δ-costs for linear and angular velocity
        self.declare_parameter("mpc_R_delta_v", 0.01)
        self.declare_parameter("mpc_R_delta_omega", 0.0015)
        # Optional time-varying weight scaling (same semantics as slip-aware MPC)
        self.declare_parameter("mpc_weight_increase_xe", 0.0)
        self.declare_parameter("mpc_weight_increase_ye", 0.0)
        self.declare_parameter("mpc_weight_increase_yaw", 0.0)

        self.declare_parameter("odometry_topic", "/Odometry_tilt_corrected_diff")
        self.declare_parameter("left_control_topic", "/left/control_message")
        self.declare_parameter("right_control_topic", "/right/control_message")
        # Stopping criterion (same semantics as MPCAutonomousController)
        self.declare_parameter("target_reached_threshold", 0.01)

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
        mpc_weight_increase_xe = float(self.get_parameter("mpc_weight_increase_xe").value)
        mpc_weight_increase_ye = float(self.get_parameter("mpc_weight_increase_ye").value)
        mpc_weight_increase_yaw = float(self.get_parameter("mpc_weight_increase_yaw").value)

        odom_topic = str(self.get_parameter("odometry_topic").value)
        left_ctrl_topic = str(self.get_parameter("left_control_topic").value)
        right_ctrl_topic = str(self.get_parameter("right_control_topic").value)
        self.target_reached_threshold = float(
            self.get_parameter("target_reached_threshold").value
        )

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.pose_initialized = False

        # Estimated velocities (from odom)
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vyaw = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.0
        self.prev_odom_time: Optional[float] = None

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
            logger=self.get_logger(),
            weight_increase_xe=mpc_weight_increase_xe,
            weight_increase_ye=mpc_weight_increase_ye,
            weight_increase_yaw=mpc_weight_increase_yaw,
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
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)
        self.current_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)

        t_now = self.get_clock().now().nanoseconds / 1e9
        if self.prev_odom_time is not None:
            dt = t_now - self.prev_odom_time
            if dt > 1e-6:
                dx = self.current_x - self.prev_x
                dy = self.current_y - self.prev_y
                self.current_vx = dx / dt
                self.current_vy = dy / dt
                dyaw = self.normalize_angle(self.current_yaw - self.prev_yaw)
                self.current_vyaw = dyaw / dt
        else:
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_vyaw = 0.0

        self.prev_x = self.current_x
        self.prev_y = self.current_y
        self.prev_yaw = self.current_yaw
        self.prev_odom_time = t_now

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

        self.target_x = float(data[0])
        self.target_y = float(data[1])
        self.target_yaw = float(data[3])
        self.has_target = True

        # Initialize straight-line path from current pose to target
        self.path_start_x = self.current_x
        self.path_start_y = self.current_y
        self.path_start_yaw = self.current_yaw
        self.path_initialized = True

        self.get_logger().info(
            f"🎯 New MPC acceleration target: x={self.target_x:.2f}, "
            f"y={self.target_y:.2f}, yaw={math.degrees(self.target_yaw):.1f}°"
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

        if distance_to_target_along_path > proximity_threshold:
            # FAR region: fixed waypoint spacing along the path
            adjusted_spacing = waypoint_spacing
            for k in range(self.mpc_horizon):
                distance_along = adjusted_spacing * (k + 1)
                t_waypoint = t_closest + distance_along / path_length
                t_waypoint = float(np.clip(t_waypoint, 0.0, 1.0))

                wx = self.path_start_x + t_waypoint * path_dx
                wy = self.path_start_y + t_waypoint * path_dy
                heading_to_target = math.atan2(path_dy, path_dx)
                wyaw = self.normalize_angle(heading_to_target)
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
                heading_to_target = math.atan2(path_dy, path_dx)
                wyaw = self.normalize_angle(heading_to_target)
                waypoint_positions.append((wx, wy, wyaw))

        # v_ref shaping near the start of the line (same gating idea as original MPC)
        path_heading = math.atan2(path_dy, path_dx)
        d_gate_v = float(self.error_ref_gate_distance)
        s_closest = float(t_closest * path_length)

        if d_gate_v > 1e-6 and s_closest <= d_gate_v:
            start_scale = max(0.0, min(1.0, float(self.error_ref_ahead_min_scale)))
            ratio_s = s_closest / d_gate_v
            v_scale_ref = start_scale + (1.0 - start_scale) * ratio_s
        else:
            v_scale_ref = 1.0

        v_ref = cruising_speed * v_scale_ref
        for k in range(self.mpc_horizon):
            wx, wy, wyaw = waypoint_positions[k]
            vx_ref = v_ref * math.cos(path_heading)
            vy_ref = v_ref * math.sin(path_heading)
            vyaw_ref = 0.0
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
    # Control loop
    # -----------------------------
    def control_loop(self) -> None:
        if not self.pose_initialized or not self.has_target:
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
            distance_along_line_remaining = math.sqrt(dx_to_target ** 2 + dy_to_target ** 2)

        if distance_along_line_remaining <= self.target_reached_threshold:
            # Final target reached
            self.has_target = False
            self.target_reached = True
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
                path_heading = math.atan2(path_dy, path_dx)

                cruising_speed = min(0.5, self.max_linear_vel)
                ref_waypoint = np.array(
                    [
                        ref_x,
                        ref_y,
                        path_heading,
                        cruising_speed * math.cos(path_heading),
                        cruising_speed * math.sin(path_heading),
                        0.0,
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

        # Approximate current body-frame v, ω from odom
        v_body = self.current_vx * math.cos(self.current_yaw) + self.current_vy * math.sin(self.current_yaw)
        omega_body = self.current_vyaw

        # Solve MPC for Δu
        du0, solve_ms, solution = self.mpc.solve(err, v_body, omega_body, ref_traj)

        # Integrate Δu into command velocities
        dv = float(du0[0])
        domega = float(du0[1])
        self.v_cmd = np.clip(self.v_cmd + dv, -self.max_linear_vel, self.max_linear_vel)
        self.omega_cmd = np.clip(self.omega_cmd + domega, -self.max_angular_vel, self.max_angular_vel)

        # Convert (v_cmd, ω_cmd) to wheel angular velocities (rad/s)
        # v = (r/2)(ωL + ωR), ω = (r/L)(ωR - ωL)
        v = self.v_cmd
        w = self.omega_cmd

        if abs(self.wheel_radius) < 1e-6 or abs(self.wheel_base) < 1e-6:
            omega_L = 0.0
            omega_R = 0.0
        else:
            omega_L = (v / self.wheel_radius) - (w * self.wheel_base / (2.0 * self.wheel_radius))
            omega_R = (v / self.wheel_radius) + (w * self.wheel_base / (2.0 * self.wheel_radius))

        # Convert to motor rev/s
        left_rps = omega_L / (2.0 * math.pi * self.gear_ratio)
        right_rps = omega_R / (2.0 * math.pi * self.gear_ratio)

        self.publish_wheel_velocities(left_rps, right_rps)

        # Publish diagnostics
        try:
            # Command Twist
            twist = Twist()
            twist.linear.x = float(v)
            twist.angular.z = float(w)
            self.cmd_pub.publish(twist)

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

        # If close enough to target, stop
        dist_to_target = math.sqrt(
            (self.target_x - self.current_x) ** 2
            + (self.target_y - self.current_y) ** 2
        )
        if dist_to_target < 0.02:
            self.get_logger().info(
                f"✅ MPC accel target reached: dist={dist_to_target:.3f} m"
            )
            self.has_target = False
            self.v_cmd = 0.0
            self.omega_cmd = 0.0
            self.send_zero_velocity()

        # Optional log for debugging
        if solve_ms is not None:
            self.get_logger().debug(f"AccelMPC solve time: {solve_ms:.3f} ms")

    # -----------------------------
    # Publishing helpers
    # -----------------------------
    def publish_wheel_velocities(self, left_rps: float, right_rps: float) -> None:
        left_cmd = -left_rps if self.invert_left else left_rps
        right_cmd = -right_rps if self.invert_right else right_rps

        left_msg = ControlMessage()
        left_msg.control_mode = 2  # VELOCITY_CONTROL
        left_msg.input_mode = 1   # PASSTHROUGH
        left_msg.input_vel = float(left_cmd)
        left_msg.input_torque = 0.0
        left_msg.input_pos = 0.0

        right_msg = ControlMessage()
        right_msg.control_mode = 2
        right_msg.input_mode = 1
        right_msg.input_vel = float(right_cmd)
        right_msg.input_torque = 0.0
        right_msg.input_pos = 0.0

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    def send_zero_velocity(self) -> None:
        self.publish_wheel_velocities(0.0, 0.0)

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


