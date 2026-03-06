#!/usr/bin/env python3
"""
Direct-input MPC autonomous controller.

This file intentionally mirrors the behavior surface of
`mpc_accel_autonomous_controller.py` while changing only the MPC internals:

- Accel controller MPC: control input is [dv, domega]
- This controller MPC:  control input is [v, omega]

All high-level node features (heartbeat, waypoint handling, auto-DC,
ramp compensation, arming/shutdown helpers, topics/services) are inherited from
the accel controller implementation for drop-in launch compatibility.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy

try:
    from mpc_accel_autonomous_controller import (
        MPCAccelController,
        OSQP_AVAILABLE,
        SCIPY_AVAILABLE,
        osqp,
        sparse,
    )
except ImportError:  # pragma: no cover
    from .mpc_accel_autonomous_controller import (  # type: ignore
        MPCAccelController,
        OSQP_AVAILABLE,
        SCIPY_AVAILABLE,
        osqp,
        sparse,
    )


class VWMPC:
    """
    LTV MPC with direct velocity inputs.

    State:
        x = [xe, ye, theta_e]^T
    Control:
        u = [v, omega]^T

    Dynamics (discrete, linearized around reference):
        x_{k+1} = A_k x_k + B_k u_k
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
        # Keep constructor parameters aligned with AccelMPC for compatibility.
        del dv_max
        del domega_max

        self.N = int(N)
        self.Ts = float(Ts)
        self.logger = logger

        self.nx = 3
        self.nu = 2
        self.nz = self.N * (self.nu + self.nx)

        self.v_min = float(v_min)
        self.v_max = float(v_max)
        self.omega_min = float(omega_min)
        self.omega_max = float(omega_max)
        self._v_min_base = self.v_min
        self._v_max_base = self.v_max

        self.Q_xe_base = float(Q_xe)
        self.Q_ye_base = float(Q_ye)
        self.Q_yaw_base = float(Q_yaw)
        self.R_delta_v = float(R_delta_v)
        self.R_delta_omega = float(R_delta_omega)

        self.weight_increase_xe = float(weight_increase_xe)
        self.weight_increase_ye = float(weight_increase_ye)
        self.weight_increase_yaw = float(weight_increase_yaw)

        self.P = None
        self.A_constr = None
        self.l_constr = None
        self.u_constr = None
        self.solver: Optional[osqp.OSQP] = None

        self.A_indices = {}
        self.B_indices = {}
        self.row_col_to_data_idx = {}
        self.prev_solution: Optional[np.ndarray] = None
        self._last_result = None
        self.solver_debug_enabled = bool(solver_debug_enabled)
        self.initialized = False

    def set_velocity_bound_scale(self, scale: float) -> None:
        try:
            s = float(scale)
        except (TypeError, ValueError):
            s = 1.0
        s = max(0.0, min(1.0, s))
        self.v_min = self._v_min_base * s
        self.v_max = self._v_max_base * s

    def setup(self) -> bool:
        if not OSQP_AVAILABLE or not SCIPY_AVAILABLE:
            if self.logger:
                self.logger.error("OSQP and scipy required for VWMPC")
            return False

        N = self.N
        nx = self.nx
        nu = self.nu
        nz = self.nz

        # 1) Cost matrix P
        P_data: List[float] = []
        P_row: List[int] = []
        P_col: List[int] = []

        for k in range(N):
            x_k_start = k * (nu + nx) + nu

            w_xe = self.Q_xe_base * (1.0 + self.weight_increase_xe * k)
            w_ye = self.Q_ye_base * (1.0 + self.weight_increase_ye * k)
            w_yaw = self.Q_yaw_base * (1.0 + self.weight_increase_yaw * k)

            P_row.extend([x_k_start + 0, x_k_start + 1, x_k_start + 2])
            P_col.extend([x_k_start + 0, x_k_start + 1, x_k_start + 2])
            P_data.extend([w_xe, w_ye, w_yaw])

        for k in range(N):
            u_k_start = k * (nu + nx)
            P_row.extend([u_k_start + 0, u_k_start + 1])
            P_col.extend([u_k_start + 0, u_k_start + 1])
            P_data.extend([self.R_delta_v, self.R_delta_omega])

        P_coo = sparse.coo_matrix((P_data, (P_row, P_col)), shape=(nz, nz))
        P_sym = (P_coo + P_coo.T) / 2.0
        self.P = (P_sym + sparse.eye(nz, format="csc") * 1e-6).tocsc()

        # 2) Constraint matrix A
        n_dynamics = N * nx
        n_input_bounds = N * 2
        n_constraints = n_dynamics + n_input_bounds

        row_indices: List[int] = []
        col_indices: List[int] = []
        data_values: List[float] = []

        self.A_indices = {}
        self.B_indices = {}

        constraint_row = 0
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

                # -A_k x_k
                if k > 0 and x_k_idx is not None:
                    for j in range(nx):
                        idx = len(data_values)
                        row_indices.append(constraint_row + i)
                        col_indices.append(x_k_idx + j)
                        data_values.append(0.0)
                        self.A_indices[k].append(idx)

                # -B_k u_k
                for j in range(nu):
                    idx = len(data_values)
                    row_indices.append(constraint_row + i)
                    col_indices.append(u_k_idx + j)
                    data_values.append(0.0)
                    self.B_indices[k].append(idx)

            constraint_row += nx

        # Input bounds (v, omega) on u_k
        for k in range(N):
            u_k_idx = k * (nu + nx)

            row_indices.append(constraint_row)
            col_indices.append(u_k_idx + 0)
            data_values.append(1.0)
            constraint_row += 1

            row_indices.append(constraint_row)
            col_indices.append(u_k_idx + 1)
            data_values.append(1.0)
            constraint_row += 1

        A_coo = sparse.coo_matrix(
            (data_values, (row_indices, col_indices)),
            shape=(n_constraints, nz),
        )
        self.A_constr = A_coo.tocsc()

        self.row_col_to_data_idx = {}
        csc_indices = self.A_constr.indices
        csc_indptr = self.A_constr.indptr
        for col in range(nz):
            for csc_idx in range(csc_indptr[col], csc_indptr[col + 1]):
                row = csc_indices[csc_idx]
                self.row_col_to_data_idx[(row, col)] = csc_idx

        for k in range(N):
            if k > 0:
                new_A_indices = []
                x_k_idx = (k - 1) * (nu + nx) + nu
                for i in range(nx):
                    for j in range(nx):
                        row = k * nx + i
                        col = x_k_idx + j
                        idx = self.row_col_to_data_idx.get((row, col))
                        if idx is not None:
                            new_A_indices.append(idx)
                self.A_indices[k] = new_A_indices
            else:
                self.A_indices[k] = []

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

        # 3) Bounds
        self.l_constr = np.zeros(n_constraints)
        self.u_constr = np.zeros(n_constraints)
        for k in range(N):
            v_idx = n_dynamics + 2 * k
            w_idx = n_dynamics + 2 * k + 1
            self.l_constr[v_idx] = self.v_min
            self.u_constr[v_idx] = self.v_max
            self.l_constr[w_idx] = self.omega_min
            self.u_constr[w_idx] = self.omega_max

        # 4) Solver setup
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
            self.logger.info("✓ VWMPC optimizer initialized")
        return True

    def update_v_bounds(self, v_scale: float) -> None:
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

    def _build_A_and_B(self, v_ref: float, omega_ref: float) -> Tuple[np.ndarray, np.ndarray]:
        Ts = self.Ts
        A_k = np.array(
            [
                [1.0, -omega_ref * Ts, 0.0],
                [omega_ref * Ts, 1.0, v_ref * Ts],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        B_k = Ts * np.array(
            [
                [-1.0, 0.0],
                [0.0, 0.0],
                [0.0, -1.0],
            ],
            dtype=float,
        )
        return A_k, B_k

    def update_matrices(self, ref_traj: List[np.ndarray]) -> None:
        if not self.initialized:
            return

        N = self.N
        nx = self.nx
        nu = self.nu
        A_data = self.A_constr.data.copy()

        for k in range(N):
            if k < len(ref_traj):
                wp = ref_traj[k]
            elif len(ref_traj) > 0:
                wp = ref_traj[-1]
            else:
                wp = np.zeros(6, dtype=float)

            v_ref = math.sqrt(float(wp[3]) ** 2 + float(wp[4]) ** 2)
            omega_ref = float(wp[5])
            A_k, B_k = self._build_A_and_B(v_ref, omega_ref)

            if k > 0 and k in self.A_indices:
                A_neg = -A_k
                for idx, matrix_idx in enumerate(self.A_indices[k]):
                    i = idx // nx
                    j = idx % nx
                    if i < nx and j < nx and matrix_idx < len(A_data):
                        A_data[matrix_idx] = A_neg[i, j]

            if k in self.B_indices:
                B_neg = -B_k
                for idx, matrix_idx in enumerate(self.B_indices[k]):
                    i = idx // nu
                    j = idx % nu
                    if i < nx and j < nu and matrix_idx < len(A_data):
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
        Solve direct-input MPC.

        Returns a *delta-shaped* command [dv, domega] so the inherited accel
        controller logic can remain unchanged while the optimization itself
        uses direct [v, omega] inputs.
        """
        if not self.initialized and not self.setup():
            return np.zeros(2), 0.0, None

        solve_start = time.time()
        self.update_matrices(ref_traj)

        # First dynamics RHS: x1 = A0 * x0 + B0 * u0
        if len(ref_traj) > 0:
            wp0 = ref_traj[0]
            v_ref0 = math.sqrt(float(wp0[3]) ** 2 + float(wp0[4]) ** 2)
            omega_ref0 = float(wp0[5])
        else:
            v_ref0 = 0.0
            omega_ref0 = 0.0

        A0, _ = self._build_A_and_B(v_ref0, omega_ref0)
        rhs0 = A0 @ current_error.reshape(3,)

        for i in range(self.nx):
            self.l_constr[i] = rhs0[i]
            self.u_constr[i] = rhs0[i]

        # k>0 dynamics equalities are homogeneous
        N = self.N
        nx = self.nx
        for k in range(1, N):
            for i in range(nx):
                row_idx = k * nx + i
                self.l_constr[row_idx] = 0.0
                self.u_constr[row_idx] = 0.0

        # Refresh direct input bounds
        n_dynamics = N * nx
        for k in range(N):
            v_idx = n_dynamics + 2 * k
            w_idx = n_dynamics + 2 * k + 1
            self.l_constr[v_idx] = self.v_min
            self.u_constr[v_idx] = self.v_max
            self.l_constr[w_idx] = self.omega_min
            self.u_constr[w_idx] = self.omega_max

        if self.prev_solution is not None and len(self.prev_solution) == self.nz:
            prev = self.prev_solution
            warm = np.zeros_like(prev)
            block = self.nu + self.nx
            warm[:-block] = prev[block:]
            warm[-block:] = prev[-block:]
            self.solver.warm_start(x=warm)

        self.solver.update(l=self.l_constr, u=self.u_constr)
        result = self.solver.solve()
        if result.info.status != "solved":
            if self.logger:
                self.logger.warn(f"VWMPC solve failed: {result.info.status}")
            return np.zeros(2), 0.0, None

        self._last_result = result
        solution = result.x
        u0_abs = solution[0 : self.nu]
        self.prev_solution = solution

        # Return delta-shaped command so parent accel-controller logic remains intact.
        du0 = np.array(
            [
                float(u0_abs[0]) - float(current_v),
                float(u0_abs[1]) - float(current_omega),
            ],
            dtype=float,
        )
        solve_time_ms = (time.time() - solve_start) * 1000.0
        return du0, solve_time_ms, solution

    def compute_solution_cost(
        self,
        solution: Optional[np.ndarray],
        ref_traj: List[np.ndarray],
    ) -> float:
        del ref_traj
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
            cost += self.R_delta_v * float(uk[0] ** 2)
            cost += self.R_delta_omega * float(uk[1] ** 2)

        return float(cost)


class MPCAutonomousController(MPCAccelController):
    """
    Feature-compatible controller with direct-input (v, omega) MPC internals.
    """

    def __init__(self) -> None:
        super().__init__()

        old_mpc = self.mpc
        self.mpc = VWMPC(
            N=int(old_mpc.N),
            Ts=float(old_mpc.Ts),
            v_min=float(old_mpc.v_min),
            v_max=float(old_mpc.v_max),
            omega_min=float(old_mpc.omega_min),
            omega_max=float(old_mpc.omega_max),
            Q_xe=float(old_mpc.Q_xe_base),
            Q_ye=float(old_mpc.Q_ye_base),
            Q_yaw=float(old_mpc.Q_yaw_base),
            R_delta_v=float(old_mpc.R_delta_v),
            R_delta_omega=float(old_mpc.R_delta_omega),
            dv_max=0.0,
            domega_max=0.0,
            logger=self.get_logger(),
            weight_increase_xe=float(old_mpc.weight_increase_xe),
            weight_increase_ye=float(old_mpc.weight_increase_ye),
            weight_increase_yaw=float(old_mpc.weight_increase_yaw),
            solver_debug_enabled=bool(self.solver_debug_enabled),
        )
        self.get_logger().info(
            "✓ MPCAutonomousController using direct-input VWMPC (u=[v, omega])"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MPCAutonomousController()
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
