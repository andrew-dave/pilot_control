#!/usr/bin/env python3

import argparse
import math
from typing import Tuple

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import numpy as np


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_errors(
    waypoint_x: float,
    waypoint_y: float,
    waypoint_yaw: float,
    current_x: float,
    current_y: float,
    current_yaw: float,
    r_goal: float,
    r_close: float,
    r_far: float,
    K_lat: float,
    K_yaw: float,
    yaw_tol: float,
) -> Tuple[float, float, float, float]:
    """Standalone replica of PoseController.compute_errors with global-goal yaw blending.

    Returns: v_e, w_e, e_lat, d_yaw
    """
    # Pose deltas
    dx = waypoint_x - current_x
    dy = waypoint_y - current_y
    d_yaw = waypoint_yaw - current_yaw
    yaw = current_yaw

    # Wrap yaw error
    while abs(d_yaw) > math.pi:
        d_yaw = d_yaw - math.copysign(2 * math.pi, d_yaw)

    # Errors in robot frame
    e_lat = -math.sin(yaw) * dx + math.cos(yaw) * dy
    v_e = math.cos(yaw) * dx + math.sin(yaw) * dy
    r_local = math.sqrt(dx * dx + dy * dy)

    # Steering components
    w_lat = K_lat * math.atan(e_lat)
    w_yaw = K_yaw * d_yaw

    # Distance-based blending uses GLOBAL goal distance
    denom = max((r_far - r_close), 1e-6)
    blend = (r_far - r_goal) / denom  # 0 → lateral; 1 → yaw
    blend = max(min(blend, 1.0), 0.0)

    w_e = (1.0 - blend) * w_lat + blend * w_yaw

    # Final yaw fine-tuning
    if r_local < r_close and abs(d_yaw) < yaw_tol:
        w_e = 0.0

    return v_e, w_e, e_lat, d_yaw


def plot_heatmaps(args):
    # Grid of dx, dy around the robot in meters
    dx = np.linspace(args.dx_min, args.dx_max, args.grid)
    dy = np.linspace(args.dy_min, args.dy_max, args.grid)
    DX, DY = np.meshgrid(dx, dy)

    # Fixed yaw and dyaw for heatmaps
    current_yaw = args.current_yaw
    d_yaw_const = args.dyaw_const
    waypoint_yaw = normalize_angle(current_yaw + d_yaw_const)

    V = np.zeros_like(DX, dtype=float)
    W = np.zeros_like(DX, dtype=float)
    EL = np.zeros_like(DX, dtype=float)
    for i in range(DX.shape[0]):
        for j in range(DX.shape[1]):
            v_e, w_e, e_lat, _ = compute_errors(
                waypoint_x=DX[i, j],
                waypoint_y=DY[i, j],
                waypoint_yaw=waypoint_yaw,
                current_x=0.0,
                current_y=0.0,
                current_yaw=current_yaw,
                r_goal=args.r_goal_heatmap,
                r_close=args.r_close,
                r_far=args.r_far,
                K_lat=args.K_lat,
                K_yaw=args.K_yaw,
                yaw_tol=args.yaw_tol,
            )
            V[i, j] = v_e
            W[i, j] = w_e
            EL[i, j] = e_lat

    fig, axs = plt.subplots(1, 3, figsize=(18, 5), constrained_layout=True)

    im0 = axs[0].imshow(
        V, extent=[args.dx_min, args.dx_max, args.dy_min, args.dy_max], origin="lower", cmap="coolwarm"
    )
    axs[0].set_title("v_e over dx,dy (m)")
    axs[0].set_xlabel("dx (m)")
    axs[0].set_ylabel("dy (m)")
    fig.colorbar(im0, ax=axs[0])

    im1 = axs[1].imshow(
        W, extent=[args.dx_min, args.dx_max, args.dy_min, args.dy_max], origin="lower", cmap="coolwarm"
    )
    axs[1].set_title("w_e over dx,dy (rad/s)")
    axs[1].set_xlabel("dx (m)")
    axs[1].set_ylabel("dy (m)")
    fig.colorbar(im1, ax=axs[1])

    im2 = axs[2].imshow(
        EL, extent=[args.dx_min, args.dx_max, args.dy_min, args.dy_max], origin="lower", cmap="coolwarm"
    )
    axs[2].set_title("e_lat over dx,dy (m)")
    axs[2].set_xlabel("dx (m)")
    axs[2].set_ylabel("dy (m)")
    fig.colorbar(im2, ax=axs[2])

    if args.output_prefix:
        fig.savefig(f"{args.output_prefix}_heatmaps.png", dpi=200)
    if args.show:
        plt.show()
    plt.close(fig)


def plot_w_vs_dyaw(args):
    current_yaw = args.current_yaw
    dx = args.dx_line
    dy = args.dy_line
    dyaws = np.linspace(-math.pi, math.pi, 721)

    r_goals = [args.r_far * 2.0, 0.5 * (args.r_far + args.r_close), max(args.r_close * 0.5, 1e-3)]
    labels = [
        f"r_goal={r_goals[0]:.3f} (FAR)",
        f"r_goal={r_goals[1]:.3f} (MID)",
        f"r_goal={r_goals[2]:.3f} (NEAR)",
    ]

    plt.figure(figsize=(8, 5))
    for rg, lbl in zip(r_goals, labels):
        W = []
        for dyg in dyaws:
            waypoint_yaw = normalize_angle(current_yaw + dyg)
            _, w_e, _, _ = compute_errors(
                waypoint_x=dx,
                waypoint_y=dy,
                waypoint_yaw=waypoint_yaw,
                current_x=0.0,
                current_y=0.0,
                current_yaw=current_yaw,
                r_goal=rg,
                r_close=args.r_close,
                r_far=args.r_far,
                K_lat=args.K_lat,
                K_yaw=args.K_yaw,
                yaw_tol=args.yaw_tol,
            )
            W.append(w_e)
        plt.plot(dyaws, W, label=lbl)

    plt.title("w_e vs d_yaw for various r_goal")
    plt.xlabel("d_yaw (rad)")
    plt.ylabel("w_e (rad/s)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    if args.output_prefix:
        plt.savefig(f"{args.output_prefix}_w_vs_dyaw.png", dpi=200)
    if args.show:
        plt.show()
    plt.close()


def plot_interactive(args):
    # Initial values (from args)
    state = {
        'K_lat': args.K_lat,
        'K_yaw': args.K_yaw,
        'r_close': args.r_close,
        'r_far': args.r_far,
        'yaw_tol': args.yaw_tol,
        'current_yaw': args.current_yaw,
        'dyaw_const': args.dyaw_const,
        'r_goal_heatmap': args.r_goal_heatmap,
        'dx_line': args.dx_line,
        'dy_line': args.dy_line,
    }

    dx = np.linspace(args.dx_min, args.dx_max, args.grid)
    dy = np.linspace(args.dy_min, args.dy_max, args.grid)
    DX, DY = np.meshgrid(dx, dy)
    dyaws = np.linspace(-math.pi, math.pi, 721)

    fig, axs = plt.subplots(1, 3, figsize=(18, 7))
    plt.subplots_adjust(bottom=0.35)

    # Placeholders; will be updated in callback
    V = np.zeros_like(DX)
    W = np.zeros_like(DX)
    EL = np.zeros_like(DX)
    imV = axs[0].imshow(V, extent=[args.dx_min, args.dx_max, args.dy_min, args.dy_max], origin="lower", cmap="coolwarm")
    axs[0].set_title("v_e over dx,dy (m)")
    axs[0].set_xlabel("dx (m)")
    axs[0].set_ylabel("dy (m)")
    cbar0 = fig.colorbar(imV, ax=axs[0])

    imW = axs[1].imshow(W, extent=[args.dx_min, args.dx_max, args.dy_min, args.dy_max], origin="lower", cmap="coolwarm")
    axs[1].set_title("w_e over dx,dy (rad/s)")
    axs[1].set_xlabel("dx (m)")
    axs[1].set_ylabel("dy (m)")
    cbar1 = fig.colorbar(imW, ax=axs[1])

    imEL = axs[2].imshow(EL, extent=[args.dx_min, args.dx_max, args.dy_min, args.dy_max], origin="lower", cmap="coolwarm")
    axs[2].set_title("e_lat over dx,dy (m)")
    axs[2].set_xlabel("dx (m)")
    axs[2].set_ylabel("dy (m)")
    cbar2 = fig.colorbar(imEL, ax=axs[2])

    # Secondary figure elements: w_e vs d_yaw as inset (reuse right axis twin)
    line_ax = axs[1].inset_axes([0.05, 0.55, 0.4, 0.4])
    line_ax.set_title("w_e vs d_yaw")
    line_ax.set_xlabel("d_yaw (rad)")
    line_ax.set_ylabel("w_e (rad/s)")
    line_ax.grid(True, alpha=0.3)
    line_far, = line_ax.plot([], [], label="FAR")
    line_mid, = line_ax.plot([], [], label="MID")
    line_near, = line_ax.plot([], [], label="NEAR")
    line_ax.legend(fontsize=8)

    # Sliders layout (2 columns)
    axcolor = 'lightgoldenrodyellow'
    # Column 1
    ax_Klat = plt.axes([0.05, 0.25, 0.4, 0.03], facecolor=axcolor)
    ax_Kyaw = plt.axes([0.05, 0.20, 0.4, 0.03], facecolor=axcolor)
    ax_rclose = plt.axes([0.05, 0.15, 0.4, 0.03], facecolor=axcolor)
    ax_rfar = plt.axes([0.05, 0.10, 0.4, 0.03], facecolor=axcolor)
    ax_yawtol = plt.axes([0.05, 0.05, 0.4, 0.03], facecolor=axcolor)
    # Column 2
    ax_cyaw = plt.axes([0.55, 0.25, 0.4, 0.03], facecolor=axcolor)
    ax_dyaw = plt.axes([0.55, 0.20, 0.4, 0.03], facecolor=axcolor)
    ax_rgoal = plt.axes([0.55, 0.15, 0.4, 0.03], facecolor=axcolor)
    ax_dxline = plt.axes([0.55, 0.10, 0.4, 0.03], facecolor=axcolor)
    ax_dyline = plt.axes([0.55, 0.05, 0.4, 0.03], facecolor=axcolor)

    sKlat = Slider(ax_Klat, 'K_lat', 0.0, 10.0, valinit=state['K_lat'], valstep=0.01)
    sKyaw = Slider(ax_Kyaw, 'K_yaw', 0.0, 10.0, valinit=state['K_yaw'], valstep=0.01)
    srclose = Slider(ax_rclose, 'r_close (m)', 0.0, 0.2, valinit=state['r_close'], valstep=0.001)
    srfar = Slider(ax_rfar, 'r_far (m)', 0.0, 0.5, valinit=state['r_far'], valstep=0.001)
    syawtol = Slider(ax_yawtol, 'yaw_tol (rad)', 0.0, 0.5, valinit=state['yaw_tol'], valstep=0.005)
    scyaw = Slider(ax_cyaw, 'current_yaw (rad)', -math.pi, math.pi, valinit=state['current_yaw'], valstep=0.005)
    sdyaw = Slider(ax_dyaw, 'dyaw_const (rad)', -math.pi, math.pi, valinit=state['dyaw_const'], valstep=0.005)
    srgoal = Slider(ax_rgoal, 'r_goal_heatmap (m)', 0.0, 1.0, valinit=state['r_goal_heatmap'], valstep=0.005)
    sdxline = Slider(ax_dxline, 'dx_line (m)', args.dx_min, args.dx_max, valinit=state['dx_line'], valstep=0.005)
    sdyline = Slider(ax_dyline, 'dy_line (m)', args.dy_min, args.dy_max, valinit=state['dy_line'], valstep=0.005)

    reset_ax = plt.axes([0.45, 0.005, 0.1, 0.035])
    reset_btn = Button(reset_ax, 'Reset', color=axcolor, hovercolor='0.975')

    def recompute_and_draw(_=None):
        # Read sliders
        K_lat = sKlat.val
        K_yaw = sKyaw.val
        r_close = srclose.val
        # Enforce r_far > r_close without recursive callbacks
        r_far_raw = srfar.val
        r_far = max(r_far_raw, r_close + 1e-6)
        if r_far != r_far_raw:
            try:
                prev_eventson = srfar.eventson
                srfar.eventson = False
                srfar.set_val(r_far)
                srfar.eventson = prev_eventson
            except Exception:
                pass
        yaw_tol = syawtol.val
        current_yaw = scyaw.val
        dyaw_const = sdyaw.val
        r_goal_heatmap = srgoal.val
        dx_line = sdxline.val
        dy_line = sdyline.val

        # Heatmaps
        waypoint_yaw = normalize_angle(current_yaw + dyaw_const)
        for i in range(DX.shape[0]):
            for j in range(DX.shape[1]):
                v_e, w_e, e_lat, _ = compute_errors(
                    waypoint_x=DX[i, j],
                    waypoint_y=DY[i, j],
                    waypoint_yaw=waypoint_yaw,
                    current_x=0.0,
                    current_y=0.0,
                    current_yaw=current_yaw,
                    r_goal=r_goal_heatmap,
                    r_close=r_close,
                    r_far=r_far,
                    K_lat=K_lat,
                    K_yaw=K_yaw,
                    yaw_tol=yaw_tol,
                )
                V[i, j] = v_e
                W[i, j] = w_e
                EL[i, j] = e_lat

        imV.set_data(V)
        imW.set_data(W)
        imEL.set_data(EL)
        imV.set_clim(vmin=np.min(V), vmax=np.max(V))
        imW.set_clim(vmin=np.min(W), vmax=np.max(W))
        imEL.set_clim(vmin=np.min(EL), vmax=np.max(EL))
        cbar0.update_normal(imV)
        cbar1.update_normal(imW)
        cbar2.update_normal(imEL)

        # Line plot for three r_goal regimes
        r_goals = [r_far * 2.0, 0.5 * (r_far + r_close), max(r_close * 0.5, 1e-3)]
        Ws = []
        for rg in r_goals:
            w_vals = []
            for dyg in dyaws:
                waypoint_yaw = normalize_angle(current_yaw + dyg)
                _, w_e, _, _ = compute_errors(
                    waypoint_x=dx_line,
                    waypoint_y=dy_line,
                    waypoint_yaw=waypoint_yaw,
                    current_x=0.0,
                    current_y=0.0,
                    current_yaw=current_yaw,
                    r_goal=rg,
                    r_close=r_close,
                    r_far=r_far,
                    K_lat=K_lat,
                    K_yaw=K_yaw,
                    yaw_tol=yaw_tol,
                )
                w_vals.append(w_e)
            Ws.append(np.array(w_vals))
        line_far.set_data(dyaws, Ws[0])
        line_mid.set_data(dyaws, Ws[1])
        line_near.set_data(dyaws, Ws[2])
        line_ax.relim()
        line_ax.autoscale_view()

        fig.canvas.draw_idle()

    def on_reset(event):
        sKlat.reset(); sKyaw.reset(); srclose.reset(); srfar.reset(); syawtol.reset()
        scyaw.reset(); sdyaw.reset(); srgoal.reset(); sdxline.reset(); sdyline.reset()

    # Wire up callbacks
    for s in [sKlat, sKyaw, srclose, srfar, syawtol, scyaw, sdyaw, srgoal, sdxline, sdyline]:
        s.on_changed(recompute_and_draw)
    reset_btn.on_clicked(on_reset)

    # Initial draw
    recompute_and_draw()
    plt.show()


def main():
    ap = argparse.ArgumentParser(
        description="Visualize compute_errors parameters (v_e, w_e) over dx, dy, d_yaw with global-goal yaw blending"
    )
    # Controller params
    ap.add_argument("--K_lat", type=float, default=1.0)
    ap.add_argument("--K_yaw", type=float, default=1.0)
    ap.add_argument("--r_close", type=float, default=0.005)
    ap.add_argument("--r_far", type=float, default=0.03)
    ap.add_argument("--yaw_tol", type=float, default=0.1)

    # Heatmap configuration
    ap.add_argument("--dx-min", dest="dx_min", type=float, default=-0.2)
    ap.add_argument("--dx-max", dest="dx_max", type=float, default=0.2)
    ap.add_argument("--dy-min", dest="dy_min", type=float, default=-0.2)
    ap.add_argument("--dy-max", dest="dy_max", type=float, default=0.2)
    ap.add_argument("--grid", type=int, default=201, help="grid size per axis for heatmaps")
    ap.add_argument("--current-yaw", dest="current_yaw", type=float, default=0.0)
    ap.add_argument("--dyaw-const", dest="dyaw_const", type=float, default=0.0, help="fixed d_yaw for heatmaps")
    ap.add_argument("--r-goal-heatmap", dest="r_goal_heatmap", type=float, default=0.5)

    # Line plot configuration
    ap.add_argument("--dx-line", dest="dx_line", type=float, default=0.0)
    ap.add_argument("--dy-line", dest="dy_line", type=float, default=0.1)

    # Output / display
    ap.add_argument("--output-prefix", type=str, default="")
    ap.add_argument("--show", action="store_true")

    args = ap.parse_args()

    # If --show is provided, open interactive with sliders; otherwise, static outputs
    if args.show:
        plot_interactive(args)
    else:
        plot_heatmaps(args)
        plot_w_vs_dyaw(args)
        print("\nSample values (center point):")
        v_e, w_e, e_lat, d_yaw = compute_errors(
            waypoint_x=0.0,
            waypoint_y=args.dy_line,
            waypoint_yaw=normalize_angle(args.current_yaw + args.dyaw_const),
            current_x=0.0,
            current_y=0.0,
            current_yaw=args.current_yaw,
            r_goal=args.r_goal_heatmap,
            r_close=args.r_close,
            r_far=args.r_far,
            K_lat=args.K_lat,
            K_yaw=args.K_yaw,
            yaw_tol=args.yaw_tol,
        )
        print(f"  v_e={v_e:.4f}, w_e={w_e:.4f}, e_lat={e_lat:.4f}, d_yaw={d_yaw:.4f}")


if __name__ == "__main__":
    main()


