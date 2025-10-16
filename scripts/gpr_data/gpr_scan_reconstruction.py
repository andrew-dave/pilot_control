#!/usr/bin/env python3
"""
gpr_scan_reconstruction.py

Reconstructs regularly spaced A-scans from an A.T-with-XYZ CSV produced by gpr_post_processing.py.

Pipeline:
- Load input CSV: first column is TWT, columns 1..N are traces; bottom 3 rows are px,py,pz per trace
- Compute initial pose as average of first K positions (default K=10, clip to available)
- Fit heading via linear regression y = m x + b over all valid (px,py)
- Generate Points Of Interest (POIs) starting from initial pose, along heading, spaced by 5 mm (configurable)
- For each POI, compute distance-weighted, decluttered average of neighboring traces within 2.5 mm radius
  * distance weight: w_d = max(0, 1 - d/R)
  * declutter weight: w_c = 1 / max(1, (# of neighbors within r_c of this neighbor)); default r_c = 1 mm
  * final weight: w = w_d * w_c, normalized per POI
- Save output CSV similar to gpr_post_processing.py: first col TWT, columns are POI traces; append x,y,z rows of POIs
- Plot result with optional interactive readout
"""

import sys
import os
import argparse
import csv
import numpy as np


def _import_matplotlib_pyplot():
    try:
        import matplotlib.pyplot as plt  # type: ignore
        return plt
    except Exception as e:
        print(f"Matplotlib unavailable ({e}); skipping plot.")
        return None


def load_at_with_xyz_csv(path: str):
    """Load A.T-with-XYZ CSV saved by gpr_post_processing.py.

    Returns dict with keys: 'twt' (twt_len,), 'A_T' (twt_len, n_cols), 'px','py','pz' (n_cols,), 'header_cols' list.
    """
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = []
        for row in reader:
            rows.append(row)

    # Convert to float with empty -> NaN
    def to_float(x):
        try:
            if x == '' or x is None:
                return float('nan')
            return float(x)
        except Exception:
            return float('nan')

    M = np.array([[to_float(c) for c in r] for r in rows], dtype=float)
    if M.size == 0:
        return {'twt': np.zeros((0,)), 'A_T': np.zeros((0, 0)), 'px': np.zeros((0,)), 'py': np.zeros((0,)), 'pz': np.zeros((0,)), 'header_cols': header}

    # Detect last three rows as px/py/pz (first column NaN)
    if M.shape[0] < 4:
        raise RuntimeError("Input CSV does not contain appended px/py/pz rows.")
    twt_len = M.shape[0] - 3
    twt = M[:twt_len, 0]
    A_T = M[:twt_len, 1:]
    px = M[twt_len + 0, 1:]
    py = M[twt_len + 1, 1:]
    pz = M[twt_len + 2, 1:]
    return {'twt': twt, 'A_T': A_T, 'px': px, 'py': py, 'pz': pz, 'header_cols': header}


def compute_initial_pose(px: np.ndarray, py: np.ndarray, pz: np.ndarray, k: int = 10):
    k = int(max(1, min(k, px.size)))
    valid = np.isfinite(px[:k]) & np.isfinite(py[:k])
    if not np.any(valid):
        return (0.0, 0.0, 0.0)
    x0 = float(np.nanmean(px[:k]))
    y0 = float(np.nanmean(py[:k]))
    z0 = float(np.nanmean(pz[:k])) if pz.size >= k else 0.0
    return (x0, y0, z0)


def fit_heading_unit_vector(px: np.ndarray, py: np.ndarray):
    mask = np.isfinite(px) & np.isfinite(py)
    if np.sum(mask) < 2:
        return (1.0, 0.0)
    try:
        m, b = np.polyfit(px[mask], py[mask], 1)
        v = np.array([1.0, float(m)], dtype=float)
        n = float(np.linalg.norm(v))
        if n <= 1e-12:
            return (1.0, 0.0)
        v = v / n
        return (float(v[0]), float(v[1]))
    except Exception:
        return (1.0, 0.0)


def generate_pois(initial_xy, heading_dxdy, spacing_m, px, py):
    start = np.array(initial_xy[:2], dtype=float)
    dir_vec = np.array(heading_dxdy, dtype=float)
    dir_vec = dir_vec / (np.linalg.norm(dir_vec) + 1e-12)
    # Project all points onto heading starting at initial
    pts = np.stack([px, py], axis=1)
    mask = np.isfinite(pts).all(axis=1)
    if not np.any(mask):
        return np.array([start])
    proj = (pts[mask] - start[None, :]) @ dir_vec
    proj = np.maximum(0.0, proj)
    max_d = float(np.max(proj)) if proj.size > 0 else 0.0
    if not np.isfinite(max_d) or max_d <= 0:
        return np.array([start])
    num = max(1, int(np.floor(max_d / float(spacing_m))))
    ds = np.linspace(0.0, num * float(spacing_m), num=num + 1, endpoint=True)
    pois = start[None, :] + ds[:, None] * dir_vec[None, :]
    return pois


def compute_decluttered_weighted_average(A_T: np.ndarray,
                                         px: np.ndarray,
                                         py: np.ndarray,
                                         pois_xy: np.ndarray,
                                         radius_m: float = 0.0025,
                                         declutter_r_m: float = 0.0010):
    """Distance-weighted, decluttered average traces at POIs.

    Returns matrix (twt_len, M) where M = len(pois).
    """
    twt_len, n_cols = A_T.shape
    M = pois_xy.shape[0]
    out = np.zeros((twt_len, M), dtype=A_T.dtype)
    # Precompute source positions
    src = np.stack([px, py], axis=1)
    valid_src = np.isfinite(src).all(axis=1)
    for i in range(M):
        poi = pois_xy[i]
        if not np.any(valid_src):
            continue
        d = np.full(n_cols, np.inf)
        d[valid_src] = np.hypot(src[valid_src, 0] - poi[0], src[valid_src, 1] - poi[1])
        mask = d <= float(radius_m)
        if not np.any(mask):
            # Fallback: nearest neighbor
            j = int(np.nanargmin(d)) if np.any(np.isfinite(d)) else 0
            out[:, i] = A_T[:, j]
            continue
        # Distance weights
        w_d = np.clip(1.0 - (d[mask] / float(radius_m)), 0.0, 1.0)
        idxs = np.nonzero(mask)[0]
        # Declutter weights based on local neighbor count within declutter_r_m among selected points
        sel_xy = src[idxs]
        if sel_xy.shape[0] > 1:
            # Pairwise distances within selected
            # For each point, count how many others within declutter radius
            declut = np.ones(sel_xy.shape[0], dtype=float)
            for k in range(sel_xy.shape[0]):
                dd = np.hypot(sel_xy[:, 0] - sel_xy[k, 0], sel_xy[:, 1] - sel_xy[k, 1])
                n_close = int(np.sum(dd <= float(declutter_r_m)))
                declut[k] = 1.0 / max(1, n_close)
        else:
            declut = np.ones(sel_xy.shape[0], dtype=float)
        w = w_d * declut
        ws = float(np.sum(w))
        if ws <= 1e-12:
            # Fallback: unweighted
            out[:, i] = np.mean(A_T[:, idxs], axis=1)
        else:
            w_norm = w / ws
            out[:, i] = (A_T[:, idxs] * w_norm[None, :]).sum(axis=1)
    return out


def plot_result(A_T_out: np.ndarray, twt: np.ndarray, pois_xyz: np.ndarray, spacing_m: float, save_path: str = None, interactive: bool = False):
    plt = _import_matplotlib_pyplot()
    if plt is None:
        return
    if A_T_out.size == 0 or twt.size == 0:
        print("Nothing to plot: empty data.")
        return
    x = np.arange(A_T_out.shape[1], dtype=float)
    v = np.percentile(np.abs(A_T_out), 98) if np.isfinite(A_T_out).any() else 1.0
    fig, ax = plt.subplots(figsize=(12, 6))
    im = ax.imshow(
        A_T_out,
        aspect="auto",
        cmap="gray",
        vmin=-v,
        vmax=v,
        extent=[float(x[0]), float(x[-1]), float(twt[-1]), float(twt[0])],
    )
    ax.set_xlabel("POI index")
    ax.set_ylabel("TWT (ms)")
    ax.set_title(f"Reconstructed Radargram (~{spacing_m*1000:.0f} mm POI spacing)")
    fig.colorbar(im, ax=ax, label="Amplitude")

    if pois_xyz is not None and pois_xyz.size > 0:
        ax2 = ax.twinx()
        ax2.plot(x, pois_xyz[:, 0], 'r-', linewidth=1.0, label='x (m)')
        ax2.plot(x, pois_xyz[:, 1], 'g-', linewidth=1.0, label='y (m)')
        ax2.plot(x, pois_xyz[:, 2], 'b-', linewidth=1.0, label='z (m)')
        ax2.set_ylabel("Position (m)")
        ax2.legend(loc='upper right')

        if interactive:
            sel_line = ax.axvline(x[0], color='y', linewidth=0.8, alpha=0.7)
            info = ax2.text(0.01, 0.98,
                            f"poi=0  x={pois_xyz[0,0]:.4f}  y={pois_xyz[0,1]:.4f}  z={pois_xyz[0,2]:.4f}",
                            transform=ax2.transAxes, va='top', ha='left', fontsize=9,
                            bbox=dict(boxstyle='round', fc='white', ec='0.7', alpha=0.8))

            def on_move(event):
                if event.inaxes not in (ax, ax2) or event.xdata is None:
                    return
                j = int(np.argmin(np.abs(x - float(event.xdata))))
                j = max(0, min(j, x.size - 1))
                sel_line.set_xdata([x[j], x[j]])
                info.set_text(f"poi={j}  x={pois_xyz[j,0]:.4f}  y={pois_xyz[j,1]:.4f}  z={pois_xyz[j,2]:.4f}")
                fig.canvas.draw_idle()

            fig.canvas.mpl_connect('motion_notify_event', on_move)

    plt.tight_layout()
    if save_path:
        try:
            plt.savefig(save_path, dpi=150)
            print(f"Saved plot: {save_path}")
        except Exception as e:
            print(f"Failed to save plot '{save_path}': {e}")
    else:
        plt.show()


def save_output_csv(base_out: str, twt: np.ndarray, A_T_out: np.ndarray, pois_xyz: np.ndarray) -> str:
    out_csv = base_out + "_reconstructed.csv"
    header_cols = ["twt"] + [f"poi{i}" for i in range(A_T_out.shape[1])]
    mat = np.column_stack((twt, A_T_out))
    # Append x,y,z rows with blank in first cell
    x_row = np.concatenate(([np.nan], pois_xyz[:, 0]))
    y_row = np.concatenate(([np.nan], pois_xyz[:, 1]))
    z_row = np.concatenate(([np.nan], pois_xyz[:, 2]))
    out_mat = np.vstack((mat, x_row[None, :], y_row[None, :], z_row[None, :]))
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f, lineterminator='\n')
        w.writerow(header_cols)
        for r in out_mat:
            w.writerow([f"{v:.10g}" if np.isfinite(v) else "" for v in r])
    print(f"Saved reconstructed CSV: {out_csv}")
    return out_csv


def main():
    parser = argparse.ArgumentParser(description="GPR scan reconstruction from A.T-with-XYZ CSV")
    parser.add_argument('input_csv', help='Path to A.T-with-XYZ CSV produced by gpr_post_processing.py')
    parser.add_argument('--spacing_m', type=float, default=0.005, help='POI spacing (m) (default: 0.005)')
    parser.add_argument('--weight_radius_m', type=float, default=0.0025, help='Distance weight radius (m) (default: 0.0025)')
    parser.add_argument('--declutter_radius_m', type=float, default=0.0010, help='Declutter radius (m) (default: 0.0010)')
    parser.add_argument('--initial_k', type=int, default=10, help='Samples to average for initial pose (default: 10)')
    parser.add_argument('--plot', action='store_true', help='Display reconstructed radargram with POIs')
    parser.add_argument('--save_plot', type=str, default=None, help='Optional path to save plot image')
    parser.add_argument('--interactive', action='store_true', help='Enable interactive readout on plot')
    args = parser.parse_args()

    if not os.path.isfile(args.input_csv):
        print(f"Input CSV not found: {args.input_csv}")
        sys.exit(1)

    data = load_at_with_xyz_csv(args.input_csv)
    twt = data['twt']
    A_T = data['A_T']
    px = data['px']
    py = data['py']
    pz = data['pz']

    if A_T.size == 0:
        print("Empty input A_T; nothing to do.")
        sys.exit(0)

    initial = compute_initial_pose(px, py, pz, k=args.initial_k)
    heading = fit_heading_unit_vector(px, py)
    pois_xy = generate_pois((initial[0], initial[1]), heading, args.spacing_m, px, py)
    pois_xyz = np.column_stack((pois_xy, np.full(pois_xy.shape[0], initial[2], dtype=float)))

    A_T_out = compute_decluttered_weighted_average(
        A_T, px, py, pois_xy,
        radius_m=args.weight_radius_m,
        declutter_r_m=args.declutter_radius_m,
    )

    base, _ = os.path.splitext(args.input_csv)
    out_csv = save_output_csv(base, twt, A_T_out, pois_xyz)

    if args.plot or (args.save_plot is not None and len(str(args.save_plot)) > 0):
        plot_result(A_T_out, twt, pois_xyz, args.spacing_m, save_path=args.save_plot, interactive=args.interactive)


if __name__ == '__main__':
    main()


