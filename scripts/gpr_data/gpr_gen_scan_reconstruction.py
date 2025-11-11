#!/usr/bin/env python3
"""
gpr_gen_scan_reconstruction.py

Reconstructs regularly spaced A-scans from an A.T-with-XYZ CSV following GENERAL TRAJECTORIES
(straight lines, curves, loops, etc.) using direct-distance POI placement.

Method (Robust to Odometry Noise):
- Generate POIs by greedy forward search: find next point at ~spacing_m from last POI
- Measures DIRECT distance from previous POI (not cumulative), avoiding error accumulation
- Local smoothing window averages nearby points to reduce position noise
- Distance-weighted averaging for trace reconstruction

Pipeline:
- Load input CSV: first column is TWT, columns 1..N are traces; bottom 3 rows are px,py,pz per trace
- Compute initial pose as average of first K positions
- Generate POIs by direct-distance search with configurable tolerance
- For each POI, compute distance-weighted, decluttered average of neighboring traces
- Save output CSV: first col TWT, columns are POI traces; append x,y,z rows of POIs
- Plot result with optional interactive tuning

Interactive Slider Mode (--sliders):
- Adjust all processing parameters in real-time with visual feedback
- POI spacing, tolerance, smoothing window, weight radius, contrast settings
"""

import sys
import os
import argparse
import csv
import numpy as np
from datetime import datetime


def _import_matplotlib_pyplot():
    try:
        import matplotlib.pyplot as plt  # type: ignore
        return plt
    except Exception as e:
        print(f"Matplotlib unavailable ({e}); skipping plot.")
        return None


def _import_matplotlib_widgets():
    try:
        from matplotlib.widgets import Slider, RadioButtons, Button  # type: ignore
        return Slider, RadioButtons, Button
    except Exception as e:
        print(f"Matplotlib widgets unavailable ({e}); interactive sliders disabled.")
        return None, None, None


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
    """Compute initial pose by averaging first K valid positions."""
    k = int(max(1, min(k, px.size)))
    valid = np.isfinite(px[:k]) & np.isfinite(py[:k])
    if not np.any(valid):
        return (0.0, 0.0, 0.0)
    x0 = float(np.nanmean(px[:k]))
    y0 = float(np.nanmean(py[:k]))
    z0 = float(np.nanmean(pz[:k])) if pz.size >= k else 0.0
    return (x0, y0, z0)


def generate_pois_direct_distance(px: np.ndarray, py: np.ndarray, pz: np.ndarray,
                                  initial_pose: tuple, spacing_m: float,
                                  tolerance: float = 0.3, smoothing_window: int = 5):
    """Generate POIs by greedy forward search using DIRECT distance from last POI.
    
    This method avoids cumulative error by measuring direct distance from the previous
    POI to candidate points, rather than summing distances along the path.
    
    Args:
        px, py, pz: Position arrays from CSV
        initial_pose: (x0, y0, z0) starting position
        spacing_m: Target spacing between POIs in meters
        tolerance: Relative tolerance for spacing (0.3 = ±30%)
        smoothing_window: Number of nearby points to average for each POI
    
    Returns:
        pois_xyz: (M, 3) array of POI positions
        poi_indices: Original indices in px/py/pz for each POI
    """
    valid = np.isfinite(px) & np.isfinite(py) & np.isfinite(pz)
    
    if not np.any(valid):
        return np.array([initial_pose]), np.array([0])
    
    valid_indices = np.where(valid)[0]
    positions = np.stack([px, py, pz], axis=1)
    
    pois_xyz = [np.array(initial_pose)]
    poi_indices = [valid_indices[0] if len(valid_indices) > 0 else 0]
    
    current_poi = np.array(initial_pose)
    search_start = 0
    
    max_iterations = len(valid_indices) * 2  # Safety limit
    iteration = 0
    
    while search_start < len(valid_indices) and iteration < max_iterations:
        iteration += 1
        
        # Compute DIRECT distances from current POI to all remaining points
        remaining_indices = valid_indices[search_start:]
        if len(remaining_indices) == 0:
            break
        
        distances = np.linalg.norm(positions[remaining_indices] - current_poi[None, :], axis=1)
        
        # Find points within tolerance of target spacing
        tolerance_range = spacing_m * tolerance
        target_mask = np.abs(distances - spacing_m) <= tolerance_range
        
        if not np.any(target_mask):
            # No points in tolerance range - try to find closest point beyond spacing
            beyond_mask = distances >= spacing_m
            if np.any(beyond_mask):
                # Take the closest point that's at least spacing_m away
                candidate_idx = np.where(beyond_mask)[0][0]
            elif np.any(distances > 0):
                # Take the furthest available point
                candidate_idx = np.argmax(distances)
            else:
                break  # No more valid points
        else:
            # Choose the point closest to exact spacing within tolerance
            target_indices = np.where(target_mask)[0]
            candidate_idx = target_indices[np.argmin(np.abs(distances[target_indices] - spacing_m))]
        
        # Get the actual index in original array (latch point)
        latch_idx = remaining_indices[candidate_idx]
        
        # Apply local smoothing around latch point
        half_win = smoothing_window // 2
        latch_pos_in_valid = np.searchsorted(valid_indices, latch_idx)
        smooth_start = max(0, latch_pos_in_valid - half_win)
        smooth_end = min(len(valid_indices), latch_pos_in_valid + half_win + 1)
        window_indices = valid_indices[smooth_start:smooth_end]
        
        if len(window_indices) > 0:
            # Average positions in window to reduce noise
            new_poi = np.mean(positions[window_indices], axis=0)
            pois_xyz.append(new_poi)
            poi_indices.append(latch_idx)
            
            # Update for next iteration
            current_poi = new_poi
            search_start = latch_pos_in_valid + 1
        else:
            break
    
    return np.array(pois_xyz), np.array(poi_indices)


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
        # Declutter weights based on local neighbor count
        sel_xy = src[idxs]
        if sel_xy.shape[0] > 1:
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
            out[:, i] = np.mean(A_T[:, idxs], axis=1)
        else:
            w_norm = w / ws
            out[:, i] = (A_T[:, idxs] * w_norm[None, :]).sum(axis=1)
    return out


def apply_contrast_enhancement(data: np.ndarray, method: str = 'percentile', 
                               percentile: float = 98.0, agc_window: int = 50) -> np.ndarray:
    """Apply contrast enhancement to radargram data."""
    if data.size == 0 or not np.isfinite(data).any():
        return data
    
    enhanced = data.copy()
    
    if method == 'agc':
        # Automatic Gain Control
        twt_len, n_traces = enhanced.shape
        half_win = max(1, agc_window // 2)
        for i in range(n_traces):
            trace = enhanced[:, i]
            rms = np.zeros_like(trace)
            for j in range(twt_len):
                start = max(0, j - half_win)
                end = min(twt_len, j + half_win + 1)
                window = trace[start:end]
                rms[j] = np.sqrt(np.mean(window**2)) if window.size > 0 else 1.0
            rms = np.where(rms < 1e-10, 1.0, rms)
            enhanced[:, i] = trace / rms
        vmax = np.percentile(np.abs(enhanced[np.isfinite(enhanced)]), 99.5)
        enhanced = np.clip(enhanced, -vmax, vmax)
        
    elif method == 'histeq':
        # Histogram equalization
        valid_data = enhanced[np.isfinite(enhanced)]
        if valid_data.size > 0:
            vmin, vmax = np.percentile(valid_data, [1, 99])
            enhanced = (enhanced - vmin) / (vmax - vmin + 1e-10)
            enhanced = np.clip(enhanced, 0, 1)
            hist, bins = np.histogram(enhanced[np.isfinite(enhanced)].flatten(), bins=256, range=(0, 1))
            cdf = hist.cumsum()
            cdf = cdf / cdf[-1]
            enhanced_flat = enhanced.flatten()
            valid_mask = np.isfinite(enhanced_flat)
            enhanced_flat[valid_mask] = np.interp(enhanced_flat[valid_mask], bins[:-1], cdf)
            enhanced = enhanced_flat.reshape(enhanced.shape)
            enhanced = 2 * enhanced - 1
            
    elif method == 'log':
        # Logarithmic scaling
        sign = np.sign(enhanced)
        enhanced = sign * np.log10(1 + np.abs(enhanced) / (np.percentile(np.abs(enhanced[np.isfinite(enhanced)]), 50) + 1e-10))
        vmax = np.percentile(np.abs(enhanced[np.isfinite(enhanced)]), 99)
        enhanced = np.clip(enhanced, -vmax, vmax)
        
    else:  # percentile (default)
        pass
    
    return enhanced


def plot_trajectory_and_pois(px: np.ndarray, py: np.ndarray, pois_xyz: np.ndarray):
    """Plot the trajectory path and POI positions for verification."""
    plt = _import_matplotlib_pyplot()
    if plt is None:
        return
    
    valid = np.isfinite(px) & np.isfinite(py)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot trajectory
    ax.plot(px[valid], py[valid], 'b.', markersize=2, alpha=0.3, label='Raw trajectory')
    
    # Plot POIs
    if pois_xyz.size > 0:
        ax.plot(pois_xyz[:, 0], pois_xyz[:, 1], 'ro-', markersize=5, linewidth=1.5, 
                label=f'POIs (n={len(pois_xyz)})')
        
        # Mark start and end
        ax.plot(pois_xyz[0, 0], pois_xyz[0, 1], 'gs', markersize=12, label='Start')
        ax.plot(pois_xyz[-1, 0], pois_xyz[-1, 1], 'r^', markersize=12, label='End')
    
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_title('GPR Trajectory and POI Placement (Direct-Distance Method)', fontsize=12)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    plt.tight_layout()
    plt.show()


def plot_result(A_T_out: np.ndarray, twt: np.ndarray, pois_xyz: np.ndarray, spacing_m: float, 
                save_path: str = None, contrast_method: str = 'percentile',
                contrast_percentile: float = 98.0, agc_window: int = 50):
    """Plot reconstructed radargram."""
    plt = _import_matplotlib_pyplot()
    if plt is None:
        return
    if A_T_out.size == 0 or twt.size == 0:
        print("Nothing to plot: empty data.")
        return
    
    # Apply contrast enhancement
    A_T_enhanced = apply_contrast_enhancement(A_T_out, method=contrast_method, 
                                              percentile=contrast_percentile, 
                                              agc_window=agc_window)
    
    x = np.arange(A_T_enhanced.shape[1], dtype=float)
    v = np.percentile(np.abs(A_T_enhanced), contrast_percentile) if np.isfinite(A_T_enhanced).any() else 1.0
    
    fig, ax = plt.subplots(figsize=(12, 6))
    im = ax.imshow(
        A_T_enhanced,
        aspect="auto",
        cmap="gray",
        vmin=-v,
        vmax=v,
        extent=[float(x[0]), float(x[-1]), float(twt[-1]), float(twt[0])],
    )
    ax.set_xlabel("POI index")
    ax.set_ylabel("TWT (ms)")
    title = f"General Trajectory Radargram (~{spacing_m*1000:.0f} mm POI spacing)"
    if contrast_method != 'percentile':
        title += f" [{contrast_method.upper()}]"
    ax.set_title(title)
    fig.colorbar(im, ax=ax, label="Amplitude")

    if pois_xyz is not None and pois_xyz.size > 0:
        ax2 = ax.twinx()
        ax2.plot(x, pois_xyz[:, 0], 'r-', linewidth=1.0, label='x (m)')
        ax2.plot(x, pois_xyz[:, 1], 'g-', linewidth=1.0, label='y (m)')
        ax2.plot(x, pois_xyz[:, 2], 'b-', linewidth=1.0, label='z (m)')
        ax2.set_ylabel("Position (m)")
        ax2.legend(loc='upper right')

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
    """Save reconstructed CSV with appended XYZ rows."""
    out_csv = base_out + "_gen_reconstructed.csv"
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
    parser = argparse.ArgumentParser(description="GPR general trajectory reconstruction from A.T-with-XYZ CSV")
    parser.add_argument('input_csv', help='Path to A.T-with-XYZ CSV produced by gpr_post_processing.py')
    parser.add_argument('--spacing_m', type=float, default=0.005, help='Target POI spacing (m) (default: 0.005)')
    parser.add_argument('--tolerance', type=float, default=0.3, help='Spacing tolerance as fraction (default: 0.3 = ±30%%)')
    parser.add_argument('--smoothing_window', type=int, default=5, help='Points to average for POI smoothing (default: 5)')
    parser.add_argument('--weight_radius_m', type=float, default=0.0025, help='Distance weight radius (m) (default: 0.0025)')
    parser.add_argument('--declutter_radius_m', type=float, default=0.0010, help='Declutter radius (m) (default: 0.0010)')
    parser.add_argument('--initial_k', type=int, default=10, help='Samples to average for initial pose (default: 10)')
    parser.add_argument('--plot', action='store_true', help='Display reconstructed radargram')
    parser.add_argument('--plot_trajectory', action='store_true', help='Display trajectory with POI placement')
    parser.add_argument('--save_plot', type=str, default=None, help='Optional path to save radargram plot')
    parser.add_argument('--contrast', type=str, default='percentile', 
                        choices=['percentile', 'agc', 'histeq', 'log'],
                        help='Contrast enhancement method')
    parser.add_argument('--contrast_percentile', type=float, default=98.0,
                        help='Percentile for amplitude clipping (default: 98.0)')
    parser.add_argument('--agc_window', type=int, default=50,
                        help='AGC window size in samples (default: 50)')
    args = parser.parse_args()

    if not os.path.isfile(args.input_csv):
        print(f"Input CSV not found: {args.input_csv}")
        sys.exit(1)

    print(f"Loading data from: {args.input_csv}")
    data = load_at_with_xyz_csv(args.input_csv)
    twt = data['twt']
    A_T = data['A_T']
    px = data['px']
    py = data['py']
    pz = data['pz']

    if A_T.size == 0:
        print("Empty input A_T; nothing to do.")
        sys.exit(0)

    print(f"Loaded {A_T.shape[0]} time samples × {A_T.shape[1]} traces")

    # Compute initial pose
    initial = compute_initial_pose(px, py, pz, k=args.initial_k)
    print(f"Initial pose: x={initial[0]:.4f}, y={initial[1]:.4f}, z={initial[2]:.4f}")

    # Generate POIs using direct-distance method
    print(f"Generating POIs with spacing={args.spacing_m*1000:.2f}mm, "
          f"tolerance=±{args.tolerance*100:.0f}%, smoothing_window={args.smoothing_window}")
    pois_xyz, poi_indices = generate_pois_direct_distance(
        px, py, pz, initial, args.spacing_m, args.tolerance, args.smoothing_window
    )
    print(f"Generated {len(pois_xyz)} POIs")

    # Compute actual spacings for statistics
    if len(pois_xyz) > 1:
        spacings = np.linalg.norm(np.diff(pois_xyz, axis=0), axis=1)
        print(f"POI spacing statistics: mean={np.mean(spacings)*1000:.2f}mm, "
              f"std={np.std(spacings)*1000:.2f}mm, "
              f"min={np.min(spacings)*1000:.2f}mm, max={np.max(spacings)*1000:.2f}mm")

    # Reconstruct traces at POIs
    print(f"Computing weighted averages (radius={args.weight_radius_m*1000:.2f}mm, "
          f"declutter={args.declutter_radius_m*1000:.2f}mm)...")
    pois_xy = pois_xyz[:, :2]
    A_T_out = compute_decluttered_weighted_average(
        A_T, px, py, pois_xy,
        radius_m=args.weight_radius_m,
        declutter_r_m=args.declutter_radius_m,
    )

    # Save output
    base, _ = os.path.splitext(args.input_csv)
    out_csv = save_output_csv(base, twt, A_T_out, pois_xyz)
    print(f"Output shape: {A_T_out.shape[0]} samples × {A_T_out.shape[1]} POIs")

    # Plot trajectory
    if args.plot_trajectory:
        plot_trajectory_and_pois(px, py, pois_xyz)

    # Plot radargram
    if args.plot or (args.save_plot is not None and len(str(args.save_plot)) > 0):
        plot_result(A_T_out, twt, pois_xyz, args.spacing_m, 
                   save_path=args.save_plot, 
                   contrast_method=args.contrast,
                   contrast_percentile=args.contrast_percentile,
                   agc_window=args.agc_window)


if __name__ == '__main__':
    main()

