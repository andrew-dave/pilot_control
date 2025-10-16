#!/usr/bin/env python3
"""
gpr_post_processing.py
Minimal post-processing utilities for GPR:
- Load a SEG-Y file to obtain the amplitude matrix A.T (time x traces)
- Load a scan CSV and extract relevant columns

Usage:
  python gpr_post_processing.py your_file.sgy [scan_log.csv]

This script prints basic summaries (shapes, header detection) and exits.
"""

import sys
import os
import argparse
import numpy as np

try:
    import segysak.segy as sgy
except Exception:
    sgy = None


def load_with_segysak(fname):
    if sgy is None:
        raise RuntimeError("segysak not installed. Install via conda-forge: conda install -c conda-forge segysak")
    ds = sgy.segy_loader(fname)
    data_var = next(iter(ds.data_vars))
    da = ds[data_var]
    # Ensure (cdp, twt)
    if da.dims == ("twt", "cdp"):
        da = da.transpose("cdp", "twt")
    return ds, da


def load_scan_csv(csv_path):
    """Load gpr_scan_controller CSV and return a dict of columns.

    - Detects the header row anywhere in the file, skipping comments and empty lines
    - Returns keys: 'event' (list[str]), 'fastlio_time_us', 'gpr_time_us',
      'pos_x','pos_y','pos_z','gpr_position','gpr_velocity'. Missing fields default to zeros/empty.
    """
    import csv

    out = {
        'event': [],
        'fastlio_time_us': [],
        'gpr_time_us': [],
        'pos_x': [], 'pos_y': [], 'pos_z': [],
        'gpr_position': [], 'gpr_velocity': [],
        'left_position': [], 'right_position': [],
    }
    with open(csv_path, 'r', newline='') as f:
        reader = csv.reader(f)
        header = None
        name_to_idx = {}

        def to_float(val, default=0.0):
            try:
                return float(val)
            except Exception:
                return float(default)

        line_no = 0
        for row in reader:
            line_no += 1
            row = [c.strip() for c in row]
            if not row or all(c == '' for c in row):
                continue
            if header is None:
                if row[0].startswith('#'):
                    continue
                required = {'event', 'pos_x', 'pos_y', 'gpr_position'}
                if required.issubset(set(row)):
                    header = row
                    name_to_idx = {name: i for i, name in enumerate(header)}
                    print(f"Detected CSV header at row {line_no}: {header}")
                    continue
                continue

            if row[0].startswith('#'):
                continue
            if {'event', 'pos_x', 'pos_y', 'gpr_position'}.issubset(set(row)):
                continue

            def col(name, row_vals, default=""):
                idx = name_to_idx.get(name)
                return row_vals[idx] if idx is not None and idx < len(row_vals) else default

            out['event'].append(col('event', row, ''))
            out['fastlio_time_us'].append(to_float(col('fastlio_time_us', row, 0.0)))
            out['gpr_time_us'].append(to_float(col('gpr_time_us', row, 0.0)))
            out['pos_x'].append(to_float(col('pos_x', row, 0.0)))
            out['pos_y'].append(to_float(col('pos_y', row, 0.0)))
            out['pos_z'].append(to_float(col('pos_z', row, 0.0)))
            out['gpr_position'].append(to_float(col('gpr_position', row, 0.0)))
            out['gpr_velocity'].append(to_float(col('gpr_velocity', row, 0.0)))
            out['left_position'].append(to_float(col('left_position', row, 0.0)))
            out['right_position'].append(to_float(col('right_position', row, 0.0)))

    for key in ['fastlio_time_us', 'gpr_time_us', 'pos_x', 'pos_y', 'pos_z', 'gpr_position', 'gpr_velocity', 'left_position', 'right_position']:
        out[key] = np.asarray(out[key], dtype=float)
    return out


def moving_average_fastlio(scan: dict, window_size: int = 5) -> dict:
    """Return a copy of scan with pos_x/pos_y/pos_z filtered by moving average over valid Fast-LIO samples.

    - Uses a validity mask fastlio_time_us>0 to avoid averaging in zeros.
    - Edges handled via normalized convolution (value_conv / weight_conv).
    """
    filtered = dict(scan)
    valid = (scan.get('fastlio_time_us', np.array([])) > 0).astype(float)
    if valid.size == 0:
        return filtered

    def smooth(series: np.ndarray) -> np.ndarray:
        if series.size != valid.size or series.size == 0:
            return series
        w = np.ones(int(max(1, window_size)), dtype=float)
        val_conv = np.convolve(series * valid, w, mode='same')
        wt_conv = np.convolve(valid, w, mode='same')
        out = np.empty_like(series)
        nz = wt_conv > 0
        out[nz] = val_conv[nz] / wt_conv[nz]
        out[~nz] = series[~nz]
        return out

    for key in ['pos_x', 'pos_y', 'pos_z']:
        if key in filtered:
            filtered[key] = smooth(np.asarray(filtered[key], dtype=float))
    return filtered


def interpolate_fastlio_by_wheels(scan: dict, invert_right: bool = True) -> dict:
    """Fill missing Fast-LIO localization by piecewise linear interpolation in CSV order.

    - Define s = (left_position + ( -right_position if invert_right else right_position)) / 2.
    - Locate consecutive valid Fast-LIO samples (A=i0, B=i1) in original order.
    - For missing rows k in (i0, i1), compute x = (s[k]-s[i0])/(s[i1]-s[i0]) and
      pos[k] = pos[i0] + x*(pos[i1]-pos[i0]) for pos_x/pos_y/pos_z.
    - For leading/trailing gaps, clamp to nearest valid endpoint.
    """
    out = dict(scan)
    left = np.asarray(scan.get('left_position', np.array([])), dtype=float)
    right = np.asarray(scan.get('right_position', np.array([])), dtype=float)
    if left.size == 0 or right.size == 0:
        return out
    s = (left + ( -right if invert_right else right)) / 2.0

    ft = np.asarray(scan.get('fastlio_time_us', np.array([])), dtype=float)
    valid = (ft > 0)
    n = s.size
    if valid.size != n or n == 0:
        return out

    # Copy arrays to output for in-place edits
    for key in ['pos_x', 'pos_y', 'pos_z']:
        if key in out:
            out[key] = np.asarray(out[key], dtype=float).copy()

    idx_valid = np.where(valid)[0]
    if idx_valid.size == 0:
        return out

    # Leading gap: clamp to first valid
    first = int(idx_valid[0])
    if first > 0:
        for key in ['pos_x', 'pos_y', 'pos_z']:
            v0 = out[key][first]
            out[key][:first] = v0

    # Interpolate between consecutive valid indices
    for i in range(idx_valid.size - 1):
        i0 = int(idx_valid[i])
        i1 = int(idx_valid[i + 1])
        if i1 <= i0 + 1:
            continue  # no gap
        s0 = s[i0]
        s1 = s[i1]
        denom = (s1 - s0)
        eps = 1e-9
        for k in range(i0 + 1, i1):
            if valid[k]:
                continue
            if abs(denom) < eps:
                # Wheel progress didn't change; keep value at A (x=0), allowing repeats as requested
                x = 0.0
            else:
                x = float(np.clip((s[k] - s0) / denom, 0.0, 1.0))
            for key in ['pos_x', 'pos_y', 'pos_z']:
                y0 = out[key][i0]
                y1 = out[key][i1]
                out[key][k] = y0 + x * (y1 - y0)

    # Trailing gap: clamp to last valid
    last = int(idx_valid[-1])
    if last < n - 1:
        for key in ['pos_x', 'pos_y', 'pos_z']:
            vL = out[key][last]
            out[key][last + 1:] = vL

    return out


def relocate_fastlio_to_nearest_gpr(scan: dict) -> dict:
    """Relocate sparse Fast-LIO poses onto nearest GPR-timestamped rows.

    - Identify rows with fastlio_time_us>0 (sources) and rows with gpr_time_us>0 (targets).
    - For each source, find nearest target by timestamp and assign its pos_x/pos_y/pos_z to that target.
    - Clear pos_x/pos_y/pos_z at all non-target rows (including original source rows).
    - If multiple sources map to the same target, keep the closest (by |dt|).
    """
    out = dict(scan)
    ft = np.asarray(scan.get('fastlio_time_us', np.array([])), dtype=float)
    gt = np.asarray(scan.get('gpr_time_us', np.array([])), dtype=float)
    pos_x = np.asarray(scan.get('pos_x', np.array([])), dtype=float)
    pos_y = np.asarray(scan.get('pos_y', np.array([])), dtype=float)
    pos_z = np.asarray(scan.get('pos_z', np.array([])), dtype=float)

    n = gt.size
    if n == 0 or ft.size == 0:
        return out

    src_idx = np.where(np.isfinite(ft) & (ft > 0))[0]
    tgt_idx = np.where(np.isfinite(gt) & (gt > 0))[0]
    if src_idx.size == 0 or tgt_idx.size == 0:
        return out

    gt_valid = gt[tgt_idx]
    order = np.argsort(gt_valid)
    gt_sorted = gt_valid[order]
    tgt_sorted = tgt_idx[order]

    # Compute nearest target and distance for each source
    pairs = []  # (abs_dt, src, tgt)
    for s in src_idx:
        t = ft[s]
        j = int(np.searchsorted(gt_sorted, t, side='left'))
        if j <= 0:
            sel = 0
        elif j >= gt_sorted.size:
            sel = gt_sorted.size - 1
        else:
            sel = j if abs(gt_sorted[j] - t) < abs(gt_sorted[j-1] - t) else (j - 1)
        tgt = int(tgt_sorted[sel])
        pairs.append((abs(gt_sorted[sel] - t), int(s), tgt))

    # Resolve collisions by choosing smallest |dt| first
    pairs.sort(key=lambda x: x[0])
    chosen_tgt = set()
    assignment = []  # (src, tgt)
    for _, s, t in pairs:
        if t in chosen_tgt:
            continue
        chosen_tgt.add(t)
        assignment.append((s, t))

    # Build new arrays: zeros everywhere, fill at assigned targets
    px_new = np.zeros(n, dtype=float)
    py_new = np.zeros(n, dtype=float)
    pz_new = np.zeros(n, dtype=float)
    ft_new = np.zeros(n, dtype=float)
    for s, t in assignment:
        if s < pos_x.size and t < n:
            px_new[t] = float(pos_x[s])
            py_new[t] = float(pos_y[s])
            pz_new[t] = float(pos_z[s])
            if s < ft.size:
                ft_new[t] = float(ft[s])

    out['pos_x'] = px_new
    out['pos_y'] = py_new
    out['pos_z'] = pz_new
    out['fastlio_time_us'] = ft_new
    return out

def find_event_indices(events, start_label: str = 'GPR_MOTOR_START', stop_label: str = 'MOTOR_STOPPING'):
    """Return indices (start_idx, stop_idx) of first occurrence of the given events; -1 if not found.

    Note: Kept for compatibility, but not used in global indexing mode.
    """
    start_idx = next((i for i, e in enumerate(events) if e == start_label), -1)
    stop_idx = next((i for i, e in enumerate(events) if e == stop_label), -1)
    return start_idx, stop_idx


def compute_initial_from_first_five(data: dict) -> float:
    """Compute initial GPR position as the average of the first five gpr_position values.

    Falls back to mean of available values; returns 0.0 if none are present.
    """
    gpr_pos = np.asarray(data.get('gpr_position', np.array([])), dtype=float)
    if gpr_pos.size == 0:
        return 0.0
    n = int(min(5, gpr_pos.size))
    return float(np.mean(gpr_pos[:n]))


def assign_locations_by_global_progress(matrix_T: np.ndarray,
                                        scan: dict,
                                        initial_gpr_pos: float,
                                        wheel_radius_m: float = 0.03,
                                        gear_ratio: float = 1.0,
                                        spacing_m: float = 0.005) -> np.ndarray:
    """Assign XYZ per A-scan by global gpr_position indexing.

    - For each CSV row k, compute trace index idx_k = floor(max(0, (gpr[k]-initial)/motor_turns_per_trace)).
    - motor_turns_per_trace = gear_ratio * (spacing_m / (2*pi*wheel_radius_m)).
    - For each trace i, choose the earliest CSV row with idx_k == i.
    - If missing, carry forward the last available pose.
    Returns array of shape (n_cols, 3).
    """
    n_cols = int(matrix_T.shape[1]) if matrix_T.ndim == 2 else 0
    if n_cols <= 0:
        return np.zeros((0, 3), dtype=float)

    gpr = np.asarray(scan.get('gpr_position', np.array([])), dtype=float)
    xs = np.asarray(scan.get('pos_x', np.array([])), dtype=float)
    ys = np.asarray(scan.get('pos_y', np.array([])), dtype=float)
    zs = np.asarray(scan.get('pos_z', np.array([])), dtype=float)

    if gpr.size == 0 or xs.size == 0 or ys.size == 0:
        # Fallback to zeros
        return np.zeros((n_cols, 3), dtype=float)

    circumference = 2.0 * np.pi * float(wheel_radius_m if wheel_radius_m else 1.0)
    wheel_revs_per_trace = float(spacing_m) / (circumference if circumference > 0 else 1.0)
    motor_revs_per_trace = float(gear_ratio if gear_ratio else 1.0) * wheel_revs_per_trace
    if motor_revs_per_trace <= 0 or not np.isfinite(motor_revs_per_trace):
        motor_revs_per_trace = 1.0

    idxs = np.floor(np.maximum(0.0, (gpr - float(initial_gpr_pos)) / motor_revs_per_trace)).astype(int)

    selected = np.full(n_cols, -1, dtype=int)
    for k, idx in enumerate(idxs):
        if 0 <= idx < n_cols and selected[idx] == -1:
            selected[idx] = k

    locs = np.zeros((n_cols, 3), dtype=float)
    # Seed with first available row if any
    first_row = int(selected[0]) if selected[0] >= 0 else 0
    locs[0, 0] = float(xs[first_row]) if first_row < xs.size else 0.0
    locs[0, 1] = float(ys[first_row]) if first_row < ys.size else 0.0
    locs[0, 2] = float(zs[first_row]) if first_row < zs.size else 0.0

    for i in range(1, n_cols):
        sel = int(selected[i])
        if sel >= 0 and sel < xs.size and sel < ys.size and sel < zs.size:
            locs[i, 0] = float(xs[sel])
            locs[i, 1] = float(ys[sel])
            locs[i, 2] = float(zs[sel])
        else:
            # carry forward last known
            locs[i, :] = locs[i - 1, :]
    return locs

def write_processed_csv(input_csv_path: str, scan: dict, output_csv_path: str = None) -> str:
    """Write a CSV similar to the input schema, populated with filtered/interpolated Fast-LIO values.

    Columns written:
      event, fastlio_time_us, gpr_time_us,
      pos_x, pos_y, pos_z,
      gpr_position, gpr_velocity,
      left_position, right_position

    Returns the output CSV path.
    """
    import csv

    if output_csv_path is None:
        base, _ = os.path.splitext(input_csv_path)
        output_csv_path = base + "_postproc.csv"

    n = len(scan.get('event', []))
    cols = [
        'event', 'fastlio_time_us', 'gpr_time_us',
        'pos_x', 'pos_y', 'pos_z',
        'gpr_position', 'gpr_velocity',
        'left_position', 'right_position',
    ]

    with open(output_csv_path, 'w', newline='') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(cols)
        for i in range(n):
            event = scan['event'][i] if i < len(scan['event']) else ''
            ft = scan['fastlio_time_us'][i] if i < len(scan['fastlio_time_us']) else 0
            gt = scan['gpr_time_us'][i] if i < len(scan['gpr_time_us']) else 0
            px = scan['pos_x'][i] if i < len(scan['pos_x']) else 0.0
            py = scan['pos_y'][i] if i < len(scan['pos_y']) else 0.0
            pz = scan['pos_z'][i] if i < len(scan['pos_z']) else 0.0
            gp = scan['gpr_position'][i] if i < len(scan['gpr_position']) else 0.0
            gv = scan['gpr_velocity'][i] if i < len(scan['gpr_velocity']) else 0.0
            lp = scan['left_position'][i] if i < len(scan['left_position']) else 0.0
            rp = scan['right_position'][i] if i < len(scan['right_position']) else 0.0

            writer.writerow([
                event,
                int(ft) if np.isfinite(ft) else 0,
                int(gt) if np.isfinite(gt) else 0,
                f"{px:.6f}", f"{py:.6f}", f"{pz:.6f}",
                f"{gp:.6f}", f"{gv:.6f}",
                f"{lp:.6f}", f"{rp:.6f}",
            ])

    return output_csv_path


def _import_matplotlib_pyplot():
    try:
        import matplotlib.pyplot as plt  # type: ignore
        return plt
    except Exception as e:
        print(f"Matplotlib unavailable ({e}); skipping plot.")
        return None


def plot_radargram_with_pose_per_trace(A_T: np.ndarray,
                                       twt: np.ndarray,
                                       cdp: np.ndarray,
                                       locs_xyz: np.ndarray,
                                       title: str = "GPR Radargram with Pose per Trace",
                                       save_path: str = None,
                                       interactive: bool = False) -> None:
    """Plot radargram (TWT x trace index/CDP) and overlay px/py/pz at each trace.

    - x-axis: trace index/CDP (no along-track distance).
    - Underlay: A_T image (twt vs trace).
    - Overlay: px, py, pz curves sampled at each trace column.
    """
    plt = _import_matplotlib_pyplot()
    if plt is None:
        return
    if A_T.size == 0 or twt.size == 0:
        print("Nothing to plot: empty A_T/twt.")
        return

    # x-axis: prefer cdp if available; else indices
    if cdp is None or np.asarray(cdp).size != A_T.shape[1]:
        x = np.arange(A_T.shape[1], dtype=float)
    else:
        x = np.asarray(cdp, dtype=float)

    v = np.percentile(np.abs(A_T), 98) if np.isfinite(A_T).any() else 1.0
    fig, ax = plt.subplots(figsize=(12, 6))
    im = ax.imshow(
        A_T,
        aspect="auto",
        cmap="gray",
        vmin=-v,
        vmax=v,
        extent=[float(x[0]), float(x[-1]), float(twt[-1]), float(twt[0])],
    )
    ax.set_xlabel("Trace (CDP)")
    ax.set_ylabel("TWT (ms)")
    ax.set_title(title)
    fig.colorbar(im, ax=ax, label="Amplitude")

    if locs_xyz is not None and locs_xyz.size > 0:
        # If locs length mismatches, resample/trim to match columns
        m = A_T.shape[1]
        pose = np.asarray(locs_xyz, dtype=float)
        if pose.shape[0] != m:
            if pose.shape[0] > 0:
                idx = np.linspace(0, pose.shape[0] - 1, num=m)
                px = np.interp(idx, np.arange(pose.shape[0]), pose[:, 0])
                py = np.interp(idx, np.arange(pose.shape[0]), pose[:, 1])
                pz = np.interp(idx, np.arange(pose.shape[0]), pose[:, 2] if pose.shape[1] > 2 else np.zeros(pose.shape[0]))
            else:
                px = np.zeros(m)
                py = np.zeros(m)
                pz = np.zeros(m)
        else:
            px, py = pose[:, 0], pose[:, 1]
            pz = pose[:, 2] if pose.shape[1] > 2 else np.zeros(m)

        ax2 = ax.twinx()
        ax2.plot(x, px, 'r-', linewidth=1.0, label='px (m)')
        ax2.plot(x, py, 'g-', linewidth=1.0, label='py (m)')
        ax2.plot(x, pz, 'b-', linewidth=1.0, label='pz (m)')
        ax2.set_ylabel("Position (m)")
        ax2.legend(loc='upper right')

        if interactive:
            # Initialize selection markers and text readout
            sel_line = ax.axvline(x[0], color='y', linewidth=0.8, alpha=0.7)
            info = ax2.text(0.01, 0.98,
                            f"trace=0  px={px[0]:.4f}  py={py[0]:.4f}  pz={pz[0]:.4f}",
                            transform=ax2.transAxes, va='top', ha='left',
                            fontsize=9, bbox=dict(boxstyle='round', fc='white', ec='0.7', alpha=0.8))

            def on_move(event):
                if event.inaxes not in (ax, ax2) or event.xdata is None:
                    return
                # Nearest trace by x
                j = int(np.argmin(np.abs(x - float(event.xdata))))
                j = max(0, min(j, m - 1))
                sel_line.set_xdata([x[j], x[j]])
                info.set_text(f"trace={j}  px={px[j]:.4f}  py={py[j]:.4f}  pz={pz[j]:.4f}")
                fig.canvas.draw_idle()

            fig.canvas.mpl_connect('motion_notify_event', on_move)

    plt.tight_layout()
    if save_path is not None and len(str(save_path)) > 0:
        try:
            plt.savefig(save_path, dpi=150)
            print(f"Saved plot: {save_path}")
        except Exception as e:
            print(f"Failed to save plot '{save_path}': {e}")
    else:
        plt.show()

def main():
    parser = argparse.ArgumentParser(description="GPR post-processing: load SEG-Y to A.T and parse scan CSV")
    parser.add_argument('segy_file', help='Path to SEG-Y file (.sgy)')
    parser.add_argument('scan_csv', nargs='?', default=None, help='Optional scan CSV from gpr_scan_controller')
    # Localization and thresholds
    parser.add_argument('--spacing_m', type=float, default=0.005, help='Along-track spacing per A-scan (m), e.g., 0.005 for 5 mm')
    parser.add_argument('--wheel_radius_m', type=float, default=0.03, help='Drive wheel radius for angle-to-distance (m)')
    parser.add_argument('--gear_ratio', type=float, default=1.0, help='Gear ratio motor->wheel (dimensionless)')
    parser.add_argument('--pre_window', type=int, default=5, help='Samples before start event to average initial pose')
    parser.add_argument('--post_window', type=int, default=5, help='Samples after stop event to average stopping pose')
    parser.add_argument('--start_label', type=str, default='GPR_MOTOR_START', help='Start event label in CSV')
    parser.add_argument('--stop_label', type=str, default='MOTOR_STOPPING', help='Stop event label in CSV')
    # Plotting
    parser.add_argument('--plot', action='store_true', help='Display radargram overlaid with px/py/pz per trace')
    parser.add_argument('--save_plot', type=str, default=None, help='Optional path to save the per-trace overlaid plot (PNG)')
    parser.add_argument('--interactive', action='store_true', help='Enable interactive px/py/pz readout by cursor (per trace)')
    args = parser.parse_args()

    if not os.path.isfile(args.segy_file):
        print(f"SEG-Y not found: {args.segy_file}")
        sys.exit(1)

    ds, da = load_with_segysak(args.segy_file)
    A = da.values              # shape (cdp, twt)
    A_T = A.T                  # shape (twt, cdp)
    cdp = da.coords.get("cdp", np.arange(A.shape[0])).values
    twt = da.coords.get("twt", np.arange(A.shape[1])).values

    print(f"Loaded SEG-Y: A shape (cdp,twt)={A.shape}, A.T shape (twt,cdp)={A_T.shape}")
    print(f"Axes: cdp len={cdp.size}, twt len={twt.size}")

    if args.scan_csv is not None and os.path.isfile(args.scan_csv):
        scan = load_scan_csv(args.scan_csv)
        print(f"Loaded scan CSV: rows={len(scan['event'])}")
        print(f"Available keys: {list(scan.keys())}")
        # 0) Relocate sparse Fast-LIO data to nearest GPR timestamp rows
        scan_relocated = relocate_fastlio_to_nearest_gpr(scan)
        # 1) Moving window average on Fast-LIO localization (disabled)
        scan_filt = scan_relocated
        # 2) Interpolate missing Fast-LIO poses by wheel encoder average (invert right)
        scan_proc = interpolate_fastlio_by_wheels(scan_filt, invert_right=True)
        # Brief summary
        valid_before = int(np.sum(np.asarray(scan.get('fastlio_time_us', np.array([]))) > 0))
        valid_after = int(np.sum(np.asarray(scan_proc.get('fastlio_time_us', np.array([]))) > 0))
        print(f"Fast-LIO valid rows (time_us>0): before={valid_before}, after={valid_after} (timestamps unchanged)")

        # 3) Global mode: compute initial from first five and map all rows to traces via global gpr_position
        initial_gpr_pos = compute_initial_from_first_five(scan_proc)
        print(f"Initial GPR position (avg first 5): {initial_gpr_pos:.6f}")
        circumference = 2.0 * np.pi * float(args.wheel_radius_m if args.wheel_radius_m else 1.0)
        wheel_revs_per_trace = float(args.spacing_m) / (circumference if circumference > 0 else 1.0)
        motor_revs_per_trace = float(args.gear_ratio if args.gear_ratio else 1.0) * wheel_revs_per_trace
        gpr = np.asarray(scan_proc.get('gpr_position', np.array([])), dtype=float)
        if gpr.size > 0 and motor_revs_per_trace > 0:
            total_revs = float(np.maximum(0.0, np.max(gpr) - initial_gpr_pos))
            expected_ascans = int(np.floor(total_revs / motor_revs_per_trace))
        else:
            expected_ascans = 0
        actual_ascans = int(A.shape[0])
        print("---- A-scan Counts (Global) ----")
        print(f"Expected (~{args.spacing_m*1000:.0f} mm): {expected_ascans}")
        print(f"Actual (SEG-Y CDP traces): {actual_ascans}")

        # Assign XYZ per A-scan using global progress
        locs_xyz = assign_locations_by_global_progress(
            A_T,
            scan_proc,
            initial_gpr_pos,
            wheel_radius_m=args.wheel_radius_m,
            gear_ratio=args.gear_ratio,
            spacing_m=args.spacing_m,
        )

        # 4) Save A.T matrix with px/py/pz appended as bottom rows
        base, _ = os.path.splitext(args.segy_file)
        out_csv_at = base + "_AT_with_xyz.csv"
        header_cols = "twt," + ",".join(str(v) for v in cdp)
        # Compose matrix: first column twt, then amplitude columns; append 3 rows for px/py/pz (NaN in first column)
        matrix_body = np.column_stack((twt, A_T))
        px = locs_xyz[:, 0] if locs_xyz.size > 0 else np.zeros((A_T.shape[1],), dtype=float)
        py = locs_xyz[:, 1] if locs_xyz.size > 0 else np.zeros((A_T.shape[1],), dtype=float)
        pz = locs_xyz[:, 2] if locs_xyz.size > 0 else np.zeros((A_T.shape[1],), dtype=float)
        px_row = np.concatenate(([np.nan], px))
        py_row = np.concatenate(([np.nan], py))
        pz_row = np.concatenate(([np.nan], pz))
        out_mat = np.vstack((matrix_body, px_row[None, :], py_row[None, :], pz_row[None, :]))

        # Use csv writer to avoid scientific notation issues and to keep header
        import csv as _csv
        with open(out_csv_at, 'w', newline='') as f:
            writer = _csv.writer(f, lineterminator='\n')
            writer.writerow(header_cols.split(','))
            for r in out_mat:
                writer.writerow([f"{v:.10g}" if np.isfinite(v) else "" for v in r])
        print(f"Saved A.T with XYZ CSV: {out_csv_at}")

        if args.plot or (args.save_plot is not None and len(str(args.save_plot)) > 0):
            plot_title = f"GPR Radargram (A.T) with Pose per Trace (~{args.spacing_m*1000:.0f} mm spacing)"
            plot_radargram_with_pose_per_trace(A_T, twt, cdp, locs_xyz, title=plot_title, save_path=args.save_plot, interactive=args.interactive)

        # Also write processed localization CSV for reference
        out_csv = write_processed_csv(args.scan_csv, scan_proc)
        print(f"Saved processed CSV: {out_csv}")
    else:
        print("No scan CSV provided or file not found; skipping CSV load.")


if __name__ == '__main__':
    main()


