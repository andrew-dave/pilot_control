#!/usr/bin/env python3
# segy_explorer.py
# Usage: python segy_explorer.py your_file.sgy

import sys
import os
import argparse
import numpy as np
# Matplotlib is imported lazily inside plotting functions to avoid
# environment issues where system matplotlib is incompatible with numpy.
import csv  
try:
    import segysak.segy as sgy
except Exception:
    sgy = None

"""Minimal SEG-Y extractor: save radargram matrix to CSV and NPZ."""

def load_with_segysak(fname):
    if sgy is None:
        raise RuntimeError("segysak not installed. Install via conda-forge: conda install -c conda-forge segysak")
    ds = sgy.segy_loader(fname)  # keep using this; robust and simple
    # Pick main data var
    data_var = next(iter(ds.data_vars))
    da = ds[data_var]
    # Ensure (cdp, twt)
    if da.dims == ("twt", "cdp"):
        da = da.transpose("cdp", "twt")
    return ds, da


def _import_matplotlib_pyplot():
    try:
        import matplotlib.pyplot as plt  # type: ignore
        return plt
    except Exception as e:
        print(f"Matplotlib unavailable ({e}); skipping plot.")
        return None


def plot_radargram(da, title="GPR Radargram"):
    plt = _import_matplotlib_pyplot()
    if plt is None:
        return
    A = da.values
    cdp = da.coords.get("cdp", np.arange(A.shape[0])).values
    twt = da.coords.get("twt", np.arange(A.shape[1])).values

    v = np.percentile(np.abs(A), 98)

    plt.figure(figsize=(12, 6))
    im = plt.imshow(
        A.T,
        aspect="auto",
        cmap="gray",
        vmin=-v,
        vmax=v,
        extent=[cdp[0], cdp[-1], twt[-1], twt[0]],  # invert Y so time increases downward
    )
    plt.xlabel("CDP (trace index / horizontal position)")
    plt.ylabel("TWT (ms)")
    plt.title(title)
    plt.colorbar(im, label="Amplitude")
    plt.tight_layout()
    plt.show()


def compute_along_track_distances(xy):
    if xy.size == 0:
        return np.zeros((0,), dtype=float)
    diffs = np.diff(np.asarray(xy, dtype=float), axis=0)
    seg = np.hypot(diffs[:, 0], diffs[:, 1])
    s = np.concatenate(([0.0], np.cumsum(seg)))
    return s


def plot_matrix_with_distance(A_T, twt, distances, title="", xlabel="Distance (m)"):
    plt = _import_matplotlib_pyplot()
    if plt is None:
        return
    if A_T.size == 0 or distances.size == 0 or twt.size == 0:
        print("Nothing to plot: empty data.")
        return
    v = np.percentile(np.abs(A_T), 98)
    plt.figure(figsize=(12, 6))
    im = plt.imshow(
        A_T,
        aspect="auto",
        cmap="gray",
        vmin=-v,
        vmax=v,
        extent=[float(distances[0]), float(distances[-1]), float(twt[-1]), float(twt[0])],
    )
    plt.xlabel(xlabel)
    plt.ylabel("TWT (ms)")
    plt.title(title)
    plt.colorbar(im, label="Amplitude")
    plt.tight_layout()
    plt.show()


def load_scan_csv(csv_path):
    """Load gpr_scan_controller CSV and return a dict of columns.

    - Detects the header row anywhere in the file (e.g., row 24), skipping comments
      and empty lines until it finds required column names.
    - Returns keys: 'event' (list[str]), 'fastlio_time_us', 'gpr_time_us',
      'pos_x','pos_y','pos_z','gpr_position','gpr_velocity'. Missing fields
      default to zeros/empty.
    """
    out = {
        'event': [],
        'fastlio_time_us': [],
        'gpr_time_us': [],
        'pos_x': [], 'pos_y': [], 'pos_z': [],
        'gpr_position': [], 'gpr_velocity': []
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
            # Normalize row values
            row = [c.strip() for c in row]
            # Skip empty rows entirely
            if not row or all(c == '' for c in row):
                continue
            # Identify header if not yet found
            if header is None:
                # Skip comment lines
                if row[0].startswith('#'):
                    continue
                # Heuristic: header must include required columns
                required = {'event', 'pos_x', 'pos_y', 'gpr_position'}
                if required.issubset(set(row)):
                    header = row
                    name_to_idx = {name: i for i, name in enumerate(header)}
                    print(f"Detected CSV header at row {line_no}: {header}")
                    continue
                # Not a header yet; keep scanning
                continue

            # From here, we have a header; skip comment lines
            if row[0].startswith('#'):
                continue
            # Skip any repeated header rows found later
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
    # Convert lists to numpy arrays for numeric fields
    for key in ['fastlio_time_us', 'gpr_time_us', 'pos_x', 'pos_y', 'pos_z', 'gpr_position', 'gpr_velocity']:
        out[key] = np.asarray(out[key], dtype=float)
    return out


def find_event_indices(events, start_label='GPR_MOTOR_START', stop_label='MOTOR_STOPPING'):
    """Return indices (start_idx, stop_idx) of first occurrence of the given events; -1 if not found."""
    start_idx = next((i for i, e in enumerate(events) if e == start_label), -1)
    stop_idx = next((i for i, e in enumerate(events) if e == stop_label), -1)
    return start_idx, stop_idx


def compute_initial_and_stopping(data, start_idx, stop_idx, pre_window=5, post_window=5):
    """Compute initial GPR position and average XY around events.

    - initial_gpr_pos: mean gpr_position over the pre_window samples before start_idx
    - initial_xy: mean (x,y) over the same window
    - stop_xy: mean (x,y) over post_window samples after stop_idx

    This is a baseline function; tune window selection later.
    """
    n = len(data['event'])
    init_slice = slice(max(0, start_idx - pre_window), max(0, start_idx)) if start_idx > 0 else slice(0, 0)
    stop_slice = slice(min(n, stop_idx + 1), min(n, stop_idx + 1 + post_window)) if 0 <= stop_idx < n - 1 else slice(n, n)

    initial_gpr_pos = float(np.mean(data['gpr_position'][init_slice])) if (init_slice.stop - init_slice.start) > 0 else float(data['gpr_position'][start_idx] if start_idx >= 0 else 0.0)
    initial_xy = (
        float(np.mean(data['pos_x'][init_slice])) if (init_slice.stop - init_slice.start) > 0 else float(data['pos_x'][start_idx] if start_idx >= 0 else 0.0),
        float(np.mean(data['pos_y'][init_slice])) if (init_slice.stop - init_slice.start) > 0 else float(data['pos_y'][start_idx] if start_idx >= 0 else 0.0),
    )
    stopping_xy = (
        float(np.mean(data['pos_x'][stop_slice])) if (stop_slice.stop - stop_slice.start) > 0 else float(data['pos_x'][stop_idx] if stop_idx >= 0 else 0.0),
        float(np.mean(data['pos_y'][stop_slice])) if (stop_slice.stop - stop_slice.start) > 0 else float(data['pos_y'][stop_idx] if stop_idx >= 0 else 0.0),
    )
    return initial_gpr_pos, initial_xy, stopping_xy


def compute_heading_unit_vector(xs, ys):
    """Compute heading unit vector in XY plane by linear fitting y = m x + b.

    Returns a unit vector (dx, dy) pointing along the fitted line (increasing x direction).
    """
    if len(xs) < 2:
        return (1.0, 0.0)
    try:
        m, b = np.polyfit(xs, ys, 1)
        # Direction vector along the line: (1, m)
        v = np.array([1.0, float(m)])
        v = v / (np.linalg.norm(v) + 1e-12)
        return (float(v[0]), float(v[1]))
    except Exception:
        return (1.0, 0.0)


def generate_locations_along_heading(start_xy, stop_xy, spacing_m=0.005):
    """Generate points from start to near stop spaced by spacing_m along the straight line.

    Uses straight segment from start to stop; caller can replace with curved path later.
    """
    start = np.array(start_xy, dtype=float)
    stop = np.array(stop_xy, dtype=float)
    total = np.linalg.norm(stop - start)
    if not np.isfinite(total) or total <= 0.0:
        return np.array([start])
    num = max(1, int(np.floor(total / float(spacing_m))))
    ts = np.linspace(0.0, 1.0, num=num, endpoint=False)
    pts = start[None, :] + ts[:, None] * (stop - start)[None, :]
    return pts


def assign_locations_to_ascans(matrix, locations_xy):
    """Map each A-scan (column) to an XY location; trims to min(len, columns)."""
    num_cols = matrix.shape[1]
    k = min(num_cols, locations_xy.shape[0])
    return locations_xy[:k, :]


def compute_revs_per_ascan(wheel_radius_m=0.03, spacing_m=0.005):
    """Return wheel revolutions corresponding to spacing_m linear travel."""
    circumference = 2.0 * np.pi * float(wheel_radius_m)
    if circumference <= 0.0:
        return 0.0
    return float(spacing_m) / circumference


def assign_locations_by_revs(matrix,
                             scan,
                             start_idx,
                             stop_idx,
                             start_xy,
                             initial_gpr_pos,
                             wheel_radius_m=0.03,
                             gear_ratio=1.0,
                             localization_spacing_m=0.005):
    """Assign XY per A-scan using nearest CSV localization at wheel-turn thresholds.

    Baseline is the averaged initial GPR motor position (initial_gpr_pos). For each
    target cumulative wheel turns (spacing_m apart), pick the CSV row within the
    [start_idx:stop_idx] window whose wheel-turn offset is closest to that target,
    and use its (pos_x,pos_y). No interpolation is done.
    """
    cols = matrix.shape[1]
    if cols <= 0:
        return np.zeros((0, 2), dtype=float)
    lo = start_idx if start_idx is not None and start_idx >= 0 else 0
    hi = stop_idx + 1 if stop_idx is not None and stop_idx >= 0 else len(scan['gpr_position'])
    lo = max(0, min(lo, len(scan['gpr_position'])))
    hi = max(lo, min(hi, len(scan['gpr_position'])))
    if hi - lo < 1:
        return np.tile(np.asarray(start_xy, dtype=float), (cols, 1))

    motor_pos = np.asarray(scan['gpr_position'][lo:hi], dtype=float)
    if motor_pos.size == 0:
        return np.tile(np.asarray(start_xy, dtype=float), (cols, 1))

    # Convert to wheel turns; use averaged initial GPR motor position as baseline
    baseline_motor = float(initial_gpr_pos)
    wheel_turns = (motor_pos - baseline_motor) / float(gear_ratio if gear_ratio else 1.0)
    # Use absolute travel
    wheel_turns_abs = np.abs(wheel_turns)

    revs_per_ascan = compute_revs_per_ascan(wheel_radius_m=wheel_radius_m, spacing_m=localization_spacing_m)
    if revs_per_ascan <= 0.0:
        return np.tile(np.asarray(start_xy, dtype=float), (cols, 1))

    # Threshold crossing indices for each subsequent A-scan
    targets = revs_per_ascan * np.arange(1, cols + 1, dtype=float)
    locs = np.zeros((cols, 2), dtype=float)
    locs[0, :] = np.asarray(start_xy, dtype=float)

    # Build arrays of xy within the window for nearest selection
    xs = np.asarray(scan['pos_x'][lo:hi], dtype=float)
    ys = np.asarray(scan['pos_y'][lo:hi], dtype=float)

    # Monotonic index advancement using searchsorted; pick nearest neighbor
    k = 0
    for i, t in enumerate(targets[:cols-1], start=1):
        j = int(np.searchsorted(wheel_turns_abs, t, side='left'))
        if j <= 0:
            sel = 0
        elif j >= wheel_turns_abs.size:
            sel = wheel_turns_abs.size - 1
        else:
            # choose nearer of j-1 and j
            if abs(wheel_turns_abs[j] - t) < abs(wheel_turns_abs[j-1] - t):
                sel = j
            else:
                sel = j - 1
        locs[i, 0] = xs[sel]
        locs[i, 1] = ys[sel]
        k = i

    if k + 1 < cols:
        locs[k + 1:, :] = locs[k, :]
    return locs

def compute_effective_columns(matrix,
                              scan,
                              start_idx,
                              stop_idx,
                              initial_gpr_pos,
                              wheel_radius_m=0.03,
                              gear_ratio=1.0,
                              localization_spacing_m=0.005):
    """Compute how many columns are supported within the start/stop window.

    Uses cumulative wheel turns inside [start_idx:stop_idx] and the requested
    spacing to determine the number of distinct along-track thresholds reached.
    Returns an integer K in [1, matrix.shape[1]].
    """
    total_cols = int(matrix.shape[1])
    if total_cols <= 0:
        return 0
    lo = start_idx if start_idx is not None and start_idx >= 0 else 0
    hi = stop_idx + 1 if stop_idx is not None and stop_idx >= 0 else len(scan['gpr_position'])
    lo = max(0, min(lo, len(scan['gpr_position'])))
    hi = max(lo, min(hi, len(scan['gpr_position'])))
    if hi - lo < 1:
        return 1

    motor_pos = np.asarray(scan['gpr_position'][lo:hi], dtype=float)
    if motor_pos.size == 0:
        return 1
    baseline_motor = float(initial_gpr_pos)
    wheel_turns = (motor_pos - baseline_motor) / float(gear_ratio if gear_ratio else 1.0)
    wheel_turns_abs = np.abs(wheel_turns)
    if wheel_turns_abs.size == 0:
        return 1

    revs_per_ascan = compute_revs_per_ascan(wheel_radius_m=wheel_radius_m, spacing_m=localization_spacing_m)
    if not np.isfinite(revs_per_ascan) or revs_per_ascan <= 0.0:
        return 1

    max_revs = float(wheel_turns_abs.max())
    # Include the first column at the start position
    k = 1 + int(np.floor(max_revs / revs_per_ascan))
    return max(1, min(k, total_cols))

def build_output_matrix(amplitudes_matrix,
                        ascan_locations_xy,
                        interest_locations_xy,
                        spacing_m=0.0025):
    """Create weighted-averaged A-scans at the locations of interest.

    - amplitudes_matrix: shape (twt, Nascans)
    - ascan_locations_xy: shape (Nascans, 2) assigned per existing column
    - interest_locations_xy: shape (M, 2) target locations (5 mm grid)
    - spacing_m: influence radius; weights = max(0, 1 - d/spacing_m)

    Returns dict with 'amplitudes' (twt, M), 'locations_xy' (M,2), 'twt' (twt,).
    """
    if amplitudes_matrix.size == 0 or ascan_locations_xy.size == 0 or interest_locations_xy.size == 0:
        return {'amplitudes': np.zeros((0, 0)), 'locations_xy': np.zeros((0, 2)), 'twt': np.zeros((0,))}

    twt_len, n_cols = amplitudes_matrix.shape
    m = interest_locations_xy.shape[0]
    result = np.zeros((twt_len, m), dtype=amplitudes_matrix.dtype)

    asc_xy = np.asarray(ascan_locations_xy, dtype=float)
    int_xy = np.asarray(interest_locations_xy, dtype=float)
    R = float(spacing_m)

    for i in range(m):
        target = int_xy[i]
        # distances to all columns
        d = np.hypot(asc_xy[:, 0] - target[0], asc_xy[:, 1] - target[1])
        w = np.clip(1.0 - (d / R), 0.0, 1.0)
        if not np.any(w > 0):
            # fallback to nearest column
            j = int(np.argmin(d))
            w = np.zeros_like(d)
            w[j] = 1.0
        # weighted average across columns; broadcast weights
        ws = w.sum()
        if ws <= 0:
            ws = 1.0
        result[:, i] = (amplitudes_matrix * w[None, :]).sum(axis=1) / ws

    twt = np.arange(twt_len)
    return {
        'amplitudes': result,
        'locations_xy': int_xy,
        'twt': twt,
    }


def main():
    if len(sys.argv) < 2:
        # print("Usage: python segy_explorer.py your_file.sgy")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="SEG-Y extractor and GPR localization")
    parser.add_argument('segy_file', help='Path to SEG-Y file (.sgy)')
    parser.add_argument('scan_csv', nargs='?', default=None, help='Optional scan CSV for localization')
    # Localization and thresholds
    parser.add_argument('--localization_spacing_m', type=float, default=0.005, help='A-scan localization spacing based on wheel turns (m)')
    parser.add_argument('--interest_spacing_m', type=float, default=0.001, help='Interest point spacing and averaging window (m)')
    parser.add_argument('--wheel_radius_m', type=float, default=0.03, help='Drive wheel radius for revolutions-to-distance (m)')
    parser.add_argument('--gear_ratio', type=float, default=1.0, help='Gear ratio from motor to wheel (dimensionless)')
    parser.add_argument('--pre_window', type=int, default=5, help='Samples before start event to average initial pose')
    parser.add_argument('--post_window', type=int, default=5, help='Samples after stop event to average stopping pose')
    parser.add_argument('--start_label', type=str, default='GPR_MOTOR_START', help='Start event label in CSV')
    parser.add_argument('--stop_label', type=str, default='MOTOR_STOPPING', help='Stop event label in CSV')
    parser.add_argument('--placement', type=str, choices=['fixed', 'revs'], default='fixed', help="XY assignment method: 'fixed' uses constant scan spacing; 'revs' uses wheel turns")
    parser.add_argument('--no_average', action='store_true', help='Disable spatial averaging; output raw traces within start/stop window')

    args = parser.parse_args()

    fname = args.segy_file
    csv_path = args.scan_csv
    ds, da = load_with_segysak(fname)
    
    A = da.values
    cdp = da.coords.get("cdp", np.arange(A.shape[0])).values
    twt = da.coords.get("twt", np.arange(A.shape[1])).values
    # Base path for output files
    base, _ = os.path.splitext(fname)

    # Save full radargram matrix
    # CSV: rows are TWT samples, columns are CDP traces; first column is TWT
    matrix_csv = base + "_radargram_matrix.csv"
    header_cols = "twt," + ",".join(str(v) for v in cdp)
    matrix = np.column_stack((twt, A.T))
    np.savetxt(matrix_csv, matrix, delimiter=",", header=header_cols, comments="")
    print(f"Saved radargram matrix CSV: {matrix_csv}")

    # Skipping NPZ save; CSV saved above in `matrix_csv`.

    # If a scan CSV is provided, run localization mapping pipeline
    if csv_path is not None and os.path.isfile(csv_path):
        scan = load_scan_csv(csv_path)
        start_idx, stop_idx = find_event_indices(scan['event'], start_label=args.start_label, stop_label=args.stop_label)

        initial_gpr_pos, initial_xy, stopping_xy = compute_initial_and_stopping(scan, start_idx, stop_idx, pre_window=args.pre_window, post_window=args.post_window)
        print(f"Initial GPR pos: {initial_gpr_pos:.6f}")
        print(f"Initial XY avg: {initial_xy}")
        print(f"Stopping XY avg: {stopping_xy}")
        # Parameter echo for traceability
        print("Localization parameters:")
        print(f"  localization_spacing_m={args.localization_spacing_m}")
        print(f"  interest_spacing_m={args.interest_spacing_m}")
        print(f"  wheel_radius_m={args.wheel_radius_m}")
        print(f"  gear_ratio={args.gear_ratio}")
        print(f"  pre_window={args.pre_window}, post_window={args.post_window}")
        print(f"  start_label='{args.start_label}', stop_label='{args.stop_label}'")

        # Fit heading over the SCANNING window (from start_idx to stop_idx)
        lo = start_idx if start_idx >= 0 else 0
        hi = stop_idx if 0 <= stop_idx <= len(scan['pos_x']) else len(scan['pos_x'])
        heading_dxdy = compute_heading_unit_vector(scan['pos_x'][lo:hi], scan['pos_y'][lo:hi])
        print(f"Heading unit vector (dx,dy): {heading_dxdy}")

        # Assign locations by revolutions per A-scan using localization spacing
        asc_locs = assign_locations_by_revs(
            A.T,
            scan,
            start_idx,
            stop_idx,
            initial_xy,
            initial_gpr_pos=initial_gpr_pos,
            wheel_radius_m=args.wheel_radius_m,
            gear_ratio=args.gear_ratio,
            localization_spacing_m=args.localization_spacing_m,
        )
        # Compose output depending on averaging preference
        if args.no_average:
            # Determine how many columns are supported by the motion window
            k_eff = compute_effective_columns(
                A.T,
                scan,
                start_idx,
                stop_idx,
                initial_gpr_pos,
                wheel_radius_m=args.wheel_radius_m,
                gear_ratio=args.gear_ratio,
                localization_spacing_m=args.localization_spacing_m,
            )
            raw_amplitudes = A.T[:, :k_eff] if k_eff > 0 else np.zeros((A.T.shape[0], 0), dtype=A.dtype)
            raw_locations = asc_locs[:k_eff, :] if k_eff > 0 else np.zeros((0, 2), dtype=float)
            result = {
                'amplitudes': raw_amplitudes,
                'locations_xy': raw_locations,
                'twt': np.arange(A.T.shape[0]),
            }
        else:
            # Distance-weighted average onto along-track interest locations
            interest_locs = generate_locations_along_heading(initial_xy, stopping_xy, spacing_m=args.interest_spacing_m)
            result = build_output_matrix(A.T, asc_locs, interest_locs, spacing_m=args.interest_spacing_m)

        # Stats: counts and amplitude ranges before/after processing
        pre_min = float(np.min(A.T)) if A.size > 0 else float('nan')
        pre_max = float(np.max(A.T)) if A.size > 0 else float('nan')
        num_initial_traces = int(A.shape[0])
        num_localized_points = int(result['amplitudes'].shape[1]) if result['amplitudes'].size > 0 else 0
        if result['amplitudes'].size > 0:
            post_min = float(np.min(result['amplitudes']))
            post_max = float(np.max(result['amplitudes']))
        else:
            post_min = float('nan')
            post_max = float('nan')

        print("---- GPR Localization Stats ----")
        print(f"Initial A-scan/CDP count: {num_initial_traces}")
        print(f"Localized points considered: {num_localized_points}")
        print(f"Amplitude min/max before: {pre_min:.6g} / {pre_max:.6g}")
        print(f"Amplitude min/max after:  {post_min:.6g} / {post_max:.6g}")
        # Save localized results as CSVs (amplitudes and locations)
        localized_ampl_csv = base + "_localized_amplitudes.csv"
        m = result['amplitudes'].shape[1]
        header_cols = "twt," + ",".join(f"loc{i}" for i in range(m))
        ampl_mat = np.column_stack((result['twt'], result['amplitudes']))
        np.savetxt(localized_ampl_csv, ampl_mat, delimiter=",", header=header_cols, comments="")
        print(f"Saved localized amplitudes CSV: {localized_ampl_csv}")

        localized_xy_csv = base + "_localized_locations_xy.csv"
        loc_idx = np.arange(m, dtype=int)
        loc_mat = np.column_stack((loc_idx, result['locations_xy']))
        np.savetxt(localized_xy_csv, loc_mat, delimiter=",", header="loc_index,x,y", comments="")
        print(f"Saved localized locations CSV: {localized_xy_csv}")

        # Plot localized radargram using along-track distance on x-axis
        distances = compute_along_track_distances(result['locations_xy'])
        plot_matrix_with_distance(
            result['amplitudes'],
            result['twt'],
            distances,
            title=f"GPR Radargram (Localized, ~{args.interest_spacing_m*1000:.0f} mm along-track)",
            xlabel="Along-track distance (m)",
        )

    # Display radargram
    plot_radargram(da, title="GPR Radargram")

if __name__ == "__main__":
    main()
