#!/usr/bin/env python3
"""
gpr_to_segy.py

Export GPR A-scans (from gpr_post_processing.py's *_AT_with_xyz.csv or
gpr_scan_reconstruction.py's *_reconstructed.csv) to SEG-Y with per-trace
positional headers for import into Geolitix.

Two coordinate frames:
  1. LIDAR/ODOM frame  -> <base>_lidar.sgy   (always produced)
     Per-trace px/py taken directly from the input CSV (Fast-LIO odom frame).
  2. GPS/UTM frame     -> <base>_gps.sgy     (only with --visual_csv)
     A 2D similarity transform odom->UTM is fitted from paired
     (odom px/py, gps lat/lon) rows in the unified_data_collector dataset CSV,
     then applied to every A-scan's lidar XY. The lidar supplies the dense,
     smooth, drift-free shape; GPS anchors it to Earth ("correct GPS using
     lidar odometry"). No UBX/PPK required -- both CSVs share the same ROS
     clock and odom frame, so the odom pose is the join key.

Coordinates are packed into trace headers as integers with SourceGroupScalar
= -100 (i.e. stored x100, header says divide-by-100 -> cm resolution).
cdpx/cdpy, sx/sy and gx/gy are all populated; CoordinateUnits = 1 (length).

Usage:
  python3 gpr_to_segy.py AT_with_xyz.csv --twt_sample_interval_us 100 \
      [--visual_csv dataset_xxx.csv] [--estimate_scale] [--out_base /path/section]

Dependencies: numpy, segyio, pyproj (pyproj only needed for the GPS export).
"""

import sys
import os
import csv
import shutil
import argparse
import numpy as np

try:
    import segyio
except Exception:
    segyio = None

try:
    from pyproj import Transformer, CRS
except Exception:
    Transformer = None
    CRS = None


# Coordinates stored as integers x100; SEG-Y scalar -100 => divide by 100.
COORD_SCALAR = -100
COORD_MULT = 100.0


# ----------------------------------------------------------------------------
# Input loading
# ----------------------------------------------------------------------------
def load_at_with_xyz_csv(path: str) -> dict:
    """Load A.T-with-XYZ CSV (gpr_post_processing.py / gpr_scan_reconstruction.py).

    First column is TWT; columns 1..N are trace amplitudes; the bottom three
    rows hold px/py/pz per trace (first cell blank/NaN). Works for both the
    *_AT_with_xyz.csv and *_reconstructed.csv layouts.

    Returns dict: 'twt' (twt_len,), 'A_T' (twt_len, N), 'px','py','pz' (N,).
    """
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [r for r in reader]

    def to_float(x):
        try:
            if x is None or x == '':
                return float('nan')
            return float(x)
        except Exception:
            return float('nan')

    M = np.array([[to_float(c) for c in r] for r in rows], dtype=float)
    if M.size == 0 or M.shape[0] < 4:
        raise RuntimeError("Input CSV empty or missing appended px/py/pz rows.")

    twt_len = M.shape[0] - 3
    twt = M[:twt_len, 0]
    A_T = M[:twt_len, 1:]
    px = M[twt_len + 0, 1:]
    py = M[twt_len + 1, 1:]
    pz = M[twt_len + 2, 1:]
    if not (A_T.shape[1] == px.size == py.size == pz.size):
        raise RuntimeError(
            f"Trace/position count mismatch: A_T cols={A_T.shape[1]}, "
            f"px={px.size}, py={py.size}, pz={pz.size}")
    return {'twt': twt, 'A_T': A_T, 'px': px, 'py': py, 'pz': pz, 'header': header}


def load_visual_csv(path: str) -> dict:
    """Load unified_data_collector dataset CSV; return paired odom/GPS samples.

    Required columns: px, py, gps_lat, gps_lon, gps_status, gps_h_acc.
    Returns dict of numpy arrays for those columns (all rows, unfiltered).
    """
    want = ['px', 'py', 'gps_lat', 'gps_lon', 'gps_status', 'gps_h_acc']
    cols = {k: [] for k in want}
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        idx = {name: i for i, name in enumerate(header)}
        missing = [k for k in want if k not in idx]
        if missing:
            raise RuntimeError(f"Visual CSV missing columns: {missing}")

        def to_float(x):
            try:
                return float(x)
            except Exception:
                return float('nan')

        for row in reader:
            if not row:
                continue
            for k in want:
                j = idx[k]
                cols[k].append(to_float(row[j]) if j < len(row) else float('nan'))

    return {k: np.asarray(v, dtype=float) for k, v in cols.items()}


# ----------------------------------------------------------------------------
# Georeferencing
# ----------------------------------------------------------------------------
def pick_utm_epsg(mean_lat: float, mean_lon: float) -> int:
    """Return EPSG for the UTM zone containing (mean_lat, mean_lon)."""
    zone = int(np.floor((mean_lon + 180.0) / 6.0)) + 1
    zone = min(max(zone, 1), 60)
    return (32600 if mean_lat >= 0 else 32700) + zone


def fit_similarity_2d(src: np.ndarray, dst: np.ndarray, estimate_scale: bool):
    """Umeyama 2D similarity fit: dst ~= s * R @ src + t.

    src, dst: (M,2). Returns (s, R(2x2), t(2,)).
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    S = src - mu_s
    D = dst - mu_d
    C = (D.T @ S) / len(src)
    U, sig, Vt = np.linalg.svd(C)
    Rm = U @ Vt
    if np.linalg.det(Rm) < 0:           # reflection guard
        U = U.copy()
        U[:, -1] *= -1.0
        Rm = U @ Vt
        sig = sig.copy()
        sig[-1] *= -1.0
    if estimate_scale:
        var_s = (S ** 2).sum() / len(src)
        s = float(sig.sum() / var_s) if var_s > 0 else 1.0
    else:
        s = 1.0
    t = mu_d - s * (Rm @ mu_s)
    return s, Rm, t


def apply_similarity(xy: np.ndarray, s: float, Rm: np.ndarray, t: np.ndarray) -> np.ndarray:
    xy = np.asarray(xy, dtype=float)
    return (s * (Rm @ xy.T)).T + t


def build_gps_transform(visual: dict, estimate_scale: bool,
                        min_pairs: int = 8, min_span_m: float = 3.0,
                        collinearity_ratio: float = 0.05):
    """Fit odom->UTM transform from visual-CSV pairs. Returns (fn, epsg, info).

    fn maps odom (N,2) -> UTM (N,2). Raises RuntimeError if the fit is
    untrustworthy (too few fixes, track too short, near-collinear path,
    or large residual vs reported GPS accuracy).
    """
    if Transformer is None:
        raise RuntimeError("pyproj not installed; cannot build GPS transform. "
                           "pip install pyproj")

    status = visual['gps_status']
    lat = visual['gps_lat']
    lon = visual['gps_lon']
    px = visual['px']
    py = visual['py']
    h_acc = visual['gps_h_acc']

    valid = (status >= 1) & np.isfinite(lat) & np.isfinite(lon) \
        & np.isfinite(px) & np.isfinite(py)
    n = int(valid.sum())
    if n < min_pairs:
        raise RuntimeError(f"Only {n} valid GPS fixes (need >= {min_pairs}).")

    lat_v, lon_v = lat[valid], lon[valid]
    odom_xy = np.column_stack((px[valid], py[valid]))

    mean_lat = float(np.mean(lat_v))
    mean_lon = float(np.mean(lon_v))
    epsg = pick_utm_epsg(mean_lat, mean_lon)
    if (lon_v.max() - lon_v.min()) > 6.0:
        print("WARNING: track spans >1 UTM zone; using zone of mean lon.")

    tf = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    E, N = tf.transform(lon_v, lat_v)
    utm_xy = np.column_stack((E, N))

    # Geometry gate: span + collinearity of the odom point cloud.
    span = float(np.linalg.norm(odom_xy.max(axis=0) - odom_xy.min(axis=0)))
    if span < min_span_m:
        raise RuntimeError(f"Odom track span {span:.2f} m < {min_span_m} m; "
                           "rotation fit unreliable.")
    Sc = odom_xy - odom_xy.mean(axis=0)
    sv = np.linalg.svd(Sc, compute_uv=False)
    if sv[0] <= 0 or (sv[1] / sv[0]) < collinearity_ratio:
        raise RuntimeError("Odom track near-collinear; rotation under-constrained. "
                           "Need an L-shaped or curved path for GPS export.")

    s, Rm, t = fit_similarity_2d(odom_xy, utm_xy, estimate_scale)
    resid = apply_similarity(odom_xy, s, Rm, t) - utm_xy
    rms = float(np.sqrt((resid ** 2).sum(axis=1).mean()))
    median_hacc = float(np.nanmedian(h_acc[valid])) if np.any(np.isfinite(h_acc[valid])) else float('nan')
    if np.isfinite(median_hacc) and median_hacc > 0 and rms > 5.0 * median_hacc:
        raise RuntimeError(
            f"Fit residual RMS {rms:.2f} m >> 5x median GPS hAcc {median_hacc:.2f} m. "
            "Odom frames may not match (node restart?) or GPS is too noisy.")

    info = {'epsg': epsg, 'n_pairs': n, 'scale': s, 'rms_m': rms,
            'median_hacc_m': median_hacc, 'span_m': span}
    return (lambda xy: apply_similarity(xy, s, Rm, t)), epsg, info


# ----------------------------------------------------------------------------
# SEG-Y writing
# ----------------------------------------------------------------------------
def write_segy(path: str, A_T: np.ndarray, twt: np.ndarray, coords_xy: np.ndarray,
               dt_raw: int, dt_units: str, epsg) -> None:
    """Write a SEG-Y (IEEE float32) with per-trace coords in cdpx/cdpy/sx/sy/gx/gy.

    dt_raw is the integer value placed verbatim in the SEG-Y 2-byte sample-interval
    fields (binary Interval + trace dt). dt_units is metadata only (us/ns/ps), echoed
    into the textual header so the importer knows the convention. GPR intervals are
    sub-microsecond, so ns/ps are the usual units that still fit the 2-byte field.
    """
    if segyio is None:
        raise RuntimeError("segyio not installed. pip install segyio")

    twt_len, n_tr = A_T.shape
    dt_raw = int(dt_raw)
    if dt_raw <= 0:
        print(f"WARNING: sample interval rounds to {dt_raw}; check --interval_units "
              "(GPR is sub-microsecond -> use ns or ps).")
    if dt_raw > 65535:
        print(f"WARNING: sample interval {dt_raw} {dt_units} exceeds the 2-byte SEG-Y "
              "field (65535); it will be clamped. Use coarser units.")
        dt_raw = 65535

    spec = segyio.spec()
    spec.format = 5                       # IEEE float32
    spec.samples = list(range(twt_len))   # sample index axis
    spec.tracecount = n_tr

    with segyio.create(path, spec) as f:
        for i in range(n_tr):
            x_i = int(round(float(coords_xy[i, 0]) * COORD_MULT))
            y_i = int(round(float(coords_xy[i, 1]) * COORD_MULT))
            f.header[i] = {
                segyio.su.tracl:  i + 1,
                segyio.su.tracr:  i + 1,
                segyio.su.cdp:    i + 1,
                segyio.su.scalco: COORD_SCALAR,
                segyio.su.sx:     x_i, segyio.su.sy:    y_i,
                segyio.su.gx:     x_i, segyio.su.gy:    y_i,
                segyio.su.cdpx:   x_i, segyio.su.cdpy:  y_i,
                segyio.su.counit: 1,      # 1 = length (m/ft), not arcseconds
                segyio.su.dt:     dt_raw,
                segyio.su.ns:     twt_len,
            }
            f.trace[i] = np.ascontiguousarray(A_T[:, i], dtype=np.float32)

        f.bin[segyio.BinField.Interval] = dt_raw
        f.bin[segyio.BinField.Samples] = twt_len
        f.bin[segyio.BinField.Format] = 5

    frame = (f"PROJECTED UTM EPSG {epsg}" if epsg
             else "LOCAL LIDAR ODOM FRAME (meters)")
    with segyio.open(path, "r+", ignore_geometry=True) as f:
        f.text[0] = segyio.tools.create_text_header({
            1: "BDR GPR EXPORT FOR GEOLITIX",
            2: frame,
            3: "TRACE COORDS IN CDPX/CDPY + SX/SY + GX/GY",
            4: f"COORD SCALAR {COORD_SCALAR} (STORED X{int(COORD_MULT)} -> CM)",
            5: f"SAMPLE INTERVAL {dt_raw} {dt_units.upper()}, NS {A_T.shape[0]}",
        })


def template_matches(template_path: str, n_tr: int, twt_len: int):
    """Return (ok, reason) if template SEG-Y has matching trace count + samples."""
    if segyio is None:
        return False, "segyio not installed"
    if not os.path.isfile(template_path):
        return False, f"template not found: {template_path}"
    with segyio.open(template_path, ignore_geometry=True) as f:
        t_tr = f.tracecount
        t_ns = len(f.samples)
    if t_tr != n_tr:
        return False, f"trace count differs (template {t_tr} vs export {n_tr})"
    if t_ns != twt_len:
        return False, f"sample count differs (template {t_ns} vs export {twt_len})"
    return True, "ok"


def write_segy_from_template(template_path: str, out_path: str,
                             coords_xy: np.ndarray, epsg) -> None:
    """Clone the original SEG-Y byte-for-byte, then patch ONLY the per-trace
    coordinate header fields. Preserves the vendor textual header (ASCII),
    binary header (revision, sample interval), trace data, and all other
    trace-header fields exactly as the acquisition unit wrote them -- so a file
    the importer already accepts stays acceptable, just with coordinates added.
    """
    if segyio is None:
        raise RuntimeError("segyio not installed. pip install segyio")
    shutil.copyfile(template_path, out_path)
    with segyio.open(out_path, "r+", ignore_geometry=True) as f:
        n = f.tracecount
        if coords_xy.shape[0] != n:
            raise RuntimeError(
                f"coords ({coords_xy.shape[0]}) != template traces ({n})")
        for i in range(n):
            x_i = int(round(float(coords_xy[i, 0]) * COORD_MULT))
            y_i = int(round(float(coords_xy[i, 1]) * COORD_MULT))
            f.header[i].update({
                segyio.su.scalco: COORD_SCALAR,
                segyio.su.sx:     x_i, segyio.su.sy:    y_i,
                segyio.su.gx:     x_i, segyio.su.gy:    y_i,
                segyio.su.cdpx:   x_i, segyio.su.cdpy:  y_i,
                segyio.su.counit: 1,      # 1 = length (m/ft), not arcseconds
            })
    _ = epsg  # textual header intentionally left untouched (vendor ASCII preserved)


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Export GPR A-scans to SEG-Y for Geolitix.")
    p.add_argument('at_with_xyz_csv', help='*_AT_with_xyz.csv or *_reconstructed.csv')
    p.add_argument('--sample_interval', type=float, default=None,
                   help='SEG-Y sample interval, in --interval_units, from GP8800 config/header.')
    p.add_argument('--interval_units', choices=['us', 'ns', 'ps'], default='ns',
                   help='Units of --sample_interval (default ns; GPR is sub-microsecond).')
    p.add_argument('--src_segy', default=None,
                   help='Original GP8800 .sgy; copy its sample interval (overrides --sample_interval).')
    p.add_argument('--template_segy', default=None,
                   help='Original GP8800 .sgy to clone byte-for-byte and patch with coords '
                        '(preserves vendor textual/binary headers; defaults to --src_segy). '
                        'Use --no_template to force building a fresh SEG-Y instead.')
    p.add_argument('--no_template', action='store_true',
                   help='Force a freshly-built SEG-Y (segyio.create) instead of clone-and-patch.')
    p.add_argument('--visual_csv', default=None,
                   help='unified_data_collector dataset CSV (enables GPS-frame export).')
    p.add_argument('--estimate_scale', action='store_true',
                   help='Estimate odom->UTM scale (default: locked to 1.0, metric->metric).')
    p.add_argument('--out_base', default=None,
                   help='Output path base (default: input CSV path without extension).')
    args = p.parse_args()

    if segyio is None:
        print("ERROR: segyio not installed. pip install segyio")
        sys.exit(1)
    if not os.path.isfile(args.at_with_xyz_csv):
        print(f"ERROR: input CSV not found: {args.at_with_xyz_csv}")
        sys.exit(1)

    # Resolve sample interval: source .sgy wins, else manual value, else 0 (+warn).
    dt_units = args.interval_units
    if args.src_segy:
        if not os.path.isfile(args.src_segy):
            print(f"ERROR: --src_segy not found: {args.src_segy}")
            sys.exit(1)
        with segyio.open(args.src_segy, ignore_geometry=True) as sf:
            dt_raw = int(sf.bin[segyio.BinField.Interval])
        dt_units = 'src'
        print(f"Sample interval copied from {args.src_segy}: {dt_raw} (raw SEG-Y units)")
    elif args.sample_interval is not None:
        dt_raw = int(round(args.sample_interval))
    else:
        dt_raw = 0
        print("WARNING: no --sample_interval / --src_segy given; writing dt=0. "
              "Set it from the GP8800 header for correct TWT scaling.")

    data = load_at_with_xyz_csv(args.at_with_xyz_csv)
    A_T = data['A_T']
    twt = data['twt']
    lidar_xy = np.column_stack((data['px'], data['py']))
    n_tr = A_T.shape[1]
    print(f"Loaded {n_tr} traces x {A_T.shape[0]} samples from {args.at_with_xyz_csv}")

    base = args.out_base if args.out_base else os.path.splitext(args.at_with_xyz_csv)[0]

    # Decide write mode: clone-and-patch the vendor file (preserves the exact
    # format Geolitix already accepts) when a matching template is available,
    # otherwise fall back to building a fresh SEG-Y from scratch.
    template = None if args.no_template else (args.template_segy or args.src_segy)
    if template:
        ok, reason = template_matches(template, n_tr, A_T.shape[0])
        if ok:
            print(f"Clone-and-patch mode: cloning {template} and patching coords only.")
        else:
            print(f"WARNING: template unusable ({reason}); building fresh SEG-Y instead.")
            template = None

    # 1) Lidar-frame export (always).
    lidar_path = base + "_lidar.sgy"
    if template:
        write_segy_from_template(template, lidar_path, lidar_xy, epsg=None)
    else:
        write_segy(lidar_path, A_T, twt, lidar_xy, dt_raw, dt_units, epsg=None)
    print(f"Wrote lidar-frame SEG-Y: {lidar_path}")

    # 2) GPS-frame export (optional).
    if args.visual_csv:
        if not os.path.isfile(args.visual_csv):
            print(f"WARNING: visual CSV not found: {args.visual_csv}; skipping GPS export.")
            return
        try:
            visual = load_visual_csv(args.visual_csv)
            transform_fn, epsg, info = build_gps_transform(visual, args.estimate_scale)
            gps_xy = transform_fn(lidar_xy)
            gps_path = base + "_gps.sgy"
            if template:
                write_segy_from_template(template, gps_path, gps_xy, epsg=epsg)
            else:
                write_segy(gps_path, A_T, twt, gps_xy, dt_raw, dt_units, epsg=epsg)
            print(f"Wrote GPS-frame SEG-Y: {gps_path}")
            print(f"  EPSG={info['epsg']} pairs={info['n_pairs']} scale={info['scale']:.6f} "
                  f"fit_RMS={info['rms_m']:.3f} m median_hAcc={info['median_hacc_m']:.3f} m "
                  f"track_span={info['span_m']:.2f} m")
        except RuntimeError as e:
            print(f"GPS-frame export SKIPPED: {e}")


if __name__ == '__main__':
    main()
