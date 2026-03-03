#!/usr/bin/env python3
"""
Stereo checkerboard calibration:
- Captures synchronized left/right image pairs.
- Estimates left and right intrinsics.
- Estimates stereo extrinsics (R, T) for the camera mount.
- Saves ROS-style camera YAML files + a stereo extrinsics YAML.
"""

import os
import glob
import time
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


# ==== Fixed video mode from robot_complete.launch.py ====
LEFT_DEVICE = "/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_3728140416020900-video-index0"
RIGHT_DEVICE = "/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_0F12140416020900-video-index0"
REQ_WIDTH = 1920
REQ_HEIGHT = 1080
REQ_FPS = 30.0
REQ_FOURCC = "MJPG"
FPS_TOL = 2.0
PATTERN_COLS = 13
PATTERN_ROWS = 9
PATTERN_CANDIDATES = [
    (13, 9),
    (12, 8),
    (11, 8),
    (10, 7),
    (9, 6),
    (8, 6),
    (7, 6),
    (6, 6),
    (5, 5),
]
MIN_PAIRS = 12
RECTIFY_ALPHA = 0.0
R_DATA_BASE = Path("/R_DATA/stereo_calibration")
LEFT_CAMERA_NAME = "left_camera"
RIGHT_CAMERA_NAME = "right_camera"
DETECT_MAX_DIM = 960
SLOW_DETECT_PERIOD = 10
AUTO_PATTERN_CHECK_PERIOD = 30


def fourcc_to_str(v: float) -> str:
    v = int(v)
    return "".join([chr((v >> (8 * i)) & 0xFF) for i in range(4)])


def set_prop(cap, prop, value):
    try:
        return cap.set(prop, value)
    except Exception:
        return False


def configure_cap(cap, width, height, fps, fourcc, strict=True, buffers=1, strict_fps=True):
    set_prop(cap, cv2.CAP_PROP_BUFFERSIZE, buffers)
    req_four = cv2.VideoWriter_fourcc(*fourcc)

    def try_order(order):
        set_prop(cap, cv2.CAP_PROP_FOURCC, req_four)
        if order == "wh-fps":
            set_prop(cap, cv2.CAP_PROP_FRAME_WIDTH, width)
            set_prop(cap, cv2.CAP_PROP_FRAME_HEIGHT, height)
            set_prop(cap, cv2.CAP_PROP_FPS, fps)
        else:
            set_prop(cap, cv2.CAP_PROP_FPS, fps)
            set_prop(cap, cv2.CAP_PROP_FRAME_WIDTH, width)
            set_prop(cap, cv2.CAP_PROP_FRAME_HEIGHT, height)

        ok, _ = cap.read()
        if not ok:
            return False, "no frame"

        got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        got_fps = float(cap.get(cv2.CAP_PROP_FPS)) or 0.0
        got_fc = fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC))

        ok_four = (got_fc == fourcc or got_fc[::-1] == fourcc)
        ok_size = (got_w == width and got_h == height)
        ok_fps = (not strict_fps) or (got_fps == 0.0) or (abs(got_fps - fps) <= FPS_TOL)

        if strict:
            if not ok_four:
                return False, f"FOURCC mismatch (got {got_fc}, want {fourcc})"
            if not ok_size:
                return False, f"size mismatch (got {got_w}x{got_h}, want {width}x{height})"
            if not ok_fps:
                return False, f"fps mismatch (got {got_fps:.2f}, want {fps}±{FPS_TOL})"

        return True, f"{got_w}x{got_h}@{got_fps:.2f} FOURCC={got_fc}"

    ok, info = try_order("wh-fps")
    if not ok:
        ok, info = try_order("fps-wh")
    return ok, info


def try_open(dev, width, height, fps, fourcc, strict=True, buffers=1, strict_fps=True):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None, "open failed"
    ok, info = configure_cap(cap, width, height, fps, fourcc, strict, buffers, strict_fps)
    if not ok:
        cap.release()
        return None, info
    return cap, info


def open_camera(preferred_dev, preferred_idx, width, height, fps, fourcc, strict=True, buffers=1, strict_fps=True):
    if preferred_dev:
        cap, info = try_open(preferred_dev, width, height, fps, fourcc, strict, buffers, strict_fps)
        if cap is not None:
            return cap, preferred_dev, info

    if preferred_idx is not None and preferred_idx >= 0:
        cap, info = try_open(preferred_idx, width, height, fps, fourcc, strict, buffers, strict_fps)
        if cap is not None:
            return cap, preferred_idx, info

    for dev in sorted(glob.glob("/dev/v4l/by-id/*")):
        cap, info = try_open(dev, width, height, fps, fourcc, strict, buffers, strict_fps)
        if cap is not None:
            return cap, dev, info

    for idx in range(10):
        path = f"/dev/video{idx}"
        if os.path.exists(path):
            cap, info = try_open(idx, width, height, fps, fourcc, strict, buffers, strict_fps)
            if cap is not None:
                return cap, idx, info

    return None, None, "no candidate devices matched requested mode"


def find_corners(gray, pattern_size, allow_slow=True):
    h, w = gray.shape[:2]
    max_dim = max(h, w)
    scale = 1.0
    if max_dim > DETECT_MAX_DIM:
        scale = float(DETECT_MAX_DIM) / float(max_dim)
        work = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    else:
        work = gray

    # Contrast normalization helps when lighting is uneven.
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    work_eq = clahe.apply(work)

    def _classic(img):
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        return cv2.findChessboardCorners(img, pattern_size, flags)

    def _sb(img):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
        return cv2.findChessboardCornersSB(img, pattern_size, flags)

    # Fast stages every frame.
    stages = [
        (_classic, work_eq),
        (_classic, work),
    ]
    # Slower exhaustive stages only when explicitly requested.
    if allow_slow:
        stages.extend(
            [
                (_sb, work_eq),
                (_sb, work),
            ]
        )

    for detector, img in stages:
        found, corners = detector(img)
        if found:
            corners = corners.astype(np.float32)
            if scale != 1.0:
                corners /= scale
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 50, 1e-4)
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
            return True, corners

    return False, None


def write_pattern_metadata(session_dir: Path, pattern_size):
    meta_path = session_dir / "pattern_size.txt"
    with open(meta_path, "w") as f:
        f.write(f"{pattern_size[0]} {pattern_size[1]}\n")


def read_pattern_metadata(session_dir: Path):
    meta_path = session_dir / "pattern_size.txt"
    if not meta_path.exists():
        return (PATTERN_COLS, PATTERN_ROWS)
    try:
        text = meta_path.read_text().strip()
        cols_str, rows_str = text.split()
        cols = int(cols_str)
        rows = int(rows_str)
        if cols >= 3 and rows >= 3:
            return (cols, rows)
    except Exception:
        pass
    return (PATTERN_COLS, PATTERN_ROWS)


def auto_select_pattern(gray_l, gray_r):
    for cand in PATTERN_CANDIDATES:
        found_l, _ = find_corners(gray_l, cand, allow_slow=False)
        found_r, _ = find_corners(gray_r, cand, allow_slow=False)
        if found_l and found_r:
            return cand
    return None


def draw_hud(frame, pair_count, text_lines):
    h, w = frame.shape[:2]
    overlay = frame.copy()
    lines = [
        "S: save pair (both checkerboards required)  C: calibrate  Q: quit",
        f"Saved stereo pairs: {pair_count}",
    ] + text_lines
    panel_h = min(h, 12 + 24 * len(lines))
    cv2.rectangle(overlay, (0, 0), (w, panel_h), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)
    for i, line in enumerate(lines):
        cv2.putText(
            frame,
            line,
            (12, 28 + 24 * i),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return frame


def mat_to_yaml(name, mat):
    rows, cols = mat.shape
    data = ", ".join([f"{x:.8f}" for x in mat.reshape(-1)])
    return f"""{name}:
  rows: {rows}
  cols: {cols}
  data: [{data}]
"""


def save_ros_yaml(path, camera_name, imsize, k, dist, rect_r, proj_p):
    w, h = imsize
    dist_list = dist.reshape(-1).tolist()
    with open(path, "w") as f:
        f.write(
            f"""image_width: {w}
image_height: {h}
camera_name: {camera_name}
{mat_to_yaml("camera_matrix", k)}
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: {len(dist_list)}
  data: [{", ".join([f"{x:.8f}" for x in dist_list])}]
{mat_to_yaml("rectification_matrix", rect_r)}
{mat_to_yaml("projection_matrix", proj_p)}
"""
        )


def save_stereo_yaml(path, imsize, pattern_size, square_size, r, t, e, f, r1, r2, p1, p2, q, stereo_rms):
    w, h = imsize
    baseline = float(np.linalg.norm(t.reshape(-1)))
    with open(path, "w") as out:
        out.write(
            f"""image_width: {w}
image_height: {h}
checkerboard:
  cols: {pattern_size[0]}
  rows: {pattern_size[1]}
  square_size_m: {square_size:.8f}
stereo_rms: {stereo_rms:.8f}
baseline_m: {baseline:.8f}
{mat_to_yaml("R", r)}
{mat_to_yaml("T", t.reshape(3, 1))}
{mat_to_yaml("E", e)}
{mat_to_yaml("F", f)}
{mat_to_yaml("R1", r1)}
{mat_to_yaml("R2", r2)}
{mat_to_yaml("P1", p1)}
{mat_to_yaml("P2", p2)}
{mat_to_yaml("Q", q)}
"""
        )


def list_pairs(pairs_dir: Path):
    left_files = sorted(pairs_dir.glob("pair_*_left.png"))
    pairs = []
    for lf in left_files:
        rf = pairs_dir / lf.name.replace("_left.png", "_right.png")
        if rf.exists():
            pairs.append((lf, rf))
    return pairs


def get_latest_session_dir(base_dir: Path):
    sessions = [p for p in sorted(base_dir.glob("session_*")) if p.is_dir()]
    if not sessions:
        return None
    return sessions[-1]


def calibrate_from_pairs(session_dir: Path, square_size: float, pattern_size=None):
    pairs_dir = session_dir / "pairs"
    pairs = list_pairs(pairs_dir)
    if len(pairs) < MIN_PAIRS:
        print(f"Need at least {MIN_PAIRS} stereo pairs; found {len(pairs)} in {pairs_dir}")
        return 1

    if pattern_size is None:
        pattern_size = read_pattern_metadata(session_dir)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []
    imgpoints_l = []
    imgpoints_r = []
    imsize = None
    used = 0

    print(f"[INFO] Reading {len(pairs)} saved stereo pairs from {pairs_dir}")
    for left_path, right_path in pairs:
        left = cv2.imread(str(left_path))
        right = cv2.imread(str(right_path))
        if left is None or right is None:
            print(f"[WARN] Unreadable pair: {left_path.name}")
            continue

        g_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        g_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        if imsize is None:
            imsize = g_l.shape[::-1]
        if g_l.shape != g_r.shape:
            print(f"[WARN] Size mismatch in pair {left_path.name}; skipping")
            continue

        found_l, corners_l = find_corners(g_l, pattern_size, allow_slow=True)
        found_r, corners_r = find_corners(g_r, pattern_size, allow_slow=True)
        if not (found_l and found_r):
            print(f"[WARN] Checkerboard not detected in both views for {left_path.name}; skipped")
            continue

        objpoints.append(objp.copy())
        imgpoints_l.append(corners_l)
        imgpoints_r.append(corners_r)
        used += 1

    if used < MIN_PAIRS:
        print(f"Only {used} usable pairs; need at least {MIN_PAIRS}.")
        return 1

    calib_flags = cv2.CALIB_RATIONAL_MODEL
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 120, 1e-7)

    print(f"[INFO] Calibrating LEFT intrinsics with {used} pairs...")
    rms_l, k_l, d_l, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints_l, imsize, None, None, flags=calib_flags, criteria=term
    )
    print(f"[INFO] LEFT RMS: {rms_l:.6f} px")

    print(f"[INFO] Calibrating RIGHT intrinsics with {used} pairs...")
    rms_r, k_r, d_r, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints_r, imsize, None, None, flags=calib_flags, criteria=term
    )
    print(f"[INFO] RIGHT RMS: {rms_r:.6f} px")

    print("[INFO] Running stereo calibration (fixing intrinsics)...")
    stereo_flags = cv2.CALIB_FIX_INTRINSIC
    stereo_rms, k_l, d_l, k_r, d_r, r, t, e, f = cv2.stereoCalibrate(
        objpoints,
        imgpoints_l,
        imgpoints_r,
        k_l,
        d_l,
        k_r,
        d_r,
        imsize,
        criteria=term,
        flags=stereo_flags,
    )

    rectify_alpha = RECTIFY_ALPHA
    r1, r2, p1, p2, q, roi1, roi2 = cv2.stereoRectify(
        k_l,
        d_l,
        k_r,
        d_r,
        imsize,
        r,
        t,
        alpha=rectify_alpha,
        flags=cv2.CALIB_ZERO_DISPARITY,
    )

    baseline = float(np.linalg.norm(t.reshape(-1)))
    print("\n=== Stereo Calibration Results ===")
    print(f"Usable stereo pairs: {used}")
    print(f"Left RMS reprojection:  {rms_l:.6f} px")
    print(f"Right RMS reprojection: {rms_r:.6f} px")
    print(f"Stereo RMS:             {stereo_rms:.6f} px")
    print(f"Baseline:               {baseline:.6f} m")
    print(f"Rectify alpha:          {rectify_alpha:.2f}")
    print(f"ROI left: {roi1}, ROI right: {roi2}")

    out_left = session_dir / "lcamera_calib.yaml"
    out_right = session_dir / "rcamera_calib.yaml"
    out_stereo = session_dir / "stereo_extrinsics.yaml"

    save_ros_yaml(out_left, LEFT_CAMERA_NAME, imsize, k_l, d_l, r1, p1)
    save_ros_yaml(out_right, RIGHT_CAMERA_NAME, imsize, k_r, d_r, r2, p2)
    save_stereo_yaml(
        out_stereo,
        imsize,
        pattern_size,
        square_size,
        r,
        t,
        e,
        f,
        r1,
        r2,
        p1,
        p2,
        q,
        stereo_rms,
    )

    npz_path = session_dir / "stereo_extrinsics.npz"
    np.savez_compressed(
        str(npz_path),
        image_size=np.array(imsize, dtype=np.int32),
        K1=k_l,
        D1=d_l,
        K2=k_r,
        D2=d_r,
        R=r,
        T=t,
        E=e,
        F=f,
        R1=r1,
        R2=r2,
        P1=p1,
        P2=p2,
        Q=q,
        roi1=np.array(roi1, dtype=np.int32),
        roi2=np.array(roi2, dtype=np.int32),
        left_rms=np.array([rms_l], dtype=np.float64),
        right_rms=np.array([rms_r], dtype=np.float64),
        stereo_rms=np.array([stereo_rms], dtype=np.float64),
        baseline_m=np.array([baseline], dtype=np.float64),
    )

    print(f"[INFO] Saved left camera YAML:  {out_left}")
    print(f"[INFO] Saved right camera YAML: {out_right}")
    print(f"[INFO] Saved stereo YAML:       {out_stereo}")
    print(f"[INFO] Saved stereo NPZ:        {npz_path}")
    return 0


def capture_mode(session_dir: Path, square_size: float):
    pairs_dir = session_dir / "pairs"
    pairs_dir.mkdir(parents=True, exist_ok=True)

    cap_l, dev_l, info_l = open_camera(
        LEFT_DEVICE,
        -1,
        REQ_WIDTH,
        REQ_HEIGHT,
        REQ_FPS,
        REQ_FOURCC,
        strict=True,
        buffers=1,
        strict_fps=True,
    )
    if cap_l is None:
        print("ERROR: failed to open LEFT camera with requested mode.", file=sys.stderr)
        return 1

    cap_r, dev_r, info_r = open_camera(
        RIGHT_DEVICE,
        -1,
        REQ_WIDTH,
        REQ_HEIGHT,
        REQ_FPS,
        REQ_FOURCC,
        strict=True,
        buffers=1,
        strict_fps=True,
    )
    if cap_r is None:
        print("ERROR: failed to open RIGHT camera with requested mode.", file=sys.stderr)
        cap_l.release()
        return 1

    print(f"[INFO] Opened LEFT  {dev_l} ({info_l})")
    print(f"[INFO] Opened RIGHT {dev_r} ({info_r})")

    pair_count = len(list_pairs(pairs_dir))
    pattern_size = (PATTERN_COLS, PATTERN_ROWS)
    write_pattern_metadata(session_dir, pattern_size)
    miss_streak = 0
    frame_idx = 0

    print(
        f"[INFO] Checkerboard expected inner corners: {pattern_size[0]}x{pattern_size[1]} "
        f"(columns x rows), square size={square_size} m"
    )
    print(f"[INFO] Auto-pattern candidates: {PATTERN_CANDIDATES}")

    try:
        while True:
            ok_l, frame_l = cap_l.read()
            ok_r, frame_r = cap_r.read()
            if not ok_l or not ok_r:
                print("[WARN] Frame read failed, retrying...")
                time.sleep(0.05)
                continue

            gray_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
            do_slow = (frame_idx % SLOW_DETECT_PERIOD == 0)
            found_l, corners_l = find_corners(gray_l, pattern_size, allow_slow=do_slow)
            found_r, corners_r = find_corners(gray_r, pattern_size, allow_slow=do_slow)
            frame_idx += 1

            vis_l = frame_l.copy()
            vis_r = frame_r.copy()
            if found_l:
                cv2.drawChessboardCorners(vis_l, pattern_size, corners_l, found_l)
            if found_r:
                cv2.drawChessboardCorners(vis_r, pattern_size, corners_r, found_r)

            status_l = "LEFT board: OK" if found_l else "LEFT board: not found"
            status_r = "RIGHT board: OK" if found_r else "RIGHT board: not found"
            can_save = found_l and found_r
            status_pair = "PAIR ready: YES" if can_save else "PAIR ready: NO"
            if can_save:
                miss_streak = 0
            else:
                miss_streak += 1
                if miss_streak % AUTO_PATTERN_CHECK_PERIOD == 0:
                    auto_pat = auto_select_pattern(gray_l, gray_r)
                    if auto_pat is not None and auto_pat != pattern_size:
                        pattern_size = auto_pat
                        write_pattern_metadata(session_dir, pattern_size)
                        print(f"[INFO] Auto-switched checkerboard pattern to {pattern_size[0]}x{pattern_size[1]} inner corners")
                    print(
                        "[HINT] Checkerboard not found in both views. "
                        f"Current inner-corner pattern is {pattern_size[0]}x{pattern_size[1]}. "
                        "Keep full board visible, "
                        "avoid motion blur/glare, and vary distance/tilt."
                    )

            vis_l = draw_hud(vis_l, pair_count, [status_l, status_pair, f"Pattern: {pattern_size[0]}x{pattern_size[1]}", f"LEFT mode: {info_l}"])
            vis_r = draw_hud(vis_r, pair_count, [status_r, status_pair, f"Pattern: {pattern_size[0]}x{pattern_size[1]}", f"RIGHT mode: {info_r}"])

            cv2.imshow("stereo_left", vis_l)
            cv2.imshow("stereo_right", vis_r)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                if not can_save:
                    print("[WARN] Checkerboard must be detected in BOTH views before saving.")
                    continue
                stamp = time.strftime("%Y%m%d_%H%M%S")
                stem = f"pair_{stamp}_{pair_count:04d}"
                lp = pairs_dir / f"{stem}_left.png"
                rp = pairs_dir / f"{stem}_right.png"
                cv2.imwrite(str(lp), frame_l)
                cv2.imwrite(str(rp), frame_r)
                pair_count += 1
                print(f"[INFO] Saved stereo pair {pair_count:04d}: {lp.name}, {rp.name}")
            if key == ord("c"):
                print("[INFO] Starting calibration from saved pairs...")
                break
    finally:
        cap_l.release()
        cap_r.release()
        cv2.destroyAllWindows()

    return calibrate_from_pairs(session_dir, square_size, pattern_size=pattern_size)


def parse_args():
    ap = argparse.ArgumentParser(
        description="Stereo checkerboard calibration for left/right intrinsics and mount extrinsics."
    )
    ap.add_argument("--mode", choices=["capture", "calibrate"], default="capture",
                    help="capture: interactive pair capture then calibrate, calibrate: run from existing pairs only")
    ap.add_argument("--square-size", type=float, default=0.02, help="Checkerboard square size in meters")
    return ap.parse_args()


def main():
    args = parse_args()
    R_DATA_BASE.mkdir(parents=True, exist_ok=True)

    if args.mode == "capture":
        ts = time.strftime("%Y%m%d_%H%M%S")
        session_dir = R_DATA_BASE / f"session_{ts}"
        session_dir.mkdir(parents=True, exist_ok=True)
        print(f"[INFO] Created calibration session: {session_dir}")
        print(f"[INFO] Using launch camera mode: {REQ_WIDTH}x{REQ_HEIGHT}@{REQ_FPS:.0f} {REQ_FOURCC}")
        print(f"[INFO] Left device:  {LEFT_DEVICE}")
        print(f"[INFO] Right device: {RIGHT_DEVICE}")
        rc = capture_mode(session_dir, args.square_size)
    else:
        session_dir = get_latest_session_dir(R_DATA_BASE)
        if session_dir is None:
            print(f"ERROR: no calibration session found under {R_DATA_BASE}", file=sys.stderr)
            print("Run with --mode capture first to collect stereo pairs.", file=sys.stderr)
            sys.exit(1)
        pat = read_pattern_metadata(session_dir)
        print(f"[INFO] Using latest calibration session: {session_dir}")
        print(f"[INFO] Using checkerboard pattern: {pat[0]}x{pat[1]} inner corners")
        rc = calibrate_from_pairs(session_dir, args.square_size, pattern_size=pat)
    sys.exit(rc)


if __name__ == "__main__":
    cv2.setUseOptimized(True)
    main()
