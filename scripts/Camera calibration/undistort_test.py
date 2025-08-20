#!/usr/bin/env python3
"""
Live undistort using calibration from a ROS camera YAML file.
Default: enforce 1920x1200 @ 114 fps, MJPG (can override via CLI).
"""
import os, glob, time, argparse, sys
import numpy as np
import cv2

# ---------- YAML loader ----------
try:
    import yaml
except ImportError:
    print("ERROR: PyYAML not installed. Run: pip install pyyaml", file=sys.stderr)
    sys.exit(1)

# Defaults (can be overridden by CLI)
DEFAULT_REQ_FOURCC = "MJPG"
DEFAULT_REQ_FPS = 114.0
FPS_TOL = 2.0

def fourcc_to_str(v):
    v = int(v)
    return "".join([chr((v >> (8*i)) & 0xFF) for i in range(4)])

def load_ros_yaml(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    try:
        w = int(data["image_width"])
        h = int(data["image_height"])
        cm = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3,3)
        dist = np.array(data["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1)
        model = str(data.get("distortion_model", "plumb_bob")).lower()
    except Exception as e:
        raise ValueError(f"YAML missing required fields: {e}")

    # rectification (if present)
    if "rectification_matrix" in data and "data" in data["rectification_matrix"]:
        R_yaml = np.array(data["rectification_matrix"]["data"], dtype=np.float64).reshape(3,3)
    else:
        R_yaml = np.eye(3, dtype=np.float64)

    # projection matrix (P) — use left 3x3 as the "newK" if provided
    newK = None
    if "projection_matrix" in data and "data" in data["projection_matrix"]:
        P = np.array(data["projection_matrix"]["data"], dtype=np.float64).reshape(3,4)
        newK = P[:, :3].copy()

    if model not in ("plumb_bob", "rational_polynomial"):
        raise ValueError(f"Unsupported distortion_model '{model}'. This script expects 'plumb_bob' (pinhole).")

    return {
        "width": w, "height": h,
        "K": cm, "D": dist, "R_yaml": R_yaml, "newK_yaml": newK,
        "model": model,
        "camera_name": data.get("camera_name", "camera")
    }

# ---------- Camera helpers ----------
def set_prop(cap, prop, val):
    try:
        return cap.set(prop, val)
    except Exception:
        return False

def configure_cap(cap, width, height, fps, fourcc, strict=True, buffers=1, strict_fps=True):
    set_prop(cap, cv2.CAP_PROP_BUFFERSIZE, buffers)
    four = cv2.VideoWriter_fourcc(*fourcc)

    def try_order(order):
        set_prop(cap, cv2.CAP_PROP_FOURCC, four)
        if order == "wh-fps":
            set_prop(cap, cv2.CAP_PROP_FRAME_WIDTH,  width)
            set_prop(cap, cv2.CAP_PROP_FRAME_HEIGHT, height)
            set_prop(cap, cv2.CAP_PROP_FPS,          fps)
        else:
            set_prop(cap, cv2.CAP_PROP_FPS,          fps)
            set_prop(cap, cv2.CAP_PROP_FRAME_WIDTH,  width)
            set_prop(cap, cv2.CAP_PROP_FRAME_HEIGHT, height)

        ok, _ = cap.read()
        if not ok: return False, "no frame from camera"

        got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        got_fps = float(cap.get(cv2.CAP_PROP_FPS)) or 0.0
        got_fc = fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC))

        ok_four = (got_fc == fourcc or got_fc[::-1] == fourcc)
        ok_size = (got_w == width and got_h == height)
        ok_fps  = (not strict_fps) or (got_fps == 0.0) or (abs(got_fps - fps) <= FPS_TOL)

        if strict:
            if not ok_four: return False, f"FOURCC mismatch (got {got_fc}, want {fourcc})"
            if not ok_size: return False, f"size mismatch (got {got_w}x{got_h}, want {width}x{height})"
            if not ok_fps:  return False, f"fps mismatch (got {got_fps:.2f}, want {fps}±{FPS_TOL})"
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
        if cap: return cap, preferred_dev, info
    if preferred_idx is not None and preferred_idx >= 0:
        cap, info = try_open(preferred_idx, width, height, fps, fourcc, strict, buffers, strict_fps)
        if cap: return cap, preferred_idx, info
    for dev in sorted(glob.glob("/dev/v4l/by-id/*")):
        cap, info = try_open(dev, width, height, fps, fourcc, strict, buffers, strict_fps)
        if cap: return cap, dev, info
    for i in range(10):
        path = f"/dev/video{i}"
        if os.path.exists(path):
            cap, info = try_open(i, width, height, fps, fourcc, strict, buffers, strict_fps)
            if cap: return cap, i, info
    return None, None, "no camera matched requested mode"

# ---------- Undistort maps ----------
def build_maps_from_yaml(yml):
    W, H = yml["width"], yml["height"]
    K, D = yml["K"], yml["D"]
    R_yaml = yml["R_yaml"]
    newK = yml["newK_yaml"]

    if newK is None:
        # If no P/newK in YAML, make a gentle crop matrix
        newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (W, H), alpha=0.15, newImgSize=(W, H), centerPrincipalPoint=True)

    # Use identity rectification unless YAML provided a meaningful R
    R = R_yaml if R_yaml is not None else np.eye(3, dtype=np.float64)

    map1, map2 = cv2.initUndistortRectifyMap(K, D, R, newK, (W, H), cv2.CV_16SC2)
    return map1, map2, newK

def put_text(img, text, y=30):
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="Live camera undistortion using ROS-style YAML calibration.")
    ap.add_argument("--yaml", type=str, default="camera_calib.yaml", help="Path to ROS camera YAML")
    ap.add_argument("--dev",  type=str, default="", help="V4L2 device path (e.g., /dev/v4l/by-id/usb-...-video-index0)")
    ap.add_argument("--cam",  type=int, default=-1, help="Numeric index if not using --dev")
    ap.add_argument("--fourcc", type=str, default=DEFAULT_REQ_FOURCC, help="Required FOURCC (default MJPG)")
    ap.add_argument("--fps",   type=float, default=DEFAULT_REQ_FPS, help="Required FPS (default 114)")
    ap.add_argument("--no-strict-fps", action="store_true", help="Do not strictly enforce reported FPS")
    ap.add_argument("--buffers", type=int, default=1, help="Capture buffer count (if supported)")
    ap.add_argument("--show-side", action="store_true", help="Show raw | undistorted side-by-side")
    args = ap.parse_args()

    # Load calibration
    try:
        yml = load_ros_yaml(args.yaml)
    except Exception as e:
        print(f"ERROR loading YAML: {e}", file=sys.stderr)
        sys.exit(1)

    W, H = yml["width"], yml["height"]
    print(f"[INFO] Loaded calibration: {W}x{H}, model={yml['model']}, camera={yml['camera_name']}")

    # Open camera strictly at YAML size + requested fps/fourcc
    cap, dev_used, info = open_camera(
        args.dev, args.cam, W, H, args.fps, args.fourcc,
        strict=True, buffers=args.buffers, strict_fps=(not args.no_strict_fps)
    )
    if cap is None:
        print("ERROR: cannot open camera with requested mode.", file=sys.stderr)
        print(f"Requested: {W}x{H}@{args.fps} FOURCC={args.fourcc}", file=sys.stderr)
        print("Tip: check 'v4l2-ctl --list-formats-ext' for supported modes.", file=sys.stderr)
        sys.exit(1)
    print(f"[INFO] Opened {dev_used}  ({info})")

    # Build undistortion maps
    map1, map2, newK = build_maps_from_yaml(yml)
    cv2.namedWindow("undistort", cv2.WINDOW_NORMAL)

    last = time.time(); frames = 0; fps_disp = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("WARN: frame grab failed"); continue

        und = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        frames += 1
        now = time.time()
        if now - last >= 0.5:
            fps_disp = frames / (now - last)
            frames = 0
            last = now

        if args.show_side:
            combo = np.hstack((frame, und))
            put_text(combo, f"raw | undistorted  ({info})")
            put_text(combo, f"FPS ~ {fps_disp:.1f}", y=60)
            cv2.imshow("undistort", combo)
        else:
            put_text(und, f"undistorted  ({info})")
            put_text(und, f"FPS ~ {fps_disp:.1f}", y=60)
            cv2.imshow("undistort", und)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    cv2.setUseOptimized(True)
    main()
