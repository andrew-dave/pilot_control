#!/usr/bin/env python3
"""
Dual camera live undistort using calibration from ROS camera YAML files.
Shows left and right cameras side by side.
Window size is adjustable and can be resized by the user.
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

def put_text(img, text, y=30, color=(255,255,255)):
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

def resize_image_to_fit(img, target_width, target_height):
    """Resize image to fit within target dimensions while maintaining aspect ratio"""
    h, w = img.shape[:2]
    
    # Calculate scaling factors
    scale_w = target_width / w
    scale_h = target_height / h
    scale = min(scale_w, scale_h)
    
    # Calculate new dimensions
    new_w = int(w * scale)
    new_h = int(h * scale)
    
    # Resize image
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
    
    # Create black canvas with target dimensions
    canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)
    
    # Center the resized image on the canvas
    y_offset = (target_height - new_h) // 2
    x_offset = (target_width - new_w) // 2
    canvas[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
    
    return canvas

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="Dual camera live undistortion using ROS-style YAML calibration.")
    ap.add_argument("--left-yaml", type=str, default="lcamera_calib.yaml", help="Path to left camera ROS YAML")
    ap.add_argument("--right-yaml", type=str, default="rcamera_calib.yaml", help="Path to right camera ROS YAML")
    ap.add_argument("--left-dev",  type=str, default="", help="Left camera V4L2 device path")
    ap.add_argument("--right-dev", type=str, default="", help="Right camera V4L2 device path")
    ap.add_argument("--left-cam",  type=int, default=0, help="Left camera numeric index")
    ap.add_argument("--right-cam", type=int, default=1, help="Right camera numeric index")
    ap.add_argument("--fourcc", type=str, default=DEFAULT_REQ_FOURCC, help="Required FOURCC (default MJPG)")
    ap.add_argument("--fps",   type=float, default=DEFAULT_REQ_FPS, help="Required FPS (default 114)")
    ap.add_argument("--no-strict-fps", action="store_true", help="Do not strictly enforce reported FPS")
    ap.add_argument("--buffers", type=int, default=1, help="Capture buffer count (if supported)")
    ap.add_argument("--show-raw", action="store_true", help="Show raw | undistorted for each camera")
    ap.add_argument("--initial-width", type=int, default=1920, help="Initial window width (default 1920)")
    ap.add_argument("--initial-height", type=int, default=600, help="Initial window height (default 600)")
    args = ap.parse_args()

    # Load calibrations
    try:
        left_yml = load_ros_yaml(args.left_yaml)
        right_yml = load_ros_yaml(args.right_yaml)
    except Exception as e:
        print(f"ERROR loading YAML: {e}", file=sys.stderr)
        sys.exit(1)

    W, H = left_yml["width"], left_yml["height"]
    print(f"[INFO] Loaded left calibration: {W}x{H}, model={left_yml['model']}, camera={left_yml['camera_name']}")
    print(f"[INFO] Loaded right calibration: {W}x{H}, model={right_yml['model']}, camera={right_yml['camera_name']}")

    # Open cameras
    left_cap, left_dev, left_info = open_camera(
        args.left_dev, args.left_cam, W, H, args.fps, args.fourcc,
        strict=True, buffers=args.buffers, strict_fps=(not args.no_strict_fps)
    )
    if left_cap is None:
        print("ERROR: cannot open left camera with requested mode.", file=sys.stderr)
        sys.exit(1)
    print(f"[INFO] Opened left camera {left_dev}  ({left_info})")

    right_cap, right_dev, right_info = open_camera(
        args.right_dev, args.right_cam, W, H, args.fps, args.fourcc,
        strict=True, buffers=args.buffers, strict_fps=(not args.no_strict_fps)
    )
    if right_cap is None:
        print("ERROR: cannot open right camera with requested mode.", file=sys.stderr)
        left_cap.release()
        sys.exit(1)
    print(f"[INFO] Opened right camera {right_dev}  ({right_info})")

    # Build undistortion maps
    left_map1, left_map2, left_newK = build_maps_from_yaml(left_yml)
    right_map1, right_map2, right_newK = build_maps_from_yaml(right_yml)
    
    # Create resizable window
    cv2.namedWindow("dual_cameras", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("dual_cameras", args.initial_width, args.initial_height)
    
    print(f"[INFO] Window created with initial size: {args.initial_width}x{args.initial_height}")
    print("[INFO] You can resize the window by dragging the corners/edges")
    print("[INFO] Press 'q' or ESC to quit, 'r' to reset window size")

    last = time.time(); frames = 0; fps_disp = 0.0
    while True:
        # Read from both cameras
        left_ok, left_frame = left_cap.read()
        right_ok, right_frame = right_cap.read()
        
        if not left_ok or not right_ok:
            print("WARN: frame grab failed from one or both cameras")
            time.sleep(0.1)
            continue

        # Undistort both frames
        left_und = cv2.remap(left_frame, left_map1, left_map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        right_und = cv2.remap(right_frame, right_map1, right_map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        # Right camera without rotation
        # right_und = cv2.rotate(right_und, cv2.ROTATE_180)  # Disabled rotation

        frames += 1
        now = time.time()
        if now - last >= 0.5:
            fps_disp = frames / (now - last)
            frames = 0
            last = now

        # Get current window size
        window_size = cv2.getWindowImageRect("dual_cameras")
        if window_size[2] > 0 and window_size[3] > 0:  # Valid window size
            target_width = window_size[2]
            target_height = window_size[3]
        else:
            # Fallback to initial size if window size is invalid
            target_width = args.initial_width
            target_height = args.initial_height

        if args.show_raw:
            # Show raw | undistorted for each camera
            left_combo = np.hstack((left_frame, left_und))
            right_combo = np.hstack((right_frame, right_und))
            # right_combo = cv2.rotate(right_combo, cv2.ROTATE_180)  # Disabled rotation
            
            # Stack vertically: left on top, right on bottom
            dual_combo = np.vstack((left_combo, right_combo))
            
            put_text(dual_combo, f"Left: raw | undistorted  ({left_info})", y=30)
            put_text(dual_combo, f"Right: raw | undistorted  ({right_info})", y=H+30)
            put_text(dual_combo, f"FPS ~ {fps_disp:.1f}", y=H+60)
        else:
            # Show just undistorted side by side
            dual_combo = np.hstack((left_und, right_und))
            put_text(dual_combo, f"Left: undistorted  ({left_info})", y=30)
            put_text(dual_combo, f"Right: undistorted  ({right_info})", y=60)
            put_text(dual_combo, f"FPS ~ {fps_disp:.1f}", y=90)

        # Resize the combined image to fit the current window size
        display_img = resize_image_to_fit(dual_combo, target_width, target_height)
        
        # Add window size info
        put_text(display_img, f"Window: {target_width}x{target_height}", y=target_height-30, color=(0,255,0))

        cv2.imshow("dual_cameras", display_img)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('r'):
            # Reset window to initial size
            cv2.resizeWindow("dual_cameras", args.initial_width, args.initial_height)
            print(f"[INFO] Window reset to {args.initial_width}x{args.initial_height}")

    left_cap.release()
    right_cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    cv2.setUseOptimized(True)
    main()
