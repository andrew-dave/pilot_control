#!/usr/bin/env python3
import os, glob, time, argparse, sys
from pathlib import Path
import numpy as np
import cv2

# ==== CONFIG YOU CARE ABOUT (defaults; can be overridden by CLI) ====
REQ_WIDTH  = 1920
REQ_HEIGHT = 1200
REQ_FPS    = 114
REQ_FOURCC = "MJPG"   # Motion-JPEG
FPS_TOL    = 1.5      # acceptable deviation in reported FPS

# --------------------------------------------------------------------

def draw_hud(frame, saved_count, msg=""):
    h, w = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0,0), (w,110), (0,0,0), -1)
    frame = cv2.addWeighted(overlay, 0.4, frame, 0.6, 0)
    for i, t in enumerate([
        "S: save (only if checkerboard found)   C: calibrate from folder   Q: quit",
        f"Saved images: {saved_count}   {msg}"
    ]):
        cv2.putText(frame, t, (12, 30+28*i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
    return frame

def find_corners(gray, pattern_size):
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
             cv2.CALIB_CB_NORMALIZE_IMAGE |
             cv2.CALIB_CB_FAST_CHECK)
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not ret:
        return False, None
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 40, 1e-3)
    cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), term)
    return True, corners

# ---------------- Camera open/config with strict checks ----------------
def fourcc_to_str(v: float) -> str:
    v = int(v)
    s = "".join([chr((v >> 8*i) & 0xFF) for i in range(4)])
    return s

def set_prop(cap, prop, value):
    try:
        return cap.set(prop, value)
    except Exception:
        return False

def configure_cap(cap, width, height, fps, fourcc, strict=True, buffersize=1):
    # Some builds support buffer size; ignore failures.
    _ = set_prop(cap, cv2.CAP_PROP_BUFFERSIZE, buffersize)

    # Order matters for some drivers; try two orders.
    four = cv2.VideoWriter_fourcc(*fourcc)

    def try_order(order_name):
        # reset attempts (not all drivers truly reset; we'll just try)
        set_prop(cap, cv2.CAP_PROP_FOURCC, four)
        if order_name == "wh-fps":
            set_prop(cap, cv2.CAP_PROP_FRAME_WIDTH,  width)
            set_prop(cap, cv2.CAP_PROP_FRAME_HEIGHT, height)
            set_prop(cap, cv2.CAP_PROP_FPS,          fps)
        else:  # fps-wh
            set_prop(cap, cv2.CAP_PROP_FPS,          fps)
            set_prop(cap, cv2.CAP_PROP_FRAME_WIDTH,  width)
            set_prop(cap, cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Read a frame to let driver lock the mode
        ok, _ = cap.read()
        if not ok:
            return False, "no frame"

        # Query back
        got_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        got_fps = float(cap.get(cv2.CAP_PROP_FPS)) or 0.0
        got_fc  = fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC))

        # Some drivers return reversed FOURCC; accept either
        ok_fourcc = (got_fc == fourcc or got_fc[::-1] == fourcc)
        ok_size   = (got_w == width and got_h == height)
        ok_fps    = (abs(got_fps - fps) <= FPS_TOL) if got_fps > 0 else True  # some drivers don't report FPS

        if strict:
            if not ok_fourcc:
                return False, f"FOURCC mismatch (got '{got_fc}', want '{fourcc}')"
            if not ok_size:
                return False, f"size mismatch (got {got_w}x{got_h}, want {width}x{height})"
            if not ok_fps:
                return False, f"fps mismatch (got {got_fps:.2f}, want {fps}±{FPS_TOL})"

        return True, f"{got_w}x{got_h}@{got_fps:.2f} FOURCC={got_fc}"

    ok, msg = try_order("wh-fps")
    if not ok:
        ok, msg = try_order("fps-wh")

    return ok, msg

def try_open(dev, width, height, fps, fourcc, strict=True, buffersize=1):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None, f"open failed"
    ok, msg = configure_cap(cap, width, height, fps, fourcc, strict=strict, buffersize=buffersize)
    if not ok:
        cap.release()
        return None, msg
    return cap, msg

def open_camera(preferred_dev, preferred_index, width, height, fps, fourcc, strict=True, buffersize=1):
    # 1) explicit device path
    if preferred_dev:
        cap, info = try_open(preferred_dev, width, height, fps, fourcc, strict, buffersize)
        if cap: return cap, preferred_dev, info

    # 2) explicit index
    if preferred_index is not None and preferred_index >= 0:
        cap, info = try_open(preferred_index, width, height, fps, fourcc, strict, buffersize)
        if cap: return cap, preferred_index, info

    # 3) stable by-id paths
    for dev in sorted(glob.glob("/dev/v4l/by-id/*")):
        cap, info = try_open(dev, width, height, fps, fourcc, strict, buffersize)
        if cap: return cap, dev, info

    # 4) numeric fallbacks
    for idx in range(10):
        path = f"/dev/video{idx}"
        if os.path.exists(path):
            cap, info = try_open(idx, width, height, fps, fourcc, strict, buffersize)
            if cap: return cap, idx, info

    return None, None, "no candidate devices matched"

# --------------------------------------------------------------------

def collect_mode(args):
    cap, dev_used, info = open_camera(
        args.dev, args.cam, args.req_width, args.req_height, args.req_fps,
        args.req_fourcc, strict=True, buffersize=args.buffers
    )
    if cap is None:
        print("ERROR: cannot open any camera with the requested mode.", file=sys.stderr)
        print(f"Requested: {args.req_width}x{args.req_height}@{args.req_fps} FOURCC={args.req_fourcc}", file=sys.stderr)
        print("Hint: ensure the camera supports MJPG at that resolution & fps (check with 'v4l2-ctl --list-formats-ext').", file=sys.stderr)
        sys.exit(1)
    print(f"[INFO] Opened camera: {dev_used}  ({info})")

    out_dir = Path(args.out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    saved = len(list(out_dir.glob("img_*.png")))
    pattern_size = (args.pattern_cols, args.pattern_rows)

    last_status = ""
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("WARN: camera read failed"); continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = find_corners(gray, pattern_size)
            vis = frame.copy()
            if found:
                cv2.drawChessboardCorners(vis, pattern_size, corners, found)
                last_status = "Checkerboard detected"
            else:
                last_status = "Checkerboard NOT found"

            vis = draw_hud(vis, saved, f"{last_status}   Mode OK: {info}")
            cv2.imshow("Capture (S=save, C=calibrate, Q=quit)", vis)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord('q'), 27):  # Q or ESC
                break
            elif key == ord('s'):
                if found:
                    ts = time.strftime("%Y%m%d_%H%M%S")
                    path = out_dir / f"img_{ts}_{saved:03d}.png"
                    cv2.imwrite(str(path), frame)
                    saved += 1
                    print(f"Saved {path}")
                else:
                    print("No checkerboard detected; not saving.")
            elif key == ord('c'):
                cap.release()
                cv2.destroyAllWindows()
                calibrate_from_folder(args)
                return
    finally:
        cap.release()
        cv2.destroyAllWindows()

def calibrate_from_folder(args):
    out_dir = Path(args.out_dir)
    images = sorted(glob.glob(str(out_dir / "*.png")) +
                    glob.glob(str(out_dir / "*.jpg")) +
                    glob.glob(str(out_dir / "*.jpeg")))
    if len(images) < 8:
        print(f"Need at least 8 images; found {len(images)} in {out_dir}")
        return

    pattern_size = (args.pattern_cols, args.pattern_rows)
    objp = np.zeros((pattern_size[1]*pattern_size[0], 3), np.float32)
    objp[:,:2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1,2)
    objp *= args.square_size

    objpoints, imgpoints = [], []
    imsize = None

    print(f"Finding corners in {len(images)} images...")
    used = 0
    for fn in images:
        img = cv2.imread(fn)
        if img is None:
            print(f"Skip unreadable {fn}"); continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if imsize is None:
            imsize = gray.shape[::-1]  # (w,h)
        found, corners = find_corners(gray, pattern_size)
        if found:
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            used += 1
        else:
            print(f"  No corners in {fn}, skipped")

    if used < 8:
        print(f"Only {used} usable images; need at least 8. Add more with varied angles/positions.")
        return

    print(f"Calibrating with {used} images at size {imsize} ...")

    flags = cv2.CALIB_RATIONAL_MODEL
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 100, 1e-6)

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, imsize, None, None, flags=flags, criteria=term
    )

    total_err = 0
    total_pts = 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2)
        total_err += err*err
        total_pts += len(objpoints[i])
    rms = np.sqrt(total_err/total_pts)
    print(f"\n=== Calibration Results ===")
    print(f"RMS reprojection error: {rms:.4f} px")
    print(f"K (camera matrix):\n{K}\n")
    print(f"dist (distortion coeffs, length {len(dist.ravel())}):\n{dist.ravel()}\n")

    w, h = imsize
    alpha = args.alpha  # 0 = crop, 1 = keep all FOV
    newK, roi = cv2.getOptimalNewCameraMatrix(
        K, dist, (w,h), alpha=alpha, newImgSize=(w,h), centerPrincipalPoint=True
    )
    print(f"newK (alpha={alpha}):\n{newK}\nROI: {roi}\n")

    save_ros_yaml(K, dist, newK, imsize, args.yaml_out, camera_name=args.camera_name)
    print(f"Saved ROS camera YAML to {args.yaml_out}")

    sample = cv2.imread(images[0])
    map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (w,h), cv2.CV_16SC2)
    undist = cv2.remap(sample, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    x,y,rw,rh = roi
    undist_cropped = undist[y:y+rh, x:x+rw]

    cv2.imshow("Sample (original)", sample)
    cv2.imshow("Sample (undistorted)", undist)
    cv2.imshow("Sample (undistorted, cropped to ROI)", undist_cropped)
    print("Press any key to exit preview...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def save_ros_yaml(K, dist, newK, imsize, out_path, camera_name="see3cam_24cUG"):
    w, h = imsize
    dist = dist.ravel().tolist()
    R = np.eye(3, dtype=float)
    P = np.zeros((3,4), dtype=float)
    P[:3,:3] = newK

    def mat_to_yaml(name, M):
        rows, cols = M.shape
        data = ", ".join([f"{x:.8f}" for x in M.reshape(-1)])
        return f"""{name}:
  rows: {rows}
  cols: {cols}
  data: [{data}]
"""

    with open(out_path, "w") as f:
        f.write(f"""image_width: {w}
image_height: {h}
camera_name: {camera_name}
{mat_to_yaml("camera_matrix", K)}
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: {len(dist)}
  data: [{", ".join([f"{x:.8f}" for x in dist])}]
{mat_to_yaml("rectification_matrix", R)}
{mat_to_yaml("projection_matrix", P)}
""")

def parse_args():
    ap = argparse.ArgumentParser(description="Capture images with 's' and calibrate with 'c' (13x9 inner corners). Enforces MJPG 1920x1200 @114fps.")
    ap.add_argument("--dev", type=str, default="", help="Explicit V4L2 device path (e.g., /dev/v4l/by-id/…); overrides --cam")
    ap.add_argument("--cam", type=int, default=-1, help="Numeric camera index if not using --dev")
    ap.add_argument("--req-width",  type=int, default=REQ_WIDTH,  help="Required width")
    ap.add_argument("--req-height", type=int, default=REQ_HEIGHT, help="Required height")
    ap.add_argument("--req-fps",    type=float, default=REQ_FPS,  help="Required FPS")
    ap.add_argument("--req-fourcc", type=str, default=REQ_FOURCC, help="Required FOURCC (e.g., MJPG)")
    ap.add_argument("--buffers",    type=int, default=1, help="Capture buffer count (if supported)")
    ap.add_argument("--out-dir", type=str, default="rcalib_images", help="Folder to save images")
    ap.add_argument("--yaml-out", type=str, default="rcamera_calib.yaml", help="Output YAML path")
    ap.add_argument("--pattern-cols", type=int, default=13, help="Checkerboard INNER corners along columns (width)")
    ap.add_argument("--pattern-rows", type=int, default=9, help="Checkerboard INNER corners along rows (height)")
    ap.add_argument("--square-size", type=float, default=0.02, help="Square size in meters (only affects translation scale)")
    ap.add_argument("--alpha", type=float, default=0.15, help="Undistort crop/keep FOV [0..1]")
    ap.add_argument("--camera-name", type=str, default="see3cam_24cug", help="ROS camera_name")
    return ap.parse_args()

if __name__ == "__main__":
    args = parse_args()
    Path(args.out_dir).mkdir(parents=True, exist_ok=True)
    collect_mode(args)
