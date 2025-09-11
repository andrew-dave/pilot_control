#!/usr/bin/env python3
"""
Open two cameras, undistort/rectify each using their own ROS-style YAML intrinsics,
then merge into a single image based on overlap (feature homography + feather blend).

Tips:
- Use stable by-id device paths with --left-dev/--right-dev (from /dev/v4l/by-id)
- Provide each camera's YAML via --left-yaml/--right-yaml
- Defaults assume MJPG @ 114 fps @ YAML resolution (same as undistort_test.py)
"""
import os, glob, time, argparse, sys
from typing import Tuple, Optional
import numpy as np
import cv2

# ---------- YAML loader ----------
try:
    import yaml
except ImportError:
    print("ERROR: PyYAML not installed. Run: pip install pyyaml", file=sys.stderr)
    sys.exit(1)

# ---------- Defaults ----------
DEFAULT_REQ_FOURCC = "MJPG"
DEFAULT_REQ_FPS = 114.0
FPS_TOL = 2.0


def fourcc_to_str(v: float) -> str:
    v = int(v)
    return "".join([chr((v >> (8*i)) & 0xFF) for i in range(4)])


def load_ros_yaml(path: str):
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

    if "rectification_matrix" in data and "data" in data["rectification_matrix"]:
        R_yaml = np.array(data["rectification_matrix"]["data"], dtype=np.float64).reshape(3,3)
    else:
        R_yaml = np.eye(3, dtype=np.float64)

    newK = None
    if "projection_matrix" in data and "data" in data["projection_matrix"]:
        P = np.array(data["projection_matrix"]["data"], dtype=np.float64).reshape(3,4)
        newK = P[:, :3].copy()

    if model not in ("plumb_bob", "rational_polynomial"):
        raise ValueError(f"Unsupported distortion_model '{model}'. Expected 'plumb_bob' or 'rational_polynomial'.")

    return {
        "width": w, "height": h,
        "K": cm, "D": dist, "R_yaml": R_yaml, "newK_yaml": newK,
        "model": model,
        "camera_name": data.get("camera_name", "camera")
    }


# ---------- Camera helpers ----------
def set_prop(cap: cv2.VideoCapture, prop: int, val) -> bool:
    try:
        return cap.set(prop, val)
    except Exception:
        return False


def configure_cap(cap: cv2.VideoCapture, width: int, height: int, fps: float, fourcc: str,
                  strict: bool=True, buffers: int=1, strict_fps: bool=True) -> Tuple[bool, str]:
    set_prop(cap, cv2.CAP_PROP_BUFFERSIZE, buffers)
    four = cv2.VideoWriter_fourcc(*fourcc)

    def try_order(order: str):
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
        if not ok:
            return False, "no frame from camera"

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


def try_open(dev, width: int, height: int, fps: float, fourcc: str,
             strict: bool=True, buffers: int=1, strict_fps: bool=True):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None, "open failed"
    ok, info = configure_cap(cap, width, height, fps, fourcc, strict, buffers, strict_fps)
    if not ok:
        cap.release()
        return None, info
    return cap, info


def open_camera(preferred_dev, preferred_idx: Optional[int], width: int, height: int, fps: float, fourcc: str,
                strict: bool=True, buffers: int=1, strict_fps: bool=True):
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
        newK, _ = cv2.getOptimalNewCameraMatrix(
            K, D, (W, H), alpha=0.15, newImgSize=(W, H), centerPrincipalPoint=True
        )

    R = R_yaml if R_yaml is not None else np.eye(3, dtype=np.float64)
    map1, map2 = cv2.initUndistortRectifyMap(K, D, R, newK, (W, H), cv2.CV_16SC2)
    return map1, map2, newK


def put_text(img, text, y=30):
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)


# ---------- UI helpers ----------
def fit_window_size(width: int, height: int, max_w: int=960, max_h: int=720) -> Tuple[int, int]:
    """Compute an initial window size that fits within max_w x max_h while preserving aspect."""
    scale = min(max_w/float(width), max_h/float(height), 1.0)
    return int(width*scale), int(height*scale)


# ---------- Stitching helpers ----------
def compute_homography_and_warp(base: np.ndarray, other: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Find homography mapping other->base and warp other into base canvas.
    Returns (warped, H, offset) where offset is translation applied to fit both.
    """
    gray_base = cv2.cvtColor(base, cv2.COLOR_BGR2GRAY)
    gray_other = cv2.cvtColor(other, cv2.COLOR_BGR2GRAY)

    # ORB features for speed; can switch to AKAZE if needed
    orb = cv2.ORB_create(4000)
    kpb, desb = orb.detectAndCompute(gray_base, None)
    kpo, deso = orb.detectAndCompute(gray_other, None)
    if desb is None or deso is None or len(kpb) < 12 or len(kpo) < 12:
        return None

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    knn = matcher.knnMatch(deso, desb, k=2)
    good = []
    for m, n in knn:
        if m.distance < 0.7 * n.distance:
            good.append(m)
    if len(good) < 12:
        return None

    src_pts = np.float32([kpo[m.queryIdx].pt for m in good]).reshape(-1,1,2)
    dst_pts = np.float32([kpb[m.trainIdx].pt for m in good]).reshape(-1,1,2)
    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
    if H is None:
        return None

    h_b, w_b = base.shape[:2]
    h_o, w_o = other.shape[:2]
    corners_o = np.float32([[0,0],[w_o,0],[w_o,h_o],[0,h_o]]).reshape(-1,1,2)
    corners_o_warp = cv2.perspectiveTransform(corners_o, H)
    corners_b = np.float32([[0,0],[w_b,0],[w_b,h_b],[0,h_b]]).reshape(-1,1,2)
    all_corners = np.concatenate((corners_b, corners_o_warp), axis=0)

    x_min, y_min = np.floor(all_corners.min(axis=0).ravel()).astype(int)
    x_max, y_max = np.ceil(all_corners.max(axis=0).ravel()).astype(int)

    tx = -x_min if x_min < 0 else 0
    ty = -y_min if y_min < 0 else 0
    T = np.array([[1,0,tx],[0,1,ty],[0,0,1]], dtype=np.float64)

    size = (x_max - x_min, y_max - y_min)
    warped_other = cv2.warpPerspective(other, T @ H, size)
    base_in_canvas = np.zeros((size[1], size[0], 3), dtype=base.dtype)
    base_in_canvas[ty:ty+h_b, tx:tx+w_b] = base

    # Feather blend using distance maps
    mask_base = np.zeros((size[1], size[0]), dtype=np.uint8)
    mask_other = np.zeros((size[1], size[0]), dtype=np.uint8)
    mask_base[ty:ty+h_b, tx:tx+w_b] = 255
    mask_other[warped_other.sum(axis=2) > 0] = 255

    if np.count_nonzero(mask_base & mask_other) == 0:
        merged = np.where(mask_other[...,None] > 0, warped_other, base_in_canvas)
        return merged.astype(base.dtype), H, np.array([tx, ty])

    dist_b = cv2.distanceTransform((mask_base>0).astype(np.uint8), cv2.DIST_L2, 5)
    dist_o = cv2.distanceTransform((mask_other>0).astype(np.uint8), cv2.DIST_L2, 5)
    w_b = dist_b / (dist_b + dist_o + 1e-6)
    w_o = 1.0 - w_b
    w_b = np.clip(w_b, 0.0, 1.0)
    w_o = np.clip(w_o, 0.0, 1.0)
    merged = (base_in_canvas * w_b[...,None] + warped_other * w_o[...,None]).astype(base.dtype)
    return merged, H, np.array([tx, ty])


def stitch_frames(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    # Try homography mapping right->left; if fails, try left->right
    out = compute_homography_and_warp(left, right)
    if out is not None:
        merged, _, _ = out
        return merged
    out2 = compute_homography_and_warp(right, left)
    if out2 is not None:
        merged, _, _ = out2
        return merged
    # Fallback: simple side-by-side
    h = max(left.shape[0], right.shape[0])
    canvas = np.zeros((h, left.shape[1] + right.shape[1], 3), dtype=left.dtype)
    canvas[:left.shape[0], :left.shape[1]] = left
    canvas[:right.shape[0], left.shape[1]:left.shape[1]+right.shape[1]] = right
    return canvas


# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="Dual-camera undistort with optional stitching")
    ap.add_argument("--left-yaml", type=str, default="lcamera_calib.yaml", help="Left camera ROS YAML")
    ap.add_argument("--right-yaml", type=str, default="rcamera_calib.yaml", help="Right camera ROS YAML")
    ap.add_argument("--left-dev",  type=str, default="", help="Left V4L2 device (e.g., /dev/v4l/by-id/…video-index0)")
    ap.add_argument("--right-dev", type=str, default="", help="Right V4L2 device (e.g., /dev/v4l/by-id/…video-index0)")
    ap.add_argument("--left-cam",  type=int, default=-1, help="Left numeric index if not using --left-dev")
    ap.add_argument("--right-cam", type=int, default=-1, help="Right numeric index if not using --right-dev")
    ap.add_argument("--fourcc", type=str, default=DEFAULT_REQ_FOURCC, help="Required FOURCC (default MJPG)")
    ap.add_argument("--fps",   type=float, default=DEFAULT_REQ_FPS, help="Required FPS (default 114)")
    ap.add_argument("--no-strict-fps", action="store_true", help="Do not strictly enforce reported FPS")
    ap.add_argument("--buffers", type=int, default=1, help="Capture buffer count (if supported)")
    ap.add_argument("--rotate-left-180", action="store_true", help="Rotate left image by 180° for display")
    ap.add_argument("--rotate-right-180", action="store_true", help="Rotate right image by 180° for display")
    ap.add_argument("--show-individual", action="store_true", help="Show individual undistorted feeds alongside merged")
    ap.add_argument("--skip-merge", action="store_true", help="Do not compute or display merged view; show only individual feeds")
    args = ap.parse_args()

    # Load calibrations
    try:
        yml_L = load_ros_yaml(args.left_yaml)
        yml_R = load_ros_yaml(args.right_yaml)
    except Exception as e:
        print(f"ERROR loading YAML: {e}", file=sys.stderr)
        sys.exit(1)

    WL, HL = yml_L["width"], yml_L["height"]
    WR, HR = yml_R["width"], yml_R["height"]
    if (WL, HL) != (WR, HR):
        print(f"WARN: Left YAML size {WL}x{HL} != Right YAML size {WR}x{HR}. Cameras will be opened at their respective sizes; merged output will handle size differences.")

    print(f"[INFO] Loaded left calib:  {WL}x{HL}, model={yml_L['model']}, camera={yml_L['camera_name']}")
    print(f"[INFO] Loaded right calib: {WR}x{HR}, model={yml_R['model']}, camera={yml_R['camera_name']}")

    # Open cameras
    capL, devL, infoL = open_camera(args.left_dev, args.left_cam, WL, HL, args.fps, args.fourcc,
                                     strict=True, buffers=args.buffers, strict_fps=(not args.no_strict_fps))
    if capL is None:
        print("ERROR: cannot open LEFT camera with requested mode.", file=sys.stderr)
        sys.exit(1)
    capR, devR, infoR = open_camera(args.right_dev, args.right_cam, WR, HR, args.fps, args.fourcc,
                                     strict=True, buffers=args.buffers, strict_fps=(not args.no_strict_fps))
    if capR is None:
        print("ERROR: cannot open RIGHT camera with requested mode.", file=sys.stderr)
        capL.release()
        sys.exit(1)
    print(f"[INFO] Opened LEFT  {devL}  ({infoL})")
    print(f"[INFO] Opened RIGHT {devR}  ({infoR})")

    # Build undistortion maps
    map1L, map2L, newKL = build_maps_from_yaml(yml_L)
    map1R, map2R, newKR = build_maps_from_yaml(yml_R)

    if not args.skip_merge:
        cv2.namedWindow("merged", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("merged", cv2.WND_PROP_ASPECT_RATIO, cv2.WINDOW_FREERATIO)
    if args.skip_merge or args.show_individual:
        cv2.namedWindow("left_undist", cv2.WINDOW_NORMAL)
        cv2.namedWindow("right_undist", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("left_undist", cv2.WND_PROP_ASPECT_RATIO, cv2.WINDOW_FREERATIO)
        cv2.setWindowProperty("right_undist", cv2.WND_PROP_ASPECT_RATIO, cv2.WINDOW_FREERATIO)
        iwl, ihl = fit_window_size(WL, HL)
        iwr, ihr = fit_window_size(WR, HR)
        cv2.resizeWindow("left_undist", iwl, ihl)
        cv2.resizeWindow("right_undist", iwr, ihr)

    last = time.time(); frames = 0; fps_disp = 0.0
    while True:
        okL, frameL = capL.read()
        okR, frameR = capR.read()
        if not okL or not okR:
            print("WARN: frame grab failed; retrying cameras in 2s")
            try:
                if capL: capL.release()
                if capR: capR.release()
            except Exception:
                pass
            time.sleep(2.0)
            # keep trying to reopen
            while True:
                capL, devL, infoL = open_camera(args.left_dev, args.left_cam, WL, HL, args.fps, args.fourcc,
                                                strict=True, buffers=args.buffers, strict_fps=(not args.no_strict_fps))
                capR, devR, infoR = open_camera(args.right_dev, args.right_cam, WR, HR, args.fps, args.fourcc,
                                                strict=True, buffers=args.buffers, strict_fps=(not args.no_strict_fps))
                if capL is not None and capR is not None:
                    print(f"[INFO] Reopened LEFT  {devL}  ({infoL})")
                    print(f"[INFO] Reopened RIGHT {devR}  ({infoR})")
                    break
                else:
                    print("WARN: reopen failed; retrying in 2s")
                    time.sleep(2.0)
            continue

        undL = cv2.remap(frameL, map1L, map2L, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        undR = cv2.remap(frameR, map1R, map2R, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        if args.rotate_left_180:
            undL = cv2.rotate(undL, cv2.ROTATE_180)
        if args.rotate_right_180:
            undR = cv2.rotate(undR, cv2.ROTATE_180)

        # If sizes differ, resize both to the smaller height to keep aspect
        if undL.shape[:2] != undR.shape[:2]:
            h_target = min(undL.shape[0], undR.shape[0])
            scaleL = h_target / undL.shape[0]
            scaleR = h_target / undR.shape[0]
            undL = cv2.resize(undL, (int(undL.shape[1]*scaleL), h_target), interpolation=cv2.INTER_AREA)
            undR = cv2.resize(undR, (int(undR.shape[1]*scaleR), h_target), interpolation=cv2.INTER_AREA)

        if not args.skip_merge:
            merged = stitch_frames(undL, undR)

        frames += 1
        now = time.time()
        if now - last >= 0.5:
            fps_disp = frames / (now - last)
            frames = 0
            last = now

        if not args.skip_merge:
            put_text(merged, f"merged  LEFT({infoL})  RIGHT({infoR})")
            put_text(merged, f"FPS ~ {fps_disp:.1f}", y=60)
            cv2.imshow("merged", merged)

        if args.skip_merge or args.show_individual:
            put_text(undL, f"LEFT  ({infoL})")
            put_text(undL, f"FPS ~ {fps_disp:.1f}", y=60)
            put_text(undR, f"RIGHT ({infoR})")
            put_text(undR, f"FPS ~ {fps_disp:.1f}", y=60)
            cv2.imshow("left_undist", undL)
            cv2.imshow("right_undist", undR)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break

    capL.release(); capR.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    cv2.setUseOptimized(True)
    main()


