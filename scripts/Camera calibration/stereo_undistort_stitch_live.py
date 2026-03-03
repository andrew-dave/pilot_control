#!/usr/bin/env python3
"""
Live stereo undistort + stitch preview.

Defaults:
- Uses camera devices/video mode from robot_complete.launch.py settings.
- Uses latest calibration from /R_DATA/stereo_calibration/session_*/.

Optional:
- --calib-file can point to:
  1) stereo_extrinsics.npz (preferred), or
  2) stereo_extrinsics.yaml (expects lcamera_calib.yaml + rcamera_calib.yaml in same folder).
"""

import argparse
import glob
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

try:
    import yaml
except ImportError:
    yaml = None


LEFT_DEVICE = "/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_3728140416020900-video-index0"
RIGHT_DEVICE = "/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_0F12140416020900-video-index0"
REQ_WIDTH = 1920
REQ_HEIGHT = 1080
REQ_FPS = 30.0
REQ_FOURCC = "MJPG"
FPS_TOL = 2.0
R_DATA_BASE = Path("/R_DATA/stereo_calibration")


def fourcc_to_str(v: float) -> str:
    v = int(v)
    return "".join([chr((v >> (8 * i)) & 0xFF) for i in range(4)])


def set_prop(cap, prop, value):
    try:
        return cap.set(prop, value)
    except Exception:
        return False


def configure_cap(cap, width, height, fps, fourcc, strict=True):
    req_four = cv2.VideoWriter_fourcc(*fourcc)
    set_prop(cap, cv2.CAP_PROP_BUFFERSIZE, 1)

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
        ok_fps = (got_fps == 0.0) or (abs(got_fps - fps) <= FPS_TOL)

        if strict:
            if not ok_four:
                return False, f"FOURCC mismatch (got {got_fc}, want {fourcc})"
            if not ok_size:
                return False, f"size mismatch (got {got_w}x{got_h}, want {width}x{height})"
            if not ok_fps:
                return False, f"fps mismatch (got {got_fps:.2f}, want {fps}+-{FPS_TOL})"
        return True, f"{got_w}x{got_h}@{got_fps:.2f} FOURCC={got_fc}"

    ok, info = try_order("wh-fps")
    if not ok:
        ok, info = try_order("fps-wh")
    return ok, info


def open_camera(dev_path):
    cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None, "open failed"
    ok, info = configure_cap(cap, REQ_WIDTH, REQ_HEIGHT, REQ_FPS, REQ_FOURCC, strict=True)
    if not ok:
        cap.release()
        return None, info
    return cap, info


def _yaml_matrix(block):
    rows = int(block["rows"])
    cols = int(block["cols"])
    data = np.array(block["data"], dtype=np.float64)
    return data.reshape(rows, cols)


def load_camera_yaml(path):
    if yaml is None:
        raise RuntimeError("PyYAML is required for YAML calibration loading. Install with: pip install pyyaml")
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    k = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    d = np.array(data["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1)
    return k, d


def load_from_npz(path):
    d = np.load(str(path))
    w, h = [int(x) for x in d["image_size"]]
    return {
        "imsize": (w, h),
        "K1": d["K1"],
        "D1": d["D1"].reshape(-1),
        "K2": d["K2"],
        "D2": d["D2"].reshape(-1),
        "R1": d["R1"],
        "R2": d["R2"],
        "P1": d["P1"],
        "P2": d["P2"],
    }


def load_from_stereo_yaml(path):
    if yaml is None:
        raise RuntimeError("PyYAML is required for YAML calibration loading. Install with: pip install pyyaml")
    with open(path, "r") as f:
        s = yaml.safe_load(f)

    p = Path(path).resolve().parent
    left_yaml = p / "lcamera_calib.yaml"
    right_yaml = p / "rcamera_calib.yaml"
    if not left_yaml.exists() or not right_yaml.exists():
        raise FileNotFoundError(
            "Expected lcamera_calib.yaml and rcamera_calib.yaml next to stereo_extrinsics.yaml"
        )

    k1, d1 = load_camera_yaml(left_yaml)
    k2, d2 = load_camera_yaml(right_yaml)

    w = int(s["image_width"])
    h = int(s["image_height"])
    return {
        "imsize": (w, h),
        "K1": k1,
        "D1": d1,
        "K2": k2,
        "D2": d2,
        "R1": _yaml_matrix(s["R1"]),
        "R2": _yaml_matrix(s["R2"]),
        "P1": _yaml_matrix(s["P1"]),
        "P2": _yaml_matrix(s["P2"]),
    }


def get_latest_session_dir(base_dir):
    sessions = [p for p in sorted(base_dir.glob("session_*")) if p.is_dir()]
    return sessions[-1] if sessions else None


def resolve_calibration(calib_file_arg):
    if calib_file_arg:
        path = Path(calib_file_arg).expanduser().resolve()
    else:
        session = get_latest_session_dir(R_DATA_BASE)
        if session is None:
            raise FileNotFoundError(f"No calibration sessions found in {R_DATA_BASE}")
        npz = session / "stereo_extrinsics.npz"
        yml = session / "stereo_extrinsics.yaml"
        if npz.exists():
            path = npz
        elif yml.exists():
            path = yml
        else:
            raise FileNotFoundError(f"No stereo_extrinsics.npz/yaml in latest session: {session}")

    if not path.exists():
        raise FileNotFoundError(f"Calibration file not found: {path}")

    if path.suffix.lower() == ".npz":
        cfg = load_from_npz(path)
    elif path.suffix.lower() in (".yaml", ".yml"):
        cfg = load_from_stereo_yaml(path)
    else:
        raise ValueError("Unsupported calibration file type. Use .npz or .yaml")

    return path, cfg


def build_maps(cfg):
    w, h = cfg["imsize"]
    m1l, m2l = cv2.initUndistortRectifyMap(
        cfg["K1"], cfg["D1"], cfg["R1"], cfg["P1"][:, :3], (w, h), cv2.CV_16SC2
    )
    m1r, m2r = cv2.initUndistortRectifyMap(
        cfg["K2"], cfg["D2"], cfg["R2"], cfg["P2"][:, :3], (w, h), cv2.CV_16SC2
    )
    return (m1l, m2l), (m1r, m2r)


def compute_homography_and_warp(base, other):
    gray_base = cv2.cvtColor(base, cv2.COLOR_BGR2GRAY)
    gray_other = cv2.cvtColor(other, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(3000)
    kpb, desb = orb.detectAndCompute(gray_base, None)
    kpo, deso = orb.detectAndCompute(gray_other, None)
    if desb is None or deso is None or len(kpb) < 12 or len(kpo) < 12:
        return None

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    knn = matcher.knnMatch(deso, desb, k=2)
    good = []
    for m, n in knn:
        if m.distance < 0.75 * n.distance:
            good.append(m)
    if len(good) < 12:
        return None

    src_pts = np.float32([kpo[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst_pts = np.float32([kpb[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    hmat, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
    if hmat is None:
        return None

    hb, wb = base.shape[:2]
    ho, wo = other.shape[:2]
    corners_o = np.float32([[0, 0], [wo, 0], [wo, ho], [0, ho]]).reshape(-1, 1, 2)
    corners_o_warp = cv2.perspectiveTransform(corners_o, hmat)
    corners_b = np.float32([[0, 0], [wb, 0], [wb, hb], [0, hb]]).reshape(-1, 1, 2)
    all_c = np.concatenate((corners_b, corners_o_warp), axis=0)
    x_min, y_min = np.floor(all_c.min(axis=0).ravel()).astype(int)
    x_max, y_max = np.ceil(all_c.max(axis=0).ravel()).astype(int)

    tx = -x_min if x_min < 0 else 0
    ty = -y_min if y_min < 0 else 0
    tmat = np.array([[1, 0, tx], [0, 1, ty], [0, 0, 1]], dtype=np.float64)
    size = (x_max - x_min, y_max - y_min)

    warped_o = cv2.warpPerspective(other, tmat @ hmat, size)
    base_canvas = np.zeros((size[1], size[0], 3), dtype=base.dtype)
    base_canvas[ty:ty + hb, tx:tx + wb] = base

    mask_b = np.zeros((size[1], size[0]), dtype=np.uint8)
    mask_o = np.zeros((size[1], size[0]), dtype=np.uint8)
    mask_b[ty:ty + hb, tx:tx + wb] = 255
    mask_o[warped_o.sum(axis=2) > 0] = 255

    overlap = cv2.bitwise_and(mask_b, mask_o)
    if np.count_nonzero(overlap) == 0:
        merged = np.where(mask_o[..., None] > 0, warped_o, base_canvas)
        return merged

    dist_b = cv2.distanceTransform((mask_b > 0).astype(np.uint8), cv2.DIST_L2, 5)
    dist_o = cv2.distanceTransform((mask_o > 0).astype(np.uint8), cv2.DIST_L2, 5)
    wbw = dist_b / (dist_b + dist_o + 1e-6)
    wow = 1.0 - wbw
    merged = (base_canvas * wbw[..., None] + warped_o * wow[..., None]).astype(base.dtype)
    return merged


def stitch_frames(left, right, cached_h=None):
    if cached_h is not None:
        hmat, tmat, size = cached_h
        warped_o = cv2.warpPerspective(right, tmat @ hmat, size)
        out = np.zeros((size[1], size[0], 3), dtype=left.dtype)
        hb, wb = left.shape[:2]
        tx = int(tmat[0, 2])
        ty = int(tmat[1, 2])
        out[ty:ty + hb, tx:tx + wb] = left
        mask = warped_o.sum(axis=2) > 0
        out[mask] = warped_o[mask]
        return out

    merged = compute_homography_and_warp(left, right)
    if merged is None:
        h = max(left.shape[0], right.shape[0])
        canvas = np.zeros((h, left.shape[1] + right.shape[1], 3), dtype=left.dtype)
        canvas[: left.shape[0], : left.shape[1]] = left
        canvas[: right.shape[0], left.shape[1] : left.shape[1] + right.shape[1]] = right
        return canvas
    return merged


def put_text(img, text, y=30):
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description="Live undistort + stitch using latest stereo calibration.")
    ap.add_argument(
        "--calib-file",
        type=str,
        default="",
        help="Optional calibration file (.npz or .yaml). If omitted, latest session calibration is used.",
    )
    ap.add_argument(
        "--refresh-homography-every",
        type=int,
        default=30,
        help="Re-estimate homography every N frames (default 30).",
    )
    args = ap.parse_args()

    try:
        calib_path, cfg = resolve_calibration(args.calib_file)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"[INFO] Using calibration: {calib_path}")
    print(f"[INFO] Left device:  {LEFT_DEVICE}")
    print(f"[INFO] Right device: {RIGHT_DEVICE}")
    print(f"[INFO] Capture mode: {REQ_WIDTH}x{REQ_HEIGHT}@{REQ_FPS:.0f} {REQ_FOURCC}")

    cap_l, info_l = open_camera(LEFT_DEVICE)
    if cap_l is None:
        print(f"ERROR opening left camera: {info_l}", file=sys.stderr)
        sys.exit(1)
    cap_r, info_r = open_camera(RIGHT_DEVICE)
    if cap_r is None:
        print(f"ERROR opening right camera: {info_r}", file=sys.stderr)
        cap_l.release()
        sys.exit(1)
    print(f"[INFO] Opened left camera  ({info_l})")
    print(f"[INFO] Opened right camera ({info_r})")

    (map1_l, map2_l), (map1_r, map2_r) = build_maps(cfg)
    cv2.namedWindow("stereo_merged", cv2.WINDOW_NORMAL)

    frame_idx = 0
    fps_last = time.time()
    fps_count = 0
    fps_disp = 0.0
    cached = None

    try:
        while True:
            ok_l, frame_l = cap_l.read()
            ok_r, frame_r = cap_r.read()
            if not ok_l or not ok_r:
                print("[WARN] Frame read failed; retrying...")
                time.sleep(0.05)
                continue

            und_l = cv2.remap(frame_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            und_r = cv2.remap(frame_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

            if frame_idx % max(1, args.refresh_homography_every) == 0:
                # Compute cached geometry from current frame pair.
                gray_l = cv2.cvtColor(und_l, cv2.COLOR_BGR2GRAY)
                gray_r = cv2.cvtColor(und_r, cv2.COLOR_BGR2GRAY)
                orb = cv2.ORB_create(3000)
                kpb, desb = orb.detectAndCompute(gray_l, None)
                kpo, deso = orb.detectAndCompute(gray_r, None)
                cached = None
                if desb is not None and deso is not None and len(kpb) >= 12 and len(kpo) >= 12:
                    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
                    knn = matcher.knnMatch(deso, desb, k=2)
                    good = [m for m, n in knn if m.distance < 0.75 * n.distance]
                    if len(good) >= 12:
                        src_pts = np.float32([kpo[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                        dst_pts = np.float32([kpb[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                        hmat, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
                        if hmat is not None:
                            hb, wb = und_l.shape[:2]
                            ho, wo = und_r.shape[:2]
                            corners_o = np.float32([[0, 0], [wo, 0], [wo, ho], [0, ho]]).reshape(-1, 1, 2)
                            corners_o_warp = cv2.perspectiveTransform(corners_o, hmat)
                            corners_b = np.float32([[0, 0], [wb, 0], [wb, hb], [0, hb]]).reshape(-1, 1, 2)
                            all_c = np.concatenate((corners_b, corners_o_warp), axis=0)
                            x_min, y_min = np.floor(all_c.min(axis=0).ravel()).astype(int)
                            x_max, y_max = np.ceil(all_c.max(axis=0).ravel()).astype(int)
                            tx = -x_min if x_min < 0 else 0
                            ty = -y_min if y_min < 0 else 0
                            tmat = np.array([[1, 0, tx], [0, 1, ty], [0, 0, 1]], dtype=np.float64)
                            size = (x_max - x_min, y_max - y_min)
                            cached = (hmat, tmat, size)

            merged = stitch_frames(und_l, und_r, cached_h=cached)

            fps_count += 1
            now = time.time()
            if now - fps_last >= 0.5:
                fps_disp = fps_count / (now - fps_last)
                fps_count = 0
                fps_last = now

            put_text(merged, f"FPS ~ {fps_disp:.1f}")
            put_text(merged, "q: quit, r: reset homography", y=58)
            cv2.imshow("stereo_merged", merged)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                cached = None
            frame_idx += 1

    finally:
        cap_l.release()
        cap_r.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    cv2.setUseOptimized(True)
    main()
