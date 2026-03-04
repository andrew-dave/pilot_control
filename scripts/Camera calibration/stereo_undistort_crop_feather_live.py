#!/usr/bin/env python3
"""
Live dual-camera panorama:
1) Undistort each camera independently (mono intrinsics only),
2) Crop borders,
3) Stitch with horizontal feather blending.

This intentionally avoids stereo extrinsics/depth and is designed to be
robust for "single wide-view" visualization.
"""

import argparse
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
RIGHT_ROTATE_180_DEFAULT = True


def fourcc_to_str(v: float) -> str:
    v = int(v)
    return "".join([chr((v >> (8 * i)) & 0xFF) for i in range(4)])


def set_prop(cap, prop, value):
    try:
        return cap.set(prop, value)
    except Exception:
        return False


def configure_cap(cap, width, height, fps, fourcc):
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


def open_camera(dev):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None, "open failed"
    ok, info = configure_cap(cap, REQ_WIDTH, REQ_HEIGHT, REQ_FPS, REQ_FOURCC)
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
        raise RuntimeError("PyYAML required for YAML loading. Install with: pip install pyyaml")
    with open(path, "r") as f:
        d = yaml.safe_load(f)
    k = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist = np.array(d["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1)
    return k, dist


def load_manual_extrinsic_yaml(path):
    if yaml is None:
        raise RuntimeError("PyYAML required for YAML loading. Install with: pip install pyyaml")
    with open(path, "r") as f:
        d = yaml.safe_load(f)
    yaw = float(d.get("yaw_deg", 90.0))
    pitch = float(d.get("pitch_deg", 0.0))
    roll = float(d.get("roll_deg", 0.0))
    baseline = float(d.get("baseline_m", 0.128))
    r_block = d.get("r_lr", None)
    t_block = d.get("t_lr_m", None)
    r_lr = None
    t_lr = None
    if isinstance(r_block, dict) and "data" in r_block:
        r_lr = np.array(r_block["data"], dtype=np.float64).reshape(3, 3)
    if isinstance(t_block, dict) and "data" in t_block:
        t_lr = np.array(t_block["data"], dtype=np.float64).reshape(3, 1)
    return yaw, pitch, roll, baseline, r_lr, t_lr


def load_intrinsics_from_calib(calib_file):
    p = Path(calib_file).expanduser().resolve()
    if p.suffix.lower() == ".npz":
        d = np.load(str(p))
        return p, d["K1"], d["D1"].reshape(-1), d["K2"], d["D2"].reshape(-1)

    if p.suffix.lower() in (".yaml", ".yml"):
        # If stereo YAML given, load mono YAMLs from same folder.
        if "stereo_extrinsics" in p.name:
            ly = p.parent / "lcamera_calib.yaml"
            ry = p.parent / "rcamera_calib.yaml"
            if not ly.exists() or not ry.exists():
                raise FileNotFoundError("Expected lcamera_calib.yaml and rcamera_calib.yaml next to stereo_extrinsics.yaml")
            k1, d1 = load_camera_yaml(ly)
            k2, d2 = load_camera_yaml(ry)
            return p, k1, d1, k2, d2
        # Otherwise treat as single camera YAML not supported here.
        raise ValueError("For YAML, pass stereo_extrinsics.yaml (not single camera YAML).")

    raise ValueError("Unsupported calibration file type. Use stereo_extrinsics.npz or stereo_extrinsics.yaml")


def latest_calib_file():
    sessions = [p for p in sorted(R_DATA_BASE.glob("session_*")) if p.is_dir()]
    if not sessions:
        raise FileNotFoundError(f"No calibration sessions found in {R_DATA_BASE}")
    s = sessions[-1]
    npz = s / "stereo_extrinsics.npz"
    yml = s / "stereo_extrinsics.yaml"
    if npz.exists():
        return npz
    if yml.exists():
        return yml
    raise FileNotFoundError(f"No stereo_extrinsics.npz/yaml in latest session: {s}")


def read_right_rotation_hint(calib_path):
    meta = Path(calib_path).parent / "right_rotation_180.txt"
    if meta.exists():
        try:
            return meta.read_text().strip() == "1"
        except Exception:
            pass
    return RIGHT_ROTATE_180_DEFAULT


def scale_k(k, sx, sy):
    ks = k.copy().astype(np.float64)
    ks[0, 0] *= sx
    ks[0, 2] *= sx
    ks[1, 1] *= sy
    ks[1, 2] *= sy
    return ks


def euler_zyx_to_rotation(yaw_deg, pitch_deg, roll_deg):
    y = np.deg2rad(float(yaw_deg))
    p = np.deg2rad(float(pitch_deg))
    r = np.deg2rad(float(roll_deg))
    cz, sz = np.cos(y), np.sin(y)
    cy, sy = np.cos(p), np.sin(p)
    cx, sx = np.cos(r), np.sin(r)
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
    return rz @ ry @ rx


def build_mono_map(k, d, calib_size, out_size, alpha=0.9, distortion_scale=0.5):
    cw, ch = calib_size
    ow, oh = out_size
    sx = float(ow) / float(cw)
    sy = float(oh) / float(ch)
    ks = scale_k(k, sx, sy)
    ds = np.array(d, dtype=np.float64).reshape(-1) * float(distortion_scale)
    newk, _ = cv2.getOptimalNewCameraMatrix(ks, ds, (ow, oh), alpha=float(alpha), newImgSize=(ow, oh))
    map1, map2 = cv2.initUndistortRectifyMap(ks, ds, None, newk, (ow, oh), cv2.CV_16SC2)
    return map1, map2


def crop_border(img, crop_px):
    if crop_px <= 0:
        return img
    h, w = img.shape[:2]
    c = int(min(crop_px, (min(h, w) // 2) - 1))
    if c <= 0:
        return img
    return img[c:h - c, c:w - c]


def estimate_overlap_and_shift(left, right, max_vshift=80):
    gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    h = gl.shape[0]
    y0 = int(0.15 * h)
    y1 = int(0.85 * h)
    gl = gl[y0:y1, :]
    gr = gr[y0:y1, :]
    wl = gl.shape[1]
    wr = gr.shape[1]
    tpl_w = max(100, int(0.30 * wr))
    search_w = max(tpl_w + 100, int(0.55 * wl))
    if search_w >= wl or tpl_w >= search_w:
        return None, 0, 0.0
    template = cv2.GaussianBlur(gr[:, :tpl_w], (5, 5), 0)
    search_full = cv2.GaussianBlur(gl[:, wl - search_w:], (5, 5), 0)

    # Restrict vertical search to avoid bad matches.
    pad = int(min(max_vshift, search_full.shape[0] // 4))
    if pad > 0:
        search = search_full
    else:
        search = search_full

    res = cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED)
    _, score, _, max_loc = cv2.minMaxLoc(res)
    if score < 0.18:
        return None, 0, score
    x_match = (wl - search_w) + int(max_loc[0])
    y_shift = int(max_loc[1])  # right must be shifted down by y_shift in the cropped band
    y_shift -= 0  # explicit no-op for readability
    # Convert shift to full-image coordinates (same because both cropped equally).
    y_shift = int(max_loc[1])
    # Since template starts at row 0 and search at row 0 in cropped views, center around zero.
    y_shift = y_shift - 0
    # Bound y shift.
    y_shift = int(np.clip(y_shift, -max_vshift, max_vshift))

    overlap = wl - x_match
    if overlap < 60 or overlap > int(0.95 * min(wl, wr)):
        return None, 0, score
    return int(overlap), y_shift, score


def stitch_feather(left, right, overlap_px, y_shift=0, feather_width=40):
    if left.shape[0] != right.shape[0]:
        h = min(left.shape[0], right.shape[0])
        wl = int(round(left.shape[1] * (h / left.shape[0])))
        wr = int(round(right.shape[1] * (h / right.shape[0])))
        left = cv2.resize(left, (wl, h), interpolation=cv2.INTER_AREA)
        right = cv2.resize(right, (wr, h), interpolation=cv2.INTER_AREA)

    # Apply small vertical shift to right image on a padded canvas.
    if y_shift != 0:
        h, w = right.shape[:2]
        shifted = np.zeros_like(right)
        if y_shift > 0:
            shifted[y_shift:, :] = right[:h - y_shift, :]
        else:
            ys = -y_shift
            shifted[:h - ys, :] = right[ys:, :]
        right = shifted

    wl, wr = left.shape[1], right.shape[1]
    ov = int(np.clip(overlap_px, 1, min(wl, wr) - 1))
    out_w = wl + wr - ov
    out = np.zeros((left.shape[0], out_w, 3), dtype=left.dtype)
    out[:, :wl] = left

    # Choose seam near best overlap region but keep feather narrow to reduce ghosting.
    l_ov = left[:, wl - ov:wl]
    r_ov = right[:, :ov]
    diff = np.mean(np.abs(l_ov.astype(np.float32) - r_ov.astype(np.float32)), axis=(0, 2))
    if diff.size >= 9:
        k = np.ones(9, dtype=np.float32) / 9.0
        smooth = np.convolve(diff, k, mode="same")
    else:
        smooth = diff
    seam = int(np.argmin(smooth))
    fw = int(np.clip(feather_width, 4, ov - 1))
    s0 = max(0, seam - fw // 2)
    s1 = min(ov, s0 + fw)
    s0 = max(0, s1 - fw)

    # Left-dominant region
    out[:, wl - ov:wl - ov + s0] = l_ov[:, :s0]
    # Narrow feather zone
    if s1 > s0:
        lf = l_ov[:, s0:s1].astype(np.float32)
        rf = r_ov[:, s0:s1].astype(np.float32)
        a = np.linspace(1.0, 0.0, s1 - s0, dtype=np.float32)[None, :, None]
        b = 1.0 - a
        out[:, wl - ov + s0:wl - ov + s1] = (lf * a + rf * b).astype(left.dtype)
    # Right-dominant overlap tail
    out[:, wl - ov + s1:wl] = r_ov[:, s1:ov]
    out[:, wl:] = right[:, ov:]
    return out


def stitch_by_manual_rotation(left, right, r_lr, feather_width=120):
    h, w = left.shape[:2]
    # Virtual camera for rotation homography.
    f = float(max(w, h))
    k = np.array([[f, 0.0, w * 0.5], [0.0, f, h * 0.5], [0.0, 0.0, 1.0]], dtype=np.float64)
    hmat = k @ r_lr @ np.linalg.inv(k)

    corners_r = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    corners_rw = cv2.perspectiveTransform(corners_r, hmat)
    corners_l = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    all_c = np.concatenate((corners_l, corners_rw), axis=0)
    x_min, y_min = np.floor(all_c.min(axis=0).ravel()).astype(int)
    x_max, y_max = np.ceil(all_c.max(axis=0).ravel()).astype(int)
    tx = -x_min if x_min < 0 else 0
    ty = -y_min if y_min < 0 else 0
    tmat = np.array([[1.0, 0.0, tx], [0.0, 1.0, ty], [0.0, 0.0, 1.0]], dtype=np.float64)
    out_w = int(x_max - x_min)
    out_h = int(y_max - y_min)
    if out_w <= 0 or out_h <= 0:
        return np.hstack((left, right))

    right_warp = cv2.warpPerspective(right, tmat @ hmat, (out_w, out_h))
    out = np.zeros((out_h, out_w, 3), dtype=left.dtype)
    out[ty:ty + h, tx:tx + w] = left

    mask_l = np.zeros((out_h, out_w), dtype=np.uint8)
    mask_r = np.zeros((out_h, out_w), dtype=np.uint8)
    mask_l[ty:ty + h, tx:tx + w] = 255
    mask_r[right_warp.sum(axis=2) > 0] = 255
    overlap = (mask_l > 0) & (mask_r > 0)
    out[mask_r > 0] = right_warp[mask_r > 0]

    if np.count_nonzero(overlap) > 0:
        ys, xs = np.where(overlap)
        x0 = int(xs.min())
        x1 = int(xs.max()) + 1
        fw = int(np.clip(feather_width, 4, max(5, x1 - x0)))
        cx = (x0 + x1) // 2
        b0 = max(x0, cx - fw // 2)
        b1 = min(x1, b0 + fw)
        b0 = max(x0, b1 - fw)
        if b1 > b0:
            l = out[:, b0:b1].astype(np.float32)
            r = right_warp[:, b0:b1].astype(np.float32)
            a = np.linspace(1.0, 0.0, b1 - b0, dtype=np.float32)[None, :, None]
            out[:, b0:b1] = (l * a + r * (1.0 - a)).astype(out.dtype)
    return out


def _mask_bbox(mask_u8):
    ys, xs = np.where(mask_u8 > 0)
    if ys.size == 0 or xs.size == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def build_manual_stitcher(frame_shape, r_lr, feather_width, output_scale):
    h, w = frame_shape[:2]
    f = float(max(w, h))
    k = np.array([[f, 0.0, w * 0.5], [0.0, f, h * 0.5], [0.0, 0.0, 1.0]], dtype=np.float64)
    hmat = k @ r_lr @ np.linalg.inv(k)

    corners_r = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    corners_rw = cv2.perspectiveTransform(corners_r, hmat)
    corners_l = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    all_c = np.concatenate((corners_l, corners_rw), axis=0)
    x_min, y_min = np.floor(all_c.min(axis=0).ravel()).astype(int)
    x_max, y_max = np.ceil(all_c.max(axis=0).ravel()).astype(int)
    tx = -x_min if x_min < 0 else 0
    ty = -y_min if y_min < 0 else 0
    out_w = int(x_max - x_min)
    out_h = int(y_max - y_min)
    if out_w <= 1 or out_h <= 1:
        return None
    m_right = np.array([[1.0, 0.0, tx], [0.0, 1.0, ty], [0.0, 0.0, 1.0]], dtype=np.float64) @ hmat

    left_mask = np.zeros((out_h, out_w), dtype=np.uint8)
    left_mask[ty:ty + h, tx:tx + w] = 255
    ones = np.full((h, w), 255, dtype=np.uint8)
    right_mask = cv2.warpPerspective(ones, m_right, (out_w, out_h), flags=cv2.INTER_NEAREST)
    union_mask = np.where((left_mask > 0) | (right_mask > 0), 255, 0).astype(np.uint8)
    overlap_mask = (left_mask > 0) & (right_mask > 0)

    w_l = (left_mask > 0).astype(np.float32)
    w_r = (right_mask > 0).astype(np.float32)
    overlap_bbox = _mask_bbox(overlap_mask.astype(np.uint8) * 255)
    if overlap_bbox is not None:
        x0, _, x1, _ = overlap_bbox
        span = max(1, x1 - x0)
        fw = int(np.clip(feather_width, 8, span))
        cx = (x0 + x1) // 2
        b0 = max(x0, cx - fw // 2)
        b1 = min(x1, b0 + fw)
        b0 = max(x0, b1 - fw)
        if b1 > b0:
            ramp = np.linspace(1.0, 0.0, b1 - b0, dtype=np.float32)[None, :]
            ov_slice = overlap_mask[:, b0:b1]
            w_l[:, b0:b1][ov_slice] = np.broadcast_to(ramp, (out_h, b1 - b0))[ov_slice]
            w_r[:, b0:b1][ov_slice] = 1.0 - w_l[:, b0:b1][ov_slice]
        if b0 > x0:
            pre = overlap_mask[:, x0:b0]
            w_l[:, x0:b0][pre] = 1.0
            w_r[:, x0:b0][pre] = 0.0
        if b1 < x1:
            post = overlap_mask[:, b1:x1]
            w_l[:, b1:x1][post] = 0.0
            w_r[:, b1:x1][post] = 1.0

    denom = w_l + w_r
    valid = denom > 1e-6
    inv_denom = np.zeros_like(denom, dtype=np.float32)
    inv_denom[valid] = 1.0 / denom[valid]

    crop_bbox = _mask_bbox(union_mask)
    if crop_bbox is None:
        crop_bbox = (0, 0, out_w, out_h)

    scale = float(max(0.2, min(1.0, output_scale)))
    scaled_size = (
        int(max(2, round((crop_bbox[2] - crop_bbox[0]) * scale))),
        int(max(2, round((crop_bbox[3] - crop_bbox[1]) * scale))),
    )

    return {
        "out_w": out_w,
        "out_h": out_h,
        "left_x": tx,
        "left_y": ty,
        "left_w": w,
        "left_h": h,
        "m_right": m_right,
        "w_l": w_l,
        "w_r": w_r,
        "valid": valid,
        "inv_denom": inv_denom,
        "crop_bbox": crop_bbox,
        "scaled_size": scaled_size,
    }


def stitch_with_precomputed_manual(left, right, cfg):
    out_h = cfg["out_h"]
    out_w = cfg["out_w"]
    left_canvas = np.zeros((out_h, out_w, 3), dtype=np.uint8)
    x = cfg["left_x"]
    y = cfg["left_y"]
    h = cfg["left_h"]
    w = cfg["left_w"]
    left_canvas[y:y + h, x:x + w] = left
    right_canvas = cv2.warpPerspective(right, cfg["m_right"], (out_w, out_h))

    w_l = cfg["w_l"][:, :, None]
    w_r = cfg["w_r"][:, :, None]
    inv = cfg["inv_denom"][:, :, None]
    merged = (left_canvas.astype(np.float32) * w_l + right_canvas.astype(np.float32) * w_r) * inv
    out = merged.astype(np.uint8)
    out[~cfg["valid"]] = 0

    x0, y0, x1, y1 = cfg["crop_bbox"]
    out = out[y0:y1, x0:x1]
    sw, sh = cfg["scaled_size"]
    if out.shape[1] != sw or out.shape[0] != sh:
        out = cv2.resize(out, (sw, sh), interpolation=cv2.INTER_AREA)
    return out


def put_text(img, text, y):
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description="Live mono-undistort + crop + feather panorama.")
    ap.add_argument("--calib-file", type=str, default="", help="Optional calibration file (.npz or stereo_extrinsics.yaml)")
    ap.add_argument("--square-size", type=float, default=0.021, help="Reserved (unused); kept for CLI compatibility.")
    ap.add_argument("--undistort-alpha", type=float, default=0.9, help="Undistort alpha [0..1], higher keeps more FOV.")
    ap.add_argument("--distortion-scale", type=float, default=0.5, help="Scale distortion coefficients [0..1].")
    ap.add_argument("--crop-px", type=int, default=24, help="Pixels cropped from each border after undistort.")
    ap.add_argument("--overlap-px", type=int, default=-1, help="Feather overlap in pixels; -1 for auto.")
    ap.add_argument("--feather-width", type=int, default=900, help="Blend width in overlap.")
    ap.add_argument("--refresh-overlap-every", type=int, default=0, help="Auto-overlap refresh period in frames (0 disables).")
    ap.add_argument("--use-manual-extrinsic", action="store_true", help="Use manual rotation extrinsic stitching instead of overlap estimator.")
    ap.add_argument("--manual-extrinsic-yaml", type=str, default="/home/raj/BDR/pilot_ws/src/pilot_control/config/manual_stereo_extrinsics.yaml", help="Manual extrinsics YAML (yaw/pitch/roll/baseline).")
    ap.add_argument("--output-scale", type=float, default=0.7, help="Scale final panorama output [0.2..1.0] for FPS.")
    ap.add_argument("--perf-mode", action="store_true", help="Performance profile (narrow blend + lower output scale).")
    args = ap.parse_args()
    if args.perf_mode:
        args.output_scale = min(args.output_scale, 0.6)
        args.feather_width = min(args.feather_width, 160)

    try:
        calib = args.calib_file if args.calib_file else str(latest_calib_file())
        calib_path, k1, d1, k2, d2 = load_intrinsics_from_calib(calib)
    except Exception as e:
        print(f"ERROR loading calibration: {e}", file=sys.stderr)
        sys.exit(1)

    right_rot180 = read_right_rotation_hint(calib_path)
    print(f"[INFO] Calibration: {calib_path}")
    print(f"[INFO] Right camera rotate180: {right_rot180}")
    manual_r = None
    manual_baseline = 0.0
    if args.use_manual_extrinsic:
        try:
            yaw, pitch, roll, manual_baseline, r_from_yaml, _ = load_manual_extrinsic_yaml(args.manual_extrinsic_yaml)
            manual_r = r_from_yaml if r_from_yaml is not None else euler_zyx_to_rotation(yaw, pitch, roll)
            print(
                f"[INFO] Manual extrinsic enabled: yaw={yaw:.1f} pitch={pitch:.1f} roll={roll:.1f} baseline={manual_baseline:.3f}m"
            )
        except Exception as e:
            print(f"ERROR loading manual extrinsics: {e}", file=sys.stderr)
            sys.exit(1)

    cap_l, info_l = open_camera(LEFT_DEVICE)
    if cap_l is None:
        print(f"ERROR opening left camera: {info_l}", file=sys.stderr)
        sys.exit(1)
    cap_r, info_r = open_camera(RIGHT_DEVICE)
    if cap_r is None:
        print(f"ERROR opening right camera: {info_r}", file=sys.stderr)
        cap_l.release()
        sys.exit(1)
    print(f"[INFO] Opened left:  {info_l}")
    print(f"[INFO] Opened right: {info_r}")

    ok_l, f0_l = cap_l.read()
    ok_r, f0_r = cap_r.read()
    if not ok_l or not ok_r:
        print("ERROR: failed initial frame read", file=sys.stderr)
        cap_l.release()
        cap_r.release()
        sys.exit(1)
    if right_rot180:
        f0_r = cv2.rotate(f0_r, cv2.ROTATE_180)

    out_w = min(f0_l.shape[1], f0_r.shape[1])
    out_h = min(f0_l.shape[0], f0_r.shape[0])
    calib_size = (1920, 1080)  # this project's calibration capture size
    map1_l, map2_l = build_mono_map(k1, d1, calib_size, (out_w, out_h), args.undistort_alpha, args.distortion_scale)
    map1_r, map2_r = build_mono_map(k2, d2, calib_size, (out_w, out_h), args.undistort_alpha, args.distortion_scale)

    cv2.namedWindow("stereo_crop_feather", cv2.WINDOW_NORMAL)
    fps_last = time.time()
    fps_count = 0
    fps_disp = 0.0
    frame_idx = 0
    overlap = max(100, out_w // 8) if args.overlap_px <= 0 else int(args.overlap_px)
    y_shift = 0
    overlap_score = 0.0
    manual_cfg = None

    if args.use_manual_extrinsic and manual_r is not None:
        und_l0 = cv2.remap(f0_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        und_r0 = cv2.remap(f0_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        und_l0 = crop_border(und_l0, args.crop_px)
        und_r0 = crop_border(und_r0, args.crop_px)
        manual_cfg = build_manual_stitcher(und_l0.shape, manual_r, args.feather_width, args.output_scale)
        if manual_cfg is None:
            print("ERROR: failed to build manual stitch configuration", file=sys.stderr)
            cap_l.release()
            cap_r.release()
            sys.exit(1)
    elif args.overlap_px <= 0:
        und_l0 = cv2.remap(f0_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        und_r0 = cv2.remap(f0_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        und_l0 = crop_border(und_l0, args.crop_px)
        und_r0 = crop_border(und_r0, args.crop_px)
        ov0, ys0, sc0 = estimate_overlap_and_shift(und_l0, und_r0)
        if ov0 is not None:
            overlap = ov0
            y_shift = ys0
            overlap_score = sc0

    try:
        while True:
            ok_l, frame_l = cap_l.read()
            ok_r, frame_r = cap_r.read()
            if not ok_l or not ok_r:
                time.sleep(0.02)
                continue
            if right_rot180:
                frame_r = cv2.rotate(frame_r, cv2.ROTATE_180)

            if frame_l.shape[1] != out_w or frame_l.shape[0] != out_h:
                frame_l = cv2.resize(frame_l, (out_w, out_h), interpolation=cv2.INTER_AREA)
            if frame_r.shape[1] != out_w or frame_r.shape[0] != out_h:
                frame_r = cv2.resize(frame_r, (out_w, out_h), interpolation=cv2.INTER_AREA)

            und_l = cv2.remap(frame_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            und_r = cv2.remap(frame_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            und_l = crop_border(und_l, args.crop_px)
            und_r = crop_border(und_r, args.crop_px)

            if (not args.use_manual_extrinsic) and args.overlap_px <= 0 and args.refresh_overlap_every > 0 and (frame_idx % max(1, args.refresh_overlap_every) == 0):
                ov, ys, sc = estimate_overlap_and_shift(und_l, und_r)
                if ov is not None:
                    overlap = ov
                    y_shift = ys
                    overlap_score = sc

            if args.use_manual_extrinsic and manual_cfg is not None:
                merged = stitch_with_precomputed_manual(und_l, und_r, manual_cfg)
            else:
                merged = stitch_feather(und_l, und_r, overlap, y_shift=y_shift, feather_width=args.feather_width)

            fps_count += 1
            now = time.time()
            if now - fps_last >= 0.5:
                fps_disp = fps_count / (now - fps_last)
                fps_count = 0
                fps_last = now

            put_text(merged, f"FPS ~ {fps_disp:.1f}", 28)
            if args.use_manual_extrinsic:
                put_text(merged, f"manual extrinsic baseline={manual_baseline:.3f}m scale={args.output_scale:.2f}", 56)
            else:
                put_text(merged, f"overlap={overlap}px yshift={y_shift}px score={overlap_score:.2f}", 56)
            put_text(merged, f"alpha={args.undistort_alpha:.2f} dist_scale={args.distortion_scale:.2f} crop={args.crop_px}px", 84)
            put_text(merged, "q: quit, r: re-estimate overlap, +/-: overlap", 112)
            cv2.imshow("stereo_crop_feather", merged)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r") and (not args.use_manual_extrinsic):
                ov, ys, sc = estimate_overlap_and_shift(und_l, und_r)
                if ov is not None:
                    overlap = ov
                    y_shift = ys
                    overlap_score = sc
            if key in (ord("+"), ord("=")):
                overlap = min(overlap + 10, min(und_l.shape[1], und_r.shape[1]) - 2)
            if key in (ord("-"), ord("_")):
                overlap = max(overlap - 10, 20)
            frame_idx += 1
    finally:
        cap_l.release()
        cap_r.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    cv2.setUseOptimized(True)
    main()

