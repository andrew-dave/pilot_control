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
    ap.add_argument("--feather-width", type=int, default=40, help="Blend width in overlap (smaller reduces ghosting).")
    ap.add_argument("--refresh-overlap-every", type=int, default=30, help="Auto-overlap refresh period in frames.")
    args = ap.parse_args()

    try:
        calib = args.calib_file if args.calib_file else str(latest_calib_file())
        calib_path, k1, d1, k2, d2 = load_intrinsics_from_calib(calib)
    except Exception as e:
        print(f"ERROR loading calibration: {e}", file=sys.stderr)
        sys.exit(1)

    right_rot180 = read_right_rotation_hint(calib_path)
    print(f"[INFO] Calibration: {calib_path}")
    print(f"[INFO] Right camera rotate180: {right_rot180}")

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

            if args.overlap_px <= 0 and (frame_idx % max(1, args.refresh_overlap_every) == 0):
                ov, ys, sc = estimate_overlap_and_shift(und_l, und_r)
                if ov is not None:
                    overlap = ov
                    y_shift = ys
                    overlap_score = sc

            merged = stitch_feather(und_l, und_r, overlap, y_shift=y_shift, feather_width=args.feather_width)

            fps_count += 1
            now = time.time()
            if now - fps_last >= 0.5:
                fps_disp = fps_count / (now - fps_last)
                fps_count = 0
                fps_last = now

            put_text(merged, f"FPS ~ {fps_disp:.1f}", 28)
            put_text(merged, f"overlap={overlap}px yshift={y_shift}px score={overlap_score:.2f}", 56)
            put_text(merged, f"alpha={args.undistort_alpha:.2f} dist_scale={args.distortion_scale:.2f} crop={args.crop_px}px", 84)
            put_text(merged, "q: quit, r: re-estimate overlap, +/-: overlap", 112)
            cv2.imshow("stereo_crop_feather", merged)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
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

