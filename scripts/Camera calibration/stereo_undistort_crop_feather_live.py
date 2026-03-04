#!/usr/bin/env python3
"""
Live stereo stitch for known intrinsics + known extrinsics.

Pipeline:
1) Read left/right intrinsics from latest calibration (or explicit file),
2) Read rig extrinsics (R, T) from manual YAML,
3) Build stereo-rectification maps once,
4) Remap each frame, crop to valid common ROI,
5) Estimate overlap once (or use fixed overlap),
6) Stitch with a narrow feather blend.
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
CALIB_CAPTURE_SIZE = (1920, 1080)


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


def load_camera_yaml(path):
    if yaml is None:
        raise RuntimeError("PyYAML required. Install with: pip install pyyaml")
    with open(path, "r") as f:
        d = yaml.safe_load(f)
    k = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist = np.array(d["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1)
    return k, dist


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


def load_intrinsics_from_calib(calib_file):
    p = Path(calib_file).expanduser().resolve()
    if p.suffix.lower() == ".npz":
        d = np.load(str(p))
        return p, d["K1"], d["D1"].reshape(-1), d["K2"], d["D2"].reshape(-1)

    if p.suffix.lower() in (".yaml", ".yml"):
        if "stereo_extrinsics" in p.name:
            ly = p.parent / "lcamera_calib.yaml"
            ry = p.parent / "rcamera_calib.yaml"
            if not ly.exists() or not ry.exists():
                raise FileNotFoundError("Expected lcamera_calib.yaml and rcamera_calib.yaml next to stereo_extrinsics.yaml")
            k1, d1 = load_camera_yaml(ly)
            k2, d2 = load_camera_yaml(ry)
            return p, k1, d1, k2, d2
        raise ValueError("For YAML, pass stereo_extrinsics.yaml (not single camera YAML).")

    raise ValueError("Unsupported calibration file type. Use stereo_extrinsics.npz or stereo_extrinsics.yaml")


def read_right_rotation_hint(calib_path):
    meta = Path(calib_path).parent / "right_rotation_180.txt"
    if meta.exists():
        try:
            return meta.read_text().strip() == "1"
        except Exception:
            pass
    return RIGHT_ROTATE_180_DEFAULT


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


def load_manual_extrinsic_yaml(path):
    if yaml is None:
        raise RuntimeError("PyYAML required. Install with: pip install pyyaml")
    with open(path, "r") as f:
        d = yaml.safe_load(f)
    yaw = float(d.get("yaw_deg", 0.0))
    pitch = float(d.get("pitch_deg", 0.0))
    roll = float(d.get("roll_deg", 0.0))
    baseline = float(d.get("baseline_m", 0.128))
    r_block = d.get("r_lr", None)
    t_block = d.get("t_lr_m", None)

    if isinstance(r_block, dict) and "data" in r_block:
        r_lr = np.array(r_block["data"], dtype=np.float64).reshape(3, 3)
    else:
        r_lr = euler_zyx_to_rotation(yaw, pitch, roll)

    if isinstance(t_block, dict) and "data" in t_block:
        t_lr = np.array(t_block["data"], dtype=np.float64).reshape(3, 1)
    else:
        t_lr = np.array([[baseline], [0.0], [0.0]], dtype=np.float64)
    return yaw, pitch, roll, baseline, r_lr, t_lr


def scale_k(k, sx, sy):
    ks = k.copy().astype(np.float64)
    ks[0, 0] *= sx
    ks[0, 2] *= sx
    ks[1, 1] *= sy
    ks[1, 2] *= sy
    return ks


def build_mono_maps(k1, d1, k2, d2, calib_size, out_size, distortion_scale):
    cw, ch = calib_size
    ow, oh = out_size
    sx = float(ow) / float(cw)
    sy = float(oh) / float(ch)
    k1s = scale_k(k1, sx, sy)
    k2s = scale_k(k2, sx, sy)
    d1s = np.array(d1, dtype=np.float64).reshape(-1) * float(distortion_scale)
    d2s = np.array(d2, dtype=np.float64).reshape(-1) * float(distortion_scale)
    n1, _ = cv2.getOptimalNewCameraMatrix(k1s, d1s, (ow, oh), alpha=0.9, newImgSize=(ow, oh))
    n2, _ = cv2.getOptimalNewCameraMatrix(k2s, d2s, (ow, oh), alpha=0.9, newImgSize=(ow, oh))
    m1a, m1b = cv2.initUndistortRectifyMap(k1s, d1s, None, n1, (ow, oh), cv2.CV_16SC2)
    m2a, m2b = cv2.initUndistortRectifyMap(k2s, d2s, None, n2, (ow, oh), cv2.CV_16SC2)
    return m1a, m1b, m2a, m2b


def rect_intersection(a, b):
    ax, ay, aw, ah = [int(v) for v in a]
    bx, by, bw, bh = [int(v) for v in b]
    x0 = max(ax, bx)
    y0 = max(ay, by)
    x1 = min(ax + aw, bx + bw)
    y1 = min(ay + ah, by + bh)
    w = x1 - x0
    h = y1 - y0
    if w <= 1 or h <= 1:
        return None
    return (x0, y0, w, h)


def inset_rect(r, inset):
    if r is None:
        return None
    x, y, w, h = [int(v) for v in r]
    c = max(0, int(inset))
    if w - 2 * c <= 1 or h - 2 * c <= 1:
        return None
    return (x + c, y + c, w - 2 * c, h - 2 * c)


def crop_rect(img, r):
    if r is None:
        return img
    x, y, w, h = [int(v) for v in r]
    ih, iw = img.shape[:2]
    x0 = max(0, min(iw - 1, x))
    y0 = max(0, min(ih - 1, y))
    x1 = max(x0 + 1, min(iw, x + w))
    y1 = max(y0 + 1, min(ih, y + h))
    return img[y0:y1, x0:x1]


def non_black_ratio(img):
    if img.size == 0:
        return 0.0
    nz = np.count_nonzero(np.any(img > 8, axis=2))
    return float(nz) / float(img.shape[0] * img.shape[1])


def build_rectify_maps(k1, d1, k2, d2, r_lr, t_lr, calib_size, out_size, rectify_alpha, distortion_scale):
    cw, ch = calib_size
    ow, oh = out_size
    sx = float(ow) / float(cw)
    sy = float(oh) / float(ch)

    k1s = scale_k(k1, sx, sy)
    k2s = scale_k(k2, sx, sy)
    d1s = np.array(d1, dtype=np.float64).reshape(-1) * float(distortion_scale)
    d2s = np.array(d2, dtype=np.float64).reshape(-1) * float(distortion_scale)
    r = np.array(r_lr, dtype=np.float64).reshape(3, 3)
    t = np.array(t_lr, dtype=np.float64).reshape(3, 1)

    r1, r2, p1, p2, _q, roi1, roi2 = cv2.stereoRectify(
        k1s, d1s, k2s, d2s, (ow, oh), r, t,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=float(rectify_alpha)
    )
    map1_l, map2_l = cv2.initUndistortRectifyMap(k1s, d1s, r1, p1, (ow, oh), cv2.CV_16SC2)
    map1_r, map2_r = cv2.initUndistortRectifyMap(k2s, d2s, r2, p2, (ow, oh), cv2.CV_16SC2)
    return map1_l, map2_l, map1_r, map2_r, roi1, roi2


def estimate_overlap_rectified(left, right):
    gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    h = gl.shape[0]
    y0 = int(0.15 * h)
    y1 = int(0.85 * h)
    gl = cv2.GaussianBlur(gl[y0:y1, :], (5, 5), 0)
    gr = cv2.GaussianBlur(gr[y0:y1, :], (5, 5), 0)
    wl = gl.shape[1]
    wr = gr.shape[1]
    tpl_w = max(80, int(0.25 * wr))
    search_w = max(tpl_w + 80, int(0.50 * wl))
    if search_w >= wl or tpl_w >= search_w:
        return None, 0.0
    template = gr[:, :tpl_w]
    search = gl[:, wl - search_w:]
    res = cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED)
    _, score, _, max_loc = cv2.minMaxLoc(res)
    if score < 0.15:
        return None, score
    x_match = (wl - search_w) + int(max_loc[0])
    overlap = wl - x_match
    if overlap < 40 or overlap > int(0.95 * min(wl, wr)):
        return None, score
    return int(overlap), score


def stitch_rectified(left, right, overlap_px, feather_width):
    if left.shape[0] != right.shape[0]:
        h = min(left.shape[0], right.shape[0])
        wl = int(round(left.shape[1] * (h / left.shape[0])))
        wr = int(round(right.shape[1] * (h / right.shape[0])))
        left = cv2.resize(left, (wl, h), interpolation=cv2.INTER_AREA)
        right = cv2.resize(right, (wr, h), interpolation=cv2.INTER_AREA)

    wl, wr = left.shape[1], right.shape[1]
    ov = int(np.clip(overlap_px, 1, min(wl, wr) - 1))
    out_w = wl + wr - ov
    out = np.zeros((left.shape[0], out_w, 3), dtype=np.uint8)
    out[:, :wl] = left

    l_ov = left[:, wl - ov:wl]
    r_ov = right[:, :ov]
    diff = np.mean(np.abs(l_ov.astype(np.float32) - r_ov.astype(np.float32)), axis=(0, 2))
    if diff.size >= 9:
        diff = np.convolve(diff, np.ones(9, dtype=np.float32) / 9.0, mode="same")
    seam = int(np.argmin(diff))
    fw = int(np.clip(feather_width, 8, ov - 1))
    s0 = max(0, seam - fw // 2)
    s1 = min(ov, s0 + fw)
    s0 = max(0, s1 - fw)

    out[:, wl - ov:wl - ov + s0] = l_ov[:, :s0]
    if s1 > s0:
        lf = l_ov[:, s0:s1].astype(np.float32)
        rf = r_ov[:, s0:s1].astype(np.float32)
        a = np.linspace(1.0, 0.0, s1 - s0, dtype=np.float32)[None, :, None]
        out[:, wl - ov + s0:wl - ov + s1] = (lf * a + rf * (1.0 - a)).astype(np.uint8)
    out[:, wl - ov + s1:wl] = r_ov[:, s1:ov]
    out[:, wl:] = right[:, ov:]
    return out


def put_text(img, text, y):
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description="Known-KDRT live stereo stitch (rectify + feather).")
    ap.add_argument("--calib-file", type=str, default="", help="Optional calibration file (.npz or stereo_extrinsics.yaml)")
    ap.add_argument("--manual-extrinsic-yaml", type=str, default="/home/raj/BDR/pilot_ws/src/pilot_control/config/manual_stereo_extrinsics.yaml", help="Manual extrinsics YAML with r_lr/t_lr_m.")
    ap.add_argument("--rectify-alpha", type=float, default=0.0, help="stereoRectify alpha [0..1], lower reduces black borders.")
    ap.add_argument("--distortion-scale", type=float, default=1.0, help="Scale distortion coefficients [0..1].")
    ap.add_argument("--crop-px", type=int, default=16, help="Extra crop inset after valid ROI.")
    ap.add_argument("--overlap-px", type=int, default=-1, help="Overlap in pixels; -1 estimates once at startup.")
    ap.add_argument("--feather-width", type=int, default=120, help="Feather width in overlap.")
    ap.add_argument("--output-scale", type=float, default=0.65, help="Scale final panorama output [0.2..1.0] for FPS.")
    ap.add_argument("--perf-mode", action="store_true", help="Use low-latency profile.")
    args = ap.parse_args()

    if args.perf_mode:
        args.output_scale = min(args.output_scale, 0.60)
        args.feather_width = min(args.feather_width, 128)

    try:
        calib = args.calib_file if args.calib_file else str(latest_calib_file())
        calib_path, k1, d1, k2, d2 = load_intrinsics_from_calib(calib)
    except Exception as e:
        print(f"ERROR loading calibration: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        yaw, pitch, roll, baseline, r_lr, t_lr = load_manual_extrinsic_yaml(args.manual_extrinsic_yaml)
    except Exception as e:
        print(f"ERROR loading manual extrinsics: {e}", file=sys.stderr)
        sys.exit(1)

    right_rot180 = read_right_rotation_hint(calib_path)
    print(f"[INFO] Calibration: {calib_path}")
    print(f"[INFO] Manual extrinsic: yaw={yaw:.1f} pitch={pitch:.1f} roll={roll:.1f} baseline={baseline:.3f}m")
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

    map1_l, map2_l, map1_r, map2_r, roi_l, roi_r = build_rectify_maps(
        k1, d1, k2, d2, r_lr, t_lr,
        CALIB_CAPTURE_SIZE, (out_w, out_h),
        args.rectify_alpha, args.distortion_scale
    )
    map_mode = "rectified"
    common_roi = rect_intersection(roi_l, roi_r)
    common_roi = inset_rect(common_roi, args.crop_px)
    if common_roi is None:
        common_roi = inset_rect((0, 0, out_w, out_h), args.crop_px)

    und_l0 = cv2.remap(f0_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    und_r0 = cv2.remap(f0_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    und_l0 = crop_rect(und_l0, common_roi)
    und_r0 = crop_rect(und_r0, common_roi)
    nb_l = non_black_ratio(und_l0)
    nb_r = non_black_ratio(und_r0)
    if nb_l < 0.05 or nb_r < 0.05:
        print(
            f"[WARN] Rectified output mostly black (nonblack L/R={nb_l:.3f}/{nb_r:.3f}); switching to undistort-only fallback."
        )
        ds_fallback = min(0.6, max(0.2, args.distortion_scale))
        map1_l, map2_l, map1_r, map2_r = build_mono_maps(k1, d1, k2, d2, CALIB_CAPTURE_SIZE, (out_w, out_h), ds_fallback)
        common_roi = inset_rect((0, 0, out_w, out_h), args.crop_px)
        map_mode = "undistort-only"
        und_l0 = cv2.remap(f0_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        und_r0 = cv2.remap(f0_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        und_l0 = crop_rect(und_l0, common_roi)
        und_r0 = crop_rect(und_r0, common_roi)

    if args.overlap_px > 0:
        overlap = int(args.overlap_px)
        overlap_score = 0.0
    else:
        ov, sc = estimate_overlap_rectified(und_l0, und_r0)
        overlap = ov if ov is not None else max(60, und_l0.shape[1] // 6)
        overlap_score = sc

    cv2.namedWindow("stereo_known_extrinsic_stitch", cv2.WINDOW_NORMAL)
    fps_last = time.time()
    fps_count = 0
    fps_disp = 0.0

    try:
        while True:
            ok_l, frame_l = cap_l.read()
            ok_r, frame_r = cap_r.read()
            if not ok_l or not ok_r:
                time.sleep(0.01)
                continue
            if right_rot180:
                frame_r = cv2.rotate(frame_r, cv2.ROTATE_180)

            if frame_l.shape[1] != out_w or frame_l.shape[0] != out_h:
                frame_l = cv2.resize(frame_l, (out_w, out_h), interpolation=cv2.INTER_AREA)
            if frame_r.shape[1] != out_w or frame_r.shape[0] != out_h:
                frame_r = cv2.resize(frame_r, (out_w, out_h), interpolation=cv2.INTER_AREA)

            und_l = cv2.remap(frame_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            und_r = cv2.remap(frame_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            und_l = crop_rect(und_l, common_roi)
            und_r = crop_rect(und_r, common_roi)

            merged = stitch_rectified(und_l, und_r, overlap, args.feather_width)
            if args.output_scale < 0.999:
                sw = int(max(2, round(merged.shape[1] * args.output_scale)))
                sh = int(max(2, round(merged.shape[0] * args.output_scale)))
                merged = cv2.resize(merged, (sw, sh), interpolation=cv2.INTER_AREA)

            fps_count += 1
            now = time.time()
            if now - fps_last >= 0.5:
                fps_disp = fps_count / (now - fps_last)
                fps_count = 0
                fps_last = now

            put_text(merged, f"FPS ~ {fps_disp:.1f}", 28)
            put_text(merged, f"mode={map_mode} alpha={args.rectify_alpha:.2f} overlap={overlap}px score={overlap_score:.2f}", 56)
            put_text(merged, f"feather={args.feather_width}px scale={args.output_scale:.2f} crop={args.crop_px}px", 84)
            put_text(merged, "q: quit, r: re-estimate overlap, +/-: overlap", 112)
            cv2.imshow("stereo_known_extrinsic_stitch", merged)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                ov, sc = estimate_overlap_rectified(und_l, und_r)
                if ov is not None:
                    overlap = ov
                    overlap_score = sc
            if key in (ord("+"), ord("=")):
                overlap = min(overlap + 10, min(und_l.shape[1], und_r.shape[1]) - 2)
            if key in (ord("-"), ord("_")):
                overlap = max(overlap - 10, 20)
    finally:
        cap_l.release()
        cap_r.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    cv2.setUseOptimized(True)
    main()
