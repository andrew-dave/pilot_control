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
RIGHT_ROTATE_180_DEFAULT = True


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
    r = d["R"] if "R" in d else None
    t = d["T"] if "T" in d else None
    return {
        "imsize": (w, h),
        "K1": d["K1"],
        "D1": d["D1"].reshape(-1),
        "K2": d["K2"],
        "D2": d["D2"].reshape(-1),
        "R": r,
        "T": t,
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
    r = _yaml_matrix(s["R"]) if "R" in s else None
    t = _yaml_matrix(s["T"]).reshape(3, 1) if "T" in s else None
    return {
        "imsize": (w, h),
        "K1": k1,
        "D1": d1,
        "K2": k2,
        "D2": d2,
        "R": r,
        "T": t,
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


def read_right_rotation_hint(calib_path: Path):
    meta = calib_path.parent / "right_rotation_180.txt"
    if meta.exists():
        try:
            return meta.read_text().strip() == "1"
        except Exception:
            pass
    return RIGHT_ROTATE_180_DEFAULT


def apply_right_rotation(img, rotate180):
    if rotate180:
        return cv2.rotate(img, cv2.ROTATE_180)
    return img


def _scale_k(k, sx, sy):
    ks = k.copy().astype(np.float64)
    ks[0, 0] *= sx
    ks[0, 2] *= sx
    ks[1, 1] *= sy
    ks[1, 2] *= sy
    return ks


def build_maps(cfg, out_size, prefer_rectify=True):
    cw, ch = cfg["imsize"]
    ow, oh = out_size
    sx = float(ow) / float(cw)
    sy = float(oh) / float(ch)

    k1 = _scale_k(cfg["K1"], sx, sy)
    k2 = _scale_k(cfg["K2"], sx, sy)
    d1 = cfg["D1"]
    d2 = cfg["D2"]

    if prefer_rectify and cfg.get("R") is not None and cfg.get("T") is not None:
        r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(
            k1,
            d1,
            k2,
            d2,
            (ow, oh),
            cfg["R"],
            cfg["T"],
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0.0,
        )
        m1l, m2l = cv2.initUndistortRectifyMap(k1, d1, r1, p1[:, :3], (ow, oh), cv2.CV_16SC2)
        m1r, m2r = cv2.initUndistortRectifyMap(k2, d2, r2, p2[:, :3], (ow, oh), cv2.CV_16SC2)
        return (m1l, m2l), (m1r, m2r), "rectified"

    new_k1, _ = cv2.getOptimalNewCameraMatrix(k1, d1, (ow, oh), alpha=0.0, newImgSize=(ow, oh))
    new_k2, _ = cv2.getOptimalNewCameraMatrix(k2, d2, (ow, oh), alpha=0.0, newImgSize=(ow, oh))
    m1l, m2l = cv2.initUndistortRectifyMap(k1, d1, None, new_k1, (ow, oh), cv2.CV_16SC2)
    m1r, m2r = cv2.initUndistortRectifyMap(k2, d2, None, new_k2, (ow, oh), cv2.CV_16SC2)
    return (m1l, m2l), (m1r, m2r), "undistort-only"


def estimate_horizontal_offset(left, right):
    # Estimate where right[:,0] should start in left image coordinates.
    gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

    h = gl.shape[0]
    y0 = int(0.15 * h)
    y1 = int(0.85 * h)
    gl = gl[y0:y1, :]
    gr = gr[y0:y1, :]

    scale = 1.0
    if max(gl.shape) > 960:
        scale = 0.5
        gl = cv2.resize(gl, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        gr = cv2.resize(gr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    wl = gl.shape[1]
    wr = gr.shape[1]
    tpl_w = max(120, int(0.35 * wr))
    search_w = max(tpl_w + 80, int(0.60 * wl))
    if search_w >= wl:
        search_w = wl - 1
    if tpl_w >= search_w:
        return None, 0.0

    template = cv2.GaussianBlur(gr[:, :tpl_w], (5, 5), 0)
    search = cv2.GaussianBlur(gl[:, wl - search_w :], (5, 5), 0)
    res = cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED)
    _, score, _, max_loc = cv2.minMaxLoc(res)
    if score < 0.20:
        return None, score

    x_in_search = int(max_loc[0])
    x_start_scaled = (wl - search_w) + x_in_search
    x_start = int(round(x_start_scaled / scale))

    overlap = left.shape[1] - x_start
    max_reasonable = int(0.95 * min(left.shape[1], right.shape[1]))
    if overlap < 60 or overlap > max_reasonable:
        return None, score
    return x_start, score


def estimate_best_layout(left, right):
    # Layout A: left image on the left, right image on the right.
    x_a, s_a = estimate_horizontal_offset(left, right)
    # Layout B: right image on the left, left image on the right.
    x_b, s_b = estimate_horizontal_offset(right, left)

    if x_a is None and x_b is None:
        return {"left_first": True, "x_start": None, "score": 0.0}
    if x_b is None:
        return {"left_first": True, "x_start": x_a, "score": s_a}
    if x_a is None:
        return {"left_first": False, "x_start": x_b, "score": s_b}
    if s_b > s_a:
        return {"left_first": False, "x_start": x_b, "score": s_b}
    return {"left_first": True, "x_start": x_a, "score": s_a}


def stitch_frames(left, right, x_start, left_first=True):
    if not left_first:
        # Reuse the same stitching math by swapping inputs.
        return stitch_frames(right, left, x_start, left_first=True)

    if left.shape[0] != right.shape[0]:
        h = min(left.shape[0], right.shape[0])
        wl = int(round(left.shape[1] * (h / left.shape[0])))
        wr = int(round(right.shape[1] * (h / right.shape[0])))
        left = cv2.resize(left, (wl, h), interpolation=cv2.INTER_AREA)
        right = cv2.resize(right, (wr, h), interpolation=cv2.INTER_AREA)

    if x_start is None:
        return np.hstack((left, right))

    wl = left.shape[1]
    wr = right.shape[1]
    x_start = int(np.clip(x_start, 0, wl - 1))
    overlap = wl - x_start
    if overlap <= 0 or overlap > min(wl, wr):
        return np.hstack((left, right))

    out_w = wl + wr - overlap
    out = np.zeros((left.shape[0], out_w, 3), dtype=left.dtype)
    out[:, :wl] = left

    left_ov = left[:, x_start:wl].astype(np.float32)
    right_ov = right[:, :overlap].astype(np.float32)
    alpha = np.linspace(1.0, 0.0, overlap, dtype=np.float32)[None, :, None]
    beta = 1.0 - alpha
    out[:, x_start:wl] = (left_ov * alpha + right_ov * beta).astype(left.dtype)
    out[:, wl:] = right[:, overlap:]
    return out


def put_text(img, text, y=30):
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)


def non_black_ratio(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return float(np.count_nonzero(g > 3)) / float(g.size)


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
        help="Re-estimate left/right overlap every N frames (default 30).",
    )
    args = ap.parse_args()

    try:
        calib_path, cfg = resolve_calibration(args.calib_file)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    right_rotate_180 = read_right_rotation_hint(Path(calib_path))

    print(f"[INFO] Using calibration: {calib_path}")
    print(f"[INFO] Left device:  {LEFT_DEVICE}")
    print(f"[INFO] Right device: {RIGHT_DEVICE}")
    print(f"[INFO] Right camera rotate180: {right_rotate_180}")
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

    ok_l, frame_l0 = cap_l.read()
    ok_r, frame_r0 = cap_r.read()
    if not ok_l or not ok_r:
        print("ERROR: could not read initial frames from both cameras.", file=sys.stderr)
        cap_l.release()
        cap_r.release()
        sys.exit(1)
    frame_r0 = apply_right_rotation(frame_r0, right_rotate_180)

    out_w = min(frame_l0.shape[1], frame_r0.shape[1])
    out_h = min(frame_l0.shape[0], frame_r0.shape[0])
    if (out_w, out_h) != cfg["imsize"]:
        print(
            f"[INFO] Calibration image size {cfg['imsize'][0]}x{cfg['imsize'][1]} "
            f"-> runtime size {out_w}x{out_h} (auto-scaled intrinsics)"
        )

    (map1_l, map2_l), (map1_r, map2_r), map_mode = build_maps(cfg, (out_w, out_h), prefer_rectify=True)
    print(f"[INFO] Map mode: {map_mode}")
    cv2.namedWindow("stereo_merged", cv2.WINDOW_NORMAL)

    frame_idx = 0
    fps_last = time.time()
    fps_count = 0
    fps_disp = 0.0
    cached_x_start = None
    cached_score = 0.0
    cached_left_first = True
    force_layout = None  # None=auto, True=left-first, False=right-first
    bad_map_count = 0

    try:
        while True:
            ok_l, frame_l = cap_l.read()
            ok_r, frame_r = cap_r.read()
            if not ok_l or not ok_r:
                print("[WARN] Frame read failed; retrying...")
                time.sleep(0.05)
                continue
            frame_r = apply_right_rotation(frame_r, right_rotate_180)

            if frame_l.shape[1] != out_w or frame_l.shape[0] != out_h:
                frame_l = cv2.resize(frame_l, (out_w, out_h), interpolation=cv2.INTER_AREA)
            if frame_r.shape[1] != out_w or frame_r.shape[0] != out_h:
                frame_r = cv2.resize(frame_r, (out_w, out_h), interpolation=cv2.INTER_AREA)

            und_l = cv2.remap(frame_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            und_r = cv2.remap(frame_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

            ratio_l = non_black_ratio(und_l)
            ratio_r = non_black_ratio(und_r)
            if ratio_l < 0.05 or ratio_r < 0.05:
                bad_map_count += 1
            else:
                bad_map_count = 0

            if bad_map_count >= 5 and map_mode != "undistort-only":
                print("[WARN] Rectified maps appear invalid (mostly black). Falling back to undistort-only maps.")
                (map1_l, map2_l), (map1_r, map2_r), map_mode = build_maps(cfg, (out_w, out_h), prefer_rectify=False)
                cached_x_start = None
                cached_score = 0.0
                cached_left_first = True
                bad_map_count = 0
                continue

            if frame_idx % max(1, args.refresh_homography_every) == 0:
                layout = estimate_best_layout(und_l, und_r)
                if layout["x_start"] is not None:
                    cached_x_start = layout["x_start"]
                    cached_score = layout["score"]
                    cached_left_first = layout["left_first"]

            use_left_first = cached_left_first if force_layout is None else force_layout
            merged = stitch_frames(und_l, und_r, cached_x_start, left_first=use_left_first)

            fps_count += 1
            now = time.time()
            if now - fps_last >= 0.5:
                fps_disp = fps_count / (now - fps_last)
                fps_count = 0
                fps_last = now

            put_text(merged, f"FPS ~ {fps_disp:.1f}")
            align_txt = "overlap: unknown"
            if cached_x_start is not None:
                align_txt = f"x_start={cached_x_start} score={cached_score:.2f}"
            put_text(merged, align_txt, y=58)
            layout_txt = "layout=auto"
            if force_layout is None:
                layout_txt = f"layout=auto ({'L->R' if cached_left_first else 'R->L'})"
            else:
                layout_txt = f"layout=forced ({'L->R' if force_layout else 'R->L'})"
            put_text(merged, layout_txt, y=86)
            put_text(merged, f"map={map_mode} nonblack L/R={ratio_l:.2f}/{ratio_r:.2f}", y=114)
            put_text(merged, "q: quit, r: reset overlap, f: toggle forced layout, a: auto layout", y=142)
            cv2.imshow("stereo_merged", merged)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                cached_x_start = None
                cached_score = 0.0
            if key == ord("f"):
                if force_layout is None:
                    force_layout = (not cached_left_first)
                else:
                    force_layout = not force_layout
            if key == ord("a"):
                force_layout = None
            frame_idx += 1

    finally:
        cap_l.release()
        cap_r.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    cv2.setUseOptimized(True)
    main()
