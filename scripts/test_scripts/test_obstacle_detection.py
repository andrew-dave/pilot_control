#!/usr/bin/env python3
"""
Automated Obstacle Detection from Point Cloud
==============================================

Detects obstacles in a 3D point cloud using the robot's driven path as a
ground-truth reference for the ground plane.

Pipeline
--------
1. Load point cloud (PCD/PLY) and extract robot path from a ROS 2 bag.
2. Identify ground points: points under the robot's footprint with z < 0.
3. Fit a ground plane (RANSAC) to those footprint-ground points.
4. Extract obstacle candidates: non-ground points within the obstacle z range.
5. Remove noisy sparse points with statistical outlier rejection.
6. Project obstacle points to 2D XY and cluster with DBSCAN.
7. Merge nearby clusters and polygonize (supports holes via grid contours).
8. Display interactive 3D + 2D visualisation.

Usage
-----
    python3 test_obstacle_detection.py \\
        --pcd scan.pcd \\
        --bag /path/to/rosbag_dir

    # Point-cloud-only mode (no bag -- uses simple z threshold for ground)
    python3 test_obstacle_detection.py --pcd scan.pcd

Dependencies (all available in the system python):
    numpy, scipy, matplotlib, rosbag2_py, rclpy
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial import ConvexHull, cKDTree
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.patches import PathPatch
from matplotlib.collections import PatchCollection
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers projection)
from matplotlib.colors import to_rgba
from matplotlib.path import Path


# ═══════════════════════════════════════════════════════════════════════════
# Constants / defaults
# ═══════════════════════════════════════════════════════════════════════════

ROBOT_LENGTH_M = 0.31
ROBOT_WIDTH_M = 0.28
ROBOT_MAX_DIM_M = 0.50  # largest effective dimension (merge gap default)


# ═══════════════════════════════════════════════════════════════════════════
# Point Cloud I/O
# ═══════════════════════════════════════════════════════════════════════════

def _load_pcd(filepath: str) -> np.ndarray:
    """Load a PCD file (ASCII or binary) and return an Nx3 float64 array."""
    with open(filepath, "rb") as f:
        fields: List[str] = []
        sizes: List[int] = []
        types: List[str] = []
        counts: List[int] = []
        width = height = 0
        data_format = "ascii"

        while True:
            raw = f.readline()
            if not raw:
                raise ValueError("Reached EOF before DATA line in PCD header")
            line = raw.decode("ascii", errors="replace").strip()

            tokens = line.split()
            if not tokens:
                continue
            key = tokens[0].upper()
            if key == "FIELDS":
                fields = [t.lower() for t in tokens[1:]]
            elif key == "SIZE":
                sizes = [int(t) for t in tokens[1:]]
            elif key == "TYPE":
                types = tokens[1:]
            elif key == "COUNT":
                counts = [int(t) for t in tokens[1:]]
            elif key == "WIDTH":
                width = int(tokens[1])
            elif key == "HEIGHT":
                height = int(tokens[1])
            elif key == "DATA":
                data_format = tokens[1].lower()
                break

        n_points = width * height if (width and height) else width
        if n_points == 0:
            raise ValueError("PCD header reports 0 points")

        # Locate x, y, z field indices
        try:
            xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")
        except ValueError as exc:
            raise ValueError(f"PCD missing x/y/z fields ({fields}): {exc}")

        if data_format == "ascii":
            points: List[List[float]] = []
            for raw_line in f:
                tok = raw_line.decode("ascii", errors="replace").split()
                if len(tok) > max(xi, yi, zi):
                    try:
                        points.append([
                            float(tok[xi]),
                            float(tok[yi]),
                            float(tok[zi]),
                        ])
                    except ValueError:
                        continue
            return np.asarray(points, dtype=np.float64)

        elif data_format == "binary":
            # Build struct format for one point row
            fmt_map = {"F": "f", "U": "I", "I": "i"}
            row_fmt = "<"
            for ftype, fsize, fcount in zip(types, sizes, counts):
                base = fmt_map.get(ftype.upper(), "f")
                if fsize == 8 and ftype.upper() == "F":
                    base = "d"
                row_fmt += base * fcount

            row_size = struct.calcsize(row_fmt)
            blob = f.read(n_points * row_size)
            actual_n = min(n_points, len(blob) // row_size)

            xyz = np.empty((actual_n, 3), dtype=np.float64)
            for i in range(actual_n):
                row = struct.unpack_from(row_fmt, blob, i * row_size)
                xyz[i, 0] = row[xi]
                xyz[i, 1] = row[yi]
                xyz[i, 2] = row[zi]
            return xyz

        else:
            raise ValueError(
                f"Unsupported PCD data format '{data_format}' "
                "(only 'ascii' and 'binary' are supported)"
            )


def _load_ply_ascii(filepath: str) -> np.ndarray:
    """Minimal PLY ASCII reader -- returns Nx3 float64."""
    with open(filepath, "r") as f:
        prop_names: List[str] = []
        n_vertices = 0
        in_header = True
        first_data_line: Optional[str] = None

        for line in f:
            line = line.strip()
            if in_header:
                if line.startswith("element vertex"):
                    n_vertices = int(line.split()[-1])
                elif line.startswith("property"):
                    prop_names.append(line.split()[-1].lower())
                elif line == "end_header":
                    in_header = False
                continue
            # First data line
            first_data_line = line
            break

        if n_vertices == 0:
            raise ValueError("PLY header reports 0 vertices")

        xi = prop_names.index("x") if "x" in prop_names else 0
        yi = prop_names.index("y") if "y" in prop_names else 1
        zi = prop_names.index("z") if "z" in prop_names else 2

        points: List[List[float]] = []
        lines_to_parse = ([first_data_line] if first_data_line else []) + f.readlines()
        for data_line in lines_to_parse:
            tok = data_line.strip().split()
            if len(tok) > max(xi, yi, zi):
                try:
                    points.append([float(tok[xi]), float(tok[yi]), float(tok[zi])])
                except ValueError:
                    continue
        return np.asarray(points[:n_vertices], dtype=np.float64)


def load_point_cloud(filepath: str) -> np.ndarray:
    """Load point cloud from PCD / PLY / XYZ.

    Tries Open3D first (handles all formats); falls back to built-in readers.
    Returns Nx3 float64 array.
    """
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(filepath)
        if pcd.is_empty():
            raise ValueError(f"Open3D loaded empty cloud: {filepath}")
        pts = np.asarray(pcd.points, dtype=np.float64)
        print(f"  [open3d] Loaded {len(pts)} points")
        return pts
    except ImportError:
        pass

    ext = filepath.rsplit(".", 1)[-1].lower()
    if ext == "pcd":
        pts = _load_pcd(filepath)
    elif ext == "ply":
        pts = _load_ply_ascii(filepath)
    elif ext == "xyz":
        pts = np.loadtxt(filepath, usecols=(0, 1, 2), dtype=np.float64)
    else:
        raise ValueError(f"Unsupported point cloud format: .{ext}")

    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"Expected Nx3 array, got shape {pts.shape}")
    pts = pts[:, :3].copy()
    print(f"  [built-in] Loaded {len(pts)} points")
    return pts


# ═══════════════════════════════════════════════════════════════════════════
# ROS 2 Bag -- path extraction
# ═══════════════════════════════════════════════════════════════════════════

def _quat_to_yaw(q) -> float:
    """Extract yaw (rotation about Z) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _prepare_bag_dir(bag_dir: str) -> Tuple[str, str]:
    """Resolve the bag directory and storage id, handling zstd compression.

    If the bag uses ``compression_format: zstd`` with ``compression_mode: FILE``
    rosbag2_py cannot open the ``.mcap.zstd`` files directly.  This helper:

    1. Looks for already-decompressed ``.mcap`` files next to the ``.mcap.zstd``.
    2. If missing, decompresses them via the ``zstd`` CLI tool.
    3. Creates a temporary bag directory with a patched ``metadata.yaml``
       (compression removed, paths point to uncompressed files) and symlinks
       to the actual data files.

    Returns ``(effective_bag_dir, storage_id)``.
    """
    import shutil
    import tempfile
    import yaml

    meta_path = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.exists(meta_path):
        raise FileNotFoundError(f"metadata.yaml not found in {bag_dir}")

    with open(meta_path, "r") as fh:
        meta = yaml.safe_load(fh)

    bag_info = meta.get("rosbag2_bagfile_information", meta)
    storage_id = bag_info.get("storage_identifier", "sqlite3")
    compression_fmt = bag_info.get("compression_format", "")
    compression_mode = bag_info.get("compression_mode", "")

    # ---- No compression: use the bag directory as-is ----
    if not compression_fmt:
        return bag_dir, storage_id

    # ---- FILE-level zstd compression ----
    if compression_fmt.lower() == "zstd" and compression_mode.upper() == "FILE":
        print(f"  Detected zstd FILE compression -- preparing decompressed bag ...")
        rel_paths = bag_info.get("relative_file_paths", [])

        # Create the temp directory first — we may need to decompress into it
        tmp_dir = tempfile.mkdtemp(prefix="bag_nocomp_")

        decompressed_names: List[str] = []
        for rp in rel_paths:
            compressed_path = os.path.join(bag_dir, rp)
            if rp.endswith(".zstd"):
                plain_name = rp[:-5]  # strip ".zstd"
            else:
                plain_name = rp

            # Check if an uncompressed copy already exists next to the source
            plain_path_src = os.path.join(bag_dir, plain_name)
            plain_path_tmp = os.path.join(tmp_dir, plain_name)

            if os.path.exists(plain_path_src):
                # Already decompressed — just symlink it
                os.symlink(os.path.abspath(plain_path_src), plain_path_tmp)
            else:
                # Decompress into the temp dir (avoids permission issues
                # when the bag directory is read-only / root-owned)
                if not os.path.exists(compressed_path):
                    raise FileNotFoundError(
                        f"Neither '{plain_path_src}' nor "
                        f"'{compressed_path}' exist"
                    )
                print(f"    Decompressing {rp} -> {tmp_dir}/{plain_name} ...")
                import subprocess
                ret = subprocess.run(
                    ["zstd", "-d", compressed_path, "-o", plain_path_tmp],
                    capture_output=True, text=True,
                )
                if ret.returncode != 0:
                    raise RuntimeError(
                        f"zstd decompression failed: {ret.stderr.strip()}"
                    )
            decompressed_names.append(plain_name)

        # Patch metadata: remove compression, fix relative_file_paths
        bag_info["compression_format"] = ""
        bag_info["compression_mode"] = ""
        bag_info["relative_file_paths"] = decompressed_names
        meta["rosbag2_bagfile_information"] = bag_info

        patched_meta = os.path.join(tmp_dir, "metadata.yaml")
        with open(patched_meta, "w") as fh:
            yaml.dump(meta, fh, default_flow_style=False, sort_keys=False)

        print(f"    Temp bag dir: {tmp_dir}")
        return tmp_dir, storage_id

    # ---- Unknown compression: warn and try as-is ----
    print(f"  [WARN] Unknown compression '{compression_fmt}' / "
          f"'{compression_mode}' -- attempting to open as-is.")
    return bag_dir, storage_id


def read_path_from_bag(
    bag_dir: str,
    topic: str = "/path",
) -> np.ndarray:
    """Read the last ``nav_msgs/Path`` message from a ROS 2 bag.

    Handles zstd-compressed MCAP bags transparently.

    Returns
    -------
    path_poses : np.ndarray, shape (M, 3)
        Each row is ``[x, y, yaw]``.
    """
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Path as RosPath

    effective_dir, storage_id = _prepare_bag_dir(bag_dir)

    reader = SequentialReader()
    storage_opts = StorageOptions(uri=effective_dir, storage_id=storage_id)
    converter_opts = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_opts, converter_opts)

    # Verify the topic exists
    available = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in available:
        raise ValueError(
            f"Topic '{topic}' not found in bag. "
            f"Available: {list(available.keys())}"
        )

    # Read ALL messages on this topic -- keep the last (most complete) one
    last_msg = None
    while reader.has_next():
        t_name, data, _ts = reader.read_next()
        if t_name == topic:
            last_msg = deserialize_message(data, RosPath)

    if last_msg is None or len(last_msg.poses) == 0:
        raise ValueError(f"No non-empty Path messages on '{topic}'")

    poses = np.empty((len(last_msg.poses), 3), dtype=np.float64)
    for i, ps in enumerate(last_msg.poses):
        poses[i, 0] = ps.pose.position.x
        poses[i, 1] = ps.pose.position.y
        poses[i, 2] = _quat_to_yaw(ps.pose.orientation)

    print(f"  Extracted {len(poses)} path poses from '{topic}'")
    return poses


# ═══════════════════════════════════════════════════════════════════════════
# Ground Detection
# ═══════════════════════════════════════════════════════════════════════════

def _is_in_oriented_rect(
    px: np.ndarray,
    py: np.ndarray,
    cx: float,
    cy: float,
    yaw: float,
    half_l: float,
    half_w: float,
) -> np.ndarray:
    """Vectorised test: are (px, py) inside an oriented rectangle?

    Returns boolean mask of same length as px.
    """
    cos_y = math.cos(-yaw)
    sin_y = math.sin(-yaw)
    dx = px - cx
    dy = py - cy
    # Rotate into rectangle-local frame
    lx = cos_y * dx - sin_y * dy
    ly = sin_y * dx + cos_y * dy
    return (np.abs(lx) <= half_l) & (np.abs(ly) <= half_w)


def extract_footprint_ground(
    points: np.ndarray,
    path_poses: np.ndarray,
    robot_length: float,
    robot_width: float,
    footprint_margin: float,
    z_max: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Select points under the robot path's footprint with z <= z_max.

    Uses a cKDTree on the path poses for efficient spatial lookup (checks the
    K nearest poses for each candidate point).

    Returns
    -------
    ground_points : Nx3 array of selected ground points
    ground_mask   : boolean mask into the original ``points`` array
    """
    # Candidate: z below threshold
    z_cand_mask = points[:, 2] <= z_max
    cand_idx = np.nonzero(z_cand_mask)[0]
    if len(cand_idx) == 0:
        return np.empty((0, 3)), np.zeros(len(points), dtype=bool)

    cand_xy = points[cand_idx, :2]

    # Build tree of path XY positions
    path_tree = cKDTree(path_poses[:, :2])

    half_l = robot_length / 2.0 + footprint_margin
    half_w = robot_width / 2.0 + footprint_margin
    search_radius = math.hypot(half_l, half_w)  # circumscribed circle

    # For each candidate point, check K nearest path poses
    K = min(5, len(path_poses))
    dists, indices = path_tree.query(cand_xy, k=K,
                                     distance_upper_bound=search_radius * 1.5)
    # Ensure 2D arrays even when K=1
    if K == 1:
        dists = dists[:, np.newaxis]
        indices = indices[:, np.newaxis]

    in_footprint = np.zeros(len(cand_idx), dtype=bool)

    for k_col in range(K):
        d_col = dists[:, k_col]
        i_col = indices[:, k_col]
        valid = d_col < np.inf  # cKDTree returns inf for no match
        if not np.any(valid):
            continue
        v_idx = np.nonzero(valid)[0]
        pose_rows = i_col[v_idx]
        cx = path_poses[pose_rows, 0]
        cy = path_poses[pose_rows, 1]
        yaw = path_poses[pose_rows, 2]

        # Vectorised per-pose check is not trivial when yaw differs per row,
        # so we iterate unique-ish yaw buckets for speed.  For moderate path
        # sizes this is fine; fall back to per-point if needed.
        for pi in range(len(v_idx)):
            pt_idx = v_idx[pi]
            hit = _is_in_oriented_rect(
                cand_xy[pt_idx:pt_idx + 1, 0],
                cand_xy[pt_idx:pt_idx + 1, 1],
                cx[pi], cy[pi], yaw[pi], half_l, half_w,
            )
            if hit[0]:
                in_footprint[pt_idx] = True

    ground_mask_full = np.zeros(len(points), dtype=bool)
    ground_mask_full[cand_idx[in_footprint]] = True
    return points[ground_mask_full], ground_mask_full


def fit_ground_plane_ransac(
    points: np.ndarray,
    n_iterations: int = 300,
    distance_threshold: float = 0.03,
) -> Tuple[np.ndarray, float, np.ndarray]:
    """Fit a plane (RANSAC) to 3D points.

    Returns
    -------
    normal : (3,) unit normal of best plane (oriented so nz > 0).
    d      : plane offset  (plane eq:  normal . p + d = 0).
    inlier_mask : boolean mask of inlier points.
    """
    n = len(points)
    if n < 3:
        raise ValueError("Need >= 3 points for plane fitting")

    rng = np.random.default_rng(42)
    best_count = 0
    best_normal = np.array([0.0, 0.0, 1.0])
    best_d = 0.0
    best_mask = np.zeros(n, dtype=bool)

    for _ in range(n_iterations):
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = points[idx]
        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            continue
        normal /= norm
        if normal[2] < 0:
            normal = -normal
        d = -np.dot(normal, p0)
        dists = np.abs(points @ normal + d)
        mask = dists <= distance_threshold
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_normal = normal
            best_d = d
            best_mask = mask

    print(
        f"      RANSAC plane: normal=[{best_normal[0]:.4f}, "
        f"{best_normal[1]:.4f}, {best_normal[2]:.4f}], d={best_d:.4f}, "
        f"inliers={best_count}/{n} ({100 * best_count / n:.1f}%)"
    )
    return best_normal, best_d, best_mask


def signed_distance_to_plane(
    points: np.ndarray, normal: np.ndarray, d: float,
) -> np.ndarray:
    """Signed distance of each point to the plane (positive = above)."""
    return points @ normal + d


# ═══════════════════════════════════════════════════════════════════════════
# Obstacle Extraction
# ═══════════════════════════════════════════════════════════════════════════

def remove_statistical_outliers(
    points: np.ndarray,
    k: int = 20,
    std_ratio: float = 1.5,
) -> Tuple[np.ndarray, np.ndarray]:
    """Remove sparse noise using k-NN distance statistics.

    Returns (filtered_points, keep_mask).
    """
    if len(points) < k + 1:
        return points, np.ones(len(points), dtype=bool)

    tree = cKDTree(points)
    dists, _ = tree.query(points, k=k + 1)  # +1 because self is dist=0
    mean_dists = dists[:, 1:].mean(axis=1)  # exclude self
    mu = mean_dists.mean()
    sigma = mean_dists.std()
    threshold = mu + std_ratio * sigma
    mask = mean_dists <= threshold
    return points[mask], mask


# ═══════════════════════════════════════════════════════════════════════════
# DBSCAN clustering (pure scipy)
# ═══════════════════════════════════════════════════════════════════════════

def dbscan(
    points: np.ndarray,
    eps: float,
    min_samples: int,
) -> np.ndarray:
    """DBSCAN clustering using scipy cKDTree.

    Parameters
    ----------
    points : (N, D) array (typically D=2 for XY).
    eps : neighbourhood radius.
    min_samples : minimum neighbours to be a core point.

    Returns
    -------
    labels : (N,) int array.  -1 = noise.
    """
    n = len(points)
    if n == 0:
        return np.array([], dtype=int)

    tree = cKDTree(points)
    # Batch query -- list of arrays of neighbour indices per point
    all_neighbours = tree.query_ball_point(points, eps)

    core_mask = np.array([len(nb) >= min_samples for nb in all_neighbours])
    labels = np.full(n, -1, dtype=int)
    cluster_id = 0

    for seed in range(n):
        if labels[seed] != -1 or not core_mask[seed]:
            continue
        # BFS from this core point
        queue = [seed]
        labels[seed] = cluster_id
        head = 0
        while head < len(queue):
            curr = queue[head]
            head += 1
            for nb in all_neighbours[curr]:
                if labels[nb] != -1:
                    continue
                labels[nb] = cluster_id
                if core_mask[nb]:
                    queue.append(nb)
        cluster_id += 1

    return labels


# ═══════════════════════════════════════════════════════════════════════════
# Polygon generation & merging
# ═══════════════════════════════════════════════════════════════════════════

def cluster_to_convex_hull(pts2d: np.ndarray) -> Optional[np.ndarray]:
    """Compute convex hull polygon for a 2D cluster.

    Returns (K, 2) array of ordered hull vertices, or *None* if degenerate.
    """
    if len(pts2d) < 3:
        if len(pts2d) == 0:
            return None
        # Buffer with a tiny circle
        centre = pts2d.mean(axis=0)
        r = 0.05
        angles = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        return np.column_stack([centre[0] + r * np.cos(angles),
                                centre[1] + r * np.sin(angles)])
    try:
        hull = ConvexHull(pts2d)
        return pts2d[hull.vertices]
    except Exception:
        return None


def merge_nearby_clusters_indices(
    clusters: List[np.ndarray],
    merge_distance: float,
) -> List[List[int]]:
    """Group clusters whose closest points are within *merge_distance*.

    Uses union-find so transitive merges are handled (A~B and B~C => A,B,C).
    Returns a list of groups of original indices.
    """
    n = len(clusters)
    if n == 0:
        return []
    if n == 1 or merge_distance <= 0:
        return [[0]]

    parent = list(range(n))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    trees = [cKDTree(c) for c in clusters]
    for i in range(n):
        for j in range(i + 1, n):
            dists_ij, _ = trees[j].query(clusters[i], k=1)
            if float(dists_ij.min()) < merge_distance:
                union(i, j)

    groups: Dict[int, List[int]] = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    return list(groups.values())


# ═══════════════════════════════════════════════════════════════════════════
# Visualisation helpers
# ═══════════════════════════════════════════════════════════════════════════

def _downsample_for_plot(pts: np.ndarray, max_pts: int = 40_000) -> np.ndarray:
    """Random downsample for faster plotting."""
    if len(pts) <= max_pts:
        return pts
    idx = np.random.default_rng(0).choice(len(pts), max_pts, replace=False)
    return pts[idx]


def _polygon_area(verts: np.ndarray) -> float:
    """Shoelace formula for polygon area."""
    n = len(verts)
    if n < 3:
        return 0.0
    x, y = verts[:, 0], verts[:, 1]
    return 0.5 * abs(float(
        np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))
    ))


def _polygon_signed_area(verts: np.ndarray) -> float:
    """Signed polygon area (positive for CCW ordering)."""
    n = len(verts)
    if n < 3:
        return 0.0
    x, y = verts[:, 0], verts[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _ensure_ccw(verts: np.ndarray) -> np.ndarray:
    if _polygon_signed_area(verts) < 0:
        return verts[::-1].copy()
    return verts


def _ensure_cw(verts: np.ndarray) -> np.ndarray:
    if _polygon_signed_area(verts) > 0:
        return verts[::-1].copy()
    return verts


def _point_in_poly(point: np.ndarray, poly: np.ndarray) -> bool:
    """Ray casting point-in-polygon (poly is (N,2))."""
    x, y = float(point[0]), float(point[1])
    inside = False
    n = len(poly)
    if n < 3:
        return False
    x0, y0 = float(poly[-1, 0]), float(poly[-1, 1])
    for i in range(n):
        x1, y1 = float(poly[i, 0]), float(poly[i, 1])
        intersects = ((y1 > y) != (y0 > y)) and (
            x < (x0 - x1) * (y - y1) / (y0 - y1 + 1e-12) + x1
        )
        if intersects:
            inside = not inside
        x0, y0 = x1, y1
    return inside


def _path_from_ring(verts: np.ndarray) -> Path:
    """Matplotlib Path for a closed polygon ring."""
    if len(verts) < 3:
        raise ValueError("Need >= 3 vertices for a ring path")
    v = np.vstack([verts, verts[:1]])
    codes = np.full(len(v), Path.LINETO, dtype=np.uint8)
    codes[0] = Path.MOVETO
    codes[-1] = Path.CLOSEPOLY
    return Path(v, codes)


def _compound_path_with_holes(outer: np.ndarray, holes: List[np.ndarray]) -> Path:
    """Compound Path where holes cut out from the outer ring."""
    outer_ccw = _ensure_ccw(outer)
    outer_path = _path_from_ring(outer_ccw)
    hole_paths: List[Path] = []
    for h in holes:
        if len(h) < 3:
            continue
        hole_paths.append(_path_from_ring(_ensure_cw(h)))
    if not hole_paths:
        return outer_path
    return Path.make_compound_path(outer_path, *hole_paths)


@dataclass
class ObstacleShape:
    """Obstacle polygon possibly with holes (e.g., a ring/wall around free space)."""
    outer: np.ndarray
    holes: List[np.ndarray]
    cluster_idx: int
    method: str  # "hull" or "grid"


def _occupancy_grid_from_points(
    pts2d: np.ndarray,
    cell_size: float,
    padding: float = 0.20,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rasterize 2D points to an occupancy grid.

    Returns
    -------
    occ : (H, W) uint8 array with 1=occupied
    x_centers : (W,) float array of cell-center x coordinates
    y_centers : (H,) float array of cell-center y coordinates
    """
    if len(pts2d) == 0:
        return np.zeros((0, 0), dtype=np.uint8), np.array([]), np.array([])
    if cell_size <= 0:
        raise ValueError("cell_size must be > 0")

    xmin = float(pts2d[:, 0].min()) - padding
    xmax = float(pts2d[:, 0].max()) + padding
    ymin = float(pts2d[:, 1].min()) - padding
    ymax = float(pts2d[:, 1].max()) + padding

    w = int(math.ceil((xmax - xmin) / cell_size))
    h = int(math.ceil((ymax - ymin) / cell_size))
    w = max(w, 1)
    h = max(h, 1)

    occ = np.zeros((h, w), dtype=np.uint8)
    ix = np.floor((pts2d[:, 0] - xmin) / cell_size).astype(int)
    iy = np.floor((pts2d[:, 1] - ymin) / cell_size).astype(int)
    ix = np.clip(ix, 0, w - 1)
    iy = np.clip(iy, 0, h - 1)
    occ[iy, ix] = 1

    x_centers = xmin + (np.arange(w, dtype=float) + 0.5) * cell_size
    y_centers = ymin + (np.arange(h, dtype=float) + 0.5) * cell_size
    return occ, x_centers, y_centers


def _inflate_occupancy(
    occ: np.ndarray,
    radius_m: float,
    cell_size: float,
) -> np.ndarray:
    """Inflate occupied cells by a radius in metres (binary dilation)."""
    if occ.size == 0:
        return occ
    if radius_m <= 1e-9:
        return occ
    r = int(math.ceil(radius_m / cell_size))
    if r <= 0:
        return occ
    # Circular structuring element
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    selem = (xx * xx + yy * yy) <= (r * r)

    try:
        from scipy import ndimage  # type: ignore
        inflated = ndimage.binary_dilation(occ.astype(bool), structure=selem)
        return inflated.astype(np.uint8)
    except Exception:
        # Fallback: no inflation (keeps pipeline running without ndimage)
        return occ


def _extract_contours_from_occupancy(
    occ: np.ndarray,
    x_centers: np.ndarray,
    y_centers: np.ndarray,
) -> List[np.ndarray]:
    """Extract contour polylines (as polygons) from a binary occupancy grid."""
    if occ.size == 0:
        return []
    # Use a hidden figure for contour tracing
    X, Y = np.meshgrid(x_centers, y_centers)
    fig = plt.figure(figsize=(4, 3))
    ax = fig.add_subplot(111)
    try:
        cs = ax.contour(X, Y, occ.astype(float), levels=[0.5])
        polys: List[np.ndarray] = []
        if cs.collections:
            for p in cs.collections[0].get_paths():
                v = p.vertices
                if v is None or len(v) < 3:
                    continue
                # Drop duplicate consecutive points
                dv = np.linalg.norm(np.diff(v, axis=0), axis=1)
                keep = np.ones(len(v), dtype=bool)
                keep[1:] = dv > 1e-9
                v = v[keep]
                if len(v) >= 3:
                    polys.append(v.astype(float))
        return polys
    finally:
        plt.close(fig)


def _group_contours_into_shapes(
    contours: List[np.ndarray],
    min_area: float = 0.02,
) -> List[Tuple[np.ndarray, List[np.ndarray]]]:
    """Build (outer, holes) shapes from a set of possibly-nested contour rings.

    Uses containment nesting depth: even depth => outer, odd depth => hole.
    """
    # Filter tiny/degenerate contours
    rings = []
    for c in contours:
        area = abs(_polygon_signed_area(c))
        if area >= min_area and len(c) >= 3:
            rings.append(c)
    if not rings:
        return []

    # Sort by absolute area descending for containment search
    areas = np.array([abs(_polygon_signed_area(r)) for r in rings])
    order = list(np.argsort(-areas))
    rings = [rings[i] for i in order]
    areas = areas[order]

    # Parent of each ring (index into rings), or -1
    parent = [-1] * len(rings)
    centroids = [r.mean(axis=0) for r in rings]
    for i in range(len(rings)):
        # Find smallest-area ring that contains this ring's centroid
        best = -1
        best_area = float("inf")
        for j in range(len(rings)):
            if areas[j] <= areas[i]:
                continue
            if not _point_in_poly(centroids[i], rings[j]):
                continue
            if areas[j] < best_area:
                best_area = float(areas[j])
                best = j
        parent[i] = best

    # Compute depth by walking parents
    depth = [0] * len(rings)
    for i in range(len(rings)):
        d = 0
        p = parent[i]
        while p != -1:
            d += 1
            p = parent[p]
        depth[i] = d

    # Build shapes from outer rings (even depth) and attach immediate odd-depth children as holes
    shapes: List[Tuple[np.ndarray, List[np.ndarray]]] = []
    for i, ring in enumerate(rings):
        if depth[i] % 2 != 0:
            continue
        holes: List[np.ndarray] = []
        for j, hr in enumerate(rings):
            if parent[j] == i and depth[j] == depth[i] + 1:
                holes.append(hr)
        shapes.append((ring, holes))
    return shapes


def polygonize_cluster_grid(
    pts2d: np.ndarray,
    cell_size: float,
    inflate_radius_m: float,
    min_contour_area: float,
) -> List[Tuple[np.ndarray, List[np.ndarray]]]:
    """Polygonize a cluster via occupancy grid -> contours -> (outer, holes)."""
    occ, x_centers, y_centers = _occupancy_grid_from_points(
        pts2d, cell_size=cell_size, padding=max(0.20, 2.0 * inflate_radius_m),
    )
    occ = _inflate_occupancy(occ, radius_m=inflate_radius_m, cell_size=cell_size)
    contours = _extract_contours_from_occupancy(occ, x_centers, y_centers)
    return _group_contours_into_shapes(contours, min_area=min_contour_area)


# ═══════════════════════════════════════════════════════════════════════════
# Visualisation
# ═══════════════════════════════════════════════════════════════════════════

def visualise_3d(
    all_points: np.ndarray,
    ground_mask: np.ndarray,
    obstacle_mask: np.ndarray,
    path_poses: Optional[np.ndarray],
    ground_z_level: float,
    obstacle_shapes: List[ObstacleShape],
) -> None:
    """Interactive lightweight 3D scatter (matplotlib mplot3d)."""
    fig = plt.figure("3D Obstacle Detection", figsize=(14, 9))
    ax = fig.add_subplot(111, projection="3d")

    # Background (other) points
    other_mask = ~ground_mask & ~obstacle_mask
    bg = _downsample_for_plot(all_points[other_mask])
    if len(bg):
        ax.scatter(bg[:, 0], bg[:, 1], bg[:, 2],
                   s=0.3, c="0.80", alpha=0.25, label="Other",
                   rasterized=True)

    # Ground
    gnd = _downsample_for_plot(all_points[ground_mask])
    if len(gnd):
        ax.scatter(gnd[:, 0], gnd[:, 1], gnd[:, 2],
                   s=0.5, c="seagreen", alpha=0.4, label="Ground")

    # Obstacles
    obs = _downsample_for_plot(all_points[obstacle_mask])
    if len(obs):
        ax.scatter(obs[:, 0], obs[:, 1], obs[:, 2],
                   s=1.5, c="crimson", alpha=0.7, label="Obstacle")

    # Robot path
    if path_poses is not None and len(path_poses):
        ax.plot(
            path_poses[:, 0], path_poses[:, 1],
            np.full(len(path_poses), ground_z_level),
            c="dodgerblue", linewidth=1.5, label="Path",
        )

    # Obstacle outlines projected at ground level (outer + holes)
    for sh in obstacle_shapes:
        outer = sh.outer
        closed = np.vstack([outer, outer[0:1]])
        ax.plot(
            closed[:, 0], closed[:, 1],
            np.full(len(closed), ground_z_level),
            c="red", linewidth=1.5, alpha=0.85,
        )
        for hole in sh.holes:
            hclosed = np.vstack([hole, hole[0:1]])
            ax.plot(
                hclosed[:, 0], hclosed[:, 1],
                np.full(len(hclosed), ground_z_level),
                c="red", linewidth=1.0, alpha=0.70, linestyle="--",
            )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("3D Point Cloud - Obstacle Detection")
    ax.legend(loc="upper left", fontsize=8, markerscale=4)
    fig.tight_layout()


def visualise_2d(
    all_points: np.ndarray,
    ground_mask: np.ndarray,
    obstacle_points_2d: np.ndarray,
    path_poses: Optional[np.ndarray],
    obstacle_shapes: List[ObstacleShape],
    cluster_points: List[np.ndarray],
) -> None:
    """2D XY projection with obstacle shapes (supports holes)."""
    fig, ax = plt.subplots(figsize=(14, 10), num="2D Obstacle Map")

    # All points (light grey background)
    bg = _downsample_for_plot(all_points, 60_000)
    ax.scatter(bg[:, 0], bg[:, 1], s=0.15, c="0.85", alpha=0.25,
               label="All pts", rasterized=True)

    # Ground points
    gnd = _downsample_for_plot(all_points[ground_mask], 30_000)
    ax.scatter(gnd[:, 0], gnd[:, 1], s=0.3, c="seagreen", alpha=0.3,
               label="Ground")

    # Obstacle clusters (colour-coded)
    cmap = plt.cm.get_cmap("tab10")
    for i, cl_pts in enumerate(cluster_points):
        colour = cmap(i % 10)
        lbl = f"Cluster {i}" if i < 10 else "_nolegend_"
        ax.scatter(cl_pts[:, 0], cl_pts[:, 1], s=2, c=[colour],
                   alpha=0.7, label=lbl)

    # Obstacle shapes (red) — uses compound paths so holes stay free
    if obstacle_shapes:
        shape_patches: List[PathPatch] = []
        for sh in obstacle_shapes:
            path = _compound_path_with_holes(sh.outer, sh.holes)
            shape_patches.append(PathPatch(path))
        pc = PatchCollection(
            shape_patches,
            facecolor=to_rgba("red", 0.18),
            edgecolor="red",
            linewidth=1.5,
        )
        ax.add_collection(pc)
        ax.plot([], [], c="red", linewidth=1.5, label="Obstacle area")

    # Shape labels (on outer boundary centroid)
    for i, sh in enumerate(obstacle_shapes):
        cx, cy = sh.outer.mean(axis=0)
        area = _polygon_area(sh.outer) - sum(_polygon_area(h) for h in sh.holes)
        ax.annotate(
            f"O{i} ({area:.3f} m\u00B2)",
            (cx, cy), fontsize=7, ha="center", va="center",
            color="darkred", fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.15", fc="white",
                      alpha=0.7, lw=0),
        )

    # Robot path
    if path_poses is not None and len(path_poses):
        ax.plot(path_poses[:, 0], path_poses[:, 1],
                c="dodgerblue", linewidth=1.2, label="Path", zorder=5)
        ax.plot(path_poses[0, 0], path_poses[0, 1],
                "o", c="lime", markersize=6, zorder=6, label="Start")
        ax.plot(path_poses[-1, 0], path_poses[-1, 1],
                "x", c="orange", markersize=8, mew=2, zorder=6, label="End")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("2D Projection - Obstacle Shapes (holes supported)")
    ax.set_aspect("equal", "box")
    ax.legend(loc="upper left", fontsize=7, markerscale=3)
    ax.grid(True, linewidth=0.3, alpha=0.5)
    fig.tight_layout()


# ═══════════════════════════════════════════════════════════════════════════
# Main pipeline
# ═══════════════════════════════════════════════════════════════════════════

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Automated obstacle detection from a 3D point cloud.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # -- I/O --
    p.add_argument("--pcd", required=True,
                   help="Input point cloud file (PCD / PLY / XYZ)")
    p.add_argument("--bag", default=None,
                   help="ROS 2 bag directory (contains metadata.yaml).  "
                        "If omitted, ground detection uses simple z threshold.")
    p.add_argument("--path-topic", default="/path",
                   help="Topic for nav_msgs/Path in the bag (default: /path)")

    # -- Robot dimensions --
    p.add_argument("--robot-length", type=float, default=ROBOT_LENGTH_M,
                   help=f"Robot length in metres (default: {ROBOT_LENGTH_M})")
    p.add_argument("--robot-width", type=float, default=ROBOT_WIDTH_M,
                   help=f"Robot width in metres (default: {ROBOT_WIDTH_M})")
    p.add_argument("--footprint-margin", type=float, default=0.10,
                   help="Extra margin around footprint for ground sampling (m)")

    # -- Ground detection --
    p.add_argument("--ground-z-max", type=float, default=0.0,
                   help="Max z for ground candidates (default: 0.0)")
    p.add_argument("--ransac-iters", type=int, default=300,
                   help="RANSAC iterations for plane fitting (default: 300)")
    p.add_argument("--ransac-thresh", type=float, default=0.03,
                   help="RANSAC distance threshold in metres (default: 0.03)")
    p.add_argument("--ground-band", type=float, default=0.05,
                   help="Half-thickness of ground band around fitted plane (m)")

    # -- Obstacle extraction --
    p.add_argument("--obstacle-z-max", type=float, default=0.30,
                   help="Max z for obstacle candidates (default: 0.30)")
    p.add_argument("--trough-depth", type=float, default=0.05,
                   help="Depth below ground plane to flag as trough (m)")
    p.add_argument("--outlier-k", type=int, default=20,
                   help="k for statistical outlier removal (default: 20)")
    p.add_argument("--outlier-std", type=float, default=1.5,
                   help="Std multiplier for outlier removal (default: 1.5)")

    # -- Clustering --
    p.add_argument("--cluster-eps", type=float, default=0.08,
                   help="DBSCAN epsilon (neighbourhood radius) in metres")
    p.add_argument("--cluster-min-pts", type=int, default=8,
                   help="DBSCAN minimum samples per cluster")

    # -- Polygonization / chaining --
    p.add_argument(
        "--polygon-mode",
        choices=["auto", "hull", "grid"],
        default="auto",
        help=("How to generate obstacle polygons from clusters. "
              "'hull' = convex hull (fast, no holes). "
              "'grid' = occupancy grid + contours (supports holes/rings). "
              "'auto' = use grid mode only when a cluster looks hollow."),
    )
    p.add_argument("--grid-cell", type=float, default=0.03,
                   help="Grid cell size in metres for grid polygonization (default: 0.03)")
    p.add_argument("--inflate-radius", type=float, default=None,
                   help=("Inflation radius (m) applied before contour extraction. "
                         "Default: 0.5 * max(robot_length, robot_width)."))
    p.add_argument("--hollow-ratio-thresh", type=float, default=0.35,
                   help=("AUTO mode: if (occupied_area / hull_area) is below this, "
                         "treat as hollow ring and use grid polygonization (default: 0.35)"))
    p.add_argument("--min-contour-area", type=float, default=0.02,
                   help="Minimum contour area (m^2) to keep (default: 0.02)")

    p.add_argument("--merge-distance", type=float, default=ROBOT_MAX_DIM_M,
                   help=f"Merge polygons closer than this (default: {ROBOT_MAX_DIM_M} m)")

    # -- Output --
    p.add_argument("--save-polygons", default=None,
                   help="Save obstacle polygons to CSV file")
    p.add_argument("--no-3d", action="store_true",
                   help="Skip the 3D visualisation window")

    return p


def run_pipeline(args: argparse.Namespace) -> None:
    sep = "=" * 68

    # ------------------------------------------------------------------
    # 1. Load point cloud
    # ------------------------------------------------------------------
    print(f"\n{sep}")
    print("STEP 1 - Load point cloud")
    print(sep)
    points = load_point_cloud(args.pcd)
    print(f"  Total points : {len(points)}")
    print(f"  X range      : [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}]")
    print(f"  Y range      : [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}]")
    print(f"  Z range      : [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")

    # ------------------------------------------------------------------
    # 2. Extract path from bag (optional)
    # ------------------------------------------------------------------
    path_poses: Optional[np.ndarray] = None
    if args.bag is not None:
        print(f"\n{sep}")
        print("STEP 2 - Extract path from rosbag")
        print(sep)
        path_poses = read_path_from_bag(args.bag, args.path_topic)
        print(f"  Path poses   : {len(path_poses)}")
        diffs = np.diff(path_poses[:, :2], axis=0)
        path_len = float(np.sum(np.linalg.norm(diffs, axis=1)))
        print(f"  Path distance: {path_len:.2f} m")
    else:
        print(f"\n{sep}")
        print("STEP 2 - No bag provided; using simple z-threshold for ground")
        print(sep)

    # ------------------------------------------------------------------
    # 3. Ground detection
    # ------------------------------------------------------------------
    print(f"\n{sep}")
    print("STEP 3 - Ground detection")
    print(sep)

    if path_poses is not None:
        # 3a. Extract ground candidates from robot footprint
        print("  3a. Extracting ground points under robot footprint ...")
        fp_ground_pts, fp_ground_mask = extract_footprint_ground(
            points, path_poses,
            robot_length=args.robot_length,
            robot_width=args.robot_width,
            footprint_margin=args.footprint_margin,
            z_max=args.ground_z_max,
        )
        print(f"      Footprint ground candidates: {len(fp_ground_pts)}")

        if len(fp_ground_pts) < 10:
            print("  [WARN] Very few footprint ground points - falling back "
                  "to z-threshold ground detection.")
            fp_ground_pts = points[points[:, 2] <= args.ground_z_max]
            fp_ground_mask = points[:, 2] <= args.ground_z_max
    else:
        fp_ground_mask = points[:, 2] <= args.ground_z_max
        fp_ground_pts = points[fp_ground_mask]
        print(f"  Ground candidates (z <= {args.ground_z_max}): "
              f"{len(fp_ground_pts)}")

    # 3b. RANSAC plane fit
    print("  3b. Fitting ground plane (RANSAC) ...")
    if len(fp_ground_pts) < 20:
        print("  [WARN] Too few ground points for RANSAC. "
              "Using median-z plane.")
        ground_normal = np.array([0.0, 0.0, 1.0])
        ground_d = -float(np.median(fp_ground_pts[:, 2])) \
            if len(fp_ground_pts) else 0.0
    else:
        ground_normal, ground_d, _inlier_mask = fit_ground_plane_ransac(
            fp_ground_pts,
            n_iterations=args.ransac_iters,
            distance_threshold=args.ransac_thresh,
        )

    # Ground z level (evaluated at centroid of path or cloud)
    if path_poses is not None:
        ref_xy = path_poses[:, :2].mean(axis=0)
    else:
        ref_xy = points[:, :2].mean(axis=0)
    ground_z_level = -(ground_normal[0] * ref_xy[0]
                       + ground_normal[1] * ref_xy[1]
                       + ground_d) / (ground_normal[2] + 1e-12)
    print(f"  Ground z at centroid: {ground_z_level:.4f} m")

    # 3c. Classify ground in full cloud using fitted plane
    signed_dists = signed_distance_to_plane(points, ground_normal, ground_d)
    ground_mask = np.abs(signed_dists) <= args.ground_band
    print(f"  Ground band +/-{args.ground_band} m: "
          f"{ground_mask.sum()} points classified as ground")

    # ------------------------------------------------------------------
    # 4. Obstacle candidate extraction
    # ------------------------------------------------------------------
    print(f"\n{sep}")
    print("STEP 4 - Obstacle extraction")
    print(sep)

    # Positive obstacles: above ground band, z <= obstacle_z_max
    above_ground = signed_dists > args.ground_band
    below_ceiling = points[:, 2] <= args.obstacle_z_max
    positive_obs_mask = above_ground & below_ceiling

    # Negative obstacles (troughs): significantly below ground band
    trough_mask = signed_dists < -(args.ground_band + args.trough_depth)
    negative_obs_mask = trough_mask & (points[:, 2] <= args.ground_z_max)

    raw_obstacle_mask = positive_obs_mask | negative_obs_mask
    print(f"  Positive obstacles (bumps, z <= {args.obstacle_z_max}): "
          f"{positive_obs_mask.sum()}")
    print(f"  Negative obstacles (troughs, depth > {args.trough_depth}): "
          f"{negative_obs_mask.sum()}")
    print(f"  Total raw obstacle candidates: {raw_obstacle_mask.sum()}")

    obstacle_pts = points[raw_obstacle_mask]

    # ------------------------------------------------------------------
    # 5. Statistical outlier removal
    # ------------------------------------------------------------------
    print(f"\n{sep}")
    print("STEP 5 - Statistical outlier removal")
    print(sep)

    if len(obstacle_pts) > 0:
        clean_obs_pts, keep_mask = remove_statistical_outliers(
            obstacle_pts, k=args.outlier_k, std_ratio=args.outlier_std,
        )
        n_removed = len(obstacle_pts) - len(clean_obs_pts)
        print(f"  Removed {n_removed} outliers "
              f"({len(clean_obs_pts)} remaining)")

        # Update full-cloud mask
        obs_indices = np.nonzero(raw_obstacle_mask)[0]
        cleaned_obstacle_mask = np.zeros(len(points), dtype=bool)
        cleaned_obstacle_mask[obs_indices[keep_mask]] = True
    else:
        clean_obs_pts = obstacle_pts
        cleaned_obstacle_mask = raw_obstacle_mask
        print("  No obstacle candidates to filter.")

    # ------------------------------------------------------------------
    # 6. 2D projection + DBSCAN clustering
    # ------------------------------------------------------------------
    print(f"\n{sep}")
    print("STEP 6 - 2D clustering (DBSCAN)")
    print(sep)

    if len(clean_obs_pts) == 0:
        print("  No obstacle points to cluster.")
        labels = np.array([], dtype=int)
        cluster_list: List[np.ndarray] = []
    else:
        obs_xy = clean_obs_pts[:, :2].copy()
        labels = dbscan(obs_xy, eps=args.cluster_eps,
                        min_samples=args.cluster_min_pts)
        unique_labels = sorted(set(labels))
        if -1 in unique_labels:
            unique_labels.remove(-1)
        n_noise = int((labels == -1).sum())
        print(f"  Clusters found : {len(unique_labels)}")
        print(f"  Noise points   : {n_noise}")

        cluster_list = []
        for cid in unique_labels:
            cl_pts = obs_xy[labels == cid]
            cluster_list.append(cl_pts)
            print(f"    Cluster {cid}: {len(cl_pts)} pts, "
                  f"span=({cl_pts[:, 0].ptp():.3f} x "
                  f"{cl_pts[:, 1].ptp():.3f}) m")

    # ------------------------------------------------------------------
    # 7. Polygon generation & chaining
    # ------------------------------------------------------------------
    print(f"\n{sep}")
    print("STEP 7 - Polygon generation & chaining")
    print(sep)

    # 7a. Build per-cluster representative hulls (used for merge grouping heuristics)
    cluster_hulls: List[np.ndarray] = []
    valid_cluster_list: List[np.ndarray] = []
    for cl_pts in cluster_list:
        hull = cluster_to_convex_hull(cl_pts)
        if hull is None:
            continue
        cluster_hulls.append(hull)
        valid_cluster_list.append(cl_pts)
    cluster_list = valid_cluster_list

    # 7b. Merge nearby clusters (union-find) and polygonize merged groups
    groups = merge_nearby_clusters_indices(cluster_list, args.merge_distance)
    if len(groups) > 1:
        print(f"  Merging enabled: {len(cluster_list)} clusters -> {len(groups)} groups "
              f"(gap threshold = {args.merge_distance} m)")
    elif groups:
        print("  Single group - no merging needed.")
    else:
        print("  No clusters to form polygons from.")

    obstacle_shapes: List[ObstacleShape] = []
    total_holes = 0
    for gid, idxs in enumerate(groups):
        merged_pts = np.vstack([cluster_list[i] for i in idxs]) if idxs else np.empty((0, 2))
        if len(merged_pts) < 3:
            continue
        merged_hull = cluster_to_convex_hull(merged_pts)
        if merged_hull is None:
            continue

        hull_area = _polygon_area(merged_hull)
        method = "hull"
        grid_shapes: List[Tuple[np.ndarray, List[np.ndarray]]] = []

        if args.polygon_mode in ("auto", "grid"):
            occ, _, _ = _occupancy_grid_from_points(
                merged_pts, cell_size=args.grid_cell, padding=0.20,
            )
            occ_area = float(occ.sum()) * (args.grid_cell ** 2)
            hollow_ratio = occ_area / max(hull_area, 1e-9)
            use_grid = (
                args.polygon_mode == "grid"
                or (hollow_ratio < args.hollow_ratio_thresh and hull_area > args.min_contour_area)
            )
            if use_grid:
                grid_shapes = polygonize_cluster_grid(
                    merged_pts,
                    cell_size=args.grid_cell,
                    inflate_radius_m=float(args.inflate_radius),
                    min_contour_area=args.min_contour_area,
                )
                if grid_shapes:
                    method = "grid"

        if method == "grid":
            hole_count = sum(len(holes) for _, holes in grid_shapes)
            total_holes += hole_count
            print(
                f"  Group {gid}: GRID shapes={len(grid_shapes)}, holes={hole_count}, "
                f"hull_area={hull_area:.3f} m\u00B2, clusters={len(idxs)}"
            )
            for (outer, holes) in grid_shapes:
                obstacle_shapes.append(
                    ObstacleShape(outer=outer, holes=holes, cluster_idx=gid, method="grid")
                )
        else:
            obstacle_shapes.append(
                ObstacleShape(outer=merged_hull, holes=[], cluster_idx=gid, method="hull")
            )
            print(
                f"  Group {gid}: HULL vertices={len(merged_hull)}, "
                f"area={hull_area:.3f} m\u00B2, clusters={len(idxs)}"
            )

    print(f"\n  >>> Obstacle shapes : {len(obstacle_shapes)} "
          f"(holes total = {total_holes})")

    # ------------------------------------------------------------------
    # 8. (Optional) save polygons
    # ------------------------------------------------------------------
    if args.save_polygons and obstacle_shapes:
        print(f"\n{sep}")
        print(f"Saving polygons -> {args.save_polygons}")
        print(sep)
        import csv
        with open(args.save_polygons, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["polygon_id", "type", "shape_id", "hole_id", "vertex_idx", "x", "y"])
            pid = 0
            for sid, sh in enumerate(obstacle_shapes):
                # outer ring
                for vi, (px, py) in enumerate(sh.outer):
                    writer.writerow([pid, "outer", sid, -1, vi, f"{px:.6f}", f"{py:.6f}"])
                pid += 1
                # holes
                for hid, hole in enumerate(sh.holes):
                    for vi, (px, py) in enumerate(hole):
                        writer.writerow([pid, "hole", sid, hid, vi, f"{px:.6f}", f"{py:.6f}"])
                    pid += 1
        print(f"  Saved {len(obstacle_shapes)} obstacle shape(s).")

    # ------------------------------------------------------------------
    # 9. Visualisation
    # ------------------------------------------------------------------
    print(f"\n{sep}")
    print("STEP 9 - Visualisation")
    print(sep)

    if not args.no_3d:
        print("  Preparing 3D view ...")
        visualise_3d(
            all_points=points,
            ground_mask=ground_mask,
            obstacle_mask=cleaned_obstacle_mask,
            path_poses=path_poses,
            ground_z_level=ground_z_level,
            obstacle_shapes=obstacle_shapes,
        )

    print("  Preparing 2D view ...")
    visualise_2d(
        all_points=points,
        ground_mask=ground_mask,
        obstacle_points_2d=(clean_obs_pts[:, :2]
                            if len(clean_obs_pts) else np.empty((0, 2))),
        path_poses=path_poses,
        obstacle_shapes=obstacle_shapes,
        cluster_points=cluster_list,
    )

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    rule = "-" * 68
    print(f"\n{rule}")
    print("SUMMARY")
    print(rule)
    print(f"  Total points loaded  : {len(points)}")
    print(f"  Ground points        : {ground_mask.sum()}")
    print(f"  Obstacle points      : {cleaned_obstacle_mask.sum()}")
    print(f"  Obstacle clusters    : {len(cluster_list)}")
    print(f"  Obstacle shapes      : {len(obstacle_shapes)}")
    obstacle_area = sum(
        _polygon_area(sh.outer) - sum(_polygon_area(h) for h in sh.holes)
        for sh in obstacle_shapes
    )
    print(f"  Obstacle area total  : {obstacle_area:.4f} m\u00B2")
    print(f"  Ground plane normal  : [{ground_normal[0]:.4f}, "
          f"{ground_normal[1]:.4f}, {ground_normal[2]:.4f}]")
    print(f"  Ground z at centroid : {ground_z_level:.4f} m")
    print(rule)

    plt.show()


# ═══════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    # Defaults derived from other args
    if args.inflate_radius is None:
        args.inflate_radius = 0.5 * float(max(args.robot_length, args.robot_width))
    if args.grid_cell <= 0:
        print("[ERROR] --grid-cell must be > 0", file=sys.stderr)
        sys.exit(1)
    if args.min_contour_area < 0:
        print("[ERROR] --min-contour-area must be >= 0", file=sys.stderr)
        sys.exit(1)

    # --- validate inputs ---
    if not os.path.exists(args.pcd):
        print(f"[ERROR] Point cloud not found: {args.pcd}", file=sys.stderr)
        sys.exit(1)
    if args.bag is not None and not os.path.isdir(args.bag):
        print(f"[ERROR] Bag directory not found: {args.bag}", file=sys.stderr)
        sys.exit(1)

    header = (
        "+" + "=" * 66 + "+\n"
        "|     AUTOMATED OBSTACLE DETECTION FROM POINT CLOUD              |\n"
        "+" + "=" * 66 + "+"
    )
    print(header)
    print(f"  Point cloud : {args.pcd}")
    print(f"  Rosbag      : {args.bag or '(none - z-threshold mode)'}")
    print(f"  Robot dims  : {args.robot_length} x {args.robot_width} m")
    print(f"  Merge gap   : {args.merge_distance} m")
    print(f"  Poly mode   : {args.polygon_mode} "
          f"(grid_cell={args.grid_cell} m, inflate={args.inflate_radius} m)")

    try:
        run_pipeline(args)
    except KeyboardInterrupt:
        print("\n[Interrupted]")
    except Exception as exc:
        print(f"\n[ERROR] {exc}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
