#!/usr/bin/env python3
"""
Point Cloud Tilt Correction Transformer
=======================================

Transforms raw point clouds saved by raw_map_saver.cpp using tilt correction
matrices saved by the GPR scanning system.

Usage:
    python3 transform_point_cloud.py --pcd input.pcd --matrices tilt_correction_matrices.npz --output corrected.pcd

The transformation applied is:
    p_corrected = R_map @ (p_raw - p0_world)

Where:
- R_map: Final transformation matrix (3x3) that combines alignment and coordinate flip
- p0_world: Origin offset vector (3,) - the first raw odometry position
- p_raw: Raw point cloud positions

Dependencies:
- numpy
"""

import argparse
import numpy as np
import os
import sys


def load_transformation_matrices(matrix_file: str) -> tuple:
    """
    Load tilt correction transformation matrices from .npz file.

    Args:
        matrix_file: Path to the .npz file containing transformation matrices

    Returns:
        Tuple of (R_map, p0_world, metadata) where:
        - R_map: 3x3 transformation matrix
        - p0_world: 3D origin offset vector
        - metadata: dict with additional info (timestamp, etc.)
    """
    if not os.path.exists(matrix_file):
        raise FileNotFoundError(f"Matrix file not found: {matrix_file}")

    data = np.load(matrix_file)

    # Required matrices
    R_map = data['R_map']  # 3x3 transformation matrix
    p0_world = data['p0_world']  # 3D origin offset

    # Optional metadata
    metadata = {}
    if 'timestamp' in data:
        metadata['timestamp'] = data['timestamp'].item() if hasattr(data['timestamp'], 'item') else data['timestamp']
    if 'pitch_angle' in data:
        metadata['pitch_angle'] = data['pitch_angle'].item() if hasattr(data['pitch_angle'], 'item') else data['pitch_angle']

    print("Loaded transformation matrices:")
    print(f"  R_map shape: {R_map.shape}")
    print(f"  p0_world: {p0_world}")
    if metadata:
        print(f"  Metadata: {metadata}")

    return R_map, p0_world, metadata


def load_point_cloud_pcd(pcd_file: str):
    """
    Load point cloud from PCD file (ASCII format).

    Args:
        pcd_file: Path to the PCD file

    Returns:
        Tuple of (points, header_lines) where:
        - points: Nx3 numpy array of point coordinates
        - header_lines: List of header lines for preserving PCD format
    """
    if not os.path.exists(pcd_file):
        raise FileNotFoundError(f"PCD file not found: {pcd_file}")

    print(f"Loading point cloud from: {pcd_file}")

    with open(pcd_file, 'r') as f:
        lines = f.readlines()

    # Parse header
    header_lines = []
    data_start = 0
    num_points = 0
    fields = []

    for i, line in enumerate(lines):
        line = line.strip()
        header_lines.append(line)

        if line.startswith('FIELDS'):
            fields = line.split()[1:]
        elif line.startswith('POINTS'):
            num_points = int(line.split()[1])
        elif line.startswith('DATA'):
            data_start = i + 1
            break

    if num_points == 0:
        raise ValueError(f"Invalid PCD file: no POINTS field found")

    # Find indices of x, y, z fields
    try:
        x_idx = fields.index('x')
        y_idx = fields.index('y')
        z_idx = fields.index('z')
    except ValueError as e:
        raise ValueError(f"PCD file missing required fields (x, y, z): {e}")

    # Parse data points
    points = []
    for line in lines[data_start:data_start + num_points]:
        if line.strip():
            values = line.strip().split()
            if len(values) > max(x_idx, y_idx, z_idx):
                try:
                    x = float(values[x_idx])
                    y = float(values[y_idx])
                    z = float(values[z_idx])
                    points.append([x, y, z])
                except (ValueError, IndexError):
                    continue

    points = np.array(points)

    if len(points) == 0:
        raise ValueError(f"Point cloud is empty: {pcd_file}")

    print(f"Loaded {len(points)} points with fields: {fields}")
    return points, header_lines


def apply_tilt_correction(points: np.ndarray, R_map: np.ndarray, p0_world: np.ndarray):
    """
    Apply tilt correction transformation to point cloud.

    Transformation: p_corrected = R_map @ (p_raw - p0_world)

    Args:
        points: Nx3 numpy array of point coordinates
        R_map: 3x3 transformation matrix
        p0_world: 3D origin offset vector

    Returns:
        Transformed Nx3 numpy array of point coordinates
    """
    print(f"Original point cloud bounds: x=[{points[:, 0].min():.3f}, {points[:, 0].max():.3f}], "
          f"y=[{points[:, 1].min():.3f}, {points[:, 1].max():.3f}], "
          f"z=[{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")

    # Apply transformation: p_corrected = R_map @ (p_raw - p0_world)
    points_centered = points - p0_world  # Subtract origin offset
    points_transformed = (R_map @ points_centered.T).T  # Apply rotation matrix

    print(f"Transformed point cloud bounds: x=[{points_transformed[:, 0].min():.3f}, {points_transformed[:, 0].max():.3f}], "
          f"y=[{points_transformed[:, 1].min():.3f}, {points_transformed[:, 1].max():.3f}], "
          f"z=[{points_transformed[:, 2].min():.3f}, {points_transformed[:, 2].max():.3f}]")

    return points_transformed


def save_point_cloud_pcd(points: np.ndarray, header_lines: list, output_file: str):
    """
    Save point cloud to PCD file (ASCII format).

    Args:
        points: Nx3 numpy array of point coordinates
        header_lines: Original header lines to preserve PCD format
        output_file: Output PCD file path
    """
    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(output_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    print(f"Saving transformed point cloud to: {output_file}")

    with open(output_file, 'w') as f:
        # Write header (update POINTS count)
        for line in header_lines:
            if line.startswith('POINTS'):
                f.write(f"POINTS {len(points)}\n")
            else:
                f.write(line + '\n')

        # Write data points
        for point in points:
            f.write(' '.join(f"{coord:.6f}" for coord in point))
            # Add placeholder values for other fields if they existed in original
            # For now, just add zeros for intensity and normals as in the original
            f.write(' 0 0 0 0 0\n')

    print(f"Successfully saved {len(points)} points")


def main():
    parser = argparse.ArgumentParser(
        description="Transform raw point clouds using tilt correction matrices",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 transform_point_cloud.py --pcd raw_map.pcd --matrices tilt_correction_matrices.npz --output corrected_map.pcd

  # Process all PCD files in a directory
  for pcd in *.pcd; do
    base=$(basename "$pcd" .pcd)
    python3 transform_point_cloud.py --pcd "$pcd" --matrices tilt_correction_matrices.npz --output "corrected_$base.pcd"
  done
        """
    )

    parser.add_argument(
        '--pcd', '-p',
        required=True,
        help='Input PCD file (raw point cloud from raw_map_saver.cpp)'
    )

    parser.add_argument(
        '--matrices', '-m',
        required=True,
        help='Input NPZ file containing tilt correction matrices'
    )

    parser.add_argument(
        '--output', '-o',
        required=True,
        help='Output PCD file for tilt-corrected point cloud'
    )

    parser.add_argument(
        '--overwrite', '-f',
        action='store_true',
        help='Overwrite output file if it exists'
    )

    args = parser.parse_args()

    # Check if output file exists
    if os.path.exists(args.output) and not args.overwrite:
        print(f"Error: Output file already exists: {args.output}")
        print("Use --overwrite to overwrite existing files")
        sys.exit(1)

    try:
        print("=" * 70)
        print("POINT CLOUD TILT CORRECTION TRANSFORMER")
        print("=" * 70)

        # Load transformation matrices
        print("\n1. Loading transformation matrices...")
        R_map, p0_world, metadata = load_transformation_matrices(args.matrices)

        # Load point cloud
        print("\n2. Loading point cloud...")
        points, header_lines = load_point_cloud_pcd(args.pcd)

        # Apply transformation
        print("\n3. Applying tilt correction transformation...")
        points_transformed = apply_tilt_correction(points, R_map, p0_world)

        # Save transformed point cloud
        print("\n4. Saving transformed point cloud...")
        save_point_cloud_pcd(points_transformed, header_lines, args.output)

        print("\n" + "=" * 70)
        print("✓ TRANSFORMATION COMPLETE")
        print(f"  Input:  {args.pcd}")
        print(f"  Output: {args.output}")
        if metadata:
            print(f"  Timestamp: {metadata.get('timestamp', 'N/A')}")
            if 'pitch_angle' in metadata:
                print(f"  Pitch correction: {np.degrees(metadata['pitch_angle']):.2f}°")
        print("=" * 70)

    except Exception as e:
        print(f"\n❌ ERROR: {str(e)}")
        sys.exit(1)


if __name__ == '__main__':
    main()
