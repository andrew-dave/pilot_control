#!/usr/bin/env python3
"""
csv_to_npy_converter.py

Convert large A.T-with-XYZ CSV files to memory-mappable .npy format.

This is a one-time conversion that enables fast, low-memory processing
of large GPR datasets using numpy memory mapping.

Usage:
    python3 csv_to_npy_converter.py input_AT_with_xyz.csv
    
Output:
    input_AT.npy      - A_T amplitude matrix (memory-mappable)
    input_xyz.npy     - Position data (px, py, pz)
    input_twt.npy     - Time axis
    input_header.txt  - Header information
"""

import sys
import os
import argparse
import csv
import numpy as np
from tqdm import tqdm


def convert_csv_to_npy_chunked(csv_path: str, chunk_size: int = 5000):
    """Convert large CSV to .npy format using chunked reading.
    
    Args:
        csv_path: Path to A.T-with-XYZ CSV
        chunk_size: Number of columns to process at once
    """
    print(f"Converting: {csv_path}")
    print(f"Using chunk size: {chunk_size} columns")
    
    # Step 1: Read header and determine dimensions
    print("\n[1/4] Reading CSV structure...")
    with open(csv_path, 'r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        
        # Count rows
        n_rows = sum(1 for _ in tqdm(reader, desc="Counting rows"))
    
    n_cols = len(header) - 1  # Exclude 'twt' column
    twt_len = n_rows - 3  # Last 3 rows are px, py, pz
    
    print(f"  Dimensions: {twt_len} time samples × {n_cols} traces")
    print(f"  Estimated size: {twt_len * n_cols * 8 / 1e9:.2f} GB")
    
    # Step 2: Prepare output path
    print("\n[2/4] Preparing output...")
    base_path = os.path.splitext(csv_path)[0]
    
    # Step 3: Load data from CSV
    print("\n[3/4] Loading data from CSV...")
    
    def to_float(x):
        try:
            if x == '' or x is None:
                return float('nan')
            return float(x)
        except Exception:
            return float('nan')
    
    with open(csv_path, 'r', newline='') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        
        rows = []
        for row in tqdm(reader, total=n_rows, desc="Reading CSV"):
            rows.append([to_float(c) for c in row])
    
    # Convert to numpy array
    M = np.array(rows, dtype=float)
    
    # Extract components
    print("\n[4/4] Extracting and saving components...")
    twt = M[:twt_len, 0].copy()
    A_T = M[:twt_len, 1:].copy()
    px = M[twt_len + 0, 1:].copy()
    py = M[twt_len + 1, 1:].copy()
    pz = M[twt_len + 2, 1:].copy()
    
    # Free memory
    del M, rows
    
    # Step 4: Save using np.save() which creates proper memory-mappable .npy format
    print("  Saving A_T matrix (this may take a moment for large files)...")
    np.save(f'{base_path}_AT.npy', A_T)
    del A_T  # Free memory
    
    print("  Saving auxiliary data...")
    np.save(f'{base_path}_twt.npy', twt)
    np.save(f'{base_path}_xyz.npy', np.column_stack([px, py, pz]))
    
    # Save header info
    with open(f'{base_path}_header.txt', 'w') as f:
        f.write(','.join(header) + '\n')
        f.write(f'twt_len={twt_len}\n')
        f.write(f'n_cols={n_cols}\n')
    
    print(f"\n✓ Conversion complete!")
    print(f"  Created files:")
    print(f"    {base_path}_AT.npy     ({os.path.getsize(f'{base_path}_AT.npy')/1e9:.2f} GB)")
    print(f"    {base_path}_xyz.npy    ({os.path.getsize(f'{base_path}_xyz.npy')/1e6:.2f} MB)")
    print(f"    {base_path}_twt.npy    ({os.path.getsize(f'{base_path}_twt.npy')/1e3:.2f} KB)")
    print(f"    {base_path}_header.txt")
    print(f"\nNow use: python3 gpr_gen_scan_reconstruction.py {base_path} --use_npy --sliders")


def main():
    parser = argparse.ArgumentParser(description="Convert large CSV to memory-mappable .npy format")
    parser.add_argument('csv_file', help='Path to A.T-with-XYZ CSV file')
    parser.add_argument('--chunk_size', type=int, default=5000, 
                        help='Number of columns to process at once (default: 5000)')
    args = parser.parse_args()
    
    if not os.path.isfile(args.csv_file):
        print(f"Error: File not found: {args.csv_file}")
        sys.exit(1)
    
    try:
        convert_csv_to_npy_chunked(args.csv_file, args.chunk_size)
    except Exception as e:
        print(f"\n✗ Conversion failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

