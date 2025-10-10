#!/usr/bin/env python3
# segy_explorer.py
# Usage: python segy_explorer.py your_file.sgy

import sys
import os
import numpy as np
import matplotlib.pyplot as plt

try:
    import segysak.segy as sgy
except Exception:
    sgy = None

"""Minimal SEG-Y extractor: save radargram matrix to CSV and NPZ."""

def load_with_segysak(fname):
    if sgy is None:
        raise RuntimeError("segysak not installed. Install via conda-forge: conda install -c conda-forge segysak")
    ds = sgy.segy_loader(fname)  # keep using this; robust and simple
    # Pick main data var
    data_var = next(iter(ds.data_vars))
    da = ds[data_var]
    # Ensure (cdp, twt)
    if da.dims == ("twt", "cdp"):
        da = da.transpose("cdp", "twt")
    return ds, da


def plot_radargram(da, title="GPR Radargram"):
    A = da.values
    cdp = da.coords.get("cdp", np.arange(A.shape[0])).values
    twt = da.coords.get("twt", np.arange(A.shape[1])).values

    v = np.percentile(np.abs(A), 98)

    plt.figure(figsize=(12, 6))
    im = plt.imshow(
        A.T,
        aspect="auto",
        cmap="gray",
        vmin=-v,
        vmax=v,
        extent=[cdp[0], cdp[-1], twt[-1], twt[0]],  # invert Y so time increases downward
    )
    plt.xlabel("CDP (trace index / horizontal position)")
    plt.ylabel("TWT (ms)")
    plt.title(title)
    plt.colorbar(im, label="Amplitude")
    plt.tight_layout()
    plt.show()


def main():
    if len(sys.argv) < 2:
        # print("Usage: python segy_explorer.py your_file.sgy")
        sys.exit(1)

    fname = sys.argv[1]
    ds, da = load_with_segysak(fname)
    
    A = da.values
    cdp = da.coords.get("cdp", np.arange(A.shape[0])).values
    twt = da.coords.get("twt", np.arange(A.shape[1])).values
    # Base path for output files
    base, _ = os.path.splitext(fname)

    # Save full radargram matrix
    # CSV: rows are TWT samples, columns are CDP traces; first column is TWT
    matrix_csv = base + "_radargram_matrix.csv"
    header_cols = "twt," + ",".join(str(v) for v in cdp)
    matrix = np.column_stack((twt, A.T))
    np.savetxt(matrix_csv, matrix, delimiter=",", header=header_cols, comments="")
    print(f"Saved radargram matrix CSV: {matrix_csv}")

    # NPZ: include full matrix and axes for programmatic use
    radar_npz = base + "_radargram.npz"
    np.savez(radar_npz, radargram=A, cdp=cdp, twt=twt)
    print(f"Saved radargram NPZ: {radar_npz}")

    # Display radargram
    plot_radargram(da, title="GPR Radargram")

if __name__ == "__main__":
    main()
