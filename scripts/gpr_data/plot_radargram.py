#!/usr/bin/env python3
"""Plot a GPR radargram from a SEG-Y (.sgy) file.

Usage:
  python3 plot_radargram.py /path/to/file.sgy [--percentile 98]

Notes:
- Requires segysak (conda: `conda install -c conda-forge segysak`).
- Matplotlib is imported lazily to avoid environment conflicts.
"""

import sys
import argparse
import numpy as np

try:
    import segysak.segy as sgy
except Exception:
    sgy = None


def load_segy(fname):
    if sgy is None:
        raise RuntimeError(
            "segysak not installed. Install via conda-forge: conda install -c conda-forge segysak"
        )
    ds = sgy.segy_loader(fname)
    print(ds)
    # Pick primary data variable
    data_var = next(iter(ds.data_vars))
    da = ds[data_var]
    # Ensure (cdp, twt) order
    if da.dims == ("twt", "cdp"):
        da = da.transpose("cdp", "twt")
    return ds, da


def _import_matplotlib_pyplot():
    try:
        import matplotlib.pyplot as plt  # type: ignore
        return plt
    except Exception as e:
        print(f"Matplotlib unavailable ({e}); skipping plot.")
        return None


def plot_radargram(da, percentile=98, title="GPR Radargram"):
    plt = _import_matplotlib_pyplot()
    if plt is None:
        return

    A = da.values
    cdp = da.coords.get("cdp", np.arange(A.shape[0])).values
    twt = da.coords.get("twt", np.arange(A.shape[1])).values

    # Contrast stretch around 0 using specified percentile of |A|
    v = np.percentile(np.abs(A), float(percentile)) if A.size else 1.0

    plt.figure(figsize=(12, 6))
    im = plt.imshow(
        A.T,
        aspect="auto",
        cmap="gray",
        vmin=-v,
        vmax=v,
        extent=[float(cdp[0]), float(cdp[-1]), float(twt[-1]), float(twt[0])],
    )
    plt.xlabel("CDP (trace index / horizontal position)")
    plt.ylabel("TWT (ms)")
    plt.title(title)
    plt.colorbar(im, label="Amplitude")
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot a radargram from a SEG-Y file")
    parser.add_argument("segy_file", help="Path to SEG-Y file (.sgy)")
    parser.add_argument("--percentile", type=float, default=98.0, help="Percentile for contrast stretch (0-100)")
    args = parser.parse_args()

    _, da = load_segy(args.segy_file)
    plot_radargram(da, percentile=args.percentile)


if __name__ == "__main__":
    main()


