#!/usr/bin/env python3
"""Plot a GPR radargram from a SEG-Y (.sgy) file.

Usage:
  python3 plot_radargram.py /path/to/file.sgy [--percentile 98] [--info] [--show-text]

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


def _decode_text_header(text_attr):
    """Best-effort textual header decode (ASCII/UTF-8/Latin1/EBCDIC cp500)."""
    if isinstance(text_attr, str):
        return text_attr
    if isinstance(text_attr, (bytes, bytearray)):
        for enc in ("ascii", "utf-8", "latin1", "cp500"):
            try:
                return text_attr.decode(enc, errors="replace")
            except Exception:
                continue
    return str(text_attr)


def print_metadata(ds, da, show_text=False):
    """Print dataset/data array summaries and attributes."""
    # Dataset summary
    print("\n=== DATASET ===")
    print(ds)
    # Dataset attributes
    if getattr(ds, "attrs", None):
        print("\n=== DATASET ATTRS ===")
        for k, v in ds.attrs.items():
            print(f"{k}: {v}")
    # DataArray summary
    print("\n=== DATA ARRAY ===")
    print(da)
    # DataArray attributes
    if getattr(da, "attrs", None):
        print("\n=== DATA ARRAY ATTRS ===")
        for k, v in da.attrs.items():
            print(f"{k}: {v}")
    # Coordinates and dims
    print("\n=== DIMS ===")
    for d in da.dims:
        print(f"{d}: {int(da.sizes.get(d, 0))}")
    print("\n=== COORDS ===")
    for name, coord in da.coords.items():
        print(f"{name}: shape={tuple(coord.shape)} attrs={dict(getattr(coord, 'attrs', {}))}")
    # Optional textual header
    if show_text:
        txt = ds.attrs.get("text") if getattr(ds, "attrs", None) else None
        print("\n=== TEXTUAL HEADER (best-effort) ===")
        if txt is None:
            print("(none)")
        else:
            s = _decode_text_header(txt)
            # Print in 80-char lines to mimic card image
            for i in range(0, len(s), 80):
                print(s[i:i+80])


def _v_from_metadata_percentiles(ds, percentile, A):
    """Try to derive symmetric display limit v from ds.attrs['percentiles'].

    Strategy:
    - Interpret ds.attrs['percentiles'] as values for percentiles from 0..100 on an
      evenly spaced grid of length N. Pick indices near p and 100-p and use the
      larger absolute value for symmetric limits.
    - Fallback to numpy-based computation on |A| if metadata is missing or invalid.
    """
    try:
        perc = getattr(ds, 'attrs', {}).get('percentiles', None)
        if perc is None:
            raise ValueError('no metadata percentiles')
        arr = np.asarray(perc, dtype=float)
        if arr.size < 3 or not np.all(np.isfinite(arr)):
            raise ValueError('bad metadata percentiles')
        n = arr.size - 1
        idx_hi = int(np.clip(np.round(float(percentile) / 100.0 * n), 0, n))
        idx_lo = int(np.clip(np.round((1.0 - float(percentile) / 100.0) * n), 0, n))
        v_candidates = [abs(float(arr[idx_hi])), abs(float(arr[idx_lo]))]
        v = float(np.nanmax(v_candidates))
        if not np.isfinite(v) or v <= 0:
            raise ValueError('nonpositive v from metadata')
        return v
    except Exception:
        # Fallback: compute from data
        return float(np.percentile(np.abs(A), float(percentile))) if A.size else 1.0


def plot_radargram(da, percentile=98, title="GPR Radargram", ds=None):
    plt = _import_matplotlib_pyplot()
    if plt is None:
        return

    A = da.values
    cdp = da.coords.get("cdp", np.arange(A.shape[0])).values
    twt = da.coords.get("twt", np.arange(A.shape[1])).values

    # Prefer metadata-derived percentile if available; fallback to data percentile
    v = _v_from_metadata_percentiles(ds, percentile, A) if ds is not None else (np.percentile(np.abs(A), float(percentile)) if A.size else 1.0)

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


def print_percentiles(ds):
    perc = getattr(ds, "attrs", {}).get("percentiles", None)
    if perc is None:
        print("No 'percentiles' found in dataset attributes.")
        return
    arr = np.asarray(perc, dtype=float)
    if arr.size <= 1:
        ps = np.array([100.0])
    else:
        step = 100.0 / float(arr.size - 1)
        ps = step * np.arange(arr.size, dtype=float)
    print("\n=== METADATA PERCENTILES ===")
    for p, v in zip(ps, arr):
        print(f"{p:6.2f}%: {v:.8g}")


def main():
    parser = argparse.ArgumentParser(description="Plot a radargram from a SEG-Y file")
    parser.add_argument("segy_file", help="Path to SEG-Y file (.sgy)")
    parser.add_argument("--percentile", type=float, default=98.0, help="Percentile for contrast stretch (0-100)")
    parser.add_argument("--info", action="store_true", help="Print dataset/data array summaries and attributes")
    parser.add_argument("--show-text", action="store_true", help="Print textual header (if present)")
    parser.add_argument("--list-percentiles", action="store_true", help="List ds.attrs['percentiles'] as p:value pairs (0..100%)")
    args = parser.parse_args()

    ds, da = load_segy(args.segy_file)
    if args.info or args.show_text:
        print_metadata(ds, da, show_text=args.show_text)
    if args.list_percentiles:
        print_percentiles(ds)
    plot_radargram(da, percentile=args.percentile, ds=ds)


if __name__ == "__main__":
    main()


