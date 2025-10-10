#!/usr/bin/env python3
# segy_explorer.py
# Usage: python segy_explorer.py your_file.sgy

import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import xarray as xr

try:
    import segysak.segy as sgy
except Exception:
    sgy = None

# --- Optional: richer header access if available ---
HAS_SEGYIO = False
try:
    import segyio
    HAS_SEGYIO = True
except Exception:
    pass

def decode_text_header(text_attr):
    """
    Try to render the 3200-byte textual header nicely.
    segysak may hand you a Python str already; sometimes it's EBCDIC-like.
    """
    # Already a "mostly" readable string?
    if isinstance(text_attr, str):
        s = text_attr
        # Try to re-decode if it looks garbled (heuristic)
        if "\x00" in s or s.count(" ") < 10:
            try:
                s = bytes(s, "latin1").decode("cp500")  # EBCDIC
            except Exception:
                pass
        return s
    # If it's bytes:
    if isinstance(text_attr, (bytes, bytearray)):
        # Try ASCII then EBCDIC
        for enc in ("ascii", "utf-8", "latin1", "cp500"):
            try:
                return text_attr.decode(enc, errors="replace")
            except Exception:
                continue
    return str(text_attr)

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

# def print_dataset_summary(ds):
#     print("\n=== DATASET SUMMARY ===")
#     print(ds)
#     print("\n=== ATTRIBUTES ===")
#     for k, v in ds.attrs.items():
#         print(f"{k}: {v}")

# def show_text_header(ds):
#     print("\n=== TEXTUAL HEADER (best-effort decode) ===")
#     txt = ds.attrs.get("text")
#     if txt is None:
#         print("(No textual header found)")
#         return
#     s = decode_text_header(txt)
#     # Print in 80-char lines like the original card image
#     for i in range(0, len(s), 80):
#         print(s[i:i+80])

def plot_radargram(da, title="GPR Radargram"):
    A = da.values
    # print(A.shape)
    cdp = da.coords.get("cdp", np.arange(A.shape[0])).values
    twt = da.coords.get("twt", np.arange(A.shape[1])).values

    # Contrast stretch around 0 using 98th percentile of |A|
    v = np.percentile(np.abs(A), 98)

    plt.figure(figsize=(12, 6))
    im = plt.imshow(
        A.T,
        aspect="auto",
        cmap="gray",
        vmin=-v, vmax=v,
        extent=[cdp[0], cdp[2], twt[-1], twt[0]],  # invert Y so time increases downward
    )
    plt.xlabel("CDP (trace index / horizontal position)")
    plt.ylabel("TWT (ms)")
    plt.title(title)
    plt.colorbar(im, label="Amplitude")
    plt.tight_layout()
    plt.show()

def plot_amplitude_histogram(da):
    A = da.values
    plt.figure(figsize=(8, 5))
    plt.hist(A.ravel(), bins=200)
    plt.title("Amplitude Histogram")
    plt.xlabel("Amplitude")
    plt.ylabel("Count")
    plt.tight_layout()
    plt.show()


def try_trace_headers_with_segyio(fname):
    """
    If segyio is present, extract a compact per-trace header table.
    We’ll grab commonly populated fields; if a field is missing in your file
    segyio returns zeros. This is *optional*.
    """
    if not HAS_SEGYIO:
        print("\n(segyio not installed — skipping trace-header table)")
        return

    print("\n=== TRACE HEADER SNAPSHOT (first 10 rows) ===")
    wanted = {
        "TraceNumber": segyio.TraceField.TraceNumber,           # bytes 1–4
        "CDP":         segyio.TraceField.CDP,                    # bytes 21–24
        "Inline3D":    segyio.TraceField.INLINE_3D,              # 189–192
        "Crossline3D": segyio.TraceField.CROSSLINE_3D,           # 193–196
        "SourceX":     segyio.TraceField.SourceX,                # 73–76
        "SourceY":     segyio.TraceField.SourceY,                # 77–80
        "GroupX":      segyio.TraceField.GroupX,                 # 81–84
        "GroupY":      segyio.TraceField.GroupY,                 # 85–88
        "ElevSrc":     segyio.TraceField.ElevationSource,        # 41–44
        "ElevRec":     segyio.TraceField.ElevationReceiver,      # 45–48
        "Year":        segyio.TraceField.YearDataRecorded,       # 157–158
        "Day":         segyio.TraceField.DayOfYear,              # 159–160
        "Hour":        segyio.TraceField.HourOfDay,              # 161–162
        "Minute":      segyio.TraceField.MinuteOfHour,           # 163–164
        "Second":      segyio.TraceField.SecondOfMinute,         # 165–166
        "CoordScalar": segyio.TraceField.SourceGroupScalar,      # 71–72 (scalar)
    }

    try:
        with segyio.open(fname, "r", strict=False, ignore_geometry=True) as f:
            rows = min(10, f.tracecount)
            cols = list(wanted.keys())
            data = {}
            for name, field in wanted.items():
                try:
                    data[name] = list(f.attributes(field)[:rows])
                except Exception:
                    data[name] = ["(na)"] * rows

            # Pretty print as a fixed-width table
            widths = {c: max(len(c), max(len(str(v)) for v in data[c])) for c in cols}
            header = " | ".join(c.ljust(widths[c]) for c in cols)
            print(header)
            print("-" * len(header))
            for i in range(rows):
                line = " | ".join(str(data[c][i]).ljust(widths[c]) for c in cols)
                print(line)
    except Exception as e:
        print(f"(segyio open failed: {e})")

def main():
    if len(sys.argv) < 2:
        # print("Usage: python segy_explorer.py your_file.sgy")
        sys.exit(1)

    fname = sys.argv[1]
    print(f"Loading: {fname}")

    ds, da = load_with_segysak(fname)
    print(ds)
    print(da)
    # print_dataset_summary(ds)
    # show_text_header(ds)
    
    A = da.values
    cdp = da.coords.get("cdp", np.arange(A.shape[0])).values
    twt = da.coords.get("twt", np.arange(A.shape[1])).values
    print(cdp.shape[0])

    # Visuals
    plot_radargram(da, title="GPR Radargram")
    # plot_amplitude_histogram(da)
    # plot_trace_rms(da)

    # Optional trace header peek (if segyio is available)
    # try_trace_headers_with_segyio(fname)

    # Bonus: basic time/depth metadata
    print("\n=== TIME / DEPTH INFO ===")
    sr = ds.attrs.get("sample_rate", None)  # ms between samples
    if sr is not None:
        print(f"Sample interval (dt): {sr} ms ({sr*1e3:.0f} µs)")
    twt = da.coords.get("twt", None)
    if twt is not None:
        print(f"TWT range: {float(np.min(twt)):.3f} .. {float(np.max(twt)):.3f} ms; samples: {twt.size}")

if __name__ == "__main__":
    main()
