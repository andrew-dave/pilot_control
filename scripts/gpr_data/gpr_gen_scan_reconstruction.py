#!/usr/bin/env python3
"""
gpr_gen_scan_reconstruction.py

Reconstructs regularly spaced A-scans from an A.T-with-XYZ CSV following GENERAL TRAJECTORIES
(straight lines, curves, loops, etc.) using direct-distance POI placement.

Method (Robust to Odometry Noise):
- Generate POIs by greedy forward search: find next point at ~spacing_m from last POI
- Measures DIRECT distance from previous POI (not cumulative), avoiding error accumulation
- Local smoothing window averages nearby points to reduce position noise
- Distance-weighted averaging for trace reconstruction

Pipeline:
- Load input CSV: first column is TWT, columns 1..N are traces; bottom 3 rows are px,py,pz per trace
- Compute initial pose as average of first K positions
- Generate POIs by direct-distance search with configurable tolerance
- For each POI, compute distance-weighted, decluttered average of neighboring traces
- Save output CSV: first col TWT, columns are POI traces; append x,y,z rows of POIs
- Plot result with optional interactive tuning

Contrast Enhancement Options:
- percentile: Basic percentile-based amplitude clipping (default)
- agc: Automatic Gain Control - normalizes by local RMS in sliding window
- histeq: Histogram equalization - spreads amplitude distribution evenly
- log: Logarithmic scaling - emphasizes weak signals

Interactive Slider Mode (--sliders):
- Adjust all processing parameters in real-time with visual feedback
- POI spacing, tolerance, smoothing window, weight radius, declutter radius
- Contrast method, AGC window, contrast percentile, colormap selection
- Find optimal parameters before saving final reconstruction
- Buttons to save reconstructed CSV and visualization plot
"""

import sys
import os
import argparse
import csv
import numpy as np
from datetime import datetime


def _import_matplotlib_pyplot():
    try:
        import matplotlib.pyplot as plt  # type: ignore
        return plt
    except Exception as e:
        print(f"Matplotlib unavailable ({e}); skipping plot.")
        return None


def _import_matplotlib_widgets():
    try:
        from matplotlib.widgets import Slider, RadioButtons, Button  # type: ignore
        return Slider, RadioButtons, Button
    except Exception as e:
        print(f"Matplotlib widgets unavailable ({e}); interactive sliders disabled.")
        return None, None, None


def load_at_with_xyz_csv(path: str):
    """Load A.T-with-XYZ CSV saved by gpr_post_processing.py.

    Returns dict with keys: 'twt' (twt_len,), 'A_T' (twt_len, n_cols), 'px','py','pz' (n_cols,), 'header_cols' list.
    """
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = []
        for row in reader:
            rows.append(row)

    # Convert to float with empty -> NaN
    def to_float(x):
        try:
            if x == '' or x is None:
                return float('nan')
            return float(x)
        except Exception:
            return float('nan')

    M = np.array([[to_float(c) for c in r] for r in rows], dtype=float)
    if M.size == 0:
        return {'twt': np.zeros((0,)), 'A_T': np.zeros((0, 0)), 'px': np.zeros((0,)), 'py': np.zeros((0,)), 'pz': np.zeros((0,)), 'header_cols': header}

    # Detect last three rows as px/py/pz (first column NaN)
    if M.shape[0] < 4:
        raise RuntimeError("Input CSV does not contain appended px/py/pz rows.")
    twt_len = M.shape[0] - 3
    twt = M[:twt_len, 0]
    A_T = M[:twt_len, 1:]
    px = M[twt_len + 0, 1:]
    py = M[twt_len + 1, 1:]
    pz = M[twt_len + 2, 1:]
    return {'twt': twt, 'A_T': A_T, 'px': px, 'py': py, 'pz': pz, 'header_cols': header}


def compute_initial_pose(px: np.ndarray, py: np.ndarray, pz: np.ndarray, k: int = 10):
    """Compute initial pose by averaging first K valid positions."""
    k = int(max(1, min(k, px.size)))
    valid = np.isfinite(px[:k]) & np.isfinite(py[:k])
    if not np.any(valid):
        return (0.0, 0.0, 0.0)
    x0 = float(np.nanmean(px[:k]))
    y0 = float(np.nanmean(py[:k]))
    z0 = float(np.nanmean(pz[:k])) if pz.size >= k else 0.0
    return (x0, y0, z0)


def generate_pois_direct_distance(px: np.ndarray, py: np.ndarray, pz: np.ndarray,
                                  initial_pose: tuple, spacing_m: float,
                                  tolerance: float = 0.3, smoothing_window: int = 5):
    """Generate POIs by greedy forward search using DIRECT distance from last POI.
    
    This method avoids cumulative error by measuring direct distance from the previous
    POI to candidate points, rather than summing distances along the path.
    
    Args:
        px, py, pz: Position arrays from CSV
        initial_pose: (x0, y0, z0) starting position
        spacing_m: Target spacing between POIs in meters
        tolerance: Relative tolerance for spacing (0.3 = ±30%)
        smoothing_window: Number of nearby points to average for each POI
    
    Returns:
        pois_xyz: (M, 3) array of POI positions
        poi_indices: Original indices in px/py/pz for each POI
    """
    valid = np.isfinite(px) & np.isfinite(py) & np.isfinite(pz)
    
    if not np.any(valid):
        return np.array([initial_pose]), np.array([0])
    
    valid_indices = np.where(valid)[0]
    positions = np.stack([px, py, pz], axis=1)
    
    pois_xyz = [np.array(initial_pose)]
    poi_indices = [valid_indices[0] if len(valid_indices) > 0 else 0]
    
    current_poi = np.array(initial_pose)
    search_start = 0
    
    max_iterations = len(valid_indices) * 2  # Safety limit
    iteration = 0
    
    while search_start < len(valid_indices) and iteration < max_iterations:
        iteration += 1
        
        # Compute DIRECT distances from current POI to all remaining points
        remaining_indices = valid_indices[search_start:]
        if len(remaining_indices) == 0:
            break
        
        distances = np.linalg.norm(positions[remaining_indices] - current_poi[None, :], axis=1)
        
        # Find points within tolerance of target spacing
        tolerance_range = spacing_m * tolerance
        target_mask = np.abs(distances - spacing_m) <= tolerance_range
        
        if not np.any(target_mask):
            # No points in tolerance range - try to find closest point beyond spacing
            beyond_mask = distances >= spacing_m
            if np.any(beyond_mask):
                # Take the closest point that's at least spacing_m away
                candidate_idx = np.where(beyond_mask)[0][0]
            elif np.any(distances > 0):
                # Take the furthest available point
                candidate_idx = np.argmax(distances)
            else:
                break  # No more valid points
        else:
            # Choose the point closest to exact spacing within tolerance
            target_indices = np.where(target_mask)[0]
            candidate_idx = target_indices[np.argmin(np.abs(distances[target_indices] - spacing_m))]
        
        # Get the actual index in original array (latch point)
        latch_idx = remaining_indices[candidate_idx]
        
        # Apply local smoothing around latch point
        half_win = smoothing_window // 2
        latch_pos_in_valid = np.searchsorted(valid_indices, latch_idx)
        smooth_start = max(0, latch_pos_in_valid - half_win)
        smooth_end = min(len(valid_indices), latch_pos_in_valid + half_win + 1)
        window_indices = valid_indices[smooth_start:smooth_end]
        
        if len(window_indices) > 0:
            # Average positions in window to reduce noise
            new_poi = np.mean(positions[window_indices], axis=0)
            pois_xyz.append(new_poi)
            poi_indices.append(latch_idx)
            
            # Update for next iteration
            current_poi = new_poi
            search_start = latch_pos_in_valid + 1
        else:
            break
    
    return np.array(pois_xyz), np.array(poi_indices)


def compute_decluttered_weighted_average(A_T: np.ndarray,
                                         px: np.ndarray,
                                         py: np.ndarray,
                                         pois_xy: np.ndarray,
                                         radius_m: float = 0.0025,
                                         declutter_r_m: float = 0.0010):
    """Distance-weighted, decluttered average traces at POIs.

    Returns matrix (twt_len, M) where M = len(pois).
    """
    twt_len, n_cols = A_T.shape
    M = pois_xy.shape[0]
    out = np.zeros((twt_len, M), dtype=A_T.dtype)
    # Precompute source positions
    src = np.stack([px, py], axis=1)
    valid_src = np.isfinite(src).all(axis=1)
    for i in range(M):
        poi = pois_xy[i]
        if not np.any(valid_src):
            continue
        d = np.full(n_cols, np.inf)
        d[valid_src] = np.hypot(src[valid_src, 0] - poi[0], src[valid_src, 1] - poi[1])
        mask = d <= float(radius_m)
        if not np.any(mask):
            # Fallback: nearest neighbor
            j = int(np.nanargmin(d)) if np.any(np.isfinite(d)) else 0
            out[:, i] = A_T[:, j]
            continue
        # Distance weights
        w_d = np.clip(1.0 - (d[mask] / float(radius_m)), 0.0, 1.0)
        idxs = np.nonzero(mask)[0]
        # Declutter weights based on local neighbor count
        sel_xy = src[idxs]
        if sel_xy.shape[0] > 1:
            declut = np.ones(sel_xy.shape[0], dtype=float)
            for k in range(sel_xy.shape[0]):
                dd = np.hypot(sel_xy[:, 0] - sel_xy[k, 0], sel_xy[:, 1] - sel_xy[k, 1])
                n_close = int(np.sum(dd <= float(declutter_r_m)))
                declut[k] = 1.0 / max(1, n_close)
        else:
            declut = np.ones(sel_xy.shape[0], dtype=float)
        w = w_d * declut
        ws = float(np.sum(w))
        if ws <= 1e-12:
            out[:, i] = np.mean(A_T[:, idxs], axis=1)
        else:
            w_norm = w / ws
            out[:, i] = (A_T[:, idxs] * w_norm[None, :]).sum(axis=1)
    return out


def apply_contrast_enhancement(data: np.ndarray, method: str = 'percentile', 
                               percentile: float = 98.0, agc_window: int = 50) -> np.ndarray:
    """Apply contrast enhancement to radargram data."""
    if data.size == 0 or not np.isfinite(data).any():
        return data
    
    enhanced = data.copy()
    
    if method == 'agc':
        # Automatic Gain Control
        twt_len, n_traces = enhanced.shape
        half_win = max(1, agc_window // 2)
        for i in range(n_traces):
            trace = enhanced[:, i]
            rms = np.zeros_like(trace)
            for j in range(twt_len):
                start = max(0, j - half_win)
                end = min(twt_len, j + half_win + 1)
                window = trace[start:end]
                rms[j] = np.sqrt(np.mean(window**2)) if window.size > 0 else 1.0
            rms = np.where(rms < 1e-10, 1.0, rms)
            enhanced[:, i] = trace / rms
        vmax = np.percentile(np.abs(enhanced[np.isfinite(enhanced)]), 99.5)
        enhanced = np.clip(enhanced, -vmax, vmax)
        
    elif method == 'histeq':
        # Histogram equalization
        valid_data = enhanced[np.isfinite(enhanced)]
        if valid_data.size > 0:
            vmin, vmax = np.percentile(valid_data, [1, 99])
            enhanced = (enhanced - vmin) / (vmax - vmin + 1e-10)
            enhanced = np.clip(enhanced, 0, 1)
            hist, bins = np.histogram(enhanced[np.isfinite(enhanced)].flatten(), bins=256, range=(0, 1))
            cdf = hist.cumsum()
            cdf = cdf / cdf[-1]
            enhanced_flat = enhanced.flatten()
            valid_mask = np.isfinite(enhanced_flat)
            enhanced_flat[valid_mask] = np.interp(enhanced_flat[valid_mask], bins[:-1], cdf)
            enhanced = enhanced_flat.reshape(enhanced.shape)
            enhanced = 2 * enhanced - 1
            
    elif method == 'log':
        # Logarithmic scaling
        sign = np.sign(enhanced)
        enhanced = sign * np.log10(1 + np.abs(enhanced) / (np.percentile(np.abs(enhanced[np.isfinite(enhanced)]), 50) + 1e-10))
        vmax = np.percentile(np.abs(enhanced[np.isfinite(enhanced)]), 99)
        enhanced = np.clip(enhanced, -vmax, vmax)
        
    else:  # percentile (default)
        pass
    
    return enhanced


def plot_result_interactive_sliders(data_dict: dict, initial_params: dict):
    """Interactive plot with parameter sliders for real-time tuning.
    
    Args:
        data_dict: dict with 'twt', 'A_T', 'px', 'py', 'pz' from load_at_with_xyz_csv
        initial_params: dict with initial values for all parameters
    """
    plt = _import_matplotlib_pyplot()
    Slider, RadioButtons, Button = _import_matplotlib_widgets()
    if plt is None or Slider is None:
        print("Interactive sliders require matplotlib with widgets support.")
        return
    
    twt = data_dict['twt']
    A_T = data_dict['A_T']
    px = data_dict['px']
    py = data_dict['py']
    pz = data_dict['pz']
    
    if A_T.size == 0:
        print("Empty data; cannot plot.")
        return
    
    # Compute initial pose (doesn't change with sliders)
    initial_k = initial_params.get('initial_k', 10)
    initial = compute_initial_pose(px, py, pz, k=initial_k)
    
    # Create figure with space for sliders at bottom
    fig = plt.figure(figsize=(14, 10))
    
    # Main plot takes upper portion
    ax_main = plt.axes([0.1, 0.35, 0.75, 0.60])
    ax_pos = plt.axes([0.87, 0.35, 0.08, 0.60])  # Position axis on right
    
    # Slider axes in lower portion
    slider_height = 0.02
    slider_spacing = 0.03
    slider_left = 0.15
    slider_width = 0.7
    
    # Reconstruction parameter sliders (top section)
    ax_spacing = plt.axes([slider_left, 0.28, slider_width, slider_height])
    ax_tolerance = plt.axes([slider_left, 0.25, slider_width, slider_height])
    ax_smoothing = plt.axes([slider_left, 0.22, slider_width, slider_height])
    ax_weight_r = plt.axes([slider_left, 0.19, slider_width, slider_height])
    ax_declutter_r = plt.axes([slider_left, 0.16, slider_width, slider_height])
    
    # Contrast/visualization sliders (bottom section)
    ax_agc_win = plt.axes([slider_left, 0.10, slider_width, slider_height])
    ax_contrast_pct = plt.axes([slider_left, 0.07, slider_width, slider_height])
    
    # Contrast method selection and buttons
    ax_contrast_method = plt.axes([0.02, 0.02, 0.15, 0.08])
    ax_colormap = plt.axes([0.88, 0.12, 0.08, 0.10])
    ax_inspect_mode = plt.axes([0.75, 0.02, 0.10, 0.08])
    ax_print_button = plt.axes([0.20, 0.02, 0.12, 0.03])
    ax_save_csv_button = plt.axes([0.35, 0.02, 0.12, 0.03])
    ax_save_plot_button = plt.axes([0.50, 0.02, 0.12, 0.03])
    
    # Add section labels
    fig.text(slider_left - 0.02, 0.30, 'Reconstruction Parameters:', 
             fontsize=10, fontweight='bold', ha='right')
    fig.text(slider_left - 0.02, 0.12, 'Contrast Enhancement:', 
             fontsize=10, fontweight='bold', ha='right', color='purple')
    fig.text(0.02, 0.105, 'Method', fontsize=9, fontweight='bold', ha='left')
    fig.text(0.88, 0.23, 'Colormap', fontsize=9, fontweight='bold', ha='left')
    fig.text(0.75, 0.105, 'Inspect', fontsize=9, fontweight='bold', ha='left')
    
    # Create sliders
    slider_spacing_m = Slider(ax_spacing, 'POI Spacing (mm)', 1.0, 20.0, 
                               valinit=initial_params.get('spacing_m', 0.005) * 1000, 
                               valstep=0.5, color='skyblue')
    slider_tolerance = Slider(ax_tolerance, 'Tolerance (±%)', 5.0, 50.0,
                              valinit=initial_params.get('tolerance', 0.3) * 100,
                              valstep=5.0, color='lightcyan')
    slider_smoothing = Slider(ax_smoothing, 'Smoothing Window', 1, 15,
                              valinit=initial_params.get('smoothing_window', 5),
                              valstep=1, color='lightsteelblue')
    slider_weight_r = Slider(ax_weight_r, 'Weight Radius (mm)', 0.5, 10.0, 
                              valinit=initial_params.get('weight_radius_m', 0.0025) * 1000, 
                              valstep=0.1, color='lightgreen')
    slider_declutter_r = Slider(ax_declutter_r, 'Declutter Radius (mm)', 0.1, 5.0, 
                                 valinit=initial_params.get('declutter_radius_m', 0.0010) * 1000, 
                                 valstep=0.1, color='lightyellow')
    slider_agc_win = Slider(ax_agc_win, 'AGC Window (samples)', 10, 200, 
                            valinit=initial_params.get('agc_window', 50), 
                            valstep=5, color='lightcoral')
    slider_contrast_pct = Slider(ax_contrast_pct, 'Contrast %ile', 90.0, 99.9, 
                                  valinit=initial_params.get('contrast_percentile', 98.0), 
                                  valstep=0.1, color='plum')
    
    # Radio buttons for contrast method
    radio_contrast = RadioButtons(ax_contrast_method, 
                                   ('percentile', 'agc', 'histeq', 'log'),
                                   active=['percentile', 'agc', 'histeq', 'log'].index(
                                       initial_params.get('contrast_method', 'percentile')))
    
    # Radio buttons for colormap selection
    radio_colormap = RadioButtons(ax_colormap,
                                   ('gray', 'seismic', 'RdBu', 'viridis', 'plasma'),
                                   active=0)
    
    # Radio buttons for inspect mode
    radio_inspect = RadioButtons(ax_inspect_mode,
                                  ('Off', 'On'),
                                  active=0)
    
    # Buttons
    btn_print = Button(ax_print_button, 'Print Params', color='lightblue', hovercolor='skyblue')
    btn_save_csv = Button(ax_save_csv_button, 'Save CSV', color='lightgreen', hovercolor='limegreen')
    btn_save_plot = Button(ax_save_plot_button, 'Save Plot', color='lightsalmon', hovercolor='salmon')
    
    # State to hold current plot objects and UI text
    state = {
        'im': None,
        'pos_lines': [],
        'colorbar': None,
        'computing': False,
        'param_info_text': None,
        'A_T_reconstructed': None,  # Current reconstruction (no contrast enhancement)
        'pois_xyz': None,  # Current POI positions
        'twt': twt,  # TWT values (constant)
        'first_update': True,  # Flag for first plot
        'last_spacing_m': initial_params.get('spacing_m', 0.005),
        'last_tolerance': initial_params.get('tolerance', 0.3),
        'last_smoothing': initial_params.get('smoothing_window', 5),
        'last_weight_r': initial_params.get('weight_radius_m', 0.0025),
        'last_declutter_r': initial_params.get('declutter_radius_m', 0.0010),
        'inspect_vline': None,  # Vertical line for trace inspection
        'inspect_text': None,  # Text box showing location info
        'inspect_enabled': False  # Whether inspection is active
    }
    
    # Add info text for active parameters
    state['param_info_text'] = fig.text(0.35, 0.06, '', fontsize=8, style='italic', 
                                        color='darkblue', ha='left',
                                        bbox=dict(boxstyle='round', fc='lightyellow', 
                                                 ec='orange', alpha=0.7))
    
    def update_plot(val=None):
        if state['computing']:
            return
        state['computing'] = True
        
        # Get current slider values
        spacing_m = slider_spacing_m.val / 1000.0
        tolerance = slider_tolerance.val / 100.0
        smoothing_window = int(slider_smoothing.val)
        weight_radius_m = slider_weight_r.val / 1000.0
        declutter_radius_m = slider_declutter_r.val / 1000.0
        agc_window = int(slider_agc_win.val)
        contrast_percentile = slider_contrast_pct.val
        contrast_method = radio_contrast.value_selected
        colormap = radio_colormap.value_selected
        
        # Check if processing parameters changed (these affect reconstruction/POI generation)
        processing_params_changed = (
            abs(spacing_m - state['last_spacing_m']) > 1e-9 or
            abs(tolerance - state['last_tolerance']) > 1e-9 or
            smoothing_window != state['last_smoothing'] or
            abs(weight_radius_m - state['last_weight_r']) > 1e-9 or
            abs(declutter_radius_m - state['last_declutter_r']) > 1e-9
        )
        
        # Store current view's LEFT and RIGHT boundary physical locations BEFORE regenerating
        old_pois_xyz = state['pois_xyz']
        view_left_xy = None
        view_right_xy = None
        if processing_params_changed and not state['first_update'] and old_pois_xyz is not None and len(old_pois_xyz) > 0:
            # Get current x-axis limits (trace indices at left and right edges of view)
            xlim = ax_main.get_xlim()
            left_idx = int(np.clip(np.round(xlim[0]), 0, len(old_pois_xyz) - 1))
            right_idx = int(np.clip(np.round(xlim[1]), 0, len(old_pois_xyz) - 1))
            
            # Store the physical XY positions at the view boundaries
            view_left_xy = old_pois_xyz[left_idx, :2].copy()
            view_right_xy = old_pois_xyz[right_idx, :2].copy()
        
        # Recompute POIs using direct-distance method
        pois_xyz, poi_indices = generate_pois_direct_distance(
            px, py, pz, initial, spacing_m, tolerance, smoothing_window
        )
        
        # Recompute reconstruction
        pois_xy = pois_xyz[:, :2]
        A_T_out = compute_decluttered_weighted_average(
            A_T, px, py, pois_xy,
            radius_m=weight_radius_m,
            declutter_r_m=declutter_radius_m,
        )
        
        # Store reconstruction and POIs in state (for saving)
        state['A_T_reconstructed'] = A_T_out
        state['pois_xyz'] = pois_xyz
        
        # Apply contrast enhancement
        A_T_enhanced = apply_contrast_enhancement(A_T_out, method=contrast_method, 
                                                  percentile=contrast_percentile, 
                                                  agc_window=agc_window)
        
        x = np.arange(A_T_enhanced.shape[1], dtype=float)
        v = np.percentile(np.abs(A_T_enhanced), contrast_percentile) if np.isfinite(A_T_enhanced).any() else 1.0
        
        # Update main image
        if state['im'] is None:
            state['im'] = ax_main.imshow(
                A_T_enhanced,
                aspect="auto",
                cmap=colormap,
                vmin=-v,
                vmax=v,
                extent=[float(x[0]), float(x[-1]) if x.size > 1 else 1.0, 
                        float(twt[-1]) if twt.size > 1 else 1.0, float(twt[0])],
            )
            ax_main.set_xlabel("POI index", fontsize=10)
            ax_main.set_ylabel("TWT (ms)", fontsize=10)
            if state['colorbar'] is None:
                state['colorbar'] = plt.colorbar(state['im'], ax=ax_main, label="Amplitude")
        else:
            state['im'].set_data(A_T_enhanced)
            state['im'].set_extent([float(x[0]), float(x[-1]) if x.size > 1 else 1.0, 
                                    float(twt[-1]) if twt.size > 1 else 1.0, float(twt[0])])
            state['im'].set_clim(-v, v)
            state['im'].set_cmap(colormap)
        
        # Update title
        title = f"General Trajectory Radargram ({spacing_m*1000:.1f} mm spacing, {A_T_out.shape[1]} POIs)"
        if contrast_method != 'percentile':
            title += f" [{contrast_method.upper()}]"
        ax_main.set_title(title, fontsize=11)
        
        # Update parameter info text based on active contrast method
        if contrast_method == 'agc':
            info_str = f"AGC: Using window={agc_window} samples, clipping at {contrast_percentile:.1f}%ile"
        elif contrast_method == 'percentile':
            info_str = f"Percentile: Clipping at {contrast_percentile:.1f}%ile"
        elif contrast_method == 'histeq':
            info_str = f"Histogram Eq: Using {contrast_percentile:.1f}%ile for final clipping"
        elif contrast_method == 'log':
            info_str = f"Logarithmic: Using {contrast_percentile:.1f}%ile for final clipping"
        else:
            info_str = ""
        state['param_info_text'].set_text(f"Active: {info_str}")
        
        # Update position plot
        for line in state['pos_lines']:
            line.remove()
        state['pos_lines'] = []
        
        if pois_xyz.size > 0:
            state['pos_lines'].append(ax_pos.plot(pois_xyz[:, 0], x, 'r-', linewidth=1.0, label='x')[0])
            state['pos_lines'].append(ax_pos.plot(pois_xyz[:, 1], x, 'g-', linewidth=1.0, label='y')[0])
            state['pos_lines'].append(ax_pos.plot(pois_xyz[:, 2], x, 'b-', linewidth=1.0, label='z')[0])
            ax_pos.set_xlabel("Position (m)", fontsize=9)
            ax_pos.set_ylabel("POI index", fontsize=9)
            ax_pos.legend(loc='upper right', fontsize=8)
            ax_pos.grid(True, alpha=0.3)
        
        # Restore view to anchor the same physical locations at left and right edges
        if view_left_xy is not None and view_right_xy is not None and len(pois_xyz) > 0:
            # Find NEW trace indices that are closest to the OLD physical boundary positions
            distances_left = np.linalg.norm(pois_xyz[:, :2] - view_left_xy[None, :], axis=1)
            distances_right = np.linalg.norm(pois_xyz[:, :2] - view_right_xy[None, :], axis=1)
            
            new_left_idx = int(np.argmin(distances_left))
            new_right_idx = int(np.argmin(distances_right))
            
            # Ensure valid range
            if new_left_idx > new_right_idx:
                new_left_idx, new_right_idx = new_right_idx, new_left_idx
            
            # Add small margin to avoid exactly zero-width view
            if new_left_idx == new_right_idx:
                new_left_idx = max(0, new_left_idx - 1)
                new_right_idx = min(len(pois_xyz) - 1, new_right_idx + 1)
            
            # Set x-axis limits to show the same physical region
            ax_main.set_xlim(new_left_idx, new_right_idx)
            ax_pos.set_ylim(new_left_idx, new_right_idx)
        
        # Update last parameter values if processing params changed
        if processing_params_changed:
            state['last_spacing_m'] = spacing_m
            state['last_tolerance'] = tolerance
            state['last_smoothing'] = smoothing_window
            state['last_weight_r'] = weight_radius_m
            state['last_declutter_r'] = declutter_radius_m
        
        # Mark that first update is complete
        if state['first_update']:
            state['first_update'] = False
        
        fig.canvas.draw_idle()
        state['computing'] = False
    
    def print_params(event):
        """Print current parameter values to console."""
        print("\n" + "="*60)
        print("CURRENT PARAMETER VALUES:")
        print("="*60)
        print(f"  --spacing_m {slider_spacing_m.val / 1000.0:.6f}")
        print(f"  --tolerance {slider_tolerance.val / 100.0:.3f}")
        print(f"  --smoothing_window {int(slider_smoothing.val)}")
        print(f"  --weight_radius_m {slider_weight_r.val / 1000.0:.6f}")
        print(f"  --declutter_radius_m {slider_declutter_r.val / 1000.0:.6f}")
        print(f"  --agc_window {int(slider_agc_win.val)}")
        print(f"  --contrast_percentile {slider_contrast_pct.val:.2f}")
        print(f"  --contrast {radio_contrast.value_selected}")
        print("="*60)
        print("Copy these values to use in command line for batch processing.")
        print("="*60 + "\n")
    
    def save_csv_callback(event):
        """Save reconstructed CSV with current processing parameters."""
        if state['A_T_reconstructed'] is None or state['pois_xyz'] is None:
            print("No reconstruction data available to save.")
            return
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        spacing_str = f"{slider_spacing_m.val:.1f}mm"
        output_path = f"gen_reconstructed_{spacing_str}_{timestamp}.csv"
        
        # Save using the same format as save_output_csv
        A_T_out = state['A_T_reconstructed']
        pois_xyz = state['pois_xyz']
        twt = state['twt']
        
        header_cols = ["twt"] + [f"poi{i}" for i in range(A_T_out.shape[1])]
        mat = np.column_stack((twt, A_T_out))
        # Append x,y,z rows with blank in first cell
        x_row = np.concatenate(([np.nan], pois_xyz[:, 0]))
        y_row = np.concatenate(([np.nan], pois_xyz[:, 1]))
        z_row = np.concatenate(([np.nan], pois_xyz[:, 2]))
        out_mat = np.vstack((mat, x_row[None, :], y_row[None, :], z_row[None, :]))
        
        try:
            with open(output_path, 'w', newline='') as f:
                w = csv.writer(f, lineterminator='\n')
                w.writerow(header_cols)
                for r in out_mat:
                    w.writerow([f"{v:.10g}" if np.isfinite(v) else "" for v in r])
            print(f"\n✓ Saved reconstructed CSV: {output_path}")
            print(f"  Parameters: spacing={slider_spacing_m.val:.1f}mm, "
                  f"tolerance=±{slider_tolerance.val:.0f}%, smoothing={int(slider_smoothing.val)}")
            print(f"  Shape: {A_T_out.shape[0]} samples × {A_T_out.shape[1]} POIs\n")
        except Exception as e:
            print(f"✗ Failed to save CSV: {e}")
    
    def save_plot_callback(event):
        """Save current radargram visualization (main plot only, no UI controls)."""
        if state['A_T_reconstructed'] is None:
            print("No reconstruction data available to save.")
            return
        
        # Generate filename with timestamp and parameters
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        spacing_str = f"{slider_spacing_m.val:.1f}mm"
        contrast_str = radio_contrast.value_selected
        output_path = f"gen_radargram_{spacing_str}_{contrast_str}_{timestamp}.png"
        
        try:
            # Create a new figure with just the radargram
            save_fig, save_ax = plt.subplots(figsize=(12, 6))
            
            # Get current visualization parameters
            spacing_m = slider_spacing_m.val / 1000.0
            agc_window = int(slider_agc_win.val)
            contrast_percentile = slider_contrast_pct.val
            contrast_method = radio_contrast.value_selected
            colormap = radio_colormap.value_selected
            
            # Apply contrast enhancement
            A_T_enhanced = apply_contrast_enhancement(state['A_T_reconstructed'], 
                                                     method=contrast_method, 
                                                     percentile=contrast_percentile, 
                                                     agc_window=agc_window)
            
            x = np.arange(A_T_enhanced.shape[1], dtype=float)
            v = np.percentile(np.abs(A_T_enhanced), contrast_percentile) if np.isfinite(A_T_enhanced).any() else 1.0
            twt = state['twt']
            
            # Plot radargram
            im = save_ax.imshow(
                A_T_enhanced,
                aspect="auto",
                cmap=colormap,
                vmin=-v,
                vmax=v,
                extent=[float(x[0]), float(x[-1]) if x.size > 1 else 1.0, 
                        float(twt[-1]) if twt.size > 1 else 1.0, float(twt[0])],
            )
            save_ax.set_xlabel("POI index", fontsize=11)
            save_ax.set_ylabel("TWT (ms)", fontsize=11)
            
            title = f"General Trajectory Radargram ({spacing_m*1000:.1f} mm spacing, {A_T_enhanced.shape[1]} POIs)"
            if contrast_method != 'percentile':
                title += f" [{contrast_method.upper()}]"
            save_ax.set_title(title, fontsize=12)
            save_fig.colorbar(im, ax=save_ax, label="Amplitude")
            
            plt.tight_layout()
            save_fig.savefig(output_path, dpi=300, bbox_inches='tight')
            plt.close(save_fig)
            
            print(f"\n✓ Saved radargram plot: {output_path}")
            print(f"  Visualization: {contrast_method} with {colormap} colormap")
            print(f"  Resolution: 300 DPI\n")
        except Exception as e:
            print(f"✗ Failed to save plot: {e}")
    
    def toggle_inspect_mode(label):
        """Toggle trace inspection mode on/off."""
        state['inspect_enabled'] = (label == 'On')
        if not state['inspect_enabled']:
            # Remove inspection visuals when turned off
            if state['inspect_vline'] is not None:
                state['inspect_vline'].remove()
                state['inspect_vline'] = None
            if state['inspect_text'] is not None:
                state['inspect_text'].remove()
                state['inspect_text'] = None
            fig.canvas.draw_idle()
    
    def on_mouse_move(event):
        """Handle mouse motion for trace inspection."""
        if not state['inspect_enabled'] or event.inaxes != ax_main:
            return
        
        if state['pois_xyz'] is None or len(state['pois_xyz']) == 0:
            return
        
        if event.xdata is None:
            return
        
        # Find nearest trace index
        trace_idx = int(np.clip(np.round(event.xdata), 0, len(state['pois_xyz']) - 1))
        
        # Get location for this trace
        x_pos = state['pois_xyz'][trace_idx, 0]
        y_pos = state['pois_xyz'][trace_idx, 1]
        z_pos = state['pois_xyz'][trace_idx, 2]
        
        # Update or create vertical line
        if state['inspect_vline'] is None:
            state['inspect_vline'] = ax_main.axvline(trace_idx, color='yellow', linewidth=1.5, 
                                                     alpha=0.8, linestyle='--')
        else:
            state['inspect_vline'].set_xdata([trace_idx, trace_idx])
        
        # Update or create text box
        info_str = f"Trace: {trace_idx}\nX: {x_pos:.4f} m\nY: {y_pos:.4f} m\nZ: {z_pos:.4f} m"
        
        if state['inspect_text'] is None:
            state['inspect_text'] = ax_main.text(0.02, 0.98, info_str,
                                                 transform=ax_main.transAxes,
                                                 verticalalignment='top',
                                                 fontsize=9,
                                                 bbox=dict(boxstyle='round', facecolor='wheat', 
                                                          alpha=0.9, edgecolor='orange', linewidth=2))
        else:
            state['inspect_text'].set_text(info_str)
        
        fig.canvas.draw_idle()
    
    # Connect sliders and controls to update function
    slider_spacing_m.on_changed(update_plot)
    slider_tolerance.on_changed(update_plot)
    slider_smoothing.on_changed(update_plot)
    slider_weight_r.on_changed(update_plot)
    slider_declutter_r.on_changed(update_plot)
    slider_agc_win.on_changed(update_plot)
    slider_contrast_pct.on_changed(update_plot)
    radio_contrast.on_clicked(update_plot)
    radio_colormap.on_clicked(update_plot)
    radio_inspect.on_clicked(toggle_inspect_mode)
    
    # Connect mouse motion event for inspection
    fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)
    
    # Connect buttons
    btn_print.on_clicked(print_params)
    btn_save_csv.on_clicked(save_csv_callback)
    btn_save_plot.on_clicked(save_plot_callback)
    
    # Initial plot
    update_plot()
    
    # Add instruction text
    fig.text(0.5, 0.005, 
             "Adjust sliders to tune parameters | 'Save CSV' saves reconstruction data | 'Save Plot' saves radargram image (300 DPI)",
             ha='center', fontsize=9, style='italic', color='gray')
    
    plt.show()


def plot_trajectory_and_pois(px: np.ndarray, py: np.ndarray, pois_xyz: np.ndarray):
    """Plot the trajectory path and POI positions for verification."""
    plt = _import_matplotlib_pyplot()
    if plt is None:
        return
    
    valid = np.isfinite(px) & np.isfinite(py)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot trajectory
    ax.plot(px[valid], py[valid], 'b.', markersize=2, alpha=0.3, label='Raw trajectory')
    
    # Plot POIs
    if pois_xyz.size > 0:
        ax.plot(pois_xyz[:, 0], pois_xyz[:, 1], 'ro-', markersize=5, linewidth=1.5, 
                label=f'POIs (n={len(pois_xyz)})')
        
        # Mark start and end
        ax.plot(pois_xyz[0, 0], pois_xyz[0, 1], 'gs', markersize=12, label='Start')
        ax.plot(pois_xyz[-1, 0], pois_xyz[-1, 1], 'r^', markersize=12, label='End')
    
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_title('GPR Trajectory and POI Placement (Direct-Distance Method)', fontsize=12)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    plt.tight_layout()
    plt.show()


def plot_result(A_T_out: np.ndarray, twt: np.ndarray, pois_xyz: np.ndarray, spacing_m: float, 
                save_path: str = None, contrast_method: str = 'percentile',
                contrast_percentile: float = 98.0, agc_window: int = 50):
    """Plot reconstructed radargram."""
    plt = _import_matplotlib_pyplot()
    if plt is None:
        return
    if A_T_out.size == 0 or twt.size == 0:
        print("Nothing to plot: empty data.")
        return
    
    # Apply contrast enhancement
    A_T_enhanced = apply_contrast_enhancement(A_T_out, method=contrast_method, 
                                              percentile=contrast_percentile, 
                                              agc_window=agc_window)
    
    x = np.arange(A_T_enhanced.shape[1], dtype=float)
    v = np.percentile(np.abs(A_T_enhanced), contrast_percentile) if np.isfinite(A_T_enhanced).any() else 1.0
    
    fig, ax = plt.subplots(figsize=(12, 6))
    im = ax.imshow(
        A_T_enhanced,
        aspect="auto",
        cmap="gray",
        vmin=-v,
        vmax=v,
        extent=[float(x[0]), float(x[-1]), float(twt[-1]), float(twt[0])],
    )
    ax.set_xlabel("POI index")
    ax.set_ylabel("TWT (ms)")
    title = f"General Trajectory Radargram (~{spacing_m*1000:.0f} mm POI spacing)"
    if contrast_method != 'percentile':
        title += f" [{contrast_method.upper()}]"
    ax.set_title(title)
    fig.colorbar(im, ax=ax, label="Amplitude")

    if pois_xyz is not None and pois_xyz.size > 0:
        ax2 = ax.twinx()
        ax2.plot(x, pois_xyz[:, 0], 'r-', linewidth=1.0, label='x (m)')
        ax2.plot(x, pois_xyz[:, 1], 'g-', linewidth=1.0, label='y (m)')
        ax2.plot(x, pois_xyz[:, 2], 'b-', linewidth=1.0, label='z (m)')
        ax2.set_ylabel("Position (m)")
        ax2.legend(loc='upper right')

    plt.tight_layout()
    if save_path:
        try:
            plt.savefig(save_path, dpi=150)
            print(f"Saved plot: {save_path}")
        except Exception as e:
            print(f"Failed to save plot '{save_path}': {e}")
    else:
        plt.show()


def save_output_csv(base_out: str, twt: np.ndarray, A_T_out: np.ndarray, pois_xyz: np.ndarray) -> str:
    """Save reconstructed CSV with appended XYZ rows."""
    out_csv = base_out + "_gen_reconstructed.csv"
    header_cols = ["twt"] + [f"poi{i}" for i in range(A_T_out.shape[1])]
    mat = np.column_stack((twt, A_T_out))
    # Append x,y,z rows with blank in first cell
    x_row = np.concatenate(([np.nan], pois_xyz[:, 0]))
    y_row = np.concatenate(([np.nan], pois_xyz[:, 1]))
    z_row = np.concatenate(([np.nan], pois_xyz[:, 2]))
    out_mat = np.vstack((mat, x_row[None, :], y_row[None, :], z_row[None, :]))
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f, lineterminator='\n')
        w.writerow(header_cols)
        for r in out_mat:
            w.writerow([f"{v:.10g}" if np.isfinite(v) else "" for v in r])
    print(f"Saved reconstructed CSV: {out_csv}")
    return out_csv


def main():
    parser = argparse.ArgumentParser(description="GPR general trajectory reconstruction from A.T-with-XYZ CSV")
    parser.add_argument('input_csv', help='Path to A.T-with-XYZ CSV produced by gpr_post_processing.py')
    parser.add_argument('--spacing_m', type=float, default=0.005, help='Target POI spacing (m) (default: 0.005)')
    parser.add_argument('--tolerance', type=float, default=0.3, help='Spacing tolerance as fraction (default: 0.3 = ±30%%)')
    parser.add_argument('--smoothing_window', type=int, default=5, help='Points to average for POI smoothing (default: 5)')
    parser.add_argument('--weight_radius_m', type=float, default=0.0025, help='Distance weight radius (m) (default: 0.0025)')
    parser.add_argument('--declutter_radius_m', type=float, default=0.0010, help='Declutter radius (m) (default: 0.0010)')
    parser.add_argument('--initial_k', type=int, default=10, help='Samples to average for initial pose (default: 10)')
    parser.add_argument('--plot', action='store_true', help='Display reconstructed radargram')
    parser.add_argument('--plot_trajectory', action='store_true', help='Display trajectory with POI placement')
    parser.add_argument('--save_plot', type=str, default=None, help='Optional path to save radargram plot')
    parser.add_argument('--sliders', action='store_true', 
                        help='Enable interactive sliders for real-time parameter tuning (overrides --plot and --save_plot)')
    parser.add_argument('--contrast', type=str, default='percentile', 
                        choices=['percentile', 'agc', 'histeq', 'log'],
                        help='Contrast enhancement method: percentile (default), agc, histeq, or log')
    parser.add_argument('--contrast_percentile', type=float, default=98.0,
                        help='Percentile for amplitude clipping (default: 98.0)')
    parser.add_argument('--agc_window', type=int, default=50,
                        help='AGC window size in samples (default: 50)')
    args = parser.parse_args()

    if not os.path.isfile(args.input_csv):
        print(f"Input CSV not found: {args.input_csv}")
        sys.exit(1)

    print(f"Loading data from: {args.input_csv}")
    data = load_at_with_xyz_csv(args.input_csv)
    twt = data['twt']
    A_T = data['A_T']
    px = data['px']
    py = data['py']
    pz = data['pz']

    if A_T.size == 0:
        print("Empty input A_T; nothing to do.")
        sys.exit(0)

    print(f"Loaded {A_T.shape[0]} time samples × {A_T.shape[1]} traces")

    # Interactive slider mode - skip saving and jump to interactive tuning
    if args.sliders:
        print("Launching interactive slider mode for parameter tuning...")
        initial_params = {
            'spacing_m': args.spacing_m,
            'tolerance': args.tolerance,
            'smoothing_window': args.smoothing_window,
            'weight_radius_m': args.weight_radius_m,
            'declutter_radius_m': args.declutter_radius_m,
            'initial_k': args.initial_k,
            'agc_window': args.agc_window,
            'contrast_percentile': args.contrast_percentile,
            'contrast_method': args.contrast,
        }
        plot_result_interactive_sliders(data, initial_params)
        return

    # Normal mode - compute reconstruction and save
    # Compute initial pose
    initial = compute_initial_pose(px, py, pz, k=args.initial_k)
    print(f"Initial pose: x={initial[0]:.4f}, y={initial[1]:.4f}, z={initial[2]:.4f}")

    # Generate POIs using direct-distance method
    print(f"Generating POIs with spacing={args.spacing_m*1000:.2f}mm, "
          f"tolerance=±{args.tolerance*100:.0f}%, smoothing_window={args.smoothing_window}")
    pois_xyz, poi_indices = generate_pois_direct_distance(
        px, py, pz, initial, args.spacing_m, args.tolerance, args.smoothing_window
    )
    print(f"Generated {len(pois_xyz)} POIs")

    # Compute actual spacings for statistics
    if len(pois_xyz) > 1:
        spacings = np.linalg.norm(np.diff(pois_xyz, axis=0), axis=1)
        print(f"POI spacing statistics: mean={np.mean(spacings)*1000:.2f}mm, "
              f"std={np.std(spacings)*1000:.2f}mm, "
              f"min={np.min(spacings)*1000:.2f}mm, max={np.max(spacings)*1000:.2f}mm")

    # Reconstruct traces at POIs
    print(f"Computing weighted averages (radius={args.weight_radius_m*1000:.2f}mm, "
          f"declutter={args.declutter_radius_m*1000:.2f}mm)...")
    pois_xy = pois_xyz[:, :2]
    A_T_out = compute_decluttered_weighted_average(
        A_T, px, py, pois_xy,
        radius_m=args.weight_radius_m,
        declutter_r_m=args.declutter_radius_m,
    )

    # Save output
    base, _ = os.path.splitext(args.input_csv)
    out_csv = save_output_csv(base, twt, A_T_out, pois_xyz)
    print(f"Output shape: {A_T_out.shape[0]} samples × {A_T_out.shape[1]} POIs")

    # Plot trajectory
    if args.plot_trajectory:
        plot_trajectory_and_pois(px, py, pois_xyz)

    # Plot radargram
    if args.plot or (args.save_plot is not None and len(str(args.save_plot)) > 0):
        plot_result(A_T_out, twt, pois_xyz, args.spacing_m, 
                   save_path=args.save_plot, 
                   contrast_method=args.contrast,
                   contrast_percentile=args.contrast_percentile,
                   agc_window=args.agc_window)


if __name__ == '__main__':
    main()

