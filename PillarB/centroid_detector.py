"""
centroid_detector.py

Given a noisy thermal frame, find the (u, v) coordinate of the hot blob
by computing the weighted centre of mass. This is the core of Pillar B
detection: raw pixel values -> (u, v) -> ground position.
"""

import numpy as np


def detect_centroid(frame, threshold=None, min_pixels=5):
    """Find the centroid of the hottest region in a frame.
    
    Parameters
    ----------
    frame : ndarray, shape (24, 32)
        Thermal frame, arbitrary units.
    threshold : float or None
        Temperature threshold. Pixels below this are ignored.
        If None, use (max + min) / 2, a simple adaptive threshold.
    min_pixels : int
        Reject detections with fewer than this many pixels above threshold.
        Prevents noise spikes from creating false detections.
    
    Returns
    -------
    u_centroid, v_centroid : float or (None, None)
        Estimated pixel coordinates of the blob centre.
        Returns (None, None) if detection failed (too few pixels, etc).
    n_pixels : int
        Number of pixels that passed the threshold.
    signal_to_noise : float
        Ratio of peak value to RMS of background pixels below threshold.
        Useful for confidence scoring later.
    
    Notes
    -----
    The centroid is a **weighted average**, not just the peak pixel:
    
        u_c = sum(u_i * frame[i]) / sum(frame[i])   (for all i above threshold)
        v_c = similar
    
    This gives subpixel precision and is the standard in computer vision.
    A simple "find the brightest pixel" would lose half the position accuracy.
    """
    
    # Adaptive threshold if not specified
    if threshold is None:
        threshold = (np.max(frame) + np.min(frame)) / 2.0
    
    # Boolean mask of pixels above threshold
    mask = frame >= threshold
    n_pixels = np.sum(mask)
    
    # Fail if too few pixels
    if n_pixels < min_pixels:
        return None, None, n_pixels, 0.0
    
    # Pixel coordinates
    v_grid, u_grid = np.meshgrid(np.arange(24), np.arange(32), indexing='ij')
    
    # Weighted centre of mass
    # The weights are the pixel values themselves
    weights = frame[mask]
    u_centroid = np.sum(u_grid[mask] * weights) / np.sum(weights)
    v_centroid = np.sum(v_grid[mask] * weights) / np.sum(weights)
    
    # Signal-to-noise: peak vs RMS of background
    peak = np.max(frame)
    background_pixels = frame[~mask]
    background_rms = np.std(background_pixels) if len(background_pixels) > 0 else 1.0
    snr = (peak - threshold) / background_rms if background_rms > 0 else 0.0
    
    return u_centroid, v_centroid, n_pixels, snr


def detect_with_logging(frame, **kwargs):
    """Wrapper that returns a dict with all diagnostics.
    
    Useful for debugging and logging.
    """
    u, v, n_px, snr = detect_centroid(frame, **kwargs)
    return {
        'u': u,
        'v': v,
        'n_pixels': n_px,
        'snr': snr,
        'valid': u is not None,
    }
