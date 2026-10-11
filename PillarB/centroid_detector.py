"""
centroid_detector.py

Given a noisy thermal frame, find hot regions by thresholding, labelling
connected components, and computing a weighted centroid per region.

This replaces "one weighted centroid of everything above threshold",
which had two failure modes verified against the current detector
(see CLAUDE.md, A3):

  - Two separate warm objects merge into a single centroid sitting in the
    cold water between them, because there was never a notion of
    "region", only "all pixels above threshold" treated as one blob.
  - An empty frame still returns a detection, because the old adaptive
    threshold (max + min) / 2 sits at roughly the background level for
    pure noise (a symmetric noise distribution has its max about as far
    above the mean as its min is below), so about half the frame's
    pixels always clear it regardless of whether anything is actually
    there.

Connected-component labelling (scipy.ndimage.label) fixes the first
problem: each contiguous blob above threshold gets its own centroid.
An absolute contrast floor on top of the old adaptive threshold fixes
the second: a region only counts if it clears the background by a
multiple of the local noise sigma, not just by sitting on the bright
side of the frame's own average.
"""

import numpy as np
from scipy import ndimage


def _adaptive_threshold(frame):
    """The original threshold: halfway between the frame's extremes.
    Kept as one half of the final threshold, since it still helps for a
    strong single blob sitting on an unusually elevated background."""
    return (np.max(frame) + np.min(frame)) / 2.0


MAD_TO_SIGMA = 1.4826   # scales MAD to match a Gaussian's standard deviation


def _background_stats(frame):
    """Robust background level and noise sigma: median and median
    absolute deviation, scaled to be comparable to a standard deviation.

    Estimating sigma from the standard deviation of sub-threshold pixels
    (the original approach) is scene-dependent: that set still contains
    the entire skirt of any real Gaussian blob, so with a true sigma of
    0.5 it measured 0.947 (CLAUDE.md, T7). The median and MAD stay close
    to the true background statistics as long as the hot region covers a
    minority of the frame, regardless of whether or where a target is
    present, so they do not need a threshold to split "background" from
    "signal" first.
    """
    median = float(np.median(frame))
    mad = float(np.median(np.abs(frame - median)))
    return median, MAD_TO_SIGMA * mad


def detect_centroids(frame, threshold=None, min_pixels=5, max_pixels=100,
                     contrast_sigma=6.0):
    """Find every hot region in a frame above a contrast-qualified threshold.

    Parameters
    ----------
    frame : ndarray, shape (24, 32)
        Thermal frame, arbitrary units.
    threshold : float or None
        Override the threshold entirely. If None, use
        max(adaptive_threshold, background + contrast_sigma * background_std).
    min_pixels, max_pixels : int
        Size band. Below it, treat as a noise spike; above it, treat as
        something wrong with the frame rather than a point-ish target
        (a person in the water does not fill a quarter of a 55x35 degree
        frame from 5 m).
    contrast_sigma : float
        How many noise sigma above the background a region must clear.
        This is the absolute contrast floor: it is what makes an empty
        frame return an empty list instead of the centroid of whatever
        noise happened to be brightest.

    Returns
    -------
    list of dict
        One entry per accepted region: 'u', 'v' (subpixel centroid),
        'n_pixels', 'snr' (peak above background over noise sigma, a
        plain ratio, NOT decibels), and 'touches_edge'. Empty list if
        nothing passes. Sorted by n_pixels, largest first.

    Notes on 'touches_edge'
    -----------------------
    True when the region's bounding box reaches column 0 or 31, or row 0
    or 23. Such a region is cut off by the frame boundary: the part of
    the blob that fell outside was never measured, so the surviving
    footprint is asymmetric and the centroid is pulled inward.

    Measured with the default synthetic blob (sigma_px 1.5), a target
    whose true centre is at u = 0 is reported at u = 0.447, and one at
    u = 31 at u = 30.554. At 5 m altitude that is 7.4 cm of ground error,
    always directed toward the frame centre. A target one column further
    in (u = 1) shows a bias of +0.007 px, so the effect is confined to
    regions that actually reach the boundary.

    The flag is deliberately conservative: it also fires on regions whose
    bounding box merely reaches the boundary without being meaningfully
    cut off, so roughly the outer two-pixel ring gets marked and the
    unflagged interior is about 73 percent of the frame. It marks rather
    than rejects; the caller decides.

    This is not noise and averaging frames does not remove it. No
    reweighting can fix it either, since the missing pixels carry no
    information (T8 addressed the background-as-uniform-weight component
    and moved the bias only from 0.475 to 0.447). The same truncation
    will occur with the real sensor.
    """
    frame = np.asarray(frame, dtype=float)
    background, sigma = _background_stats(frame)
    adaptive = _adaptive_threshold(frame)
    contrast_floor = background + contrast_sigma * sigma

    if threshold is None:
        threshold = max(adaptive, contrast_floor)

    mask = frame >= threshold
    labels, n_labels = ndimage.label(mask)

    v_grid, u_grid = np.meshgrid(np.arange(frame.shape[0]), np.arange(frame.shape[1]),
                                 indexing='ij')

    detections = []
    for region_id in range(1, n_labels + 1):
        region_mask = labels == region_id
        n_px = int(np.sum(region_mask))
        if n_px < min_pixels or n_px > max_pixels:
            continue

        raw = frame[region_mask]
        # Weight by temperature excess above background, not raw pixel
        # value. The background is a uniform offset present on every
        # thresholded pixel; left in, it acts as a flat weight that pulls
        # the centroid toward the geometric middle of the region instead
        # of toward the true peak, and does so worst exactly where a
        # blob is truncated by the frame edge, since the surviving
        # footprint is already asymmetric there (T8). Clipped at zero:
        # a pixel just above threshold can sit fractionally below the
        # background estimate once noise is accounted for, and a
        # negative weight would pull the centroid the wrong way.
        weights = np.clip(raw - background, 0.0, None)
        sum_w = np.sum(weights)
        if sum_w <= 0:
            # Every pixel in this region is at or below the background
            # estimate (only possible right at the contrast floor).
            # Fall back to the raw weights rather than divide by zero.
            weights, sum_w = raw, np.sum(raw)
        u_c = float(np.sum(u_grid[region_mask] * weights) / sum_w)
        v_c = float(np.sum(v_grid[region_mask] * weights) / sum_w)
        peak = float(np.max(raw))
        snr = (peak - background) / sigma if sigma > 0 else 0.0

        # Bounding box of the region, used only to decide whether the
        # blob was cut off by the frame boundary.
        rows_hit = np.flatnonzero(region_mask.any(axis=1))
        cols_hit = np.flatnonzero(region_mask.any(axis=0))
        touches_edge = bool(cols_hit[0] == 0 or cols_hit[-1] == frame.shape[1] - 1
                            or rows_hit[0] == 0 or rows_hit[-1] == frame.shape[0] - 1)

        detections.append({'u': u_c, 'v': v_c, 'n_pixels': n_px, 'snr': snr,
                           'touches_edge': touches_edge})

    detections.sort(key=lambda d: d['n_pixels'], reverse=True)
    return detections


def detect_centroid(frame, **kwargs):
    """Backward-compatible single-detection wrapper around
    detect_centroids(). New code should call detect_centroids() and
    handle a list; this is for callers still expecting the old
    (u, v, n_pixels, snr) single-result shape, which by construction
    cannot see more than one target even if the frame has several.

    Returns (None, None, 0, 0.0) if nothing was detected.
    """
    detections = detect_centroids(frame, **kwargs)
    if not detections:
        return None, None, 0, 0.0
    best = detections[0]
    return best['u'], best['v'], best['n_pixels'], best['snr']


def detect_with_logging(frame, **kwargs):
    """Wrapper that returns a dict with all diagnostics for the single
    strongest detection. Useful for debugging and logging.
    """
    u, v, n_px, snr = detect_centroid(frame, **kwargs)
    return {
        'u': u,
        'v': v,
        'n_pixels': n_px,
        'snr': snr,
        'valid': u is not None,
    }
