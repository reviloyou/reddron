"""
synthetic_frames.py

Generate fake 32x24 thermal frames with a hot Gaussian blob at a
specified pixel location, plus realistic Gaussian noise.

This lets you test the entire Pillar B pipeline without a camera.
You control the true position and can verify the centroid detector works.
"""

import numpy as np


def create_synthetic_frame(u_true, v_true, amplitude=20.0, sigma_px=1.5,
                           background=15.0, noise_sigma=0.5, rng=None):
    """Generate a 32x24 frame with a Gaussian blob at (u_true, v_true).
    
    Parameters
    ----------
    u_true, v_true : float
        True pixel coordinates of the blob centre.
    amplitude : float
        Peak temperature above background, in arbitrary units (K or counts).
    sigma_px : float
        Standard deviation of the Gaussian blob, in pixels.
    background : float
        Background temperature, uniform across frame.
    noise_sigma : float
        Standard deviation of Gaussian measurement noise per pixel.
    rng : np.random.Generator
        Random number generator. If None, creates a new one.
    
    Returns
    -------
    frame : ndarray, shape (24, 32)
        Synthetic thermal frame.
    
    Notes
    -----
    The frame is float, not integer. Real MLX90640 returns raw ADC counts
    which you then convert to temperature, but for testing, float is fine.
    """
    if rng is None:
        rng = np.random.default_rng()
    
    # Build the grid of pixel indices
    v_grid, u_grid = np.ogrid[0:24, 0:32]
    
    # Gaussian blob: exp(-(r^2 / (2*sigma^2)))
    # where r^2 = (u - u_true)^2 + (v - v_true)^2
    r_squared = (u_grid - u_true) ** 2 + (v_grid - v_true) ** 2
    blob = amplitude * np.exp(-r_squared / (2 * sigma_px ** 2))
    
    # Add uniform background
    frame = background + blob
    
    # Add Gaussian noise to every pixel
    noise = rng.normal(0.0, noise_sigma, size=(24, 32))
    frame = frame + noise
    
    return frame


def batch_synthetic_frames(positions, n_frames_per_pos=10, rng=None, **kwargs):
    """Generate multiple frames at different true positions.
    
    Parameters
    ----------
    positions : list of (u, v) tuples
        True positions where to place the blob.
    n_frames_per_pos : int
        How many independent frames per position (to accumulate noise samples).
    rng : np.random.Generator
        Random number generator.
    **kwargs
        Passed to create_synthetic_frame (amplitude, sigma_px, etc).
    
    Yields
    ------
    frame : ndarray
    u_true, v_true : float
        The true position for this frame.
    """
    if rng is None:
        rng = np.random.default_rng()
    
    for u, v in positions:
        for _ in range(n_frames_per_pos):
            frame = create_synthetic_frame(u, v, rng=rng, **kwargs)
            yield frame, u, v
