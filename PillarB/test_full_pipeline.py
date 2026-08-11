"""
test_full_pipeline.py

End-to-end test of Pillar B: generate synthetic frames, detect centroids,
transform to ground position, compare against known truth. This proves the
entire pipeline works before you touch hardware.
"""

import numpy as np
import pillar_b as pb
import synthetic_frames as sf
import centroid_detector as cd


def run_full_pipeline_test(n_frames=200, heights=[5.0], attitudes=None, rng=None):
    """Run synthetic frames through the entire Pillar B chain.
    
    Parameters
    ----------
    n_frames : int
        Total number of frames to generate.
    heights : list of float
        Altitudes to test at (metres).
    attitudes : list of (roll, pitch, yaw) tuples in radians, or None
        If None, generate random attitudes with small perturbations
        (typical flight) instead of level hover.
    rng : np.random.Generator
        Random number generator.
    
    Yields
    ------
    result : dict
        'frame' : raw 32x24 array
        'u_true', 'v_true' : ground truth pixel position
        'u_est', 'v_est' : detected centroid
        'n_detections' : number of regions detect_centroids() returned
        'north_true', 'east_true' : ground truth position (metres from FC)
        'north_est', 'east_est' : estimated ground position
        'error_m' : Euclidean distance error
        'off_nadir' : angle from nadir for this true position
        'valid' : exactly one detection, matched against ground truth

    Each synthetic frame has exactly one blob (synthetic_frames.py does
    not generate multiple), so a valid result here means
    detect_centroids() found exactly one region. Zero regions is a missed
    detection; more than one is a spurious split of a single blob, and
    both count as failures rather than silently picking one candidate,
    since either would hide a real regression in the detector.
    """
    if rng is None:
        rng = np.random.default_rng(42)
    
    if attitudes is None:
        # Generate small random perturbations around level flight
        attitudes = [pb.euler_to_rotation(*rng.normal(0.0, 0.1, 3))
                     for _ in range(n_frames)]
    
    # Test positions spread across the frame, including edges
    test_positions = [
        (pb.C_U, pb.C_V),           # centre
        (0, pb.C_V),                # left edge
        (31, pb.C_V),               # right edge
        (pb.C_U, 0),                # top edge
        (pb.C_U, 23),               # bottom edge
    ]
    
    # Replicate to fill n_frames
    positions = test_positions * (n_frames // len(test_positions) + 1)
    positions = positions[:n_frames]
    
    R_bc = pb.mounting_matrix(12.5)
    
    for i, (h, R_nb, (u_true, v_true)) in enumerate(
            zip(np.tile(heights, n_frames // len(heights) + 1)[:n_frames],
                attitudes,
                positions)):
        
        # Generate frame
        frame = sf.create_synthetic_frame(u_true, v_true, rng=rng)

        # Detect regions. Each synthetic frame has one true blob, so
        # "valid" means detect_centroids() found exactly one region, not
        # just "at least one" (a spurious split is also a failure).
        detections = cd.detect_centroids(frame)
        valid_detection = len(detections) == 1
        if valid_detection:
            u_est, v_est = detections[0]['u'], detections[0]['v']
            n_px, snr = detections[0]['n_pixels'], detections[0]['snr']
        else:
            u_est = v_est = None
            n_px, snr = sum(d['n_pixels'] for d in detections), 0.0

        # Compute ground truth position
        if valid_detection:
            # True transform
            north_true, east_true, off_nadir = pb.pixel_to_ground_ned(
                u_true, v_true, h, R_nb, pb.ZERO_LEVER, R_bc)
            
            # Estimated transform
            try:
                north_est, east_est, _ = pb.pixel_to_ground_ned(
                    u_est, v_est, h, R_nb, pb.ZERO_LEVER, R_bc)
            except ValueError:
                # Ray does not point downward (very rare with small attitudes)
                north_est, east_est = np.nan, np.nan
        else:
            north_true = east_true = north_est = east_est = np.nan
            off_nadir = np.nan
        
        # Error
        if valid_detection and not np.isnan(north_est):
            error_m = np.hypot(north_est - north_true, east_est - east_true)
        else:
            error_m = np.nan
        
        yield {
            'frame_index': i,
            'frame': frame,
            'u_true': u_true,
            'v_true': v_true,
            'u_est': u_est,
            'v_est': v_est,
            'n_detections': len(detections),
            'n_pixels': n_px,
            'snr': snr,
            'north_true': north_true,
            'east_true': east_true,
            'north_est': north_est,
            'east_est': east_est,
            'error_m': error_m,
            'off_nadir': off_nadir,
            'valid': valid_detection,
            'height': h,
        }


# Placeholder for when hardware is absent
pb.ZERO_LEVER = np.array([0.0, 0.0, 0.0])


if __name__ == '__main__':
    print("Running full Pillar B pipeline test...\n")
    
    errors = []
    valid_count = 0
    
    for result in run_full_pipeline_test(n_frames=100, heights=[5.0]):
        if result['valid']:
            valid_count += 1
            errors.append(result['error_m'])
            
            if result['frame_index'] < 5 or result['frame_index'] % 20 == 0:
                print(
                    f"Frame {result['frame_index']:3d}: "
                    f"true ({result['u_true']:.1f}, {result['v_true']:.1f}) "
                    f"-> est ({result['u_est']:.2f}, {result['v_est']:.2f})  "
                    f"error {result['error_m']:.3f} m  "
                    f"SNR {result['snr']:.1f}x  "
                    f"off-nadir {result['off_nadir']:.1f} deg"
                )
    
    errors = np.array(errors)
    print(f"\n{valid_count} of 100 detections succeeded.\n")
    print(f"Localisation error:")
    print(f"  mean   {errors.mean():.3f} m")
    print(f"  median {np.median(errors):.3f} m")
    print(f"  p95    {np.percentile(errors, 95):.3f} m")
    print(f"  p99    {np.percentile(errors, 99):.3f} m")
