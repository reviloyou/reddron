"""
pillar_b.py

Thermal pixel -> ground position for the maritime SAR UAV.

Frames
------
C (camera): x_c right along increasing u, y_c down along increasing v,
            z_c out along the boresight.
B (body):   FRD. x_b nose, y_b right, z_b down. Matches ArduPilot.
N (nav):    NED. x_n north, y_n east, z_n down.

Everything in this file is SI. Angles are radians internally; the
configuration constants at the top are in degrees for readability.
"""

import numpy as np

# ----------------------------------------------------------------------
# Sensor configuration -- MLX90640
# ----------------------------------------------------------------------

N_COLS = 32          # pixel columns, indexed by u = 0..31
N_ROWS = 24          # pixel rows,    indexed by v = 0..23

FOV_COLS_DEG = 55.0  # FULL field of view across the 32-pixel axis
FOV_ROWS_DEG = 35.0  # FULL field of view across the 24-pixel axis

# Principal point: the pixel coordinate of the optical axis.
# With 32 columns indexed 0..31 the centre falls BETWEEN pixel 15 and 16.
C_U = (N_COLS - 1) / 2.0      # 15.5
C_V = (N_ROWS - 1) / 2.0      # 11.5

# Focal length expressed in PIXEL units.
# Derivation: at the edge of the field, the ray makes angle (FOV/2) with
# the boresight. That edge sits (N/2) pixels from the principal point.
# tan(FOV/2) = (N/2) / f   =>   f = (N/2) / tan(FOV/2)
F_U_NOMINAL = (N_COLS / 2.0) / np.tan(np.radians(FOV_COLS_DEG / 2.0))
F_V_NOMINAL = (N_ROWS / 2.0) / np.tan(np.radians(FOV_ROWS_DEG / 2.0))

# These two disagree by ~19%, which is physically impossible for square 
# detector pixels behind one lens. One datasheet number is defined
# differently (corner-to-corner vs edge-to-edge) or rounded. Replace both
# with measured values from calibrate_intrinsics() before flight.
F_U = F_U_NOMINAL
F_V = F_V_NOMINAL

# ----------------------------------------------------------------------
# Mounting configuration
# ----------------------------------------------------------------------

# The 55 deg axis (u, 32 px) runs LEFT-RIGHT across the airframe.
# The 35 deg axis (v, 24 px) runs FORE-AFT.
# Image top (v = 0) points toward the nose.

# Camera is tilted forward from nadir by this angle. Positive = looking
# ahead of the aircraft. Chosen to cancel the nose-down pitch the quad
# holds during forward cruise.
FORWARD_TILT_DEG = 12.5

# Camera optical centre relative to the flight controller, in body FRD,
LEVER_ARM_B = np.array([0.050, 0.000, 0.040])   # 5 cm fwd, 0cm sideways, 4 cm below (placeholder)

EARTH_RADIUS_M = 6371000.0


# ----------------------------------------------------------------------
# Step 1: pixel -> ray in camera frame
# ----------------------------------------------------------------------

def pixel_to_ray_camera(u, v, f_u=F_U, f_v=F_V):
    """Return an UNNORMALISED direction vector in the camera frame.

    The division (u - C_U) / f_u is pixels divided by pixels, so the
    result is dimensionless. It is literally tan(angle off the boresight)
    in the horizontal direction. Same for v. The third component is 1 by
    construction, which is what makes the first two components tangents.
    """
    return np.array([(u - C_U) / f_u,
                     (v - C_V) / f_v,
                     1.0])


# ----------------------------------------------------------------------
# Step 2: camera -> body (constant, set by how the pod is bolted on)
# ----------------------------------------------------------------------

def _rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0.0, s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0, c]])


def mounting_matrix(forward_tilt_deg=FORWARD_TILT_DEG):
    """R_bc: rotate a camera-frame vector into the body frame.

    Two parts composed:
      1. Axis remap for a downward pod with image-top toward the nose.
         x_c (right)     -> +y_b
         y_c (down rows) -> -x_b   (increasing v moves aft)
         z_c (boresight) -> +z_b   (down)
      2. Forward tilt about the body y (right) axis, which swings the
         boresight from straight down toward the nose.
    """
    axis_remap = np.array([[0.0, -1.0, 0.0],
                           [1.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0]])
    return _rot_y(np.radians(forward_tilt_deg)) @ axis_remap


# ----------------------------------------------------------------------
# Step 3: body -> nav, from the EKF attitude quaternion
# ----------------------------------------------------------------------

def quaternion_to_rotation(q):
    """R_nb from a MAVLink ATTITUDE_QUATERNION (w, x, y, z), Hamilton."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def euler_to_rotation(roll, pitch, yaw):
    """R_nb from Euler angles in radians, 3-2-1 (yaw, then pitch, then roll).

    Only used for testing and for readable inputs. In flight, prefer the
    quaternion path: it has fewer places to get a sign or an ordering wrong.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


# ----------------------------------------------------------------------
# LiDAR: slant range -> vertical height
# ----------------------------------------------------------------------

def lidar_range_to_height(slant_range_m, roll, pitch):
    """The TFmini-S points along +z_b, which is only 'down' when level.

    As soon as the aircraft tilts, the beam runs at an angle and measures
    a longer path. Project it back onto the vertical.
    """
    return slant_range_m * np.cos(roll) * np.cos(pitch)


def lidar_is_valid(slant_range_m, roll, pitch,
                   max_tilt_deg=15.0, min_m=0.3, max_m=11.0):
    """Reject readings we should not trust, so the LiDAR can run
    continuously instead of only in hover."""
    tilt = np.arccos(np.cos(roll) * np.cos(pitch))
    return (np.degrees(tilt) <= max_tilt_deg
            and min_m <= slant_range_m <= max_m)


# ----------------------------------------------------------------------
# Steps 4-5: lever arm and ground intersection
# ----------------------------------------------------------------------

def pixel_to_ground_ned(u, v, height_m, R_nb,
                        lever_arm_b=LEVER_ARM_B,
                        R_bc=None):
    """Full chain. Returns (north_m, east_m) of the target relative to the
    flight controller's position, plus diagnostics.

    height_m is the vertical distance from the FLIGHT CONTROLLER to the
    ground (what a LiDAR or barometer referenced to the FC reports), not
    from the camera. The camera sits lever_arm_b away from the FC, so this
    function derives the camera's own height internally
    (h_cam = height_m - cam_offset_n[2]) before tracing the ray. Passing a
    camera-to-ground height here double-counts the lever arm's vertical
    component.
    """
    if R_bc is None:
        R_bc = mounting_matrix()

    r_c = pixel_to_ray_camera(u, v)          # step 1, dimensionless
    r_b = R_bc @ r_c                         # step 2
    r_n = R_nb @ r_b                         # step 3

    if r_n[2] <= 1e-6:
        raise ValueError("Ray does not point downward; no ground intersection.")

    cam_offset_n = R_nb @ lever_arm_b        # step 4
    h_cam = height_m - cam_offset_n[2]

    s = h_cam / r_n[2]                       # step 5, metres
    north = cam_offset_n[0] + s * r_n[0]
    east = cam_offset_n[1] + s * r_n[1]

    off_nadir_deg = np.degrees(np.arccos(r_n[2] / np.linalg.norm(r_n)))
    return north, east, off_nadir_deg


# ----------------------------------------------------------------------
# Step 6: metres -> geodetic
# ----------------------------------------------------------------------

def ned_to_latlon(lat_deg, lon_deg, north_m, east_m):
    """Local flat-earth offset. Valid over the few hundred metres we care
    about. The cos(lat) term is essential this far north: at Alta a degree
    of longitude is about 38.2 km, not 111.3 km.
    """
    dlat = np.degrees(north_m / EARTH_RADIUS_M)
    dlon = np.degrees(east_m / (EARTH_RADIUS_M * np.cos(np.radians(lat_deg))))
    return lat_deg + dlat, lon_deg + dlon


# ----------------------------------------------------------------------
# Bench calibration
# ----------------------------------------------------------------------

def calibrate_intrinsics(observations, axis="u"):
    """Least-squares focal length from bench measurements.

    observations: list of (pixel_index, lateral_offset_m, distance_m)
    for a hot target moved sideways in front of a level camera.

    Model: (p - c) / f = X / L, so p - c = f * (X / L). One unknown,
    no intercept, so f = sum(a*b) / sum(a*a) with a = X/L, b = p - c.
    """
    c = C_U if axis == "u" else C_V
    a = np.array([x / L for _, x, L in observations])
    b = np.array([p - c for p, _, _ in observations])
    f = float(a @ b / (a @ a))
    residual_px = b - f * a
    return f, float(np.sqrt(np.mean(residual_px ** 2)))
