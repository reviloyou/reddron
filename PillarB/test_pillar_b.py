"""Hand-checkable tests for the Pillar B transform, then a noise study."""

import sys

import numpy as np
import pillar_b as pb

ZERO_LEVER = np.zeros(3)
H = 5.0
LEVEL = pb.euler_to_rotation(0.0, 0.0, 0.0)

results = []


def check(label, got, expected, tol=0.01):
    ok = abs(got - expected) <= tol
    print(f"  [{'ok ' if ok else 'FAIL'}] {label:<46} "
          f"got {got:+8.4f}  expected {expected:+8.4f}")
    results.append(ok)
    return ok


print("Focal lengths from the nominal datasheet field of view")
print(f"  f_u = {pb.F_U_NOMINAL:.3f} px   f_v = {pb.F_V_NOMINAL:.3f} px")
print(f"  disagreement = {100*abs(pb.F_U_NOMINAL-pb.F_V_NOMINAL)/pb.F_V_NOMINAL:.1f}%"
      "  -> must be calibrated on the bench\n")

# ----------------------------------------------------------------------
print("A. Textbook checks with the camera pointing straight down "
      "(tilt = 0, no lever arm)")
flat = pb.mounting_matrix(0.0)

n, e, g = pb.pixel_to_ground_ned(pb.C_U, pb.C_V, H, LEVEL, ZERO_LEVER, flat)
check("1. centre pixel, level -> north", n, 0.0)
check("1. centre pixel, level -> east", e, 0.0)

n, e, g = pb.pixel_to_ground_ned(31, pb.C_V, H, LEVEL, ZERO_LEVER, flat)
check("2. last column centre -> east", e, 2.52, tol=0.02)
check("2. field edge (analytic 5*tan27.5)", H * np.tan(np.radians(27.5)), 2.60)

pitched = pb.euler_to_rotation(0.0, np.radians(10.0), 0.0)
n, e, g = pb.pixel_to_ground_ned(pb.C_U, pb.C_V, H, pitched, ZERO_LEVER, flat)
check("3. centre pixel, 10 deg nose UP -> north", n, +0.88, tol=0.02)

pitched_dn = pb.euler_to_rotation(0.0, np.radians(-10.0), 0.0)
n, e, g = pb.pixel_to_ground_ned(pb.C_U, pb.C_V, H, pitched_dn, ZERO_LEVER, flat)
check("3b. 10 deg nose DOWN looks BEHIND -> north", n, -0.88, tol=0.02)

# ----------------------------------------------------------------------
print("\nB. Checks with the real 12.5 deg forward mounting tilt")
tilted = pb.mounting_matrix(12.5)

n, e, g = pb.pixel_to_ground_ned(pb.C_U, pb.C_V, H, LEVEL, ZERO_LEVER, tilted)
check("4. centre pixel in hover -> north", n, H * np.tan(np.radians(12.5)))
check("4. off-nadir angle (deg)", g, 12.5)

cruise = pb.euler_to_rotation(0.0, np.radians(-12.5), 0.0)
n, e, g = pb.pixel_to_ground_ned(pb.C_U, pb.C_V, H, cruise, ZERO_LEVER, tilted)
check("5. tilt cancels 12.5 deg cruise pitch", n, 0.0)
check("5. off-nadir angle in cruise (deg)", g, 0.0)

for row, name, expected_north, expected_off_nadir in [
        (0, "front row v=0", 2.807, 29.31),
        (23, "rear row v=23", -0.377, 4.31)]:
    n, e, g = pb.pixel_to_ground_ned(pb.C_U, row, H, LEVEL, ZERO_LEVER, tilted)
    check(f"5b. {name} -> north", n, expected_north, tol=0.01)
    check(f"5c. {name} -> off-nadir (deg)", g, expected_off_nadir, tol=0.02)

# ----------------------------------------------------------------------
print("\nC. Yaw check: rotate 90 deg, a forward target must move east")
yawed = pb.euler_to_rotation(0.0, 0.0, np.radians(90.0))
n, e, g = pb.pixel_to_ground_ned(pb.C_U, pb.C_V, H, yawed, ZERO_LEVER, tilted)
check("6. yaw 90 -> north component", n, 0.0)
check("6. yaw 90 -> east component", e, H * np.tan(np.radians(12.5)))

# ----------------------------------------------------------------------
print("\nG. Lever arm checks (non-zero LEVER_ARM_B)")
print("  height_m is the FLIGHT CONTROLLER's height above ground, not the "
     "camera's. Every check above uses ZERO_LEVER, which hides that\n  "
     "distinction entirely: with no lever arm the two are the same "
     "number. These do not.")

# Straight-down ray (flat mounting, centre pixel, level attitude): a
# forward-only lever arm shows up as a pure north offset, hand-computed
# exactly since s * r_n[0] = 0 on a vertical ray.
lever_fwd = np.array([1.0, 0.0, 0.0])
n, e, g = pb.pixel_to_ground_ned(pb.C_U, pb.C_V, H, LEVEL, lever_fwd, flat)
check("7. lever arm 1 m fwd, straight-down ray -> north", n, 1.0, tol=1e-6)
check("7. lever arm 1 m fwd, straight-down ray -> east", e, 0.0, tol=1e-6)

# Same lever arm, yaw 90 deg: the lever arm itself is rotated by R_nb
# (cam_offset_n = R_nb @ lever_arm_b), so 1 m forward becomes 1 m east.
# This is what "the rotation of the lever arm" means and ZERO_LEVER can
# never exercise it, since 0 rotated by anything is still 0.
n, e, g = pb.pixel_to_ground_ned(pb.C_U, pb.C_V, H, yawed, lever_fwd, flat)
check("8. lever arm 1 m fwd, yaw 90 -> north", n, 0.0, tol=1e-6)
check("8. lever arm 1 m fwd, yaw 90 -> east", e, 1.0, tol=1e-6)

# The real LEVER_ARM_B, tilted mounting, level attitude, centre pixel:
# analytic closed form north = lever_x + (height_m - lever_z) * tan(tilt).
# height_m - lever_z is exactly h_cam, the camera's own height, which is
# the quantity the docstring bug would have skipped subtracting. Against
# the ZERO_LEVER answer for the same geometry (check 4, H*tan(12.5) =
# 1.1085 m) this lands about 4 cm away, the size of double-count CLAUDE.md
# warns about once real CAD numbers are in LEVER_ARM_B.
lever_real = pb.LEVER_ARM_B
expected_north = lever_real[0] + (H - lever_real[2]) * np.tan(np.radians(12.5))
n, e, g = pb.pixel_to_ground_ned(pb.C_U, pb.C_V, H, LEVEL, lever_real, tilted)
check("9. real LEVER_ARM_B, hover -> north", n, expected_north, tol=1e-6)
check("9. real LEVER_ARM_B, hover -> east", e, lever_real[1], tol=1e-6)
print(f"  shift from the ZERO_LEVER answer for the same geometry: "
     f"{(n - H * np.tan(np.radians(12.5))) * 100:+.1f} cm")

# ----------------------------------------------------------------------
print("\nD. Geodetic conversion at Alta, 100 m north and 100 m east")
lat0, lon0 = 69.9689, 23.2717
lat1, lon1 = pb.ned_to_latlon(lat0, lon0, 100.0, 100.0)
m_per_deg_lon = pb.EARTH_RADIUS_M * np.cos(np.radians(lat0)) * np.pi / 180.0
print(f"  metres per degree of longitude at 69.97 N: {m_per_deg_lon:.0f} m")
print(f"  (a mid-latitude constant of 111320 would be wrong by "
      f"{111320/m_per_deg_lon:.2f}x)")
print(f"  dlat {lat1-lat0:+.6f} deg   dlon {lon1-lon0:+.6f} deg")

# ----------------------------------------------------------------------
print("\nE. Monte Carlo error budget, 20000 runs, target at frame centre")
rng = np.random.default_rng(16)
SIGMA_ATT = np.radians(1.0)
SIGMA_H = 0.06
SIGMA_PX = 0.5

# The thermal frame and the MAVLink attitude it gets paired with arrive at
# different instants. Until frames carry a timestamp attitude_interpolator
# can SLERP against (A6, reserved, not implemented here), whatever attitude
# is "current" when a frame is processed is off by this latency, and the
# aircraft has moved ground_speed * latency in the meantime. That is a pure
# translation: it does not depend on attitude, pixel position or height, so
# unlike the other three sources it is added directly to the ground
# position rather than run back through pixel_to_ground_ned.
GROUND_SPEED_MS = 3.0     # cruise/search speed, matches the CLAUDE.md worked example
LATENCY_MEAN_S = 0.050    # MAVLink-over-UART + ESP32 processing, assumed until measured
LATENCY_JITTER_S = 0.020  # placeholder spread on the same assumption


def latency_translation_error(ground_speed_ms, latency_s, rng):
    """Displacement from pairing a frame with a stale attitude sample.
    Heading is drawn uniformly at random: this is a generic error budget,
    not a specific flight leg, so the direction of travel is not fixed."""
    displacement = ground_speed_ms * max(latency_s, 0.0)
    heading = rng.uniform(0.0, 2 * np.pi)
    return displacement * np.cos(heading), displacement * np.sin(heading)


n_true, e_true, _ = pb.pixel_to_ground_ned(
    pb.C_U, pb.C_V, H, LEVEL, ZERO_LEVER, tilted)

errs = []
errs_no_latency = []
for _ in range(20000):
    R = pb.euler_to_rotation(*rng.normal(0.0, SIGMA_ATT, 3))
    h = H + rng.normal(0.0, SIGMA_H)
    u = pb.C_U + rng.normal(0.0, SIGMA_PX)
    v = pb.C_V + rng.normal(0.0, SIGMA_PX)
    n, e, _ = pb.pixel_to_ground_ned(u, v, h, R, ZERO_LEVER, tilted)
    errs_no_latency.append(np.hypot(n - n_true, e - e_true))
    lat_s = rng.normal(LATENCY_MEAN_S, LATENCY_JITTER_S)
    dn, de = latency_translation_error(GROUND_SPEED_MS, lat_s, rng)
    n += dn
    e += de
    errs.append(np.hypot(n - n_true, e - e_true))

errs = np.array(errs)
errs_no_latency = np.array(errs_no_latency)
print(f"  without latency: mean {errs_no_latency.mean():.3f} m   "
     f"median {np.median(errs_no_latency):.3f} m")
print(f"  new combined total (with latency):")
print(f"    mean   {errs.mean():.3f} m")
print(f"    median {np.median(errs):.3f} m")
print(f"    p95    {np.percentile(errs, 95):.3f} m")

print("\n  Contribution of each source in isolation (RMS, metres):")
for name, sa, sh, sp in [("attitude 1 deg", SIGMA_ATT, 0, 0),
                         ("height 6 cm", 0, SIGMA_H, 0),
                         ("pixel 0.5 px", 0, 0, SIGMA_PX)]:
    d = []
    for _ in range(8000):
        R = pb.euler_to_rotation(*rng.normal(0.0, sa, 3)) if sa else LEVEL
        h = H + (rng.normal(0.0, sh) if sh else 0.0)
        u = pb.C_U + (rng.normal(0.0, sp) if sp else 0.0)
        v = pb.C_V + (rng.normal(0.0, sp) if sp else 0.0)
        n, e, _ = pb.pixel_to_ground_ned(u, v, h, R, ZERO_LEVER, tilted)
        d.append(np.hypot(n - n_true, e - e_true))
    print(f"    {name:<18} {np.sqrt(np.mean(np.square(d))):.3f}")

d = []
for _ in range(8000):
    lat_s = rng.normal(LATENCY_MEAN_S, LATENCY_JITTER_S)
    dn, de = latency_translation_error(GROUND_SPEED_MS, lat_s, rng)
    d.append(np.hypot(dn, de))
print(f"    {'latency 50+-20 ms':<18} {np.sqrt(np.mean(np.square(d))):.3f}  "
     f"(at {GROUND_SPEED_MS:.0f} m/s ground speed)")

print(f"\n  Latency alone at fixed ground speed, reference points from "
     f"CLAUDE.md (ground speed {GROUND_SPEED_MS:.0f} m/s):")
for latency_ms, expected_m in [(50, 0.15), (100, 0.30), (200, 0.60)]:
    displacement = GROUND_SPEED_MS * (latency_ms / 1000.0)
    check(f"latency {latency_ms} ms -> {displacement:.2f} m", displacement, expected_m, tol=0.005)
print(f"  at 100 ms this is {0.30 / errs_no_latency.mean():.1f}x the "
     f"current mean error budget without latency")

# ----------------------------------------------------------------------
print("\nF. Calibration routine on synthetic bench data (true f_u = 30.0)")
obs = []
for x in [-0.6, -0.4, -0.2, 0.2, 0.4, 0.6]:
    L = 2.0
    p = pb.C_U + 30.0 * (x / L) + rng.normal(0.0, 0.3)
    obs.append((p, x, L))
f_hat, rms = pb.calibrate_intrinsics(obs, axis="u")
print(f"  recovered f_u = {f_hat:.3f} px   residual {rms:.3f} px")

# ----------------------------------------------------------------------
n_fail = results.count(False)
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
if n_fail:
    sys.exit(1)
