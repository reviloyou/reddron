"""Hand-checkable tests for the Pillar B transform, then a noise study."""

import numpy as np
import pillar_b as pb

ZERO_LEVER = np.zeros(3)
H = 5.0
LEVEL = pb.euler_to_rotation(0.0, 0.0, 0.0)


def check(label, got, expected, tol=0.01):
    ok = abs(got - expected) <= tol
    print(f"  [{'ok ' if ok else 'FAIL'}] {label:<46} "
          f"got {got:+8.4f}  expected {expected:+8.4f}")
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

for row, name in [(0, "front row v=0"), (23, "rear row v=23")]:
    n, e, g = pb.pixel_to_ground_ned(pb.C_U, row, H, LEVEL, ZERO_LEVER, tilted)
    print(f"  [   ] {name:<46} north {n:+7.3f} m   off-nadir {g:5.2f} deg")

# ----------------------------------------------------------------------
print("\nC. Yaw check: rotate 90 deg, a forward target must move east")
yawed = pb.euler_to_rotation(0.0, 0.0, np.radians(90.0))
n, e, g = pb.pixel_to_ground_ned(pb.C_U, pb.C_V, H, yawed, ZERO_LEVER, tilted)
check("6. yaw 90 -> north component", n, 0.0)
check("6. yaw 90 -> east component", e, H * np.tan(np.radians(12.5)))

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

n_true, e_true, _ = pb.pixel_to_ground_ned(
    pb.C_U, pb.C_V, H, LEVEL, ZERO_LEVER, tilted)

errs = []
for _ in range(20000):
    R = pb.euler_to_rotation(*rng.normal(0.0, SIGMA_ATT, 3))
    h = H + rng.normal(0.0, SIGMA_H)
    u = pb.C_U + rng.normal(0.0, SIGMA_PX)
    v = pb.C_V + rng.normal(0.0, SIGMA_PX)
    n, e, _ = pb.pixel_to_ground_ned(u, v, h, R, ZERO_LEVER, tilted)
    errs.append(np.hypot(n - n_true, e - e_true))

errs = np.array(errs)
print(f"  mean   {errs.mean():.3f} m")
print(f"  median {np.median(errs):.3f} m")
print(f"  p95    {np.percentile(errs, 95):.3f} m")

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

# ----------------------------------------------------------------------
print("\nF. Calibration routine on synthetic bench data (true f_u = 30.0)")
obs = []
for x in [-0.6, -0.4, -0.2, 0.2, 0.4, 0.6]:
    L = 2.0
    p = pb.C_U + 30.0 * (x / L) + rng.normal(0.0, 0.3)
    obs.append((p, x, L))
f_hat, rms = pb.calibrate_intrinsics(obs, axis="u")
print(f"  recovered f_u = {f_hat:.3f} px   residual {rms:.3f} px")
