"""Tests for attitude_interpolator.py, following the nine cases in
attitude_interpolator_spec.pdf.

The reference quaternions here are built independently of the module
under test, so a shared sign or ordering error cannot cancel itself out.
"""

import sys

import numpy as np

from attitude_interpolator import AttitudeBuffer, _slerp

results = []


def check(label, ok, detail=""):
    print(f"  [{'ok ' if ok else 'FAIL'}] {label}" + (f"   {detail}" if detail else ""))
    results.append(bool(ok))
    return ok


def q_yaw(deg):
    """Hamilton (w, x, y, z) for a rotation of deg about the down axis."""
    h = np.radians(deg) / 2.0
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)])


def q_axis(axis, deg):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    h = np.radians(deg) / 2.0
    return np.concatenate(([np.cos(h)], axis * np.sin(h)))


def angle_between_deg(qa, qb):
    """Rotation angle separating two quaternions, sign-insensitive."""
    d = abs(float(np.dot(np.asarray(qa), np.asarray(qb))))
    return np.degrees(2.0 * np.arccos(min(1.0, d)))


# ----------------------------------------------------------------------
print("T1. Constant yaw rate, 30 deg/s sampled at 10 Hz")
# The analytic answer is known at every instant, so this is the core
# correctness test: interpolation must reproduce it between samples.
buf = AttitudeBuffer(capacity=64)
for i in range(20):
    buf.add(i * 100, q_yaw(30.0 * i * 0.1))

worst = 0.0
for t_ms in range(0, 1900, 7):          # 7 ms step lands off every sample
    got = buf.at(t_ms)
    worst = max(worst, angle_between_deg(got, q_yaw(30.0 * t_ms / 1000.0)))
check("interpolated attitude matches the analytic rotation", worst < 0.01,
      f"worst {worst:.6f} deg over 272 queries")

# ----------------------------------------------------------------------
print("\nT2. Sign flip (quaternion double cover)")
# q and -q are the same orientation. Nothing guarantees the EKF keeps a
# consistent sign between messages. Without the dot-product sign fix, the
# interpolation takes the 356 deg path instead of the 4 deg one, and the
# midpoint of q and -q has a norm near zero, which amplifies floating
# point noise into an arbitrary orientation.
buf = AttitudeBuffer()
buf.add(0, q_yaw(0.0))
buf.add(100, -q_yaw(4.0))
mid = np.array(buf.at(50))
check("midpoint takes the short path", angle_between_deg(mid, q_yaw(2.0)) < 0.01,
      f"err {angle_between_deg(mid, q_yaw(2.0)):.6f} deg")

naive = (q_yaw(0.0) + (-q_yaw(4.0))) / 2.0
check("naive midpoint would have collapsed", np.linalg.norm(naive) < 0.05,
      f"norm {np.linalg.norm(naive):.4f} vs 1.0 for the correct result")

# ----------------------------------------------------------------------
print("\nT3. Exact hit on a stored timestamp")
buf = AttitudeBuffer()
buf.add(0, q_yaw(0.0))
buf.add(100, q_yaw(10.0))
check("returns the stored sample unchanged",
      angle_between_deg(buf.at(100), q_yaw(10.0)) < 1e-9)
check("also at the oldest sample",
      angle_between_deg(buf.at(0), q_yaw(0.0)) < 1e-9)

# ----------------------------------------------------------------------
print("\nT4. Boundaries: refuse rather than extrapolate")
buf = AttitudeBuffer()
buf.add(100, q_yaw(0.0))
buf.add(200, q_yaw(5.0))
before, after = buf.at(50), buf.at(300)
check("query before the oldest sample returns None", before is None)
check("query after the newest sample returns None", after is None)
check("counters incremented correctly",
      buf.rejected_too_old == 1 and buf.rejected_too_new == 1,
      f"too_old {buf.rejected_too_old}, too_new {buf.rejected_too_new}")

empty = AttitudeBuffer()
empty.add(0, q_yaw(0.0))
check("fewer than two samples returns None", empty.at(0) is None)

# ----------------------------------------------------------------------
print("\nT5. Gap refusal")
# Interpolating across a long gap fabricates data: the aircraft could
# have done anything between the two samples.
buf = AttitudeBuffer(max_gap_ms=200)
buf.add(0, q_yaw(0.0))
buf.add(500, q_yaw(20.0))
check("refuses to bridge a 500 ms gap with max_gap_ms=200",
      buf.at(250) is None and buf.rejected_gap == 1)

buf2 = AttitudeBuffer(max_gap_ms=200)
buf2.add(0, q_yaw(0.0))
buf2.add(150, q_yaw(5.0))
check("still serves a 150 ms gap", buf2.at(75) is not None)

# ----------------------------------------------------------------------
print("\nT6. Ring buffer wraparound")
# Where hand-written ring buffers usually break: old entries must be
# evicted in order, the newest must stay queryable, and queries into the
# evicted region must be refused rather than returning wrapped garbage.
buf = AttitudeBuffer(capacity=4, max_gap_ms=1000)
for i in range(10):
    buf.add(i * 100, q_yaw(3.0 * i))

check("newest sample still queryable after 10 adds into capacity 4",
      angle_between_deg(buf.at(900), q_yaw(27.0)) < 1e-6)
check("evicted sample is refused, not wrapped", buf.at(100) is None)
check("interpolation still correct inside the surviving window",
      angle_between_deg(buf.at(750), q_yaw(22.5)) < 0.01)
check("oldest surviving sample is the expected one",
      buf.at(600) is not None and buf.at(500) is None)

# ----------------------------------------------------------------------
print("\nT7. Unit norm, including from denormalised input")
buf = AttitudeBuffer()
buf.add(0, q_yaw(0.0) * 3.0)
buf.add(100, q_yaw(10.0) * 0.2)
q = np.array(buf.at(50))
check("output is unit length despite scaled inputs",
      abs(np.linalg.norm(q) - 1.0) < 1e-9,
      f"norm {np.linalg.norm(q):.12f}")

worst_norm = 0.0
rng = np.random.default_rng(4)
for _ in range(500):
    b = AttitudeBuffer()
    a0, a1 = rng.uniform(-90, 90, 2)
    b.add(0, q_axis(rng.normal(size=3), a0))
    b.add(100, q_axis(rng.normal(size=3), a1))
    r = np.array(b.at(rng.integers(1, 100)))
    worst_norm = max(worst_norm, abs(np.linalg.norm(r) - 1.0))
check("unit norm over 500 random attitude pairs", worst_norm < 1e-9,
      f"worst deviation {worst_norm:.2e}")

# ----------------------------------------------------------------------
print("\nT8. Out-of-order and duplicate timestamps")
# Must be rejected on insert. Stored and left alone, a duplicate produces
# a division by zero at query time.
buf = AttitudeBuffer()
buf.add(0, q_yaw(0.0))
buf.add(100, q_yaw(5.0))
check("duplicate timestamp rejected", buf.add(100, q_yaw(9.0)) is False)
check("backwards timestamp rejected", buf.add(50, q_yaw(9.0)) is False)
check("buffer contents unaffected by the rejected adds",
      angle_between_deg(buf.at(100), q_yaw(5.0)) < 1e-9)

# ----------------------------------------------------------------------
print("\nT9. SLERP against NLERP (measurement, not a pass/fail criterion)")


def nlerp(q0, q1, frac):
    if np.dot(q0, q1) < 0:
        q1 = -q1
    q = (1.0 - frac) * q0 + frac * q1
    return q / np.linalg.norm(q)


print("   rotation between samples   max angular diff   ground error at 5 m")
for rot in [3.75, 7.5, 15.0, 30.0, 90.0]:
    q0, q1 = q_yaw(0.0), q_yaw(rot)
    worst_d = max(angle_between_deg(_slerp(q0, q1, t), nlerp(q0, q1, t))
                  for t in np.linspace(0, 1, 201))
    print(f"   {rot:6.2f} deg                {worst_d:10.6f} deg   "
          f"{5.0 * np.radians(worst_d) * 100:8.4f} cm")
print("   3.75 deg is 30 deg/s across one 125 ms MLX90640 subpage interval.")
print("   SLERP is used for behaviour under dropped messages, not for")
print("   accuracy at nominal rates, where the difference is not measurable.")

# ----------------------------------------------------------------------
print("\nT10. Staleness cost, i.e. why this module exists")
print("   attitude rate   worst staleness   ground error at 5 m, 30 deg/s yaw")
for rate in [4, 10, 25, 50]:
    stale = 0.5 / rate
    print(f"   {rate:5d} Hz          {stale*1000:6.1f} ms          "
          f"{np.radians(30.0 * stale) * 5.0 * 100:6.1f} cm")
print("   Compare against the 20.8 cm total error budget from test_pillar_b.py.")

# ----------------------------------------------------------------------
n_fail = results.count(False)
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
if n_fail:
    sys.exit(1)
