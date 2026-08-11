"""Tests for logformat.py: the height-projection shortcut it uses, the
log round trip bit-identical acceptance check from CLAUDE.md, and
resilience to a truncated or corrupted file."""

import math
import os
import sys
import tempfile

import numpy as np

import pillar_b as pb
import synthetic_frames as sf
import centroid_detector as cd
import logformat as lf

results = []


def check(label, ok):
    print(f"  [{'ok ' if ok else 'FAIL'}] {label}")
    results.append(ok)
    return ok


def axis_quat(axis, angle_rad):
    """Hamilton (w, x, y, z) quaternion for a rotation of angle_rad about a
    unit axis. Built independently of pillar_b so it is not circular with
    the function under test."""
    half = angle_rad / 2.0
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    w = math.cos(half)
    xyz = axis * math.sin(half)
    return np.array([w, xyz[0], xyz[1], xyz[2]], dtype=np.float64)


# ----------------------------------------------------------------------
print("A. Height-from-quaternion shortcut vs the existing tested "
      "lidar_range_to_height(slant_range, roll, pitch)")
rng = np.random.default_rng(3)
max_height_err = 0.0
for _ in range(2000):
    roll, pitch, yaw = rng.uniform(-0.5, 0.5, 3)   # +-28.6 deg, comfortably inside LIDAR_MAX_TILT
    R_nb = pb.euler_to_rotation(roll, pitch, yaw)
    slant = rng.uniform(0.3, 11.0)
    h_direct = pb.lidar_range_to_height(slant, roll, pitch)
    h_shortcut = slant * R_nb[2, 2]
    max_height_err = max(max_height_err, abs(h_direct - h_shortcut))
check(f"R_nb[2,2] projection matches lidar_range_to_height "
     f"(max abs diff over 2000 random attitudes = {max_height_err:.2e} m)",
     max_height_err < 1e-9)

# ----------------------------------------------------------------------
print("\nB. Log round trip is bit-identical to running the pipeline directly")

tmpdir = tempfile.mkdtemp(prefix="pillarb_logformat_test_")
log_path = os.path.join(tmpdir, "roundtrip.pblog")

H = 5.0
ZERO_LEVER = np.zeros(3)
R_bc = pb.mounting_matrix(12.5)
rng = np.random.default_rng(11)

test_cases = [
    (axis_quat([0, 1, 0], math.radians(12.5)), pb.C_U, pb.C_V, H),
    (axis_quat([1, 0, 0], math.radians(-6.0)), 4.0, 6.0, H),
    (axis_quat([0, 0, 1], math.radians(30.0)), 27.0, 18.0, H + 1.3),
]
# a compound rotation (roll, then pitch, then yaw) via quaternion multiplication
def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])
compound = qmul(qmul(axis_quat([0, 0, 1], math.radians(15.0)),
                     axis_quat([0, 1, 0], math.radians(-8.0))),
                axis_quat([1, 0, 0], math.radians(3.0)))
test_cases.append((compound, 10.0, 20.0, H))

writer = lf.LogWriter(log_path)
direct_results = []
for i, (quat, u_true, v_true, height) in enumerate(test_cases):
    frame = sf.create_synthetic_frame(u_true, v_true, rng=rng)
    frame32 = frame.astype(np.float32)   # the precision the log actually stores

    # The log stores the quaternion and the slant range as float32. "Running
    # the pipeline directly" has to start from that same rounded precision,
    # not the original float64 values, or a bit-identical match is
    # impossible by construction rather than by anything the log does wrong.
    quat32 = quat.astype(np.float32)
    slant_range = np.float32(5.5)

    R_nb = pb.quaternion_to_rotation(quat32)
    height_direct = slant_range * R_nb[2, 2]

    detections = cd.detect_centroids(frame32)
    u_est, v_est = detections[0]['u'], detections[0]['v']
    north, east, off_nadir = pb.pixel_to_ground_ned(
        u_est, v_est, height_direct, R_nb, ZERO_LEVER, R_bc)
    direct_results.append((frame32, R_nb, height_direct, u_est, v_est, north, east))

    writer.write_frame(
        time_boot_ms=1000 * i, subpage=i % 2, pixels=frame,
        lidar_distance_m=slant_range, lidar_strength=8000 + i,
        quat=quat32, lat_deg=69.9689, lon_deg=23.2717, alt_m=42.0,
    )
writer.close()

read_back = list(lf.read_frames(log_path))
check(f"read back {len(read_back)} of {len(test_cases)} written frames",
     len(read_back) == len(test_cases))

all_bit_identical = True
for i, (rec, direct) in enumerate(zip(read_back, direct_results)):
    frame32_direct, R_nb_direct, height_direct, u_direct, v_direct, north_direct, east_direct = direct
    frame_logged, height_logged, R_nb_logged = lf.frame_record_to_transform_inputs(rec)

    frame_ok = np.array_equal(frame_logged, frame32_direct)
    height_ok = height_logged == height_direct
    R_ok = np.array_equal(R_nb_logged, R_nb_direct)

    detections = cd.detect_centroids(frame_logged)
    u_est, v_est = detections[0]['u'], detections[0]['v']
    north, east, off_nadir = pb.pixel_to_ground_ned(
        u_est, v_est, height_logged, R_nb_logged, ZERO_LEVER, R_bc)
    uv_ok = (u_est == u_direct) and (v_est == v_direct)
    pos_ok = (north == north_direct) and (east == east_direct)

    ok = frame_ok and height_ok and R_ok and uv_ok and pos_ok
    all_bit_identical &= ok
    if not ok:
        print(f"    frame {i}: frame_ok={frame_ok} height_ok={height_ok} "
             f"R_ok={R_ok} uv_ok={uv_ok} pos_ok={pos_ok}")

check("all frames bit-identical: logged pipeline output == direct pipeline output",
     all_bit_identical)

# ----------------------------------------------------------------------
print("\nC. Resilience: truncated (interrupted session) and corrupted files")

trunc_path = os.path.join(tmpdir, "truncated.pblog")
writer = lf.LogWriter(trunc_path)
for i in range(5):
    frame = sf.create_synthetic_frame(pb.C_U, pb.C_V, rng=rng)
    writer.write_frame(i, 0, frame, 5.0, 8000, np.array([1.0, 0, 0, 0], dtype=np.float32),
                       69.9, 23.2, 10.0)
writer.close()

with open(trunc_path, "rb") as f:
    full_bytes = f.read()
# cut off partway through the last record, simulating power loss mid-write
cut_bytes = full_bytes[:-(lf.RECORD_SIZE // 2)]
with open(trunc_path, "wb") as f:
    f.write(cut_bytes)

try:
    recovered = list(lf.read_frames(trunc_path))
    crashed = False
except Exception as exc:
    recovered = []
    crashed = True
check("truncated file does not raise", not crashed)
check(f"truncated file recovers the 4 complete records written before the cut "
     f"(got {len(recovered)})", len(recovered) == 4)

corrupt_path = os.path.join(tmpdir, "corrupt.pblog")
writer = lf.LogWriter(corrupt_path)
for i in range(5):
    frame = sf.create_synthetic_frame(pb.C_U, pb.C_V, rng=rng)
    writer.write_frame(i, 0, frame, 5.0, 8000, np.array([1.0, 0, 0, 0], dtype=np.float32),
                       69.9, 23.2, 10.0)
writer.close()

with open(corrupt_path, "rb") as f:
    raw = bytearray(f.read())
# flip a byte in the middle of record index 2's payload, leaving its magic intact
third_record_start = len(lf.FILE_MAGIC) + 2 * lf.RECORD_SIZE
raw[third_record_start + 20] ^= 0xFF
with open(corrupt_path, "wb") as f:
    f.write(raw)

try:
    recovered2 = list(lf.read_frames(corrupt_path))
    crashed2 = False
except Exception as exc:
    recovered2 = []
    crashed2 = True
check("corrupted middle record does not raise", not crashed2)
check(f"corrupted middle record is skipped, the other 4 still read back "
     f"(got {len(recovered2)})", len(recovered2) == 4)

# ----------------------------------------------------------------------
n_fail = results.count(False)
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
if n_fail:
    sys.exit(1)
