"""Tests for orchestrator.py.

Driven by synthetic sensor streams at the rates the real system runs at:
attitude at 50 Hz, LiDAR at 100 Hz, thermal subpages at 8 Hz (so each
subpage 125 ms after the last, complete frames at 4 Hz).

The interesting cases are the degraded ones. A chain that works when
every message arrives on time proves very little, because the whole
point of the refusal paths is what happens when they do not.

Pair-and-average: on_subpage now holds the first half of each frame and
returns an empty list. When the matching half arrives, the two ground
positions are averaged and returned together. Tests that check the
return value of on_subpage must therefore check at the sp1 call, not
the sp0 call. Tests that check per-subpage counts (frames_processed,
rejected_*) are unaffected.
"""

import math
import os
import sys
import tempfile

import numpy as np

import pillar_b as pb
import centroid_detector as cd
import clock_sync as cs
import synthetic_frames as sf
import logformat as lf
from attitude_interpolator import AttitudeBuffer
from orchestrator import Orchestrator, LIDAR_MAX_AGE_MS, PAIR_TIMEOUT_MS

results = []


def check(label, ok, detail=""):
    print(f"  [{'ok ' if ok else 'FAIL'}] {label}" + (f"   {detail}" if detail else ""))
    results.append(bool(ok))
    return ok


def q_yaw(deg):
    """Hamilton (w, x, y, z) for a rotation about the down axis."""
    h = math.radians(deg) / 2.0
    return np.array([math.cos(h), 0.0, 0.0, math.sin(h)])


LEVEL_Q = q_yaw(0.0)
H = 5.0
ZERO_LEVER = np.zeros(3)

# A subpage is timestamped when it is captured, but it cannot be handed
# to the orchestrator until it has been clocked out over I2C. At 400 kHz
# that readout is a few tens of milliseconds.
#
# This delay is not an inconvenience: it is what makes the chain work at
# all. The interpolator refuses to extrapolate, so a subpage can only be
# served once an attitude sample at or after its capture time exists. If
# subpages were dispatched at the instant of capture, roughly half of
# them would arrive before the next attitude message and be refused as
# too_new through no fault of the link.
SUBPAGE_READOUT_MS = 25


def run_session(orch, subpage_times, frame_for, duration_ms,
                yaw_rate_deg_s=0.0, attitude_hz=50, lidar_hz=100,
                attitude_stops_ms=None, lidar_stops_ms=None,
                distance_m=H, position_at_ms=0,
                lat=69.9689, lon=23.2717):
    """Drive the orchestrator with three independent streams, delivered
    in true timestamp order.

    Interleaving matters and is not a detail: the attitude ring buffer
    holds a bounded window of history, so feeding a whole flight of
    attitude before the first subpage would evict everything the early
    subpages need and report it, correctly, as too_old. Real streams
    arrive mixed together, and the test has to as well.

    attitude_stops_ms / lidar_stops_ms simulate a stream dying partway
    through and never resuming.
    """
    events = []
    if position_at_ms is not None:
        events.append((position_at_ms, 0, "pos", None))

    att_step = 1000 // attitude_hz
    for t in range(0, duration_ms + 1, att_step):
        if attitude_stops_ms is not None and t > attitude_stops_ms:
            break
        events.append((t, 1, "att", t))

    lid_step = 1000 // lidar_hz
    for t in range(0, duration_ms + 1, lid_step):
        if lidar_stops_ms is not None and t > lidar_stops_ms:
            break
        events.append((t, 2, "lid", t))

    for i, t in enumerate(subpage_times):
        events.append((t + SUBPAGE_READOUT_MS, 3, "sub", (t, i)))

    # Sort by dispatch time, then by kind so that sensor data arriving in
    # the same millisecond as a query is stored before the query runs.
    events.sort(key=lambda e: (e[0], e[1]))

    out = []
    for _, _, kind, payload in events:
        if kind == "pos":
            orch.on_position(position_at_ms, lat, lon, 42.0)
        elif kind == "att":
            orch.on_attitude(payload, q_yaw(yaw_rate_deg_s * payload / 1000.0))
        elif kind == "lid":
            orch.on_lidar(payload, distance_m, 8000)
        else:
            t_capture, i = payload
            out.append((t_capture, orch.on_subpage(t_capture, i % 2, frame_for(i))))
    return out


# ----------------------------------------------------------------------
print("A. Nominal run: 50 Hz attitude, 100 Hz LiDAR, 8 Hz subpages")

rng = np.random.default_rng(7)
frame = sf.create_synthetic_frame(pb.C_U, pb.C_V, rng=rng)

orch = Orchestrator(lever_arm_b=ZERO_LEVER)
subpage_times = list(range(500, 1500, 125))
n_subpages = len(subpage_times)  # 8 subpages = 4 pairs
session = run_session(orch, subpage_times, lambda i: frame, duration_ms=2000)
# Paired results come from sp1 calls; sp0 calls return [].
paired_sessions = [res for _, res in session if res]
last_results = paired_sessions[-1]

check("every frame pair produced exactly one detection",
      orch.detections_reported == n_subpages // 2,
      f"{orch.detections_reported} detections over {n_subpages // 2} pairs")
check("no frame was rejected",
      orch.rejected_no_attitude == 0 and orch.rejected_no_lidar == 0
      and orch.rejected_lidar_stale == 0 and orch.rejected_lidar_unusable == 0)
check("all subpages counted as processed",
      orch.frames_processed == n_subpages, f"{orch.frames_processed}")
check("complete frame pairs equal half the subpage count",
      orch.frames_paired == n_subpages // 2, f"{orch.frames_paired}")

# At level attitude with no yaw, sp0 and sp1 see the same centroid, so
# the paired average equals each individual result. The wiring check is
# still valid: paired north/east must match a direct transform call.
r = last_results[0]
R_nb = pb.quaternion_to_rotation(LEVEL_Q)
n_direct, e_direct, _ = pb.pixel_to_ground_ned(
    r['u'], r['v'], H, R_nb, ZERO_LEVER, pb.mounting_matrix())
check("ground offset matches a direct transform call",
      abs(r['north_m'] - n_direct) < 1e-6 and abs(r['east_m'] - e_direct) < 1e-6,
      f"north {r['north_m']:.6f} vs {n_direct:.6f}")
check("height came from the LiDAR slant range projected onto the vertical",
      abs(r['height_m'] - H) < 1e-9, f"{r['height_m']:.6f} m")
# Loose by design: measured centroid, not the exact centre pixel.
check("off-nadir angle is the 12.5 deg boresight at frame centre",
      abs(r['off_nadir_deg'] - 12.5) < 0.1, f"{r['off_nadir_deg']:.3f} deg")

lat_check, lon_check = pb.ned_to_latlon(69.9689, 23.2717, r['north_m'], r['east_m'])
check("lat/lon converted from the stored position fix",
      abs(r['lat_deg'] - lat_check) < 1e-12 and abs(r['lon_deg'] - lon_check) < 1e-12)
check("centre-frame detection is not edge flagged", r['touches_edge'] is False)

# ----------------------------------------------------------------------
print("\nB. Each subpage gets its own attitude, and the pair averages them")
# Under yaw, two subpages 125 ms apart are captured at genuinely
# different orientations. The paired result is the average of the two
# per-subpage ground positions, not either one alone. This test verifies
# that the per-subpage attitude lookup is real (the two results differ)
# and that the averaging is correct (paired lies between them).

YAW_RATE = 30.0
orch = Orchestrator(lever_arm_b=ZERO_LEVER)
frame_side = sf.create_synthetic_frame(4.0, pb.C_V, rng=rng)
session = run_session(orch, [1000, 1125], lambda i: frame_side,
                      duration_ms=2000, yaw_rate_deg_s=YAW_RATE)
res_sp0 = session[0][1]   # sp0 holds as pending, returns []
res_sp1 = session[1][1]   # sp1 pairs with sp0, returns averaged result

check("sp0 is held (returns empty), sp1 completes the pair",
      res_sp0 == [] and len(res_sp1) == 1)

# Compute what sp0 and sp1 would give individually, analytically.
det = cd.detect_centroids(frame_side)[0]
R_sp0 = pb.quaternion_to_rotation(q_yaw(YAW_RATE * 1.000))
R_sp1 = pb.quaternion_to_rotation(q_yaw(YAW_RATE * 1.125))
n_sp0, e_sp0, _ = pb.pixel_to_ground_ned(
    det['u'], det['v'], H, R_sp0, ZERO_LEVER, pb.mounting_matrix())
n_sp1, e_sp1, _ = pb.pixel_to_ground_ned(
    det['u'], det['v'], H, R_sp1, ZERO_LEVER, pb.mounting_matrix())

moved = math.hypot(n_sp1 - n_sp0, e_sp1 - e_sp0)
arm = math.hypot(n_sp0, e_sp0)
expected_chord = 2.0 * arm * math.sin(math.radians(YAW_RATE * 0.125) / 2.0)
check("per-subpage positions differ by one subpage interval of yaw",
      abs(moved - expected_chord) < 0.01,
      f"moved {moved:.3f} m, expected {expected_chord:.3f} m")
check("that disagreement is large enough to matter", moved > 0.10,
      f"{moved * 100:.1f} cm, against a 20.8 cm total error budget")

n_avg_expected = 0.5 * (n_sp0 + n_sp1)
e_avg_expected = 0.5 * (e_sp0 + e_sp1)
r = res_sp1[0]
check("paired result is the average of the two per-subpage ground positions",
      abs(r['north_m'] - n_avg_expected) < 0.05
      and abs(r['east_m'] - e_avg_expected) < 0.05,
      f"N {r['north_m']:.3f} vs expected avg {n_avg_expected:.3f}")

# ----------------------------------------------------------------------
print("\nC. Attitude dropout mid-sequence")
# Stream dies at 1000 ms. Subpages keep arriving. Every subpage past
# 1000 ms is refused because it queries past the newest stored sample.
# The sp0 that WAS stored just before the dropout is orphaned when the
# matching sp1 is refused and the next sp0 evicts it.

orch = Orchestrator(lever_arm_b=ZERO_LEVER)
run_session(orch, list(range(500, 2000, 125)), lambda i: frame,
            duration_ms=2000, attitude_stops_ms=1000)

check("pairs before the dropout completed successfully",
      orch.frames_paired >= 2, f"{orch.frames_paired} pairs")
check("subpages after the dropout were rejected for missing attitude",
      orch.rejected_no_attitude >= 6, f"{orch.rejected_no_attitude}")
check("the interpolator recorded them as extrapolation attempts",
      orch.attitude.rejected_too_new >= 6,
      f"rejected_too_new {orch.attitude.rejected_too_new}")
check("the pending half from just before the dropout was eventually orphaned",
      orch.halves_timed_out >= 1, f"halves_timed_out {orch.halves_timed_out}")

# A gap too large to bridge is a different refusal from running past the
# end of the buffer, and gets its own counter.
orch_gap = Orchestrator(lever_arm_b=ZERO_LEVER, attitude_max_gap_ms=200)
orch_gap.on_lidar(0, H, 8000)
orch_gap.on_attitude(0, LEVEL_Q)
orch_gap.on_attitude(500, q_yaw(15.0))
out = orch_gap.on_subpage(250, 0, frame)
check("a 500 ms attitude gap is refused rather than bridged",
      out == [] and orch_gap.attitude.rejected_gap == 1
      and orch_gap.rejected_no_attitude == 1)

# ----------------------------------------------------------------------
print("\nD. LiDAR failures, each counted separately")

# D1: no LiDAR reading has ever arrived.
orch = Orchestrator(lever_arm_b=ZERO_LEVER)
orch.on_attitude(0, LEVEL_Q)
orch.on_attitude(100, LEVEL_Q)
orch.on_subpage(50, 0, frame)
check("no LiDAR at all is refused and counted",
      orch.rejected_no_lidar == 1 and orch.frames_processed == 0)

# D2: LiDAR stops, so the newest reading ages past the threshold.
orch = Orchestrator(lever_arm_b=ZERO_LEVER)
run_session(orch, list(range(500, 2000, 125)), lambda i: frame,
            duration_ms=2000, lidar_stops_ms=1000)
check("pairs while the rangefinder was alive worked",
      orch.frames_paired >= 2, f"{orch.frames_paired}")
check("subpages after it went quiet were refused",
      orch.rejected_lidar_stale >= 6,
      f"rejected_lidar_stale {orch.rejected_lidar_stale}")

# The boundary: a reading exactly at the threshold is still usable, one
# millisecond older is not. Use sp0 then sp1 with different LiDAR ages
# so we can test both without the sp0 pending half blocking sp1.
orch = Orchestrator(lever_arm_b=ZERO_LEVER)
for t in range(0, 400, 20):   # dense attitude, so only the LiDAR is under test
    orch.on_attitude(t, LEVEL_Q)
orch.on_lidar(100, H, 8000)
orch.on_subpage(100 + LIDAR_MAX_AGE_MS, 0, frame)    # sp0 at exactly the limit
check(f"a reading exactly {LIDAR_MAX_AGE_MS} ms old is still accepted",
      orch.frames_processed == 1 and orch.rejected_lidar_stale == 0)
orch.on_subpage(100 + LIDAR_MAX_AGE_MS + 1, 1, frame)  # sp1 one ms older
check("one millisecond older is refused",
      orch.rejected_lidar_stale == 1)

# D3: reading present and fresh, but outside the rangefinder's honest band.
orch = Orchestrator(lever_arm_b=ZERO_LEVER)
orch.on_attitude(0, LEVEL_Q)
orch.on_attitude(100, LEVEL_Q)
orch.on_lidar(50, 25.0, 8000)          # past the 11 m ceiling
orch.on_subpage(50, 0, frame)
check("a 25 m range is refused as unusable, not silently trusted",
      orch.rejected_lidar_unusable == 1 and orch.frames_processed == 0)

# D4: fresh, in band, but the aircraft is tilted too far for the beam to
# be measuring anything useful beneath it.
orch = Orchestrator(lever_arm_b=ZERO_LEVER)
tilted = pb.euler_to_rotation(0.0, math.radians(40.0), 0.0)
w = math.sqrt(1.0 + tilted[0, 0] + tilted[1, 1] + tilted[2, 2]) / 2.0
q_tilt = np.array([w,
                   (tilted[2, 1] - tilted[1, 2]) / (4 * w),
                   (tilted[0, 2] - tilted[2, 0]) / (4 * w),
                   (tilted[1, 0] - tilted[0, 1]) / (4 * w)])
orch.on_attitude(0, q_tilt)
orch.on_attitude(100, q_tilt)
orch.on_lidar(50, H, 8000)
orch.on_subpage(50, 0, frame)
check("40 deg of tilt puts the rangefinder outside its valid cone",
      orch.rejected_lidar_unusable == 1 and orch.frames_processed == 0)

# ----------------------------------------------------------------------
print("\nE. Position handling")

orch = Orchestrator(lever_arm_b=ZERO_LEVER)
orch.on_attitude(0, LEVEL_Q)
orch.on_attitude(200, LEVEL_Q)
orch.on_attitude(300, LEVEL_Q)
orch.on_lidar(100, H, 8000)
# sp0 before any GPS fix.
orch.on_subpage(100, 0, frame)
check("a subpage before any GPS fix is still processed, not dropped",
      orch.frames_processed == 1 and orch.rejected_no_attitude == 0)
# Complete the pair so we can inspect the result.
# LiDAR age at sp1@200: 200 - 100 = 100 ms = exactly at limit.
out = orch.on_subpage(200, 1, frame)
check("its ground offset in metres is still valid",
      len(out) == 1
      and abs(out[0]['north_m'] - H * math.tan(math.radians(12.5))) < 0.05,
      f"north {out[0]['north_m']:.3f} m")
check("its lat/lon are NaN rather than invented",
      math.isnan(out[0]['lat_deg']) and math.isnan(out[0]['lon_deg']))
check("and the frame pair is flagged as having had no fix",
      orch.frames_without_position == 1)

# ----------------------------------------------------------------------
print("\nF. Empty frames, and out-of-order messages")

orch = Orchestrator(lever_arm_b=ZERO_LEVER)
orch.on_attitude(0, LEVEL_Q)
orch.on_attitude(200, LEVEL_Q)
orch.on_lidar(100, H, 8000)
noise = 15.0 + np.random.default_rng(3).normal(0.0, 0.5, size=(24, 32))
out = orch.on_subpage(100, 0, noise)
check("a noise-only frame yields no detection and is held as pending",
      out == [] and orch.frames_no_detection == 1)
check("but the subpage still counts as processed, not rejected",
      orch.frames_processed == 1 and orch.rejected_no_attitude == 0)

orch = Orchestrator(lever_arm_b=ZERO_LEVER)
orch.on_attitude(100, LEVEL_Q)
check("a backwards attitude timestamp is rejected and counted",
      orch.on_attitude(50, LEVEL_Q) is False
      and orch.attitude_rejected_out_of_order == 1)
orch.on_lidar(100, H, 8000)
check("a duplicate LiDAR timestamp is rejected and counted",
      orch.on_lidar(100, H, 8000) is False
      and orch.lidar_rejected_out_of_order == 1)

# ----------------------------------------------------------------------
print("\nG. Logging: one record per complete frame pair")
# The binary record is 3132 bytes. At 8 subpages per second and one
# record per subpage, that is 25.1 kB/s and 2.26 MB over a 90 s flight,
# against roughly 1.5 MB of LittleFS on a 4 MB ESP32. One record per
# complete frame halves it to 12.5 kB/s and 1.13 MB.
#
# The record stores the sp0 timestamp and the sp0 attitude quaternion
# (the start of the physical frame). The sp1 quaternion is not stored;
# the binary layout is frozen.

tmpdir = tempfile.mkdtemp(prefix="pillarb_orch_test_")
log_path = os.path.join(tmpdir, "session.pblog")

orch = Orchestrator(log_path=log_path, lever_arm_b=ZERO_LEVER)
log_subpages = list(range(500, 1000, 125))   # [500, 625, 750, 875]
n_log_sp = len(log_subpages)                 # 4 subpages, 2 pairs
run_session(orch, log_subpages, lambda i: frame, duration_ms=1200)
orch.close()

records = list(lf.read_frames(log_path))
check("one record per complete frame pair",
      len(records) == n_log_sp // 2,
      f"{len(records)} records, {n_log_sp // 2} expected from {n_log_sp} subpages")
check("each record is for a complete frame (subpage field is 0)",
      all(r.subpage == 0 for r in records),
      f"subpages {[r.subpage for r in records]}")
check("timestamps are the sp0 capture times (start of each physical frame)",
      [r.time_boot_ms for r in records] == log_subpages[::2],
      f"{[r.time_boot_ms for r in records]} vs {log_subpages[::2]}")
check("pixel data survives the round trip",
      np.array_equal(records[0].frame, frame.astype(np.float32)))
check("frames_logged matches what came back",
      orch.frames_logged == len(records))

# The same session through the SD-style buffered writer (F7). Only two
# records against a batch of four, so this also proves close() drains a
# partial batch rather than leaving it in RAM.
log_path_sd = os.path.join(tmpdir, "session_sd.pblog")
orch_sd = Orchestrator(log_path=log_path_sd, lever_arm_b=ZERO_LEVER,
                       log_kwargs={'batch_records': lf.SD_BATCH_RECORDS})
run_session(orch_sd, log_subpages, lambda i: frame, duration_ms=1200)
orch_sd.close()
with open(log_path, 'rb') as f_ref, open(log_path_sd, 'rb') as f_sd:
    same_bytes = f_ref.read() == f_sd.read()
check("buffered SD-style logging through the orchestrator writes identical bytes",
      same_bytes)
c_sd = orch_sd.counters()
log_keys = {k for k in c_sd if k.startswith('log_')}

# Attitude dies after 2 s, so paired frames stop reaching the log with a
# part-filled SD batch in the queue. The age bound must still write it
# while the camera keeps producing subpages, not leave it in RAM.
log_path_age = os.path.join(tmpdir, "session_age.pblog")
orch_age = Orchestrator(log_path=log_path_age, lever_arm_b=ZERO_LEVER,
                        log_kwargs={'batch_records': lf.SD_BATCH_RECORDS})
run_session(orch_age, list(range(500, 6000, 125)), lambda i: frame,
            duration_ms=6000, attitude_stops_ms=2000)
c_age = orch_age.counters()
check("records stranded in a part-filled batch are written once they are a second old",
      c_age['log_records_pending'] == 0
      and c_age['log_records_written'] == c_age['frames_logged'] > 0
      and c_age['frames_logged'] % lf.SD_BATCH_RECORDS != 0,
      f"logged {c_age['frames_logged']}, written {c_age['log_records_written']}, "
      f"pending {c_age['log_records_pending']}")
orch_age.close()

check("log counters surface in counters(), survive close(), same keys without a log",
      c_sd['log_records_written'] == len(records) and c_sd['log_records_pending'] == 0
      and c_sd['log_write_failures'] == 0 and c_sd['log_card_mounted'] is None
      and log_keys == {k for k in Orchestrator().counters() if k.startswith('log_')},
      f"written {c_sd['log_records_written']}, batches {c_sd['log_batches_written']}")

# ----------------------------------------------------------------------
print("\nH. Counter bookkeeping over a long degraded run")
# 60 s at realistic rates with attitude dying halfway, which is the shape
# of a real link failure. Every subpage must land in exactly one bucket.

orch = Orchestrator(lever_arm_b=ZERO_LEVER)
run_session(orch, list(range(0, 60000, 125)), lambda i: frame,
            duration_ms=60000, attitude_stops_ms=30000)

c = orch.counters()
accounted = (c['frames_processed'] + c['rejected_no_attitude']
             + c['rejected_no_lidar'] + c['rejected_lidar_stale']
             + c['rejected_lidar_unusable'])
check("every subpage is accounted for in exactly one bucket",
      accounted == c['subpages_received'],
      f"{accounted} of {c['subpages_received']}")
check("the run really did degrade partway through",
      c['rejected_no_attitude'] > 100 and c['frames_processed'] > 100,
      f"processed {c['frames_processed']}, dropped {c['rejected_no_attitude']}")
check("interpolator served exactly the subpages that were processed",
      c['attitude_queries_served'] == c['frames_processed'],
      f"served {c['attitude_queries_served']}, processed {c['frames_processed']}")

print("\n  Counters at the end of the degraded run:")
for k, v in c.items():
    print(f"    {k:<38} {v}")

# ----------------------------------------------------------------------
print("\nI. Pair-and-average is measurably flatter than individual subpage results")
# Under constant yaw, each subpage of a physical frame is captured 125 ms
# apart. Emitting once per subpage (old behaviour) produced results at
# 125 ms intervals spanning the full yaw range of the run. Emitting once
# per pair spans the same wall-clock time but at 250 ms intervals, and
# each result represents the mid-yaw of the two contributing subpages
# rather than either extreme. Over a fixed time window, the paired
# sequence covers less total yaw than the individual subpage sequence,
# so its spread is smaller.

orch_I = Orchestrator(lever_arm_b=ZERO_LEVER)
sp_times_I = list(range(0, 1000, 125))   # 8 subpages, 4 pairs
session_I = run_session(orch_I, sp_times_I, lambda i: frame_side,
                        duration_ms=1500, yaw_rate_deg_s=YAW_RATE)

paired_north_I = [res[0]['north_m'] for _, res in session_I if res]
check("four complete pairs were emitted", len(paired_north_I) == 4,
      f"{len(paired_north_I)} pairs")

# What each subpage would give individually (using its own correct attitude):
det_side = cd.detect_centroids(frame_side)[0]
indiv_north_I = []
for t_sp in sp_times_I:
    R = pb.quaternion_to_rotation(q_yaw(YAW_RATE * t_sp / 1000.0))
    n, _, _ = pb.pixel_to_ground_ned(
        det_side['u'], det_side['v'], H, R, ZERO_LEVER, pb.mounting_matrix())
    indiv_north_I.append(n)

spread_indiv = max(indiv_north_I) - min(indiv_north_I)
spread_paired = max(paired_north_I) - min(paired_north_I)
check("paired-average spread is smaller than individual-subpage spread",
      spread_paired < spread_indiv,
      f"paired {spread_paired:.4f} m vs individual {spread_indiv:.4f} m")

# First pair's north should match the analytic average of sp0 and sp1.
n0_I, _, _ = pb.pixel_to_ground_ned(
    det_side['u'], det_side['v'], H,
    pb.quaternion_to_rotation(q_yaw(YAW_RATE * 0.000)), ZERO_LEVER,
    pb.mounting_matrix())
n1_I, _, _ = pb.pixel_to_ground_ned(
    det_side['u'], det_side['v'], H,
    pb.quaternion_to_rotation(q_yaw(YAW_RATE * 0.125)), ZERO_LEVER,
    pb.mounting_matrix())
check("first pair's north matches the analytic average of its two subpages",
      abs(paired_north_I[0] - 0.5 * (n0_I + n1_I)) < 0.05,
      f"got {paired_north_I[0]:.4f}, expected {0.5*(n0_I+n1_I):.4f}")

# ----------------------------------------------------------------------
print("\nJ. Pending half times out when the matching subpage never arrives")
# sp0 is stored as pending. If no matching sp1 arrives within
# PAIR_TIMEOUT_MS, the pending half is emitted as unpaired at the next
# on_subpage call. Discarding it would risk losing a real SAR contact.

orch_J = Orchestrator(lever_arm_b=ZERO_LEVER)
for t in range(0, 2000, 20):
    orch_J.on_attitude(t, LEVEL_Q)
orch_J.on_lidar(0, H, 8000)
orch_J.on_position(0, 69.9689, 23.2717, 42.0)

# sp0@1: LiDAR age = 1 ms, accepted. Stored as pending.
orch_J.on_subpage(1, 0, frame)
check("pending half is held (not yet emitted)",
      orch_J.frames_paired == 0 and orch_J.frames_processed == 1)

# sp0@252: age = 252 - 1 = 251 ms > PAIR_TIMEOUT_MS (250). Evict.
# LiDAR age at 252 ms: 252 - 0 = 252 > LIDAR_MAX_AGE_MS. Rejected.
# So: evicted detection from sp0@1 is returned; sp0@252 itself rejected.
out_J = orch_J.on_subpage(252, 0, frame)
check("timed-out pending half is returned as unpaired at the next call",
      orch_J.halves_timed_out == 1 and len(out_J) >= 1,
      f"timed_out {orch_J.halves_timed_out}, out len {len(out_J)}")
check("orphaned detection is flagged paired=False and counted as unmatched",
      out_J[0]['paired'] is False and orch_J.detections_unmatched_sp0 == 1
      and orch_J.detections_matched == 0)

# ----------------------------------------------------------------------
print("\nK. Same subpage index twice evicts the pending half immediately")
# If sp0 arrives twice in a row, the first sp0 was captured in a frame
# whose sp1 was lost. The first sp0 is orphaned at the arrival of the
# second sp0, even if the timeout has not yet elapsed.

orch_K = Orchestrator(lever_arm_b=ZERO_LEVER)
for t in range(0, 500, 20):
    orch_K.on_attitude(t, LEVEL_Q)
orch_K.on_lidar(0, H, 8000)
orch_K.on_position(0, 69.9689, 23.2717, 42.0)

orch_K.on_subpage(1, 0, frame)                   # sp0 -> pending
out_K = orch_K.on_subpage(50, 0, frame)          # sp0 again -> evict first sp0
check("second sp0 arrival evicts the first (counted as timed-out)",
      orch_K.halves_timed_out == 1 and orch_K.frames_processed == 2,
      f"timed_out {orch_K.halves_timed_out}, processed {orch_K.frames_processed}")
check("evicted half's detection is returned as unpaired in the same call",
      len(out_K) == 1 and out_K[0]['paired'] is False
      and orch_K.detections_unmatched_sp0 == 1)

# A target seen only in the last subpage before shutdown: close() must
# hand it back, not count it as reported and then discard it.
flushed_K = orch_K.close()
check("close() returns the pending half's detection, and keeps it in results_at_close",
      len(flushed_K) == 1 and flushed_K[0]['paired'] is False
      and orch_K.results_at_close == flushed_K
      and orch_K.detections_reported == 2)

# ----------------------------------------------------------------------
print("\nL. Unmatched detections when halves have different counts")
# sp0 finds two targets; sp1 finds only one. The nearest pair is matched
# and averaged; the unmatched sp0 detection is emitted with paired=False
# rather than being silently discarded.

# Two blobs at u=5 and u=26 (21 px apart), hot against a uniform
# background. MAD will be near zero so both are above the sigma
# threshold. Each blob is 2x3 = 6 pixels, above min_pixels=5.
bg = np.full((24, 32), 20.0)
frame_two = bg.copy()
frame_two[10:12, 4:7] = 60.0     # left blob, centroid near u=5, v=10.5
frame_two[10:12, 24:27] = 60.0   # right blob, centroid near u=25, v=10.5

frame_one_left = bg.copy()
frame_one_left[10:12, 4:7] = 60.0  # only the left blob

orch_L = Orchestrator(lever_arm_b=ZERO_LEVER)
for t in range(0, 300, 20):
    orch_L.on_attitude(t, LEVEL_Q)
orch_L.on_lidar(0, H, 8000)
orch_L.on_position(0, 69.9689, 23.2717, 42.0)

orch_L.on_subpage(50, 0, frame_two)      # sp0: two blobs
out_L = orch_L.on_subpage(100, 1, frame_one_left)  # sp1: one blob

check("one detection matched, one unmatched from sp0",
      orch_L.detections_matched == 1 and orch_L.detections_unmatched_sp0 == 1
      and orch_L.detections_unmatched_sp1 == 0,
      f"matched {orch_L.detections_matched}, unmatched_sp0 {orch_L.detections_unmatched_sp0}")
check("two results returned in total",
      len(out_L) == 2, f"{len(out_L)} results")
check("matched has paired=True, unmatched has paired=False",
      any(r['paired'] is True for r in out_L)
      and any(r['paired'] is False for r in out_L))

# ----------------------------------------------------------------------
# M to P: the real camera's path, on two clocks.
#
# True time is the flight controller's clock in microseconds since it
# first booted. The ESP32 was switched on 37.25 s later and its crystal
# runs 20 ppm fast. Attitude is packed by the FC at 50 Hz inside its
# 2.5 ms loop, stamped in whole milliseconds, and reaches the ESP32 after
# its wire time plus a random wait (exponential, 2 ms mean) plus up to
# 40 ms for a third of the messages, first in first out. The ESP32 stamps
# each on parsing. The LiDAR is stamped by the ESP32 at 100 Hz. Each
# subpage integrates for 125 ms, is stamped by the ESP32 when its
# data-ready flag is seen at the END of that (plus up to 1 ms of polling),
# and reaches the orchestrator 45 ms later, after readout and USB.
ESP_AT_FC_ZERO_US = -37_250_000.0
ESP_DRIFT_PPM = 20.0
WIRE_US = cs.wire_time_us(cs.ATTITUDE_QUATERNION_WIRE_BYTES)


def esp_of(t_us):
    return int(round(ESP_AT_FC_ZERO_US + t_us * (1.0 + ESP_DRIFT_PPM * 1e-6)))


def attitude_events(rng, t0_s, t1_s, yaw_rate_deg_s, fc_boot_s=0.0, fifo=None):
    fifo = fifo if fifo is not None else {'last': -1e18}
    ev, t = [], t0_s * 1e6
    while t < t1_s * 1e6:
        t_pack = t + rng.uniform(0.0, 2500.0)
        lat = WIRE_US + rng.exponential(2000.0)
        if rng.random() < 1 / 3:
            lat += rng.uniform(0.0, 40_000.0)
        arrive = max(t_pack + lat, fifo['last'] + 20.0)
        fifo['last'] = arrive
        fc_ms = int((t_pack - fc_boot_s * 1e6) // 1000)
        ev.append((arrive, 1, 'att', (fc_ms, q_yaw(yaw_rate_deg_s * t_pack / 1e6),
                                      esp_of(arrive))))
        t += 20_000.0
    return ev


def lidar_events(t0_s, t1_s):
    return [(t, 2, 'lid', esp_of(t)) for t in np.arange(t0_s * 1e6, t1_s * 1e6, 10_000.0)]


def subpage_events(rng, first_mid_s, n, frame_for, first_index=0):
    ev = []
    for k in range(n):
        i = first_index + k
        t_mid = first_mid_s * 1e6 + k * 125_000.0
        t_ready = t_mid + 62_500.0 + rng.uniform(0.0, 1000.0)
        ev.append((t_ready + 45_000.0, 3, 'sub', (esp_of(t_ready), i % 2, frame_for(i), t_mid, i)))
    return ev


def dispatch(orch, events, fc_boot_s=0.0):
    """Feed events in true-time order. Returns [(index, t_mid_us, results)]."""
    out = []
    for t, _, kind, p in sorted(events, key=lambda e: (e[0], e[1])):
        if kind == 'att':
            orch.on_attitude(p[0], p[1], esp_rx_us=p[2], wire_bytes=cs.ATTITUDE_QUATERNION_WIRE_BYTES)
        elif kind == 'lid':
            orch.on_lidar_esp(p, H, 8000)
        elif kind == 'pos':
            orch.on_position(int((t - fc_boot_s * 1e6) // 1000), 69.9689, 23.2717, 42.0)
        else:
            out.append((p[4], p[3], orch.on_subpage_esp(p[0], p[1], p[2])))
    return out


def truth_ne(det, t_us):
    R = pb.quaternion_to_rotation(q_yaw(YAW_RATE * t_us / 1e6))
    n, e, _ = pb.pixel_to_ground_ned(det['u'], det['v'], H, R, ZERO_LEVER, pb.mounting_matrix())
    return n, e


def bucket_sum(c):
    return (c['frames_processed'] + c['rejected_no_clock_sync'] + c['rejected_no_attitude']
            + c['rejected_no_lidar'] + c['rejected_lidar_stale'] + c['rejected_lidar_unusable'])


# ----------------------------------------------------------------------
print("\nM. Real camera path: ESP32-stamped subpages, FC-stamped attitude")

det_side = cd.detect_centroids(frame_side)[0]
rng_m = np.random.default_rng(21)
events_m = (attitude_events(rng_m, 40.0, 60.0, YAW_RATE) + lidar_events(40.0, 60.0)
            + [(40.1e6, 0, 'pos', None)]
            + subpage_events(rng_m, 40.3, 150, lambda i: frame_side))
log_path_m = os.path.join(tmpdir, "two_clocks.pblog")
orch_M = Orchestrator(log_path=log_path_m, lever_arm_b=ZERO_LEVER)
session_m = dispatch(orch_M, events_m)
c_m = orch_M.counters()
orch_M.close()

refused_first = [i for i, _, res in session_m[:16] if res == []]
check("subpages before the clock estimate exists are refused and counted, not guessed",
      c_m['rejected_no_clock_sync'] > 0 and c_m['clock_rejected_no_estimate'] > 0
      and c_m['frames_processed'] > 0,
      f"{c_m['rejected_no_clock_sync']} refused in the first seconds, then "
      f"{c_m['frames_processed']} processed")

t_mid_of = {i: t for i, t, _ in session_m}
time_err_ms, pos_err_m = [], []
for i, t_mid, res in session_m:
    for r in res:
        if not r['paired']:
            continue
        # A pair is emitted by its second subpage, i, and averages it with
        # i - 1. It carries the time of whichever of the two is subpage 0:
        # after a refusal the halves can pair as sp1 then sp0.
        sp0 = i if i % 2 == 0 else i - 1
        time_err_ms.append(r['time_boot_ms'] - t_mid_of[sp0] / 1000.0)
        n0, e0 = truth_ne(det_side, t_mid_of[i - 1])
        n1, e1 = truth_ne(det_side, t_mid)
        pos_err_m.append(math.hypot(r['north_m'] - 0.5 * (n0 + n1), r['east_m'] - 0.5 * (e0 + e1)))
time_err_ms = np.array(time_err_ms)
pos_err_m = np.array(pos_err_m)
check("every paired frame was looked up within 2 ms of its true mid-exposure FC time",
      len(time_err_ms) > 50 and np.abs(time_err_ms).max() <= 2.0,
      f"{len(time_err_ms)} pairs, error {time_err_ms.mean():+.2f} ms mean, "
      f"{np.abs(time_err_ms).max():.2f} ms worst (the ESP32 clock alone is 37 s out)")
check("so at 30 deg/s yaw the ground position is right to the millimetre",
      pos_err_m.max() < 0.005,
      f"worst {pos_err_m.max() * 1000:.1f} mm against the true-attitude average")
check("every subpage lands in exactly one bucket, clock refusals included",
      bucket_sum(c_m) == c_m['subpages_received'],
      f"{bucket_sum(c_m)} of {c_m['subpages_received']}")
records_m = list(lf.read_frames(log_path_m))
first_pair_t = [r['time_boot_ms'] for _, _, res in session_m for r in res if r['paired']]
check("log records carry the FC time the attitude was looked up at, same as the results",
      len(records_m) == c_m['frames_logged'] > 0
      and [rec.time_boot_ms for rec in records_m] == first_pair_t,
      f"{len(records_m)} records")
check("the clock's counters surface in counters() under clock_",
      c_m['clock_queries_served'] > 0 and c_m['clock_fc_reboots'] == 0
      and {k for k in Orchestrator().counters() if k.startswith('clock_')}
      == {k for k in c_m if k.startswith('clock_')})

# ----------------------------------------------------------------------
print("\nN. The exposure midpoint is an error both halves share: averaging keeps it")
# The same events through an orchestrator that takes the data-ready time
# as the moment of the image. Both halves are then looked up 62.5 ms late,
# in the same direction, and the average of two equally late positions is
# equally late.

orch_N = Orchestrator(lever_arm_b=ZERO_LEVER, exposure_mid_offset_us=0)
session_n = dispatch(orch_N, events_m)
err_n, late_n = [], []
for i, t_mid, res in session_n:
    for r in res:
        if not r['paired']:
            continue
        t0_mid = t_mid_of[i - 1]
        n0, e0 = truth_ne(det_side, t0_mid)
        n1, e1 = truth_ne(det_side, t_mid)
        n0l, e0l = truth_ne(det_side, t0_mid + cs.EXPOSURE_MID_OFFSET_US)
        n1l, e1l = truth_ne(det_side, t_mid + cs.EXPOSURE_MID_OFFSET_US)
        err_n.append(math.hypot(r['north_m'] - 0.5 * (n0 + n1), r['east_m'] - 0.5 * (e0 + e1)))
        late_n.append(math.hypot(0.5 * (n0l + n1l) - 0.5 * (n0 + n1),
                                 0.5 * (e0l + e1l) - 0.5 * (e0 + e1)))
err_n, late_n = np.array(err_n), np.array(late_n)
check("uncorrected, the paired result is off by the full 62.5 ms of rotation",
      abs(np.median(err_n) - np.median(late_n)) < 0.2 * np.median(late_n),
      f"{np.median(err_n) * 100:.1f} cm, a 62.5 ms lag predicts {np.median(late_n) * 100:.1f} cm")
check("corrected, that error is gone",
      pos_err_m.max() < 0.15 * np.median(err_n),
      f"{np.median(pos_err_m) * 1000:.1f} mm median with the correction")

# ----------------------------------------------------------------------
print("\nO. Flight controller reboot mid-run")
# The FC reboots at 50 s of true time, is silent for 3 s while it boots,
# then counts time_boot_ms from zero again. The ESP32, the camera and the
# LiDAR carry on.

T_R = 50.0
rng_o = np.random.default_rng(22)
fifo_o = {'last': -1e18}
att_before = attitude_events(rng_o, 40.0, T_R, YAW_RATE, fifo=fifo_o)
att_after = attitude_events(rng_o, T_R + 3.0, 62.0, YAW_RATE, fc_boot_s=T_R, fifo=fifo_o)
subs_o = subpage_events(rng_o, 40.3, 170, lambda i: frame_side)
log_path_o = os.path.join(tmpdir, "reboot.pblog")
orch_O = Orchestrator(log_path=log_path_o, lever_arm_b=ZERO_LEVER,
                      log_kwargs={'batch_records': lf.SD_BATCH_RECORDS})
t_first_new = min(e[0] for e in att_after)
ev_o = att_before + att_after + lidar_events(40.0, 62.0) + subs_o + [(40.1e6, 0, 'pos', None)]
part1 = [e for e in ev_o if (e[0], e[1]) < (t_first_new, 1)]
part2 = [e for e in ev_o if (e[0], e[1]) >= (t_first_new, 1)]
sess_o1 = dispatch(orch_O, part1)
first_new = min(part2, key=lambda e: (e[0], e[1]))
dispatch(orch_O, [first_new])
c_at = orch_O.counters()
sess_o2 = dispatch(orch_O, [e for e in part2 if e is not first_new]
                   + [(T_R * 1e6 + 3.5e6, 0, 'pos', None)], fc_boot_s=T_R)
orch_O.close()
c_o = orch_O.counters()     # after close(): the last part-filled batch is written

check("the reboot is detected once, from the first attitude sample after it",
      c_at['fc_reboots'] == 1 and c_at['clock_resets'] == 1)

# In that run the log's age bound had already written everything during
# the FC's 3 s of silence. The reboot's own flush, directly: two paired
# frames waiting in a part-filled SD batch and one half waiting for its
# partner when time_boot_ms steps back.
orch_R = Orchestrator(log_path=os.path.join(tmpdir, "reboot_flush.pblog"),
                      lever_arm_b=ZERO_LEVER, log_kwargs={'batch_records': lf.SD_BATCH_RECORDS})
orch_R.on_position(0, 69.9689, 23.2717, 42.0)
for t in range(0, 1200, 20):
    orch_R.on_attitude(t, LEVEL_Q)
for t in range(0, 1200, 10):
    orch_R.on_lidar(t, H, 8000)
for i, t in enumerate(range(500, 1125, 125)):       # 5 subpages: 2 pairs and a held sp0
    orch_R.on_subpage(t, i % 2, frame)
held = orch_R._pending_half is not None
pending_R = orch_R.counters()['log_records_pending']
orch_R.on_attitude(100, LEVEL_Q)          # time_boot_ms back to 100 ms: the FC rebooted
c_R = orch_R.counters()
flushed_R = orch_R.close()
check("on a reboot, queued log records are written at once, not left for an age check "
      "across two epochs",
      pending_R == 2 and c_R['fc_reboots'] == 1 and c_R['log_records_pending'] == 0
      and c_R['log_records_written'] == 2,
      f"{pending_R} pending before, {c_R['log_records_pending']} after")
check("and the half waiting for its partner is emitted unpaired, not lost",
      held and len(flushed_R) == 1 and flushed_R[0]['paired'] is False
      and not math.isnan(flushed_R[0]['lat_deg']),
      "returned by close() here; by the next on_subpage() call in flight")

# The frozen interpolator on its own, fed the same samples, which is what
# the orchestrator did before it knew about reboots.
bare = AttitudeBuffer(capacity=128, max_gap_ms=200)
for _, _, _, p in sorted(att_before + att_after):
    bare.add(p[0], p[1])
refused_bare = sum(1 for e in att_after if e[3][0] <= att_before[-1][3][0])
check("every post-reboot attitude sample is stored, where a buffer that is not "
      "restarted refuses them",
      c_o['attitude_rejected_out_of_order'] == 0 and refused_bare == len(att_after),
      f"0 refused; an unrestarted buffer refuses all {refused_bare} as out of order")

post = [(i, t_mid, res) for i, t_mid, res in sess_o2 if t_mid > T_R * 1e6 + 3.0e6]
t_mid_o = {i: t for i, t, _ in sess_o1 + sess_o2}
errs_o, old_epoch = [], 0
for i, t_mid, res in post:
    for r in res:
        sp0 = i if i % 2 == 0 else i - 1
        if r['paired'] and sp0 in t_mid_o:
            truth_ms = (t_mid_o[sp0] - T_R * 1e6) / 1000.0
            errs_o.append(r['time_boot_ms'] - truth_ms)
            old_epoch += r['time_boot_ms'] > 40_000
errs_o = np.array(errs_o)
check("while the clock reconverges, subpages are refused and counted",
      c_o['rejected_no_clock_sync'] > c_m['rejected_no_clock_sync'],
      f"rejected_no_clock_sync {c_o['rejected_no_clock_sync']}")
check("afterwards every pair is in the new epoch and within 2 ms",
      len(errs_o) > 20 and old_epoch == 0 and np.abs(errs_o).max() <= 2.0,
      f"{len(errs_o)} pairs after the reboot, worst {np.abs(errs_o).max():.2f} ms, "
      f"{old_epoch} on the old clock")
records_o = list(lf.read_frames(log_path_o))
check("every paired frame reached the log, both sides of the reboot",
      len(records_o) == c_o['frames_logged'] > 0 and c_o['log_records_written'] == len(records_o)
      and records_o[-1].time_boot_ms < records_o[0].time_boot_ms,
      f"{len(records_o)} records, times {records_o[0].time_boot_ms} ... "
      f"{records_o[-1].time_boot_ms} ms")
check("and every subpage is still in exactly one bucket",
      bucket_sum(c_o) == c_o['subpages_received'])

# ----------------------------------------------------------------------
print("\nP. Link loss and wiring errors on the real camera path")
# Attitude dies at 46 s. The interpolator refuses first (nothing after the
# last sample), then the clock estimate goes stale and refuses too. Paired
# frames stop reaching the log with a batch part-filled; the log's age
# bound must still write it, on the clock estimate's last fit.

rng_p = np.random.default_rng(23)
ev_p = (attitude_events(rng_p, 40.0, 46.0, 0.0) + lidar_events(40.0, 52.0)
        + subpage_events(rng_p, 40.3, 90, lambda i: frame) + [(40.1e6, 0, 'pos', None)])
orch_P = Orchestrator(log_path=os.path.join(tmpdir, "link_loss.pblog"), lever_arm_b=ZERO_LEVER,
                      log_kwargs={'batch_records': lf.SD_BATCH_RECORDS})
dispatch(orch_P, ev_p)
c_p = orch_P.counters()
orch_P.close()
check("after the link dies, refusals move from the interpolator to the clock",
      c_p['attitude_rejected_too_new'] > 0 and c_p['clock_rejected_stale'] > 0
      and c_p['rejected_no_clock_sync'] > c_p['clock_rejected_no_estimate'],
      f"too_new {c_p['attitude_rejected_too_new']}, clock stale {c_p['clock_rejected_stale']}")
check("records stranded in a part-filled batch are still written within the age bound",
      c_p['log_records_pending'] == 0 and c_p['log_records_written'] == c_p['frames_logged'] > 0,
      f"logged {c_p['frames_logged']}, written {c_p['log_records_written']}")
check("every subpage accounted for", bucket_sum(c_p) == c_p['subpages_received'])

orch_W = Orchestrator(lever_arm_b=ZERO_LEVER)
orch_W.on_lidar(0, H, 8000)
try:
    orch_W.on_subpage_esp(1_000_000, 0, frame)
    raised_a = False
except ValueError:
    raised_a = True
orch_W = Orchestrator(lever_arm_b=ZERO_LEVER)
orch_W.on_lidar_esp(1_000_000, H, 8000)
try:
    orch_W.on_subpage(1000, 0, frame)
    raised_b = False
except ValueError:
    raised_b = True
check("LiDAR on one clock and subpages on the other is a wiring error, and raises",
      raised_a and raised_b)

orch_W = Orchestrator(lever_arm_b=ZERO_LEVER)
orch_W.on_lidar_esp(20_000_000, H, 8000)
stale_dup = orch_W.on_lidar_esp(19_995_000, H, 8000)
restart = orch_W.on_lidar_esp(15_000, H, 8000)
check("a LiDAR reading 5 ms back is refused; 20 s back is the ESP32 restarting",
      stale_dup is False and restart is True and orch_W.lidar_rejected_out_of_order == 1
      and orch_W.esp_restarts == 1)

# ----------------------------------------------------------------------
n_fail = results.count(False)
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
if n_fail:
    sys.exit(1)
