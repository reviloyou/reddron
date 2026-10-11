"""Tests for logformat.py: the height-projection shortcut it uses, the
log round trip bit-identical acceptance check from CLAUDE.md, resilience
to a truncated or corrupted file, and the buffered queue-and-drain mode
used for SD logging (F7)."""

import errno
import math
import os
import sys
import tempfile
import threading
import time

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
# F7: buffered mode and the queue-and-drain interface.

rng = np.random.default_rng(29)
N_REC = 23          # deliberately not a multiple of SD_BATCH_RECORDS
RECORDS = []
for i in range(N_REC):
    RECORDS.append(dict(
        time_boot_ms=250 * i, subpage=0,
        pixels=sf.create_synthetic_frame(rng.uniform(2, 29), rng.uniform(2, 21), rng=rng),
        lidar_distance_m=5.0 + 0.01 * i, lidar_strength=7000 + i,
        quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        lat_deg=69.9689 + 1e-6 * i, lon_deg=23.2717, alt_m=42.0))


def fresh(name):
    path = os.path.join(tmpdir, name)
    if os.path.exists(path):
        os.remove(path)
    return path


def read_bytes(path):
    with open(path, "rb") as f:
        return f.read()


def times_in(path, stats=None):
    return [r.time_boot_ms for r in lf.read_frames(path, stats)]


# Reference: the laptop default, fsync per record.
ref_path = fresh("ref_unbuffered.pblog")
w = lf.LogWriter(ref_path)
for rec in RECORDS:
    w.write_frame(**rec)
w.close()
REF = read_bytes(ref_path)
ALL_TIMES = [rec['time_boot_ms'] for rec in RECORDS]

print("\nD. Buffered mode writes a byte-identical file")

sd_path = fresh("sd_batched.pblog")
w = lf.LogWriter(sd_path, batch_records=lf.SD_BATCH_RECORDS)
for rec in RECORDS:
    w.write_frame(**rec)
pending_before_close = w.records_pending
w.close()
check(f"batch_records={lf.SD_BATCH_RECORDS} file is byte-identical to fsync-per-record",
      read_bytes(sd_path) == REF)
expected_batches = N_REC // lf.SD_BATCH_RECORDS + (1 if N_REC % lf.SD_BATCH_RECORDS else 0)
check(f"one fsync per batch: {w.batches_written} batches for {N_REC} records "
      f"(expected {expected_batches}), partial tail of {pending_before_close} drained at close",
      w.batches_written == expected_batches
      and pending_before_close == N_REC % lf.SD_BATCH_RECORDS)

manual_path = fresh("manual_drain.pblog")
w = lf.LogWriter(manual_path, auto_drain=False, queue_capacity=64)
drain_after = {0, 4, 5, 10, 17}
for i, rec in enumerate(RECORDS):
    w.write_frame(**rec)
    if i in drain_after:
        w.drain()
w.close()
check("auto_drain=False with irregular drains is byte-identical too",
      read_bytes(manual_path) == REF)
check("bytes_written counts the whole file, header included",
      w.bytes_written == len(REF))

# ----------------------------------------------------------------------
print("\nE. A crash mid-buffer loses only the queued tail")

crash_path = fresh("crash.pblog")
w = lf.LogWriter(crash_path, batch_records=lf.SD_BATCH_RECORDS)
for rec in RECORDS[:10]:          # drains after 4 and 8, leaves 2 queued
    w.write_frame(**rec)
# Read while the writer is still open and never closed: that is the file
# a power loss leaves behind. The two queued records only ever existed in
# RAM.
survived = times_in(crash_path)
check(f"after 10 writes with N={lf.SD_BATCH_RECORDS}, the 8 drained records are on disk "
      f"(got {len(survived)}) and 2 are pending (got {w.records_pending})",
      survived == ALL_TIMES[:8] and w.records_pending == 2)
check("what survived is byte-identical to the same prefix of the reference",
      read_bytes(crash_path) == REF[:len(lf.FILE_MAGIC) + 8 * lf.RECORD_SIZE])

# Power lost halfway through writing the second batch: a torn batch.
torn_path = fresh("torn_batch.pblog")
with open(torn_path, "wb") as f:
    f.write(read_bytes(crash_path)[:len(lf.FILE_MAGIC) + 6 * lf.RECORD_SIZE + 1000])
st = {}
torn_times = times_in(torn_path, st)
check(f"torn batch: the 6 complete records before the tear read back (got {len(torn_times)}), "
      f"the fragment is reported as a trailing partial ({st['trailing_partial']} bytes)",
      torn_times == ALL_TIMES[:6] and st['trailing_partial'] == 1000)
w.close()   # cleanup; the crash was simulated by reading before this

# ----------------------------------------------------------------------
print("\nF. Drain slower than fill")

# Deterministic. Capacity 8, the owner drains rarely.
slow_path = fresh("slow_drain.pblog")
w = lf.LogWriter(slow_path, auto_drain=False, queue_capacity=8)
accepted = []
for i in range(3):
    accepted.append(w.write_frame(**RECORDS[i]))
w.drain()
for i in range(3, 15):            # 12 offered into an empty queue of 8
    accepted.append(w.write_frame(**RECORDS[i]))
w.drain()
for i in range(15, 17):
    accepted.append(w.write_frame(**RECORDS[i]))
w.close()
kept = [RECORDS[i]['time_boot_ms'] for i in range(17) if accepted[i]]
check("full queue refuses the newest records and says so: 8 of 12 accepted, 4 refused",
      accepted[3:15] == [True] * 8 + [False] * 4)
check(f"refusals are counted (records_dropped_queue_full = {w.records_dropped_queue_full})",
      w.records_dropped_queue_full == 4)
check("the file holds exactly the accepted records, in order",
      times_in(slow_path) == kept)
c = w.counters()
check("conservation: enqueued == written + pending + lost, offered == enqueued + dropped",
      c['records_enqueued'] == c['records_written'] + c['records_pending'] + c['records_lost_at_close']
      and c['records_enqueued'] + c['records_dropped_queue_full'] == 17)

# Threaded, deterministic drops. The drain thread is held until the
# producer has offered everything, so the queue is certain to overflow.
gate_path = fresh("threaded_gate.pblog")
w = lf.LogWriter(gate_path, auto_drain=False, queue_capacity=lf.DEFAULT_QUEUE_CAPACITY)
go = threading.Event()
drainer = threading.Thread(target=lambda: (go.wait(), w.drain()))
drainer.start()
offered = RECORDS * 3            # 69 records, times repeat per lap; order is what matters
results_gate = [w.write_frame(**rec) for rec in offered]
go.set()
drainer.join()
w.close()
check(f"threaded, drain held back: {sum(results_gate)} accepted (the queue capacity), "
      f"{w.records_dropped_queue_full} refused and counted",
      sum(results_gate) == lf.DEFAULT_QUEUE_CAPACITY
      and w.records_dropped_queue_full == len(offered) - lf.DEFAULT_QUEUE_CAPACITY
      and times_in(gate_path) == [r['time_boot_ms'] for r in offered[:lf.DEFAULT_QUEUE_CAPACITY]])

# Threaded, genuinely concurrent: a drain task that sleeps between
# drains, a producer that does not wait for it. With room in the queue
# nothing may be lost or reordered, and the file must match the
# reference byte for byte.
conc_path = fresh("threaded_concurrent.pblog")
w = lf.LogWriter(conc_path, auto_drain=False, queue_capacity=1000)
stop = threading.Event()


def drain_task():
    while not stop.is_set():
        w.drain()
        time.sleep(0.002)
    w.drain()


drainer = threading.Thread(target=drain_task)
drainer.start()
for rec in RECORDS:
    w.write_frame(**rec)
    time.sleep(0.0005)
stop.set()
drainer.join()
w.close()
check(f"concurrent producer and drain task: byte-identical to the reference "
      f"({w.batches_written} batches, 0 refused)",
      read_bytes(conc_path) == REF and w.records_dropped_queue_full == 0)

# ----------------------------------------------------------------------
print("\nG. Device failure mid-batch: counted, retried, no duplicated bytes")


class FailingDevice:
    """Stands in for the log file. Accepts `budget` bytes, reports a
    short write for the call that crosses the budget (as write(2) does on
    a regular file), then refuses with ENOSPC until healed."""

    def __init__(self, real, budget):
        self.real = real
        self.budget = budget
        self.healed = False

    def write(self, b):
        if not self.healed:
            if self.budget <= 0:
                raise OSError(errno.ENOSPC, "No space left on device (simulated)")
            if len(b) > self.budget:
                n = self.real.write(bytes(b[:self.budget]))
                self.budget -= n
                return n
            self.budget -= len(b)
        return self.real.write(b)

    def fileno(self):
        return self.real.fileno()

    def close(self):
        self.real.close()


fail_path = fresh("device_failure.pblog")
w = lf.LogWriter(fail_path, batch_records=lf.SD_BATCH_RECORDS)
device = FailingDevice(w._f, budget=6 * lf.RECORD_SIZE + 100)
w._f = device
for rec in RECORDS[:8]:           # batch 1 fine, batch 2 fails 100 bytes into record 7
    w.write_frame(**rec)
st = {}
on_disk = times_in(fail_path, st)
check(f"failure is counted (write_failures = {w.write_failures}) and the unwritten "
      f"records stay queued (pending = {w.records_pending})",
      w.write_failures == 1 and w.records_pending == 2 and w.records_written == 6)
check(f"meanwhile the file reads back the 6 complete records plus a "
      f"{st['trailing_partial']}-byte torn fragment",
      on_disk == ALL_TIMES[:6] and st['trailing_partial'] == 100)
device.healed = True
for rec in RECORDS[8:]:
    w.write_frame(**rec)
w.close()
st = {}
check("after the fault clears, the next drain resumes mid-record: the file is "
      "byte-identical to an uninterrupted run",
      read_bytes(fail_path) == REF)
check("and reads back cleanly, nothing skipped, nothing duplicated",
      times_in(fail_path, st) == ALL_TIMES and st['skipped_bytes'] == 0)

# A fault that never clears: the queue fills, refusals are counted, and
# what could not be written by close is counted as lost. Nothing vanishes
# without a number attached to it.
dead_path = fresh("device_dead.pblog")
w = lf.LogWriter(dead_path, batch_records=lf.SD_BATCH_RECORDS, queue_capacity=8)
w._f = FailingDevice(w._f, budget=2 * lf.RECORD_SIZE)
for rec in RECORDS:
    w.write_frame(**rec)
w.close()
c = w.counters()
check(f"dead device: written {c['records_written']}, refused {c['records_dropped_queue_full']}, "
      f"lost at close {c['records_lost_at_close']}, all {N_REC} accounted for",
      c['records_written'] == 2
      and c['records_written'] + c['records_dropped_queue_full'] + c['records_lost_at_close'] == N_REC
      and c['records_lost_at_close'] == 8 and c['write_failures'] > 0)

# ----------------------------------------------------------------------
print("\nH. Reader: preallocation padding, stale records, and open-time checks")

for fill, name in [(b"\x00", "0x00"), (b"\xff", "0xFF")]:
    pad_path = fresh(f"padded_{name}.pblog")
    with open(pad_path, "wb") as f:
        f.write(REF[:len(lf.FILE_MAGIC) + 5 * lf.RECORD_SIZE] + fill * 65536)
    st = {}
    t = times_in(pad_path, st)
    check(f"untruncated SD file ending in {name}: 5 records, 65536 bytes reported as "
          f"padding, not corruption (skipped {st['skipped_bytes']})",
          t == ALL_TIMES[:5] and st['padding_bytes'] == 65536 and st['skipped_bytes'] == 0)

torn_pad_path = fresh("torn_then_padding.pblog")
with open(torn_pad_path, "wb") as f:
    f.write(REF[:len(lf.FILE_MAGIC) + 5 * lf.RECORD_SIZE + 700] + b"\x00" * 65536)
st = {}
t = times_in(torn_pad_path, st)
check(f"torn record then padding: 5 records, the fragment counted as skipped "
      f"({st['skipped_bytes']}), the zeros as padding ({st['padding_bytes']})",
      t == ALL_TIMES[:5] and st['skipped_bytes'] == 700 and st['padding_bytes'] == 65536)

back_path = fresh("time_backwards.pblog")
w = lf.LogWriter(back_path)
for t_ms in [0, 1000, 2000, 500, 600]:
    w.write_frame(**dict(RECORDS[0], time_boot_ms=t_ms))
w.close()
st = {}
t = times_in(back_path, st)
check("time_boot_ms going backwards is flagged, and every record is still yielded",
      t == [0, 1000, 2000, 500, 600] and st['time_backwards'] == 1)

append_path = fresh("append.pblog")
w = lf.LogWriter(append_path)
for rec in RECORDS[:2]:
    w.write_frame(**rec)
w.close()
w = lf.LogWriter(append_path)
for rec in RECORDS[2:5]:
    w.write_frame(**rec)
w.close()
check("reopening an existing log appends without a second header",
      read_bytes(append_path) == REF[:len(lf.FILE_MAGIC) + 5 * lf.RECORD_SIZE])

alien_path = fresh("not_a_log.bin")
with open(alien_path, "wb") as f:
    f.write(b"some other file")
try:
    lf.LogWriter(alien_path)
    refused = False
except ValueError:
    refused = True
check("opening a non-empty file that is not a Pillar B log is refused at open",
      refused and read_bytes(alien_path) == b"some other file")

# ----------------------------------------------------------------------
print("\nI. Review fixes: close() races, interrupts, fsync failures, age bound")


class SlowDevice:
    """Signals when a write starts, then takes its time, so another
    thread can be made to act while a drain is in progress."""

    def __init__(self, real, delay_s):
        self.real = real
        self.delay_s = delay_s
        self.writing = threading.Event()

    def write(self, b):
        self.writing.set()
        time.sleep(self.delay_s)
        return self.real.write(b)

    def fileno(self):
        return self.real.fileno()

    def close(self):
        self.real.close()


# close() against a writer thread's drain and a producer, deterministic.
# close() is made to sit inside its final drain; meanwhile a second drain
# and a late write_frame arrive. Before the fix the second drain could
# write to a closed file or pop from a cleared queue.
race_path = fresh("close_race.pblog")
w = lf.LogWriter(race_path, auto_drain=False, queue_capacity=64)
slow = SlowDevice(w._f, 0.05)
w._f = slow
for rec in RECORDS[:3]:
    w.write_frame(**rec)
thread_errors = []
late_drain = []


def guarded(fn):
    try:
        fn()
    except Exception as exc:      # recorded, then asserted empty
        thread_errors.append(repr(exc))


closer = threading.Thread(target=guarded, args=(w.close,))
closer.start()
slow.writing.wait(5.0)            # close() is now inside its drain, holding the lock
drainer = threading.Thread(target=guarded, args=(lambda: late_drain.append(w.drain()),))
drainer.start()
try:
    w.write_frame(**RECORDS[3])
    late_write = "accepted"
except ValueError:
    late_write = "refused"
closer.join()
drainer.join()
check(f"close() during a writer-thread drain: nothing raises in either thread "
      f"({thread_errors or 'none'}), the late drain finds nothing to do, a late write is refused",
      thread_errors == [] and late_drain == [0] and late_write == "refused")
check("and the file, records_written and file_bytes all agree",
      times_in(race_path) == ALL_TIMES[:3] and w.records_written == 3
      and w.file_bytes == os.path.getsize(race_path))

# The same, randomised: a writer thread draining, a producer writing, and
# close() arriving at an arbitrary point.
bad_runs = []
for run_i in range(30):
    path = fresh(f"close_stress_{run_i}.pblog")
    w = lf.LogWriter(path, batch_records=1 + run_i % 2, auto_drain=bool(run_i % 2),
                     queue_capacity=8)
    stop = threading.Event()
    errs = []

    def writer_task():
        while not stop.is_set():
            try:
                w.drain()
            except Exception as exc:
                errs.append(repr(exc))
            time.sleep(0.0005)

    def producer():
        for rec in RECORDS:
            try:
                w.write_frame(**rec)
            except ValueError:    # closed under it: the documented refusal
                return
            except Exception as exc:
                errs.append(repr(exc))
                return

    threads = [threading.Thread(target=writer_task), threading.Thread(target=producer)]
    for t in threads:
        t.start()
    time.sleep(0.001 * (run_i % 7))
    try:
        w.close()
    except Exception as exc:
        errs.append(repr(exc))
    stop.set()
    for t in threads:
        t.join()
    c = w.counters()
    on_disk = len(list(lf.read_frames(path)))
    if (errs or on_disk != c['records_written'] or os.path.getsize(path) != c['file_bytes']
            or c['records_enqueued'] != c['records_written'] + c['records_lost_at_close']):
        bad_runs.append((run_i, errs, on_disk, c['records_written']))
check(f"30 randomised close() races: no exceptions, file == counters in every run "
      f"({len(bad_runs)} bad)", not bad_runs)

# Ctrl+C lands just after an fsync, inside the drain. Before the fix the
# byte accounting was skipped and close() wrote the record a second time.
real_fsync = os.fsync
ki_path = fresh("keyboard_interrupt.pblog")
w = lf.LogWriter(ki_path)
w.write_frame(**RECORDS[0])
w.write_frame(**RECORDS[1])
armed = [True]


def fsync_then_interrupt(fd):
    real_fsync(fd)
    if armed[0]:
        armed[0] = False
        raise KeyboardInterrupt


os.fsync = fsync_then_interrupt
try:
    try:
        w.write_frame(**RECORDS[2])
        interrupted = False
    except KeyboardInterrupt:
        interrupted = True
finally:
    os.fsync = real_fsync
w.close()
st = {}
check("Ctrl+C mid-drain propagates, and close() does not write the record twice",
      interrupted and times_in(ki_path, st) == ALL_TIMES[:3]
      and read_bytes(ki_path) == REF[:len(lf.FILE_MAGIC) + 3 * lf.RECORD_SIZE]
      and w.records_written == 3)

# A write that succeeds followed by an fsync that fails: the records are
# counted as unconfirmed, and close() retries the fsync.
fs_path = fresh("fsync_failure.pblog")
w = lf.LogWriter(fs_path, batch_records=lf.SD_BATCH_RECORDS)
fsync_calls = [0]


def fsync_fails_once(fd):
    fsync_calls[0] += 1
    if fsync_calls[0] == 2:
        raise OSError(errno.EIO, "Input/output error (simulated fsync)")
    real_fsync(fd)


os.fsync = fsync_fails_once
try:
    for rec in RECORDS[:8]:       # batch 1 syncs, batch 2's fsync fails
        w.write_frame(**rec)
    unsynced_mid = w.records_unsynced
    w.close()
finally:
    os.fsync = real_fsync
check(f"failed fsync: {unsynced_mid} records counted as unconfirmed, close() retried "
      f"the fsync ({fsync_calls[0]} calls) and cleared them",
      unsynced_mid == 4 and fsync_calls[0] == 3 and w.records_unsynced == 0
      and w.write_failures == 1 and times_in(fs_path) == ALL_TIMES[:8])

# Records stop arriving before a batch fills: the age bound writes them.
age_path = fresh("age_bound.pblog")
w = lf.LogWriter(age_path, batch_records=lf.SD_BATCH_RECORDS)
w.write_frame(**RECORDS[0])       # t = 0
w.write_frame(**RECORDS[1])       # t = 250
not_yet = w.drain_if_older_than(1000, 1000)
now = w.drain_if_older_than(1001, 1000)
check("drain_if_older_than writes a part-filled batch once its oldest record is too old",
      not_yet == 0 and now == 2 and times_in(age_path) == ALL_TIMES[:2])
w.close()

# file_bytes is the file's length even when a reboot reopened the file,
# which is what the port contract truncates to.
reopen_path = fresh("reopen.pblog")
w = lf.LogWriter(reopen_path)
for rec in RECORDS[:5]:
    w.write_frame(**rec)
w.close()
w = lf.LogWriter(reopen_path)
for rec in RECORDS[5:7]:
    w.write_frame(**rec)
w.close()
check(f"after reopening, file_bytes ({w.file_bytes}) is the file length, "
      f"bytes_written ({w.bytes_written}) only this session's",
      w.file_bytes == os.path.getsize(reopen_path)
      and w.bytes_written == 2 * lf.RECORD_SIZE)

gap_path = fresh("time_gap.pblog")
w = lf.LogWriter(gap_path)
for t_ms in [0, 250, 5000, 5250]:
    w.write_frame(**dict(RECORDS[0], time_boot_ms=t_ms))
w.close()
st = {}
check("a forward jump in time_boot_ms is flagged as a gap, not as going backwards",
      times_in(gap_path, st) == [0, 250, 5000, 5250]
      and st['time_gaps'] == 1 and st['time_backwards'] == 0)

# ----------------------------------------------------------------------
n_fail = results.count(False)
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
if n_fail:
    sys.exit(1)
