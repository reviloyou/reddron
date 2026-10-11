"""
logformat.py

Raw per-frame logging format for Pillar B. One record per thermal
frame: timestamp, all 768 raw pixel values, LiDAR distance and strength,
attitude quaternion, GPS position, and the MLX90640 subpage index.

Format
Fixed-size binary records, one after another, after an 8-byte file header.
No delimiters between fields inside a record (the layout is fixed), but
every record starts with its own 4-byte magic marker and ends with a
CRC32 of its payload. This means the reader does not have to trust byte
offsets: it scans for the magic, validates the checksum, and resyncs on
anything that does not check out. That is the same approach main.cpp
already uses for the TFmini-S UART frames (look for 0x59 0x59, verify the
checksum, drop and resync on failure), applied here so a session
interrupted mid-write (power loss, USB drop) never corrupts records
written before the interruption, and the damage from a partial record is
contained to that one record.

Binary was chosen over text because 768 floats per frame at 8 Hz is not
comfortable as CSV, and a fixed C-like struct layout is also the easiest
thing to port when this gets translated to the ESP32 firmware later.
Readability without custom tooling is covered by to_csv(), which unpacks
a log to one row per frame (pixels flattened into columns).

Queue and drain
---------------
LogWriter is split into a producer side and a consumer side, because on
the ESP32 they run on different cores:

    write_frame()  producer. Packs the record and appends it to a bounded
                   in-RAM queue. Never touches the file unless auto_drain
                   is on. On the ESP32 this is xQueueSend with a zero
                   timeout, called from the sensor path on core 1.
    drain()        consumer. Writes every queued record in one batch and
                   fsyncs once. On the ESP32 this is the body of a
                   FreeRTOS writer task pinned to core 0.

A write stall inline in the sensor path loses MAVLink messages for the
duration of the stall and puts a hole in the attitude buffer, which is
the error attitude_interpolator.py exists to remove. The split keeps the
stall on the other core.

If the drain falls so far behind that the queue is full, write_frame()
refuses the NEWEST record, returns False and counts it in
records_dropped_queue_full. Blocking the producer instead would move the
stall back into the sensor path, and evicting the oldest record would cut
a hole into the middle of the file rather than stopping at the point of
failure.

Modes
-----
batch_records=1, auto_drain=True (the default): every write_frame()
drains immediately, so on a healthy device each record is written and
fsync'd before write_frame() returns. This is the laptop behaviour, where
an interrupted session must be durable to the last record. On a failing
device write_frame() still returns True (queued) rather than raising; see
"Write failures". True means accepted into the queue, never "durable":
records_written is the durable count.

batch_records=SD_BATCH_RECORDS, auto_drain=True: records are written in
batches of N with one fsync per batch. An fsync per record on an SD card
forces a physical write per record and maximises stalls.

auto_drain=False: write_frame() only enqueues, and whoever owns the log
calls drain() from its own thread or task. This is the structure the
ESP32 port has.

What a power loss costs in buffered mode
----------------------------------------
A record is durable once the batch containing it has been written and
fsync'd. An unexpected power loss loses whatever is still queued: at most
N - 1 records waiting for the batch to fill, or N if the loss lands while
that batch is being written (a torn batch, whose complete records before
the tear still read back; the reader skips the fragment). Nothing written
before that is affected.

The orchestrator logs one record per paired frame, which is 4 per second
at the 8 Hz subpage setting, so SD_BATCH_RECORDS = 4 bounds the loss at
one second of flight. Losing the last second is acceptable; losing the
flight is not. A larger N saves little (a 4-record batch is already
12.5 kB, well past the point where per-write overhead dominates) and
costs tail.

That bound assumes records keep arriving. If they stop (attitude or
LiDAR drops out, so no paired frames reach the log) a part-filled batch
would otherwise sit in RAM indefinitely, and the records lost would be
the ones just before the fault, the ones an investigation needs most.
drain_if_older_than() drains when the oldest queued record is older than
a given age on the time_boot_ms clock; the orchestrator calls it on every
subpage with LOG_MAX_PENDING_MS, so the one-second bound holds as long as
the camera is running. With auto_drain=False the bound is set by how
often the owner drains, and the ESP32 writer task must drain on a timer
as well as on a full batch for the same reason.

Write failures
--------------
drain() never raises on an I/O error. A card that fills or stops
accepting writes mid-flight increments write_failures, records the error
in last_error, prints one warning to stderr on the transition from
healthy to failing, and leaves the unwritten records queued for the next
drain. Bytes the device did accept before failing are not written again:
the next drain resumes at the exact byte where the previous one stopped,
so the file stays byte-identical to an uninterrupted run instead of
containing a duplicated fragment. If the fault persists, the queue fills
and further records are refused and counted, not silently lost.

The byte accounting is committed even when something other than an I/O
error interrupts a drain (Ctrl+C during a laptop capture is the common
one); the interruption then propagates, but the bytes that reached the
file are never written a second time by the drain in close().

A write that succeeds followed by an fsync that fails leaves the records
in the file but not confirmed durable. They are counted in
records_unsynced, the next drain retries the fsync even with nothing
queued, and close() makes a final attempt and reports what is still
unconfirmed.

close() holds the drain lock throughout: it refuses new records first,
then drains, then closes the file, so a writer thread that calls drain()
concurrently waits and then finds nothing to do. Stop the producer before
close(); write_frame() after close() raises.

ESP32 port contract (not implemented here)
------------------------------------------
The laptop has no SD card and no internal flash split, so the following
is specified here for the C++ translation rather than exercised in
Python. counters() already carries the two card-only fields, as None, so
the interface is the same on both sides.

  * Library: SdFat, not the Arduino SD.h. SdFat reads exFAT directly, so
    a 64 GB SDXC card works as shipped, and it can preallocate a
    contiguous file so in-flight writes do not update the allocation
    table.
  * Boot: mount the card, create the session file, preallocate 64 MB
    (a 90 s flight is 1.13 MB at 12.5 kB/s, so this covers over an
    hour), write FILE_MAGIC, sync, read the header back, and only then
    report ready. A card that fails any step sets card_mounted false and
    must be surfaced before takeoff: the worst outcome is a perfect
    flight with no log, discovered after landing.
  * What preallocated space contains depends on the filesystem (SdFat
    2.3.1, FsFile::preAllocate). On exFAT, the card as shipped, the file
    gets a zero valid length: anything past the last write whose
    directory entry was synced reads back as zeros, which read_frames()
    reports as trailing padding. The writer's sync after every batch is
    what keeps that valid length current. On FAT16/FAT32 the clusters
    hold "uninitialized data", whatever an earlier file left there, which
    can include complete records from an earlier flight with valid magic
    and CRC. If the session ends without the truncate (power loss), the
    reader would walk straight into them. On a FAT32 card, erase the
    preallocated range at boot (FsFile::contiguousRange gives its
    sectors, SdCard::erase clears them); on either filesystem, refuse to
    fly if the file is not contiguous. The erase is mandatory on FAT32,
    not a nicety: stale records whose timestamps happen to be later than
    the current flight's cannot be told apart from the record content.
    read_frames() flags timestamps going backwards and forward jumps, but
    a stale record that lands just after the flight's last one passes
    both checks. firmware/src/sd_check.cpp exercises preallocation and
    reports the filesystem and contiguity.
  * Landing: drain, truncate the file to file_bytes (not bytes_written,
    which counts only this session's bytes and is smaller when a reboot
    reopened an existing file), close. The truncate is a filesystem
    write, harmless on the ground.
  * Tasks: the writer task is pinned to core 0, sensor ingest and the
    orchestrator chain to core 1. The queue holds pointers into a static
    pool of queue_capacity record buffers allocated at boot; no heap
    allocation in flight.
  * RAM: DEFAULT_QUEUE_CAPACITY = 16 records is 50 kB, and absorbs a
    4 s writer outage at 4 records per second. The SD specification
    caps a single write's busy time at a few hundred milliseconds, so
    this covers a worst-case card stall with a wide margin.
  * Preallocation reduces stalls; it does not remove them. The card's
    controller erases blocks and levels wear on its own schedule, which
    no filesystem choice affects. The queue and the writer task are
    required regardless.
  * Storage split. Bulk frame logs go to SD. Internal flash (LittleFS)
    keeps what must survive a missing or failed card:
      - calibration constants (F_U, F_V, LEVER_ARM_B) once measured. If
        they lived only on SD, a card failure would mean flying with
        wrong intrinsics and producing plausible wrong positions, the
        worst failure mode available because nothing about it looks
        broken;
      - a small circular buffer of the last few seconds of records
        (5 s is 20 records, 63 kB), so a mid-flight card failure does
        not lose everything;
      - the counters and the end-of-run summary, small, written at
        landing.
    The principle is bulk-sequential versus small-durable, not data
    versus metadata.
"""

import csv
import os
import struct
import sys
import threading
import zlib
from collections import deque, namedtuple

import numpy as np

import pillar_b as pb

N_PIXELS = 768   # PillarB.pillar_b.N_COLS * N_ROWS, kept independent on purpose:
                 # this module must not need to import sensor geometry to log a frame.

FILE_MAGIC = b"PBLOG1\n\x00"     # 8 bytes, once at the start of the file
RECORD_MAGIC = b"RDF1"           # 4 bytes, once at the start of every record

# '<' little-endian, explicit widths, no implicit alignment padding beyond
# what is written out explicitly below.
_RECORD_FMT = "<4sIB3x768ffH2x4fddfI"
_RECORD_STRUCT = struct.Struct(_RECORD_FMT)
RECORD_SIZE = _RECORD_STRUCT.size

FrameRecord = namedtuple(
    "FrameRecord",
    "time_boot_ms subpage frame lidar_distance_m lidar_strength quat lat_deg lon_deg alt_m",
)


def _pack_record(time_boot_ms, subpage, pixels, lidar_distance_m, lidar_strength,
                 quat, lat_deg, lon_deg, alt_m):
    pixels = np.asarray(pixels, dtype="<f4").reshape(-1)
    if pixels.size != N_PIXELS:
        raise ValueError(f"expected {N_PIXELS} pixels, got {pixels.size}")
    quat = np.asarray(quat, dtype="<f4").reshape(-1)
    if quat.size != 4:
        raise ValueError(f"expected a 4-element quaternion (w, x, y, z), got {quat.size}")
    if subpage not in (0, 1):
        raise ValueError(f"subpage must be 0 or 1, got {subpage}")

    payload = struct.pack(
        "<IB3x768ffH2x4fddf",
        int(time_boot_ms) & 0xFFFFFFFF, int(subpage),
        *pixels.tolist(),
        float(lidar_distance_m), int(lidar_strength) & 0xFFFF,
        *quat.tolist(),
        float(lat_deg), float(lon_deg), float(alt_m),
    )
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    return RECORD_MAGIC + payload + struct.pack("<I", crc)


def _unpack_record(chunk):
    """Return a FrameRecord, or None if the magic/CRC does not check out."""
    if len(chunk) != RECORD_SIZE or chunk[:4] != RECORD_MAGIC:
        return None
    payload = chunk[4:-4]
    crc_stored = struct.unpack("<I", chunk[-4:])[0]
    if zlib.crc32(payload) & 0xFFFFFFFF != crc_stored:
        return None

    fields = struct.unpack("<IB3x768ffH2x4fddf", payload)
    time_boot_ms = fields[0]
    subpage = fields[1]
    pixels = np.array(fields[2:2 + N_PIXELS], dtype=np.float32).reshape(24, 32)
    lidar_distance_m = fields[2 + N_PIXELS]
    lidar_strength = fields[3 + N_PIXELS]
    quat = np.array(fields[4 + N_PIXELS:8 + N_PIXELS], dtype=np.float32)
    lat_deg, lon_deg, alt_m = fields[8 + N_PIXELS:11 + N_PIXELS]

    return FrameRecord(time_boot_ms, subpage, pixels, lidar_distance_m,
                       lidar_strength, quat, lat_deg, lon_deg, alt_m)


# Records per batch for SD logging. See "What a power loss costs in
# buffered mode" above: at 4 records per second this bounds the loss at
# one second.
SD_BATCH_RECORDS = 4

# Queue bound, in records. See the RAM note in the ESP32 port contract.
DEFAULT_QUEUE_CAPACITY = 16

# A forward jump between consecutive records larger than this is reported
# by read_frames(). Records normally arrive every 250 ms (one per paired
# frame at 4 Hz), so a second without one means frames stopped reaching
# the log, or the reader is no longer reading this session's records.
TIME_GAP_MS = 1000


class LogWriter:
    """Append-only writer with a bounded queue between producer and disk.

    time_boot_ms must be the MAVLink ATTITUDE_QUATERNION time_boot_ms for
    this frame, not the ESP32's arrival time for the frame or the attitude
    message. Arrival time carries transmission latency that varies frame
    to frame, and modelling that latency is what T6 exists for; folding it
    invisibly into the logged timestamp would make that impossible to
    recover later.

    Parameters
    ----------
    path : str
        Log file. Created with a FILE_MAGIC header if new or empty. An
        existing non-empty file must already start with FILE_MAGIC:
        appending records to anything else would produce a file the
        reader refuses as a whole, so that is refused here, at open,
        rather than discovered after landing.
    batch_records : int
        With auto_drain, drain once this many records are queued.
    auto_drain : bool
        Drain from inside write_frame() when a batch is full. False
        means the owner calls drain() itself, from its own thread.
    queue_capacity : int
        Maximum records held in RAM. Must hold at least one batch.

    Counters are plain attributes (see counters()). The invariant
    records_enqueued == records_written + records_pending
    + records_lost_at_close holds at every point, and every record
    offered to write_frame() is either enqueued or counted in
    records_dropped_queue_full. records_written counts records whose
    bytes reached the file; records_unsynced is the subset of those not
    yet confirmed by an fsync. file_bytes is the file's length, which is
    what the ESP32 port truncates to at landing.
    """

    def __init__(self, path, batch_records=1, auto_drain=True,
                 queue_capacity=DEFAULT_QUEUE_CAPACITY):
        if batch_records < 1:
            raise ValueError(f"batch_records must be at least 1, got {batch_records}")
        if queue_capacity < batch_records:
            raise ValueError(f"queue_capacity ({queue_capacity}) must hold at least "
                             f"one batch ({batch_records})")
        self.path = path
        self.batch_records = int(batch_records)
        self.auto_drain = bool(auto_drain)
        self.queue_capacity = int(queue_capacity)

        # (time_boot_ms, packed record) not yet written, oldest first. The
        # timestamp is kept for drain_if_older_than(). _head_written is
        # how many bytes of the oldest record already reached the file in
        # a drain that stopped partway, so the next drain resumes there.
        self._queue = deque()
        self._head_written = 0
        self._unsynced = 0
        self._lock = threading.Lock()         # guards the queue and counters
        self._drain_lock = threading.Lock()   # one drain (or close) at a time

        self.records_enqueued = 0
        self.records_written = 0
        self.records_dropped_queue_full = 0
        self.records_lost_at_close = 0
        self.batches_written = 0
        self.write_failures = 0
        self.bytes_written = 0
        self.file_bytes = 0
        self.last_error = None
        self.closed = False
        self._failing = False

        new_file = not os.path.exists(path) or os.path.getsize(path) == 0
        if not new_file:
            with open(path, "rb") as f:
                header = f.read(len(FILE_MAGIC))
            if header != FILE_MAGIC:
                raise ValueError(f"{path}: exists and is not a Pillar B log "
                                 f"(header {header!r}), refusing to append")
            self.file_bytes = os.path.getsize(path)

        # Unbuffered, so every byte handed to the OS is counted exactly and
        # a failed write leaves nothing hidden in a Python-side buffer.
        self._f = open(path, "ab", buffering=0)
        if new_file:
            # Not routed through drain(): a sink that cannot take its own
            # header is not ready, and that must raise here, at open.
            self._write_all(FILE_MAGIC)
            os.fsync(self._f.fileno())
            self.bytes_written += len(FILE_MAGIC)
            self.file_bytes += len(FILE_MAGIC)

    def _write_all(self, data):
        view = memoryview(data)
        written = 0
        while written < len(view):
            n = self._f.write(view[written:])
            if not n:
                raise OSError("short write: the device accepted no bytes")
            written += n
        return written

    @property
    def records_pending(self):
        with self._lock:
            return len(self._queue)

    @property
    def records_unsynced(self):
        with self._lock:
            return self._unsynced

    def write_frame(self, time_boot_ms, subpage, pixels, lidar_distance_m,
                    lidar_strength, quat, lat_deg, lon_deg, alt_m):
        """Enqueue one record. Returns True if it was accepted, False if the
        queue was full and the record was refused (and counted).

        A malformed record raises ValueError here, at the caller, before
        anything is queued: that is a programming error in the producer,
        not a condition to count and continue past.
        """
        record = _pack_record(time_boot_ms, subpage, pixels, lidar_distance_m,
                              lidar_strength, quat, lat_deg, lon_deg, alt_m)
        with self._lock:
            if self.closed:
                raise ValueError(f"{self.path}: write_frame() on a closed LogWriter")
            if len(self._queue) >= self.queue_capacity:
                self.records_dropped_queue_full += 1
                accepted = False
            else:
                self._queue.append((int(time_boot_ms), record))
                self.records_enqueued += 1
                accepted = True
            batch_full = self.auto_drain and len(self._queue) >= self.batch_records
        if batch_full:
            self.drain()
        return accepted

    def drain(self):
        """Write every queued record in one batch and fsync once.

        Returns the number of records that became fully written. Safe to
        call from a different thread than write_frame(); records enqueued
        while a drain is in progress wait for the next one. A no-op once
        the writer is closed.
        """
        with self._drain_lock:
            with self._lock:
                if self.closed:
                    return 0
            return self._drain_locked()

    def drain_if_older_than(self, now_ms, max_age_ms):
        """Drain if the oldest queued record's time_boot_ms is more than
        max_age_ms before now_ms (same clock). Keeps the loss bound when
        records stop arriving before a batch fills. Returns the number of
        records written, 0 if nothing was old enough."""
        with self._lock:
            if self.closed or not self._queue:
                return 0
            oldest_ms = self._queue[0][0]
        if now_ms - oldest_ms <= max_age_ms:
            return 0
        return self.drain()

    def _drain_locked(self):
        """The drain body. Caller holds _drain_lock."""
        with self._lock:
            batch = [rec for _, rec in self._queue]
            head_written = self._head_written
            sync_owed = self._unsynced > 0
        if not batch and not sync_owed:
            return 0

        view = memoryview(b"".join(batch)[head_written:])
        written = 0
        outcome = None    # 'ok' or 'io_error'; None if something else interrupted
        error = None
        try:
            while written < len(view):
                n = self._f.write(view[written:])
                if not n:
                    raise OSError("short write: the device accepted no bytes")
                written += n
            os.fsync(self._f.fileno())
            outcome = 'ok'
        except OSError as exc:
            error = exc
            outcome = 'io_error'
        finally:
            # Committed whatever interrupted the drain. Bytes the OS
            # accepted are in the file whether or not the fsync then
            # succeeded, and writing them again would duplicate them.
            done, first_failure, recovered = self._commit(batch, head_written, written,
                                                          outcome, error)

        if recovered:
            print(f"warning: {self.path}: writes recovered after "
                  f"{self.write_failures} failure(s)", file=sys.stderr)
        elif first_failure:
            print(f"warning: {self.path}: write failed ({error!r}); "
                  f"{len(batch) - done} record(s) kept queued for retry, "
                  f"{self.records_unsynced} written but not yet confirmed by fsync",
                  file=sys.stderr)
        return done

    def _commit(self, batch, head_written, written, outcome, error):
        with self._lock:
            remaining = head_written + written
            done = 0
            for rec in batch:
                if remaining < len(rec):
                    break
                remaining -= len(rec)
                done += 1
            for _ in range(done):
                self._queue.popleft()
            self._head_written = remaining
            self.records_written += done
            self.bytes_written += written
            self.file_bytes += written
            first_failure = recovered = False
            if outcome == 'ok':
                self._unsynced = 0
                if batch:
                    self.batches_written += 1
                recovered = self._failing
                self._failing = False
            else:
                self._unsynced += done
                if outcome == 'io_error':
                    self.write_failures += 1
                    self.last_error = repr(error)
                    first_failure = not self._failing
                    self._failing = True
            return done, first_failure, recovered

    def counters(self):
        """All counters as a dict, for telemetry and the end-of-run summary.

        card_mounted and preallocated_bytes_remaining are None here: the
        laptop has no card. The ESP32 port fills them (see the port
        contract in the module docstring), so the dict has the same keys
        on both sides.
        """
        with self._lock:
            return {
                'records_enqueued': self.records_enqueued,
                'records_pending': len(self._queue),
                'records_written': self.records_written,
                'records_unsynced': self._unsynced,
                'records_dropped_queue_full': self.records_dropped_queue_full,
                'records_lost_at_close': self.records_lost_at_close,
                'batches_written': self.batches_written,
                'write_failures': self.write_failures,
                'bytes_written': self.bytes_written,
                'file_bytes': self.file_bytes,
                'card_mounted': None,
                'preallocated_bytes_remaining': None,
            }

    def close(self):
        """Refuse new records, drain what is queued (retrying an owed
        fsync), then close. Anything a failing device still refuses is
        counted in records_lost_at_close and reported.

        Holds the drain lock throughout, so a writer thread's concurrent
        drain() waits and then finds the writer closed."""
        with self._drain_lock:
            with self._lock:
                if self.closed:
                    return
                self.closed = True
            self._drain_locked()
            with self._lock:
                lost = len(self._queue)
                self.records_lost_at_close += lost
                self._queue.clear()
                self._head_written = 0
                unsynced = self._unsynced
            self._f.close()
        if lost:
            print(f"warning: {self.path}: {lost} record(s) could not be written "
                  f"before close and are lost ({self.last_error})", file=sys.stderr)
        if unsynced:
            print(f"warning: {self.path}: {unsynced} record(s) are in the file but "
                  f"were never confirmed by fsync ({self.last_error})", file=sys.stderr)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


def _trailing_padding(tail):
    """Length of the run of 0x00 or 0xFF bytes that ends `tail`, counted
    as erased preallocated space when it is the whole tail or at least a
    record long. A shorter run after other unreadable bytes is more likely
    part of a torn record, and stays counted as skipped."""
    for fill in (b"\x00", b"\xff"):
        stripped = tail.rstrip(fill)
        run = len(tail) - len(stripped)
        if run and (not stripped or run >= RECORD_SIZE):
            return run
    return 0


def read_frames(path, stats=None):
    """Yield FrameRecord objects from a log file written by LogWriter.

    Scans for RECORD_MAGIC rather than trusting fixed offsets, so a torn
    write anywhere in the file (not just at the end) is contained to the
    record(s) it touched: everything before and after reads back cleanly.
    A trailing partial record (interrupted session, still writing when
    power was lost) is silently within spec, not an error, since there is
    nothing to recover; anything else unreadable is reported, once, as a
    total byte count, per "never silently drop data" -- the caller is told
    something was lost, even though a corrupt binary record cannot be kept
    the way a suspect CSV row can.

    Three conditions are counted separately from corruption. Each is a
    warning, because each has a benign and a damaging explanation that the
    bytes alone cannot tell apart:

      * Trailing padding: the file ends in a run of 0x00 or 0xFF. On an SD
        session file that was never truncated (power lost before landing)
        this is unused preallocated space. On a file that was never
        preallocated it is damage: some filesystems read back a
        committed-but-unwritten extension as zeros after a crash, which is
        a lost write.
      * time_boot_ms going backwards. Within one session it never does. It
        happens after a flight controller reboot, or when the reader has
        walked into records an earlier flight left in unerased
        preallocated space.
      * time_boot_ms jumping forward by more than TIME_GAP_MS: a logging
        gap (attitude or LiDAR out, so no paired frames), or stale records
        from an earlier flight whose timestamps happen to be later. A
        stale record that lands close after the last real one passes this
        check too, which is why the ESP32 port must erase its
        preallocated range at boot.
    In every case the records are still yielded (flagged, not filtered).

    stats : dict or None
        If given, filled with 'records', 'skipped_bytes',
        'trailing_partial', 'padding_bytes', 'time_backwards' and
        'time_gaps' once the generator is exhausted, for callers and
        tests that need the numbers rather than the stderr warnings.
    """
    with open(path, "rb") as f:
        header = f.read(len(FILE_MAGIC))
        if header != FILE_MAGIC:
            raise ValueError(f"{path}: not a Pillar B log (missing or bad file header)")
        data = f.read()

    pos = 0
    n = len(data)
    skipped_bytes = 0
    trailing_partial = 0
    padding_bytes = 0
    time_backwards = 0
    time_gaps = 0
    n_records = 0
    last_t = None

    while pos < n:
        idx = data.find(RECORD_MAGIC, pos)
        if idx == -1:
            tail = data[pos:]
            pad = _trailing_padding(tail)
            padding_bytes += pad
            skipped_bytes += len(tail) - pad
            break
        skipped_bytes += idx - pos

        if idx + RECORD_SIZE > n:
            trailing_partial = n - idx
            break

        chunk = data[idx:idx + RECORD_SIZE]
        record = _unpack_record(chunk)
        if record is None:
            skipped_bytes += 1
            pos = idx + 1   # resync: try the next byte, this magic hit was not real
            continue

        if last_t is not None:
            if record.time_boot_ms < last_t:
                time_backwards += 1
            elif record.time_boot_ms - last_t > TIME_GAP_MS:
                time_gaps += 1
        last_t = record.time_boot_ms
        n_records += 1
        yield record
        pos = idx + RECORD_SIZE

    if stats is not None:
        stats.update(records=n_records, skipped_bytes=skipped_bytes,
                     trailing_partial=trailing_partial, padding_bytes=padding_bytes,
                     time_backwards=time_backwards, time_gaps=time_gaps)

    if skipped_bytes:
        print(f"warning: {path}: skipped {skipped_bytes} unreadable byte(s) "
             f"(corrupt or misaligned records)", file=sys.stderr)
    if trailing_partial:
        print(f"warning: {path}: {trailing_partial} trailing byte(s) after the last "
             f"full record, consistent with a session interrupted mid-write", file=sys.stderr)
    if padding_bytes:
        print(f"warning: {path}: {padding_bytes} trailing 0x00/0xFF byte(s) after the last "
              f"record: unused preallocated space if this is an SD log that was never "
              f"truncated, otherwise zero-filled damage (a lost write)", file=sys.stderr)
    if time_backwards:
        print(f"warning: {path}: time_boot_ms went backwards {time_backwards} time(s): "
              f"flight controller reboot, or records from an earlier session in "
              f"unerased preallocated space. Records kept; check before trusting.",
              file=sys.stderr)
    if time_gaps:
        print(f"warning: {path}: time_boot_ms jumped forward by more than {TIME_GAP_MS} ms "
              f"{time_gaps} time(s): a logging gap, or records from an earlier session "
              f"in unerased preallocated space. Records kept; check before trusting.",
              file=sys.stderr)


def frame_record_to_transform_inputs(record):
    """Convert a stored FrameRecord into (frame, height_m, R_nb): the
    inputs centroid_detector.detect_centroid() and
    pillar_b.pixel_to_ground_ned() expect.

    height_m is the LiDAR slant range projected onto the vertical. The
    LiDAR points along body +z regardless of camera tilt (nadir-fixed), so
    that projection is slant_range * R_nb[2, 2]: the nav-frame z-component
    of the body +z axis, i.e. cos(total tilt from vertical). This is
    exactly what pillar_b.lidar_range_to_height(slant_range, roll, pitch)
    computes, without needing roll and pitch pulled back out of the
    quaternion; test_logformat.py checks the two agree numerically across
    random attitudes.
    """
    R_nb = pb.quaternion_to_rotation(record.quat)
    height_m = record.lidar_distance_m * R_nb[2, 2]
    return record.frame, height_m, R_nb


def to_csv(log_path, csv_path):
    """Convert a binary log to CSV, one row per frame, for reading without
    this module. Pixels are flattened row-major (v-major) into 768
    columns px0000..px0767, matching frame.reshape(-1) with frame shape
    (24, 32)."""
    fieldnames = (["time_boot_ms", "subpage", "lidar_distance_m", "lidar_strength",
                  "quat_w", "quat_x", "quat_y", "quat_z",
                  "lat_deg", "lon_deg", "alt_m"]
                 + [f"px{i:04d}" for i in range(N_PIXELS)])
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(fieldnames)
        for rec in read_frames(log_path):
            writer.writerow(
                [rec.time_boot_ms, rec.subpage, rec.lidar_distance_m, rec.lidar_strength,
                *rec.quat.tolist(), rec.lat_deg, rec.lon_deg, rec.alt_m]
                + rec.frame.reshape(-1).tolist()
            )


if __name__ == "__main__":
    ap_path = sys.argv[1] if len(sys.argv) > 1 else None
    if ap_path is None:
        sys.exit("usage: python logformat.py <log_path> [csv_out_path]")
    out_path = sys.argv[2] if len(sys.argv) > 2 else ap_path + ".csv"
    to_csv(ap_path, out_path)
    print(f"wrote {out_path}")
