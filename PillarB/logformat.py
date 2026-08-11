"""
logformat.py

Raw per-frame logging format for Pillar B. One record per thermal
frame: timestamp, all 768 raw pixel values, LiDAR distance and strength,
attitude quaternion, GPS position, and the MLX90640 subpage index.

Format
------
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

Every write is flushed and fsync'd immediately: append-safe means the
record just written is durable before write_frame() returns, not just
buffered.
"""

import csv
import os
import struct
import sys
import zlib
from collections import namedtuple

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


class LogWriter:
    """Append-only writer. One write_frame() call is one durable record.

    time_boot_ms must be the MAVLink ATTITUDE_QUATERNION time_boot_ms for
    this frame, not the ESP32's arrival time for the frame or the attitude
    message. Arrival time carries transmission latency that varies frame
    to frame, and modelling that latency is what T6 exists for; folding it
    invisibly into the logged timestamp would make that impossible to
    recover later.
    """

    def __init__(self, path):
        self.path = path
        new_file = not os.path.exists(path) or os.path.getsize(path) == 0
        self._f = open(path, "ab")
        if new_file:
            self._f.write(FILE_MAGIC)
            self._f.flush()
            os.fsync(self._f.fileno())

    def write_frame(self, time_boot_ms, subpage, pixels, lidar_distance_m,
                    lidar_strength, quat, lat_deg, lon_deg, alt_m):
        record = _pack_record(time_boot_ms, subpage, pixels, lidar_distance_m,
                              lidar_strength, quat, lat_deg, lon_deg, alt_m)
        self._f.write(record)
        self._f.flush()
        os.fsync(self._f.fileno())

    def close(self):
        self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


def read_frames(path):
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

    while pos < n:
        idx = data.find(RECORD_MAGIC, pos)
        if idx == -1:
            skipped_bytes += n - pos
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

        yield record
        pos = idx + RECORD_SIZE

    if skipped_bytes:
        print(f"warning: {path}: skipped {skipped_bytes} unreadable byte(s) "
             f"(corrupt or misaligned records)", file=sys.stderr)
    if trailing_partial:
        print(f"warning: {path}: {trailing_partial} trailing byte(s) after the last "
             f"full record, consistent with a session interrupted mid-write", file=sys.stderr)


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
