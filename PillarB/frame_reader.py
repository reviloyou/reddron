# The MLX90640 compensation in this file, the code under the headings
# "Melexis parameter extraction" and "Melexis per-frame calculations", is
# a Python port of functions/MLX90640_API.c from melexis/mlx90640-library,
# used under the Apache License 2.0. The original carries this notice:
#
#   @copyright (C) 2017 Melexis N.V.
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
# Changed from the original: translated from C to Python, with the
# departures listed in the docstring below. The full licence text is in
# testdata/LICENSE-APACHE-2.0.

"""
frame_reader.py

Reads the raw MLX90640 stream that firmware/src/mlx90640_stream.cpp
sends over USB, runs the Melexis temperature compensation on the laptop,
and yields frames with the same shape synthetic_frames.py produces: a
(24, 32) float array, row v, column u. It replaces synthetic_frames.py as
a data source; nothing downstream changes.

Why compensation happens here and not on the ESP32: the ESP32 does less
work in its hot loop, and the raw measurement stays recoverable. A
compensation bug found later means re-running this file over a saved raw
capture, not re-flying. Pass save_path to iter_serial() to keep one.

The compensation is a line-by-line port of the Melexis reference driver,
melexis/mlx90640-library, functions/MLX90640_API.c at commit
f6be7ca1d4a55146b705f3d347f84b773b29cc86 (Apache-2.0). Function names
below map one to one onto the C (extract_parameters is
MLX90640_ExtractParameters and so on). Parameter extraction reproduces
the C's float32 rounding at every store, because its results are
quantized to 16- and 8-bit integers where a different rounding could
flip a value. calculate_to() runs in float64: against the C's float32 it
differs by about 1e-4 K, far below the sensor's own noise.

Deliberate departures from the C, each flagged where it happens:
  * The C's rescaling loops never terminate when every alpha is
    non-positive or every kta or kv is zero (a corrupt EEPROM dump). Here
    they stop after _MAX_SCALE_STEPS and the EEPROM is reported invalid.
  * MLX90640_GetFrameData silently keeps the previous auxiliary words
    when the new ones read 0x7FFF. That behaviour is kept, because it is
    what the reference does, but it is counted (aux_invalid) instead of
    silent.

Things this file cannot settle and that the bench must:
  * Orientation. The frame is the sensor's native row-major order,
    pixel p at row p // 32, column p % 32. Whether native row 0 is the
    nose side (pillar_b's v = 0) depends on how the sensor is mounted
    and whether the optics invert the image. Hold something warm ahead
    of the aircraft and check it appears near row 0 before trusting a
    single ground position; HARDWARE_GUIDE.md has the step.
  * Timestamps. time_us is on the ESP32's clock and marks when the
    subpage's data became ready, which is the END of its measurement. It
    is passed on raw: Orchestrator.on_subpage_esp() moves it to the
    middle of the exposure and onto the flight controller's clock at
    lookup time (clock_sync.py). What the bench still has to settle is
    whether the exposure really spans the whole subpage period, which is
    what that correction assumes (CODE_WALKTHROUGH.md section 7).

Attitude pairs
--------------
The pillarb_stream firmware build runs the MAVLink link in the same
program as the camera and sends every attitude message it parses as an
MLXA packet: the message's own time_boot_ms and the ESP32's micros() when
it was parsed. Both clocks, so the offset between them can be estimated
now and re-estimated later from a saved capture. feed_events() returns
these AttitudePair events interleaved with the Subpages in stream order;
drive() hands both to an orchestrator. The mlx90640_stream build sends
no MLXA packets and nothing changes for it.
"""

import struct
import sys
import zlib
from collections import namedtuple

import numpy as np

import clock_sync as cs

# --------------------------------------------------------------------------
# Stream format. Must match firmware/src/mlx90640_stream.cpp.
# --------------------------------------------------------------------------

MAGIC_SUBPAGE = b"MLXS"
MAGIC_EEPROM = b"MLXE"
MAGIC_TEXT = b"MLXT"
MAGIC_ATTITUDE = b"MLXA"
_MAGICS = {MAGIC_SUBPAGE: "subpage", MAGIC_EEPROM: "eeprom", MAGIC_TEXT: "text",
           MAGIC_ATTITUDE: "attitude"}

N_RAM_WORDS = 832      # 768 pixels at 0x0400, then 64 auxiliary words at 0x0700
N_EE_WORDS = 832       # 0x2400..0x273F
TEXT_MAX = 200

_SUBPAGE_FMT = struct.Struct(f"<IIIIIIHH{N_RAM_WORDS}H")
_EEPROM_FMT = struct.Struct(f"<I{N_EE_WORDS}H")
# seq, rx_us, time_boot_ms, q w x y z, flags (bit 0: Euler fallback),
# wire_bytes, reserved
_ATTITUDE_FMT = struct.Struct("<III4fBBH")
SUBPAGE_PAYLOAD = _SUBPAGE_FMT.size    # 1692
EEPROM_PAYLOAD = _EEPROM_FMT.size      # 1668
ATTITUDE_PAYLOAD = _ATTITUDE_FMT.size  # 32
BAUD = 921600


def refresh_rate_hz(control):
    """Per-subpage refresh rate from control register 0x800D, bits 7..9:
    code 0 is 0.5 Hz, each step doubles, code 4 is 8 Hz."""
    return 0.5 * 2 ** ((int(control) >> 7) & 0x7)

# Melexis example values: emissivity for a generic surface, and the
# open-air offset between the sensor's own ambient temperature and the
# reflected temperature (TA_SHIFT in the Melexis examples).
EMISSIVITY = 0.95
TA_SHIFT = 8.0


def _frame(magic, payload):
    body = struct.pack("<H", len(payload)) + payload
    return magic + body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def encode_subpage_packet(seq, t_ready_us, status, control, ram, read_us=0,
                          i2c_errors=0, missed_subpages=0, tx_dropped=0):
    """The bytes the firmware sends for one subpage. Used to build test
    streams and simulations; the firmware is the authority."""
    ram = np.asarray(ram, dtype=np.uint16).reshape(-1)
    if ram.size != N_RAM_WORDS:
        raise ValueError(f"expected {N_RAM_WORDS} RAM words, got {ram.size}")
    payload = _SUBPAGE_FMT.pack(seq & 0xFFFFFFFF, t_ready_us & 0xFFFFFFFF, read_us,
                                i2c_errors, missed_subpages, tx_dropped,
                                status & 0xFFFF, control & 0xFFFF, *ram.tolist())
    return _frame(MAGIC_SUBPAGE, payload)


def encode_eeprom_packet(seq, ee):
    ee = np.asarray(ee, dtype=np.uint16).reshape(-1)
    if ee.size != N_EE_WORDS:
        raise ValueError(f"expected {N_EE_WORDS} EEPROM words, got {ee.size}")
    return _frame(MAGIC_EEPROM, _EEPROM_FMT.pack(seq & 0xFFFFFFFF, *ee.tolist()))


def encode_text_packet(text):
    return _frame(MAGIC_TEXT, text.encode("ascii", errors="replace")[:TEXT_MAX])


def encode_attitude_packet(seq, rx_us, time_boot_ms, quat, from_euler=False,
                           wire_bytes=cs.ATTITUDE_QUATERNION_WIRE_BYTES):
    """The bytes the pillarb_stream firmware sends for one parsed attitude
    message."""
    w, x, y, z = (float(v) for v in quat)
    return _frame(MAGIC_ATTITUDE, _ATTITUDE_FMT.pack(
        seq & 0xFFFFFFFF, rx_us & 0xFFFFFFFF, time_boot_ms & 0xFFFFFFFF, w, x, y, z,
        1 if from_euler else 0, wire_bytes & 0xFF, 0))


class StreamParser:
    """Splits a byte stream into packets. Scans for a magic, checks the
    length against the packet type and the CRC, and resyncs one byte
    further on any failure, so a dropped or corrupted byte costs the
    packet it landed in and nothing else.

    Counters: bytes_received, bytes_discarded (every byte not inside a
    valid packet), crc_failures, length_errors, trailing_partial (bytes
    of an incomplete packet left at finish()), and packets per type. A
    rising discard rate is how a bad cable is diagnosed rather than
    guessed at.
    """

    def __init__(self):
        self._buf = bytearray()
        self.bytes_received = 0
        self.bytes_discarded = 0
        self.crc_failures = 0
        self.length_errors = 0
        self.trailing_partial = 0
        self.packets = {"subpage": 0, "eeprom": 0, "text": 0, "attitude": 0}

    @staticmethod
    def _length_ok(kind, length):
        if kind == "subpage":
            return length == SUBPAGE_PAYLOAD
        if kind == "eeprom":
            return length == EEPROM_PAYLOAD
        if kind == "attitude":
            return length == ATTITUDE_PAYLOAD
        return length <= TEXT_MAX

    def feed(self, data):
        """Returns a list of (kind, payload bytes) for every complete,
        valid packet now available."""
        buf = self._buf
        buf += data
        self.bytes_received += len(data)
        out = []
        pos = 0
        while True:
            hits = [i for i in (buf.find(m, pos) for m in _MAGICS) if i >= 0]
            if not hits:
                # A magic may straddle the end of this chunk: keep 3 bytes.
                keep = min(3, len(buf) - pos)
                self.bytes_discarded += len(buf) - pos - keep
                pos = len(buf) - keep
                break
            idx = min(hits)
            self.bytes_discarded += idx - pos
            pos = idx
            if len(buf) - pos < 6:
                break
            kind = _MAGICS[bytes(buf[pos:pos + 4])]
            length = buf[pos + 4] | (buf[pos + 5] << 8)
            if not self._length_ok(kind, length):
                self.length_errors += 1
                self.bytes_discarded += 1
                pos += 1
                continue
            total = 6 + length + 4
            if len(buf) - pos < total:
                break
            crc = struct.unpack_from("<I", buf, pos + 6 + length)[0]
            if zlib.crc32(buf[pos + 4:pos + 6 + length]) & 0xFFFFFFFF != crc:
                self.crc_failures += 1
                self.bytes_discarded += 1
                pos += 1
                continue
            out.append((kind, bytes(buf[pos + 6:pos + 6 + length])))
            self.packets[kind] += 1
            pos += total
        del buf[:pos]
        return out

    def finish(self):
        """End of stream: whatever is left is an incomplete packet."""
        self.trailing_partial = len(self._buf)
        self._buf.clear()


# --------------------------------------------------------------------------
# Melexis parameter extraction (MLX90640_ExtractParameters and helpers).
# --------------------------------------------------------------------------

F32 = np.float32
_MAX_SCALE_STEPS = 64
SCALEALPHA = 0.000001

MLX90640_NO_ERROR = 0
MLX90640_BROKEN_PIXELS_NUM_ERROR = 3
MLX90640_OUTLIER_PIXELS_NUM_ERROR = 4
MLX90640_BAD_PIXELS_NUM_ERROR = 5
MLX90640_ADJACENT_BAD_PIXELS_ERROR = 6


def _ms(w):
    return (int(w) >> 8) & 0xFF


def _ls(w):
    return int(w) & 0xFF


def _nib(w, k):
    """MLX90640_NIBBLEk: k = 1 is the least significant nibble."""
    return (int(w) >> (4 * (k - 1))) & 0xF


def _signed(v, bits):
    return v - (1 << bits) if v >= (1 << (bits - 1)) else v


def _wrap16(a):
    """Store into int16_t, two's complement wrap."""
    a = np.asarray(a, dtype=np.int64)
    return (((a + 32768) % 65536) - 32768).astype(np.int16)


class MLX90640Params:
    """paramsMLX90640, one attribute per struct field, same names."""

    def __repr__(self):
        return (f"MLX90640Params(kVdd={self.kVdd}, vdd25={self.vdd25}, "
                f"KtPTAT={self.KtPTAT}, vPTAT25={self.vPTAT25}, gainEE={self.gainEE}, "
                f"resolutionEE={self.resolutionEE}, calibrationModeEE={self.calibrationModeEE}, "
                f"broken={self.broken_pixel_list()}, outlier={self.outlier_pixel_list()})")

    def broken_pixel_list(self):
        return [int(p) for p in self.brokenPixels if p != 0xFFFF]

    def outlier_pixel_list(self):
        return [int(p) for p in self.outlierPixels if p != 0xFFFF]


_PIX = np.arange(768)
_ROW = _PIX // 32
_COL = _PIX % 32
_SPLIT = 2 * (_PIX // 32 - (_PIX // 64) * 2) + _PIX % 2


def _row_col_nibbles(ee, first_row_word, first_col_word):
    rows = np.array([_signed(_nib(ee[first_row_word + k // 4], k % 4 + 1), 4) for k in range(24)])
    cols = np.array([_signed(_nib(ee[first_col_word + k // 4], k % 4 + 1), 4) for k in range(32)])
    return rows, cols


def _rescale_to_int8(temp_arr):
    """The shared tail of ExtractKtaPixelParameters and
    ExtractKvPixelParameters: find the largest power of two that brings
    the biggest magnitude up to 63.4, then round half away from zero."""
    temp = F32(np.max(np.abs(temp_arr)))
    scale = 0
    while float(temp) < 63.4:
        if scale >= _MAX_SCALE_STEPS:     # departure: the C loops forever here
            raise ValueError("EEPROM kta/kv data invalid (all zero)")
        temp = F32(temp * F32(2))
        scale += 1
    t = (temp_arr.astype(np.float64) * 2.0 ** scale).astype(F32).astype(np.float64)
    q = np.where(t < 0, np.trunc(t - 0.5), np.trunc(t + 0.5))
    return q.astype(np.int8), scale


def _extract_vdd(ee, p):
    kVdd = _signed(_ms(ee[51]), 8)
    vdd25 = _ls(ee[51])
    vdd25 = ((vdd25 - 256) << 5) - 8192
    p.kVdd = int(_wrap16(32 * kVdd))
    p.vdd25 = int(_wrap16(vdd25))


def _extract_ptat(ee, p):
    KvPTAT = (int(ee[50]) & 0xFC00) >> 10
    if KvPTAT > 31:
        KvPTAT -= 64
    p.KvPTAT = F32(F32(KvPTAT) / F32(4096))
    KtPTAT = int(ee[50]) & 0x03FF
    if KtPTAT > 511:
        KtPTAT -= 1024
    p.KtPTAT = F32(F32(KtPTAT) / F32(8))
    # int16_t local stored into the uint16_t struct field: the raw word.
    p.vPTAT25 = int(ee[49]) & 0xFFFF
    p.alphaPTAT = F32((int(ee[16]) & 0xF000) / 2.0 ** 14 + 8.0)


def _extract_misc(ee, p):
    p.gainEE = int(_wrap16(int(ee[48])))
    p.tgc = F32(F32(_signed(_ls(ee[60]), 8)) / F32(32.0))
    p.resolutionEE = (int(ee[56]) & 0x3000) >> 12
    p.KsTa = F32(F32(_signed(_ms(ee[60]), 8)) / F32(8192.0))


def _extract_ksto(ee, p):
    step = ((int(ee[63]) & 0x3000) >> 12) * 10
    ct = [-40, 0, _nib(ee[63], 2), _nib(ee[63], 3), 400]
    ct[2] = ct[2] * step
    ct[3] = ct[2] + ct[3] * step
    p.ct = [int(c) for c in _wrap16(ct)]
    scale = F32(1 << (_nib(ee[63], 1) + 8))
    p.ksTo = np.array([F32(_signed(_ls(ee[61]), 8)) / scale,
                       F32(_signed(_ms(ee[61]), 8)) / scale,
                       F32(_signed(_ls(ee[62]), 8)) / scale,
                       F32(_signed(_ms(ee[62]), 8)) / scale,
                       -0.0002], dtype=F32)


def _extract_cp(ee, p):
    alpha_scale = _nib(ee[32], 4) + 27
    off0 = int(ee[58]) & 0x03FF
    if off0 > 511:
        off0 -= 1024
    off1 = (int(ee[58]) & 0xFC00) >> 10
    if off1 > 31:
        off1 -= 64
    off1 = off1 + off0
    a0 = int(ee[57]) & 0x03FF
    if a0 > 511:
        a0 -= 1024
    a0 = F32(a0 / 2.0 ** alpha_scale)
    a1 = (int(ee[57]) & 0xFC00) >> 10
    if a1 > 31:
        a1 -= 64
    a1 = F32(F32(F32(1) + F32(F32(a1) / F32(128))) * a0)
    p.cpKta = F32(_signed(_ls(ee[59]), 8) / 2.0 ** (_nib(ee[56], 2) + 8))
    p.cpKv = F32(_signed(_ms(ee[59]), 8) / 2.0 ** _nib(ee[56], 3))
    p.cpAlpha = np.array([a0, a1], dtype=F32)
    p.cpOffset = [int(v) for v in _wrap16([off0, off1])]


def _extract_alpha(ee, p):
    acc_rem = _nib(ee[32], 1)
    acc_col_scale = _nib(ee[32], 2)
    acc_row_scale = _nib(ee[32], 3)
    alpha_scale = _nib(ee[32], 4) + 30
    alpha_ref = int(ee[33])
    acc_row, acc_col = _row_col_nibbles(ee, 34, 40)

    raw = ((ee[64:64 + 768].astype(np.int64) & 0x03F0) >> 4)
    raw = np.where(raw > 31, raw - 64, raw)
    at = raw.astype(F32) * F32(1 << acc_rem)
    # (int + int + int) is summed as int, then converted and added in float.
    int_sum = alpha_ref + acc_row[_ROW] * (1 << acc_row_scale) + acc_col[_COL] * (1 << acc_col_scale)
    at = int_sum.astype(F32) + at
    at = (at.astype(np.float64) / 2.0 ** alpha_scale).astype(F32)
    at = at - F32(F32(p.tgc * F32(p.cpAlpha[0] + p.cpAlpha[1])) / F32(2))
    with np.errstate(divide="ignore"):
        at = (SCALEALPHA / at.astype(np.float64)).astype(F32)

    temp = F32(np.max(at))
    scale = 0
    while float(temp) < 32767.4:
        if scale >= _MAX_SCALE_STEPS:     # departure: the C loops forever here
            raise ValueError("EEPROM alpha data invalid (no positive sensitivity)")
        temp = F32(temp * F32(2))
        scale += 1
    t = (at.astype(np.float64) * 2.0 ** scale).astype(F32).astype(np.float64)
    p.alpha = np.floor(t + 0.5).astype(np.int64).astype(np.uint16)
    p.alphaScale = scale


def _extract_offset(ee, p):
    occ_rem = _nib(ee[16], 1)
    occ_col_scale = _nib(ee[16], 2)
    occ_row_scale = _nib(ee[16], 3)
    offset_ref = int(_wrap16(int(ee[17])))
    occ_row, occ_col = _row_col_nibbles(ee, 18, 24)
    raw = (ee[64:64 + 768].astype(np.int64) & 0xFC00) >> 10
    raw = np.where(raw > 31, raw - 64, raw)
    raw = _wrap16(raw * (1 << occ_rem)).astype(np.int64)
    p.offset = _wrap16(offset_ref + occ_row[_ROW] * (1 << occ_row_scale)
                       + occ_col[_COL] * (1 << occ_col_scale) + raw)


def _extract_kta(ee, p):
    kta_rc = np.array([_signed(_ms(ee[54]), 8), _signed(_ms(ee[55]), 8),
                       _signed(_ls(ee[54]), 8), _signed(_ls(ee[55]), 8)])
    scale1 = _nib(ee[56], 2) + 8
    scale2 = _nib(ee[56], 1)
    raw = (ee[64:64 + 768].astype(np.int64) & 0x000E) >> 1
    raw = np.where(raw > 3, raw - 8, raw)
    kt = raw.astype(F32) * F32(1 << scale2)
    kt = kta_rc[_SPLIT].astype(F32) + kt
    kt = (kt.astype(np.float64) / 2.0 ** scale1).astype(F32)
    p.kta, p.ktaScale = _rescale_to_int8(kt)


def _extract_kv(ee, p):
    kv_t = np.array([_signed(_nib(ee[52], 4), 4), _signed(_nib(ee[52], 2), 4),
                     _signed(_nib(ee[52], 3), 4), _signed(_nib(ee[52], 1), 4)])
    scale = _nib(ee[56], 3)
    kv = (kv_t[_SPLIT].astype(F32).astype(np.float64) / 2.0 ** scale).astype(F32)
    p.kv, p.kvScale = _rescale_to_int8(kv)


def _extract_cilc(ee, p):
    mode = (int(ee[10]) & 0x0800) >> 4
    p.calibrationModeEE = mode ^ 0x80
    c0 = int(ee[53]) & 0x003F
    if c0 > 31:
        c0 -= 64
    c1 = (int(ee[53]) & 0x07C0) >> 6
    if c1 > 15:
        c1 -= 32
    c2 = (int(ee[53]) & 0xF800) >> 11
    if c2 > 15:
        c2 -= 32
    p.ilChessC = np.array([F32(c0) / F32(16.0), F32(c1) / F32(2.0), F32(c2) / F32(8.0)],
                          dtype=F32)


def _check_adjacent(pix1, pix2):
    lp1, lp2 = pix1 >> 5, pix2 >> 5
    cp1, cp2 = pix1 - (lp1 << 5), pix2 - (lp2 << 5)
    if -2 < lp1 - lp2 < 2 and -2 < cp1 - cp2 < 2:
        return -MLX90640_ADJACENT_BAD_PIXELS_ERROR
    return 0


def _extract_deviating_pixels(ee, p):
    broken = [0xFFFF] * 5
    outlier = [0xFFFF] * 5
    n_broken = n_outlier = 0
    pix = 0
    while pix < 768 and n_broken < 5 and n_outlier < 5:
        word = int(ee[pix + 64])
        if word == 0:
            broken[n_broken] = pix
            n_broken += 1
        elif word & 0x0001:
            outlier[n_outlier] = pix
            n_outlier += 1
        pix += 1
    p.brokenPixels = broken
    p.outlierPixels = outlier

    if n_broken > 4:
        return -MLX90640_BROKEN_PIXELS_NUM_ERROR
    if n_outlier > 4:
        return -MLX90640_OUTLIER_PIXELS_NUM_ERROR
    if n_broken + n_outlier > 4:
        return -MLX90640_BAD_PIXELS_NUM_ERROR
    for a in range(n_broken):
        for b in range(a + 1, n_broken):
            warn = _check_adjacent(broken[a], broken[b])
            if warn:
                return warn
    for a in range(n_outlier):
        for b in range(a + 1, n_outlier):
            warn = _check_adjacent(outlier[a], outlier[b])
            if warn:
                return warn
    for a in range(n_broken):
        for b in range(n_outlier):
            warn = _check_adjacent(broken[a], outlier[b])
            if warn:
                return warn
    return MLX90640_NO_ERROR


def extract_parameters(ee):
    """MLX90640_ExtractParameters. Returns (params, error), error being
    0 or the C's negative deviating-pixel code. The call order matters
    and is the C's: CP before alpha, because alpha uses cpAlpha and tgc."""
    ee = np.asarray(ee, dtype=np.uint16).reshape(-1)
    if ee.size != N_EE_WORDS:
        raise ValueError(f"expected {N_EE_WORDS} EEPROM words, got {ee.size}")
    p = MLX90640Params()
    _extract_vdd(ee, p)
    _extract_ptat(ee, p)
    _extract_misc(ee, p)      # gain, tgc, resolution, KsTa, in the C's order
    _extract_ksto(ee, p)
    _extract_cp(ee, p)
    _extract_alpha(ee, p)
    _extract_offset(ee, p)
    _extract_kta(ee, p)
    _extract_kv(ee, p)
    _extract_cilc(ee, p)
    error = _extract_deviating_pixels(ee, p)
    return p, error


# --------------------------------------------------------------------------
# Melexis per-frame calculations.
# --------------------------------------------------------------------------

def get_vdd(frame, p):
    """MLX90640_GetVdd. frame is the 834-word frameData."""
    resolution_ram = (int(frame[832]) & 0x0C00) >> 10
    correction = F32(2.0 ** p.resolutionEE / 2.0 ** resolution_ram)
    t = F32(correction * F32(_signed(int(frame[810]), 16)))
    t = F32(t - F32(p.vdd25))
    t = F32(t / F32(p.kVdd))
    return F32(float(t) + 3.3)


def get_ta(frame, p):
    """MLX90640_GetTa."""
    vdd = get_vdd(frame, p)
    ptat = F32(_signed(int(frame[800]), 16))
    denom = F32(F32(ptat * p.alphaPTAT) + F32(_signed(int(frame[768]), 16)))
    ptat_art = F32(float(F32(ptat / denom)) * 2.0 ** 18)
    ta = F32(float(ptat_art) / (1.0 + float(p.KvPTAT) * (float(vdd) - 3.3)) - p.vPTAT25)
    return F32(F32(ta / p.KtPTAT) + F32(25))


_IL = _PIX // 32 - (_PIX // 64) * 2
_CHESS = _IL ^ (_PIX - (_PIX // 2) * 2)
_CONVERSION = ((_PIX + 2) // 4 - (_PIX + 3) // 4 + (_PIX + 1) // 4 - _PIX // 4) * (1 - 2 * _IL)


def subpage_mask(frame):
    """Pixels this subpage updates: chess or interleaved, per the control
    register the frame was read with."""
    mode = (int(frame[832]) & 0x1000) >> 5
    pattern = _IL if mode == 0 else _CHESS
    return pattern == int(frame[833])


def calculate_to(frame, p, emissivity, tr, result):
    """MLX90640_CalculateTo. Writes object temperatures (degC) into
    result (768 floats) for this subpage's pixels only, leaving the other
    half untouched, exactly as the C does. Returns the subpage mask."""
    sub = int(frame[833])
    vdd = float(get_vdd(frame, p))
    ta = float(get_ta(frame, p))
    ta4 = (ta + 273.15) ** 4
    tr4 = (tr + 273.15) ** 4
    ta_tr = tr4 - (tr4 - ta4) / emissivity

    kta_scale = 2.0 ** p.ktaScale
    kv_scale = 2.0 ** p.kvScale
    alpha_scale = 2.0 ** p.alphaScale
    ks_to = p.ksTo.astype(np.float64)
    ct = np.array(p.ct, dtype=np.float64)
    alpha_corr = np.empty(4)
    alpha_corr[0] = 1.0 / (1.0 + ks_to[0] * 40)
    alpha_corr[1] = 1.0
    alpha_corr[2] = 1.0 + ks_to[1] * ct[2]
    alpha_corr[3] = alpha_corr[2] * (1.0 + ks_to[2] * (ct[3] - ct[2]))

    gain = float(p.gainEE) / _signed(int(frame[778]), 16)
    mode = (int(frame[832]) & 0x1000) >> 5

    common = (1.0 + float(p.cpKta) * (ta - 25)) * (1.0 + float(p.cpKv) * (vdd - 3.3))
    ir_cp = [_signed(int(frame[776]), 16) * gain, _signed(int(frame[808]), 16) * gain]
    ir_cp[0] -= p.cpOffset[0] * common
    if mode == p.calibrationModeEE:
        ir_cp[1] -= p.cpOffset[1] * common
    else:
        ir_cp[1] -= (p.cpOffset[1] + float(p.ilChessC[0])) * common

    sel = subpage_mask(frame)
    raw = frame[:768][sel].astype(np.int64)
    raw = np.where(raw > 32767, raw - 65536, raw).astype(np.float64)
    ir = raw * gain
    kta = p.kta[sel].astype(np.float64) / kta_scale
    kv = p.kv[sel].astype(np.float64) / kv_scale
    ir = ir - p.offset[sel].astype(np.float64) * (1 + kta * (ta - 25)) * (1 + kv * (vdd - 3.3))
    if mode != p.calibrationModeEE:
        ir = ir + float(p.ilChessC[2]) * (2 * _IL[sel] - 1) - float(p.ilChessC[1]) * _CONVERSION[sel]
    ir = ir - float(p.tgc) * ir_cp[sub]
    ir = ir / emissivity

    with np.errstate(divide="ignore", invalid="ignore"):
        ac = SCALEALPHA * alpha_scale / p.alpha[sel].astype(np.float64)
        ac = ac * (1 + float(p.KsTa) * (ta - 25))
        sx = ac * ac * ac * (ir + ac * ta_tr)
        sx = np.sqrt(np.sqrt(sx)) * ks_to[1]
        to = np.sqrt(np.sqrt(ir / (ac * (1 - ks_to[1] * 273.15) + sx) + ta_tr)) - 273.15
        rng = np.where(to < ct[1], 0, np.where(to < ct[2], 1, np.where(to < ct[3], 2, 3)))
        to = np.sqrt(np.sqrt(ir / (ac * alpha_corr[rng] * (1 + ks_to[rng] * (to - ct[rng])))
                             + ta_tr)) - 273.15
    result[sel] = to
    return sel


def _get_median(values):
    v = sorted(values)
    n = len(v)
    return (v[n // 2] + v[n // 2 - 1]) / 2.0 if n % 2 == 0 else v[n // 2]


def _is_pixel_bad(pixel, p):
    return any(pixel == p.outlierPixels[i] or pixel == p.brokenPixels[i] for i in range(5))


def bad_pixels_correction(pixels, to, mode, p):
    """MLX90640_BadPixelsCorrection. pixels is p.brokenPixels or
    p.outlierPixels (0xFFFF-terminated), mode 1 for chess readout, 0 for
    interleaved. Replaces each listed pixel with an estimate from its
    neighbours, in place. These are the pixels the factory calibration
    marked defective; left alone, a stuck hot pixel is a permanent false
    target."""
    for pixel in pixels:
        if pixel == 0xFFFF:
            break
        line = pixel >> 5
        column = pixel - (line << 5)
        if mode == 1:
            if line == 0:
                if column == 0:
                    to[pixel] = to[33]
                elif column == 31:
                    to[pixel] = to[62]
                else:
                    to[pixel] = (to[pixel + 31] + to[pixel + 33]) / 2.0
            elif line == 23:
                if column == 0:
                    to[pixel] = to[705]
                elif column == 31:
                    to[pixel] = to[734]
                else:
                    to[pixel] = (to[pixel - 33] + to[pixel - 31]) / 2.0
            elif column == 0:
                to[pixel] = (to[pixel - 31] + to[pixel + 33]) / 2.0
            elif column == 31:
                to[pixel] = (to[pixel - 33] + to[pixel + 31]) / 2.0
            else:
                to[pixel] = _get_median([to[pixel - 33], to[pixel - 31],
                                         to[pixel + 31], to[pixel + 33]])
        else:
            if column == 0:
                to[pixel] = to[pixel + 1]
            elif column in (1, 30):
                to[pixel] = (to[pixel - 1] + to[pixel + 1]) / 2.0
            elif column == 31:
                to[pixel] = to[pixel - 1]
            elif not _is_pixel_bad(pixel - 2, p) and not _is_pixel_bad(pixel + 2, p):
                ap0 = to[pixel + 1] - to[pixel + 2]
                ap1 = to[pixel - 1] - to[pixel - 2]
                if abs(ap0) > abs(ap1):
                    to[pixel] = to[pixel - 1] + ap1
                else:
                    to[pixel] = to[pixel + 1] + ap0
            else:
                to[pixel] = (to[pixel - 1] + to[pixel + 1]) / 2.0


# MLX90640_GetFrameData's ValidateAuxData: aux words that must not read
# 0x7FFF (the value RAM holds when a word was not measured).
_AUX_CHECKED = np.array([0] + list(range(8, 19)) + list(range(20, 23)) + list(range(24, 33))
                        + list(range(40, 51)) + list(range(52, 55)) + list(range(56, 64)))


# --------------------------------------------------------------------------
# The reader.
# --------------------------------------------------------------------------

Subpage = namedtuple("Subpage", "time_ms subpage frame seq time_us ta vdd mode")
Subpage.__doc__ = """One compensated subpage.

time_ms   ESP32 clock, when the subpage's data became ready (the end of
          its measurement). Not flight-controller time: give time_us to
          Orchestrator.on_subpage_esp(), which converts it.
subpage   0 or 1, the argument Orchestrator.on_subpage() takes.
frame     (24, 32) degC, the full image as it stands after this subpage:
          this subpage's half fresh, the other half from the previous
          subpage. The same shape synthetic_frames.create_synthetic_frame
          returns.
seq       the firmware's subpage counter.
time_us   the unwrapped microsecond timestamp time_ms comes from.
ta, vdd   the sensor's own ambient temperature and supply voltage.
mode      'chess' or 'interleaved', from the control register.
"""

AttitudePair = namedtuple("AttitudePair", "rx_us time_boot_ms quat from_euler wire_bytes seq")
AttitudePair.__doc__ = """One attitude message as the ESP32 parsed it, with both clocks.

rx_us         ESP32 micros() when the message was parsed, unwrapped on the
              same timeline as Subpage.time_us.
time_boot_ms  the flight controller's stamp, from inside the message.
quat          (4,) float, w x y z, body FRD to NED.
from_euler    True if it came from ATTITUDE (msg 30), the Euler fallback.
wire_bytes    the message's length on the wire, for the known part of its
              latency.
seq           the firmware's attitude packet counter.
"""


class FrameReader:
    """Turns stream bytes into compensated Subpage tuples and, from the
    pillarb_stream build, AttitudePair tuples.

    feed(bytes) returns the Subpages that became available; feed_events()
    returns Subpages and AttitudePairs in stream order. Subpages
    only start coming out once both halves of the image have been seen
    at least once, since before that half the frame has never been
    measured. Subpages that arrive before the first EEPROM packet are
    held (up to hold_limit) and processed as soon as it arrives. Every
    path that does not yield a subpage has a counter; see counters().
    """

    def __init__(self, emissivity=EMISSIVITY, ta_shift=TA_SHIFT,
                 correct_bad_pixels=True, hold_limit=64):
        self.emissivity = emissivity
        self.ta_shift = ta_shift
        self.correct_bad_pixels = correct_bad_pixels
        self.hold_limit = hold_limit
        self.parser = StreamParser()

        self.params = None
        self.eeprom_error = None
        self._ee = None
        self._held = []
        self._last_aux = None
        self._image = np.full(768, np.nan)
        self._seen = [False, False]
        self._last_seq = None
        self._last_att_seq = None
        self._newest_us = None

        self.texts = []
        self.subpages_yielded = 0
        self.subpages_lost = 0            # firmware seq gaps: lost on the wire or never sent
        self.firmware_restarts = 0        # seq went backwards
        self.aux_invalid = 0              # aux words read 0x7FFF, previous ones reused
        self.aux_unusable = 0             # ... and there were no previous ones: skipped
        self.frame_data_invalid = 0       # pixel lines read 0x7FFF: skipped
        self.held_before_eeprom = 0
        self.dropped_before_eeprom = 0    # hold_limit exceeded before any EEPROM arrived
        self.warmup_subpages = 0          # compensated, but the other half never measured yet
        self.eeprom_changes = 0           # a different EEPROM: a different sensor
        self.eeprom_invalid = 0           # extraction failed outright
        self.nan_pixels = 0               # compensation produced NaN (sqrt of a negative)
        self.fw_i2c_errors = 0            # the firmware's own counters, latest values
        self.fw_missed_subpages = 0
        self.fw_tx_dropped = 0
        # The sensor reports a refresh rate other than clock_sync.SUBPAGE_RATE_HZ,
        # which the exposure-midpoint correction is derived from. The subpage
        # is still yielded; this says the correction no longer fits it.
        self.refresh_rate_mismatch = 0
        self.attitude_yielded = 0
        self.attitude_lost = 0            # attitude seq gaps: USB drops, CRC failures
        self.attitude_seq_restarts = 0    # attitude seq went backwards: firmware restart
        self.attitude_from_euler = 0      # samples from the Euler fallback (msg 30)
        self.attitude_ignored_by_feed = 0 # attitude pairs feed() returned no way to use

    def counters(self):
        c = {k: getattr(self, k) for k in (
            "subpages_yielded", "subpages_lost", "firmware_restarts", "aux_invalid",
            "aux_unusable", "frame_data_invalid", "held_before_eeprom", "dropped_before_eeprom",
            "warmup_subpages", "eeprom_changes", "eeprom_invalid", "nan_pixels",
            "fw_i2c_errors", "fw_missed_subpages", "fw_tx_dropped", "refresh_rate_mismatch",
            "attitude_yielded", "attitude_lost", "attitude_seq_restarts", "attitude_from_euler",
            "attitude_ignored_by_feed")}
        c.update(bytes_received=self.parser.bytes_received,
                 bytes_discarded=self.parser.bytes_discarded,
                 crc_failures=self.parser.crc_failures,
                 length_errors=self.parser.length_errors,
                 trailing_partial=self.parser.trailing_partial,
                 packets_subpage=self.parser.packets["subpage"],
                 packets_eeprom=self.parser.packets["eeprom"],
                 packets_text=self.parser.packets["text"],
                 packets_attitude=self.parser.packets["attitude"])
        return c

    def feed(self, data):
        """The Subpages that became available. An attitude pair in the
        stream is counted in attitude_ignored_by_feed rather than returned:
        use feed_events() for a pillarb_stream capture."""
        out = []
        for ev in self.feed_events(data):
            if isinstance(ev, Subpage):
                out.append(ev)
            else:
                self.attitude_ignored_by_feed += 1
        return out

    def feed_events(self, data):
        """Subpages and AttitudePairs, in the order they arrived. The order
        is the point: the orchestrator's attitude buffer holds a bounded
        window, so attitude has to reach it interleaved with the subpages,
        as it happened. Subpages held for a missing EEPROM come out when it
        arrives, after the attitude that came meanwhile; the orchestrator
        refuses what it can then no longer serve, and counts it."""
        out = []
        for kind, payload in self.parser.feed(data):
            if kind == "text":
                msg = payload.decode("ascii", errors="replace")
                self.texts.append(msg)
                print(f"[mlx90640] {msg}", file=sys.stderr)
            elif kind == "eeprom":
                out.extend(self._on_eeprom(payload))
            elif kind == "attitude":
                out.append(self._on_attitude(payload))
            else:
                if self.params is None:
                    if len(self._held) < self.hold_limit:
                        self._held.append(payload)
                        self.held_before_eeprom += 1
                    else:
                        self.dropped_before_eeprom += 1
                    continue
                res = self._on_subpage(payload)
                if res is not None:
                    out.append(res)
        return out

    def finish(self):
        self.parser.finish()

    def _on_eeprom(self, payload):
        fields = _EEPROM_FMT.unpack(payload)
        ee = np.array(fields[1:], dtype=np.uint16)
        if self._ee is not None and np.array_equal(ee, self._ee):
            return []
        try:
            params, error = extract_parameters(ee)
        except ValueError as exc:
            self.eeprom_invalid += 1
            print(f"[frame_reader] EEPROM rejected: {exc}", file=sys.stderr)
            return []
        if self._ee is not None:
            # A different sensor: its image and aux history do not apply.
            self.eeprom_changes += 1
            self._image[:] = np.nan
            self._seen = [False, False]
            self._last_aux = None
        self._ee = ee
        self.params = params
        self.eeprom_error = error
        if error:
            print(f"[frame_reader] EEPROM deviating-pixel code {error}: "
                  f"broken {params.broken_pixel_list()}, outlier {params.outlier_pixel_list()}",
                  file=sys.stderr)
        held, self._held = self._held, []
        out = []
        for payload in held:
            res = self._on_subpage(payload)
            if res is not None:
                out.append(res)
        return out

    def _unwrap_us(self, t_raw):
        """Place a 32-bit micros() on the stream's 64-bit timeline: of the
        values it could stand for, the one nearest the newest time seen so
        far. Nearest, not only forwards, because subpages and attitude pairs
        share the clock but are not sent in stamp order: a subpage is
        stamped before its 40 ms I2C read, and attitude parsed during that
        read goes out first. Correct as long as packets are sent less than
        35 minutes (half the wrap) out of stamp order."""
        if self._newest_us is None:
            self._newest_us = t_raw
            return t_raw
        t = (self._newest_us & ~0xFFFFFFFF) + t_raw
        if t - self._newest_us > (1 << 31):
            t -= 1 << 32
        elif self._newest_us - t > (1 << 31):
            t += 1 << 32
        if t > self._newest_us:
            self._newest_us = t
        return t

    def _on_attitude(self, payload):
        seq, rx_raw, t_boot, w, x, y, z, flags, wire_bytes, _ = _ATTITUDE_FMT.unpack(payload)
        if self._last_att_seq is not None:
            if seq > self._last_att_seq:
                self.attitude_lost += seq - self._last_att_seq - 1
            else:
                self.attitude_seq_restarts += 1
                self._newest_us = None
        self._last_att_seq = seq
        if flags & 1:
            self.attitude_from_euler += 1
        self.attitude_yielded += 1
        return AttitudePair(rx_us=self._unwrap_us(rx_raw), time_boot_ms=t_boot,
                            quat=np.array([w, x, y, z]), from_euler=bool(flags & 1),
                            wire_bytes=wire_bytes, seq=seq)

    def _on_subpage(self, payload):
        fields = _SUBPAGE_FMT.unpack(payload)
        seq, t_raw, _read_us, i2c_err, missed, tx_drop, status, control = fields[:8]
        ram = np.array(fields[8:], dtype=np.uint16)
        self.fw_i2c_errors, self.fw_missed_subpages, self.fw_tx_dropped = i2c_err, missed, tx_drop
        if refresh_rate_hz(control) != cs.SUBPAGE_RATE_HZ:
            self.refresh_rate_mismatch += 1

        if self._last_seq is not None:
            if seq > self._last_seq:
                self.subpages_lost += seq - self._last_seq - 1
            else:
                self.firmware_restarts += 1
                self._newest_us = None
        self._last_seq = seq
        time_us = self._unwrap_us(t_raw)

        frame = np.empty(834, dtype=np.uint16)
        frame[:832] = ram
        frame[832] = control
        frame[833] = status & 0x0001
        sub = int(frame[833])

        aux = frame[768:832]
        if np.any(aux[_AUX_CHECKED] == 0x7FFF):
            self.aux_invalid += 1
            if self._last_aux is None:
                self.aux_unusable += 1
                return None
            frame[768:832] = self._last_aux
        else:
            self._last_aux = aux.copy()

        for line in range(24):
            if frame[line * 32] == 0x7FFF and line % 2 == sub:
                self.frame_data_invalid += 1
                return None

        p = self.params
        ta = get_ta(frame, p)
        vdd = get_vdd(frame, p)
        sel = calculate_to(frame, p, self.emissivity, float(ta) - self.ta_shift, self._image)
        self.nan_pixels += int(np.count_nonzero(np.isnan(self._image[sel])))
        mode = (int(control) & 0x1000) >> 12
        if self.correct_bad_pixels:
            bad_pixels_correction(p.brokenPixels, self._image, mode, p)
            bad_pixels_correction(p.outlierPixels, self._image, mode, p)

        self._seen[sub] = True
        if not all(self._seen):
            self.warmup_subpages += 1
            return None
        self.subpages_yielded += 1
        return Subpage(time_ms=int(time_us // 1000), subpage=sub,
                       frame=self._image.reshape(24, 32).copy(), seq=seq, time_us=time_us,
                       ta=float(ta), vdd=float(vdd), mode="chess" if mode else "interleaved")


def iter_file(path, chunk_size=65536, reader=None, events=False):
    """Subpages from a raw capture saved by iter_serial(save_path=...).
    With events=True, Subpages and AttitudePairs in stream order, as
    feed_events() returns them."""
    reader = reader or FrameReader()
    feed = reader.feed_events if events else reader.feed
    with open(path, "rb") as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            yield from feed(chunk)
    reader.finish()


def iter_serial(port, baud=BAUD, save_path=None, reader=None, events=False):
    """Subpages live from the ESP32 (with events=True, AttitudePairs too).
    If save_path is given every byte received is also appended there,
    before parsing, so the capture can be re-read later with iter_file()
    whatever this version of the compensation, or of the clock estimate,
    does. Needs pyserial (already used by the LiDAR capture script)."""
    import serial   # only needed live; the tests and iter_file() do not use it

    reader = reader or FrameReader()
    feed = reader.feed_events if events else reader.feed
    sink = open(save_path, "ab") if save_path else None
    try:
        with serial.Serial(port, baud, timeout=0.2) as ser:
            while True:
                chunk = ser.read(8192)
                if not chunk:
                    continue
                if sink:
                    sink.write(chunk)
                yield from feed(chunk)
    finally:
        if sink:
            sink.close()
        reader.finish()


def drive(events, orch):
    """Hand feed_events() output to an orchestrator.Orchestrator in stream
    order: each AttitudePair to on_attitude() with both its clocks, each
    Subpage to on_subpage_esp(). Returns the detections. This module does
    not import the orchestrator; it only calls it."""
    out = []
    for ev in events:
        if isinstance(ev, AttitudePair):
            orch.on_attitude(ev.time_boot_ms, ev.quat, esp_rx_us=ev.rx_us,
                             wire_bytes=ev.wire_bytes)
        else:
            out.extend(orch.on_subpage_esp(ev.time_us, ev.subpage, ev.frame))
    return out


def _summarise(events, reader, period_s=1.0):
    """Once a second: rate, sensor state, image statistics and counters.
    This is the output HARDWARE_GUIDE.md tells you to expect. From the
    pillarb_stream build a second line follows with the attitude rate and
    the ESP32-to-flight-controller clock estimate."""
    clock = cs.ClockOffsetEstimator()
    window = []
    n_att = 0
    last_print = None
    for ev in events:
        if isinstance(ev, AttitudePair):
            n_att += 1
            clock.add(ev.rx_us, ev.time_boot_ms,
                      floor_us=cs.wire_time_us(ev.wire_bytes))
            continue
        sp = ev
        window.append(sp)
        t = sp.time_us / 1e6
        if last_print is None:
            last_print = t
        if t - last_print >= period_s:
            rate = (len(window) - 1) / (t - last_print) if len(window) > 1 else 0.0
            img = sp.frame
            hot = np.unravel_index(np.nanargmax(img), img.shape)
            c = reader.counters()
            print(f"{rate:4.1f} subpages/s  Ta {sp.ta:5.1f} C  Vdd {sp.vdd:4.2f} V  {sp.mode:<11}"
                  f" To min/med/max {np.nanmin(img):5.1f}/{np.nanmedian(img):5.1f}/"
                  f"{np.nanmax(img):5.1f} C at row {hot[0]} col {hot[1]}"
                  f" | lost {c['subpages_lost']} crc {c['crc_failures']}"
                  f" discarded {c['bytes_discarded']} B i2c {c['fw_i2c_errors']}"
                  f" tx_drop {c['fw_tx_dropped']}")
            if c['packets_attitude']:
                est = clock.estimate()
                fit = (f"ESP32 - FC {est.offset_us / 1000:+.2f} ms, drift {est.drift_ppm:+.0f} ppm"
                       if est is not None else "no clock estimate yet")
                print(f"     attitude {n_att / (t - last_print):4.1f} Hz  {fit}"
                      f" | att lost {c['attitude_lost']} euler {c['attitude_from_euler']}"
                      f" rate mismatch {c['refresh_rate_mismatch']}")
            window = [sp]
            n_att = 0
            last_print = t


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Read the MLX90640 stream from the ESP32.")
    ap.add_argument("port", nargs="?", help="serial port, e.g. COM5")
    ap.add_argument("--file", help="replay a raw capture instead of a live port")
    ap.add_argument("--save", help="append every raw byte to this file (live only)")
    args = ap.parse_args()
    rdr = FrameReader()
    if args.file:
        _summarise(iter_file(args.file, reader=rdr, events=True), rdr)
    elif args.port:
        try:
            _summarise(iter_serial(args.port, save_path=args.save, reader=rdr, events=True), rdr)
        except KeyboardInterrupt:
            pass
    else:
        ap.error("give a serial port or --file")
    print(rdr.counters(), file=sys.stderr)
