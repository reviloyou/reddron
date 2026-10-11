"""Tests for frame_reader.py (F3).

A. The Melexis port against Melexis's own example data: calibration
   parameters, per-pixel coefficients and the temperature of every pixel
   after both subpages. This is the test that says the compensation is
   right, not merely self-consistent. The data is in
   testdata/mlx90640_melexis_example.json with its provenance and licence.
B. Stream framing: CRC, resync on garbage and corruption, arbitrary
   chunking, truncation.
C. Reader robustness: lost and restarted sequences, timestamp wrap, the
   0x7FFF validity rules from MLX90640_GetFrameData, subpages that arrive
   before the EEPROM, a changed or corrupt EEPROM.
D. Bad-pixel correction.
E. Real Melexis frames through the detector and the orchestrator with
   nothing changed downstream.

What this cannot test: the ESP32 side (mlx90640_stream.cpp compiles, but
has not run against a sensor), image orientation on the airframe, and
the mapping from the ESP32 clock to the flight controller's.
"""

import json
import os
import sys
import tempfile
import zlib

import numpy as np

import centroid_detector as cd
import frame_reader as fr
import synthetic_frames as sf
from orchestrator import Orchestrator

results = []


def check(label, ok, detail=""):
    print(f"  [{'ok ' if ok else 'FAIL'}] {label}" + (f"   {detail}" if detail else ""))
    results.append(bool(ok))
    return ok


HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "testdata", "mlx90640_melexis_example.json")) as f:
    G = json.load(f)
EE = np.array(G["eeprom_words"], dtype=np.uint16)
RAM = [np.array(G["frame0_ram_words"], dtype=np.uint16),
       np.array(G["frame1_ram_words"], dtype=np.uint16)]
STATUS = [G["frame0_status"], G["frame1_status"]]
CTRL = G["control_register"]
TEMPS = np.array(G["temperatures"])
# The expected temperatures are given to 3 decimals, so 0.5 mK is their own
# rounding. The port runs in float64 where the C runs in float32, which is
# worth another fraction of a mK. 2 mK leaves room for both and nothing else.
TOL_K = 0.002
NEW_DATA = 0x0008   # the status bit the firmware saw when it read the subpage


def frame_words(which, ram=None, control=CTRL):
    ram = RAM[which] if ram is None else ram
    return np.concatenate([ram, [control, STATUS[which] & 1]]).astype(np.uint16)


def sub_packet(seq, t_us, which, ram=None, control=CTRL, **kw):
    ram = RAM[which] if ram is None else ram
    return fr.encode_subpage_packet(seq, t_us, STATUS[which] | NEW_DATA, control, ram, **kw)


# ----------------------------------------------------------------------
print("A. The Melexis port against the Melexis example data")

p, err = fr.extract_parameters(EE)
s = G["scalars"]
check("EEPROM extraction reports no deviating-pixel error", err == 0, f"code {err}")
exact = {"kVdd": p.kVdd, "vdd25": p.vdd25, "KtPTAT": float(p.KtPTAT), "vPTAT25": p.vPTAT25,
         "alphaPTAT": float(p.alphaPTAT), "gainEE": p.gainEE, "cpKv": float(p.cpKv),
         "resolutionEE": p.resolutionEE}
check("integer and exactly representable parameters match exactly",
      all(exact[k] == s[k] for k in exact), str({k: (exact[k], s[k]) for k in exact if exact[k] != s[k]}))
# The example prints these to 6 significant decimals; the port holds the
# full float32 value.
approx = {"KvPTAT": float(p.KvPTAT), "cpKta": float(p.cpKta), "KsTa": float(p.KsTa)}
check("parameters printed to 6 decimals match to that precision",
      all(abs(approx[k] - s[k]) < 5e-7 for k in approx),
      ", ".join(f"{k} {approx[k]:.9f}" for k in approx))
check("compensation-pixel alpha and offset, corner temperatures, ksTo",
      np.allclose(p.cpAlpha, s["cpAlpha"], rtol=1e-7) and list(p.cpOffset) == s["cpOffset"]
      and list(p.ct[:4]) == s["ct"] and np.allclose(p.ksTo[:4], s["ksTo"], rtol=2e-3),
      f"ct {p.ct}, ksTo[0] {float(p.ksTo[0]):.7f}")
check("all 768 pixel offsets match exactly",
      np.array_equal(p.offset, np.array(G["offset_pixels"])))
kta_err = np.max(np.abs(p.kta / 2.0 ** p.ktaScale - np.array(G["kta_pixels"])))
kv_err = np.max(np.abs(p.kv / 2.0 ** p.kvScale - np.array(G["kv_pixels"])))
check("pixel kta and kv match to well within one quantization step",
      kta_err < 0.5 * 2.0 ** -p.ktaScale and kv_err < 0.5 * 2.0 ** -p.kvScale,
      f"kta err {kta_err:.1e} (step {2.0 ** -p.ktaScale:.1e}), kv err {kv_err:.1e}")
alpha = fr.SCALEALPHA * 2.0 ** p.alphaScale / p.alpha.astype(float)
alpha_err = np.max(np.abs(alpha / np.array(G["alpha_pixels"]) - 1))
check("pixel alpha matches to the 16-bit storage precision",
      alpha_err < 5e-5, f"max relative error {alpha_err:.1e}")

masks = [fr.subpage_mask(frame_words(k)) for k in (0, 1)]
check("in chess mode the two subpages update complementary halves",
      masks[0].sum() == 384 and masks[1].sum() == 384 and not np.any(masks[0] & masks[1]))

img = np.full(768, np.nan)
for k in (0, 1):
    fw = frame_words(k)
    fr.calculate_to(fw, p, 1.0, float(fr.get_ta(fw, p)) - fr.TA_SHIFT, img)
diff = np.abs(img - TEMPS)
check(f"every pixel's temperature after both subpages matches Melexis within {TOL_K * 1000:.0f} mK",
      np.nanmax(diff) < TOL_K and not np.isnan(img).any(),
      f"max {np.nanmax(diff) * 1000:.2f} mK over 768 pixels")

# The same through the byte stream and the reader.
stream = (fr.encode_eeprom_packet(0, EE) + sub_packet(1, 1_000_000, 0)
          + sub_packet(2, 1_125_000, 1))
rd = fr.FrameReader(emissivity=1.0)
out = rd.feed(stream)
check("through the stream: the first subpage is warm-up, the second yields the full image",
      len(out) == 1 and rd.warmup_subpages == 1 and out[0].subpage == 1,
      f"{len(out)} yielded, warm-up {rd.warmup_subpages}")
check("and that image matches Melexis within the same tolerance, as a (24, 32) frame",
      out[0].frame.shape == (24, 32)
      and np.max(np.abs(out[0].frame.reshape(-1) - TEMPS)) < TOL_K,
      f"max {np.max(np.abs(out[0].frame.reshape(-1) - TEMPS)) * 1000:.2f} mK, "
      f"Ta {out[0].ta:.2f} C, Vdd {out[0].vdd:.3f} V, {out[0].mode}")

# ----------------------------------------------------------------------
print("\nB. Stream framing")


def firmware_crc32(data):
    """The loop in mlx90640_stream.cpp, transcribed, to show it is the same
    function as zlib.crc32 that the reader checks with."""
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (0xEDB88320 ^ (c >> 1)) if c & 1 else (c >> 1)
        table.append(c)
    c = 0xFFFFFFFF
    for b in data:
        c = table[(c ^ b) & 0xFF] ^ (c >> 8)
    return c ^ 0xFFFFFFFF


rng = np.random.default_rng(5)
blobs = [rng.integers(0, 256, n, dtype=np.uint8).tobytes() for n in (0, 1, 7, 1694)]
check("the firmware's CRC loop is zlib.crc32",
      all(firmware_crc32(b) == zlib.crc32(b) & 0xFFFFFFFF for b in blobs))

packets = [fr.encode_text_packet("hello from the ESP32"), fr.encode_eeprom_packet(0, EE),
           sub_packet(1, 1_000_000, 0, read_us=40000, i2c_errors=2, missed_subpages=1,
                      tx_dropped=3),
           sub_packet(2, 1_125_000, 1)]
clean = b"".join(packets)
parser = fr.StreamParser()
got = parser.feed(clean)
kinds = [k for k, _ in got]
check("a clean stream parses into its packets, nothing discarded",
      kinds == ["text", "eeprom", "subpage", "subpage"] and parser.bytes_discarded == 0)
fields = fr._SUBPAGE_FMT.unpack(got[2][1])
check("subpage header fields decode as the firmware packs them",
      fields[:8] == (1, 1_000_000, 40000, 2, 1, 3, STATUS[0] | NEW_DATA, CTRL)
      and np.array_equal(np.array(fields[8:], dtype=np.uint16), RAM[0]))

garbage = b"\x00junk\xffMLX" + bytes(range(50))
parser = fr.StreamParser()
got = parser.feed(garbage + packets[1] + garbage + packets[2] + garbage)
parser.finish()
check("garbage between packets is discarded and counted to the byte",
      [k for k, _ in got] == ["eeprom", "subpage"]
      and parser.bytes_discarded + parser.trailing_partial == 3 * len(garbage),
      f"discarded {parser.bytes_discarded}, trailing {parser.trailing_partial}")

bad = bytearray(packets[2])
bad[500] ^= 0x40
parser = fr.StreamParser()
got = parser.feed(packets[1] + bytes(bad) + packets[3])
check("a corrupted byte costs exactly its own packet: CRC failure counted, neighbours kept",
      [k for k, _ in got] == ["eeprom", "subpage"] and parser.crc_failures == 1
      and parser.bytes_discarded == len(bad),
      f"crc failures {parser.crc_failures}, discarded {parser.bytes_discarded} of {len(bad)}")

parser = fr.StreamParser()
got = []
for i in range(len(clean)):
    got.extend(parser.feed(clean[i:i + 1]))
check("fed one byte at a time, the same packets come out",
      [k for k, _ in got] == kinds and parser.bytes_discarded == 0)

parser = fr.StreamParser()
got = parser.feed(clean[:-100])
parser.finish()
check("a stream cut mid-packet reports the partial packet at finish()",
      len(got) == 3 and parser.trailing_partial == len(packets[3]) - 100)

wrong_len = fr.MAGIC_SUBPAGE + (1234).to_bytes(2, "little") + b"\x00" * 20
parser = fr.StreamParser()
got = parser.feed(wrong_len + packets[3])
check("a magic followed by an impossible length is rejected, the stream recovers",
      [k for k, _ in got] == ["subpage"] and parser.length_errors == 1)

# ----------------------------------------------------------------------
print("\nC. Reader robustness")

base = fr.encode_eeprom_packet(0, EE)

rd = fr.FrameReader(emissivity=1.0)
out = rd.feed(base + sub_packet(1, 1_000_000, 0) + sub_packet(2, 1_125_000, 1)
              + sub_packet(4, 1_375_000, 1) + sub_packet(1, 2_000, 0))
check("a sequence gap counts lost subpages, a sequence restart counts a firmware restart",
      rd.subpages_lost == 1 and rd.firmware_restarts == 1,
      f"lost {rd.subpages_lost}, restarts {rd.firmware_restarts}")
check("the firmware's own counters are passed through",
      rd.counters()["fw_i2c_errors"] == 0 and "fw_tx_dropped" in rd.counters())

rd = fr.FrameReader(emissivity=1.0)
t0 = (1 << 32) - 60_000
out = rd.feed(base + sub_packet(1, t0, 0) + sub_packet(2, (t0 + 125_000) & 0xFFFFFFFF, 1)
              + sub_packet(3, (t0 + 250_000) & 0xFFFFFFFF, 0))
check("a micros() wrap between subpages keeps time monotonic",
      [o.time_us for o in out] == [t0 + 125_000, t0 + 250_000],
      f"{[o.time_us for o in out]}")

bad_aux = RAM[1].copy()
bad_aux[768] = 0x7FFF
rd = fr.FrameReader(emissivity=1.0)
out = rd.feed(base + sub_packet(1, 1_000_000, 0) + sub_packet(2, 1_125_000, 1, ram=bad_aux))
# What the Melexis driver computes in that case: subpage 1's pixels with
# subpage 0's (stale) aux words.
stale = np.concatenate([RAM[1][:768], RAM[0][768:]])
ref = np.full(768, np.nan)
fr.calculate_to(frame_words(0), p, 1.0, 0.0, ref)
fr.calculate_to(frame_words(1, ram=stale), p, 1.0, 0.0, ref)
check("aux words reading 0x7FFF: the previous aux is reused, exactly as the Melexis driver does, "
      "and counted",
      rd.aux_invalid == 1 and len(out) == 1
      and np.max(np.abs(out[0].frame.reshape(-1) - ref)) < 1e-9,
      f"aux_invalid {rd.aux_invalid}; stale aux moves the image by up to "
      f"{np.max(np.abs(ref - TEMPS)):.3f} K, which is why it is counted")
rd = fr.FrameReader(emissivity=1.0)
out = rd.feed(base + sub_packet(1, 1_000_000, 0, ram=np.where(np.arange(832) == 768, 0x7FFF,
                                                               RAM[0]).astype(np.uint16)))
check("and with no previous aux to fall back on, the subpage is skipped and counted",
      rd.aux_unusable == 1 and out == [])

bad_px = RAM[0].copy()
bad_px[0] = 0x7FFF     # line 0, a subpage 0 line
rd = fr.FrameReader(emissivity=1.0)
out = rd.feed(base + sub_packet(1, 1_000_000, 0, ram=bad_px) + sub_packet(2, 1_125_000, 1))
check("pixel data reading 0x7FFF on the subpage's own line is skipped and counted",
      rd.frame_data_invalid == 1 and out == [] and rd.warmup_subpages == 1)

rd = fr.FrameReader(emissivity=1.0)
early = rd.feed(sub_packet(1, 1_000_000, 0) + sub_packet(2, 1_125_000, 1))
late = rd.feed(base)
check("subpages that arrive before the EEPROM are held, then processed when it arrives",
      early == [] and rd.held_before_eeprom == 2 and len(late) == 1
      and np.max(np.abs(late[0].frame.reshape(-1) - TEMPS)) < TOL_K)
rd = fr.FrameReader(emissivity=1.0, hold_limit=1)
rd.feed(sub_packet(1, 1_000_000, 0) + sub_packet(2, 1_125_000, 1))
check("beyond hold_limit they are counted, not silently lost",
      rd.held_before_eeprom == 1 and rd.dropped_before_eeprom == 1)

other = EE.copy()
other[33] ^= 0x0100    # a different alphaRef: a different sensor
rd = fr.FrameReader(emissivity=1.0)
rd.feed(base + sub_packet(1, 1_000_000, 0) + sub_packet(2, 1_125_000, 1))
out = rd.feed(fr.encode_eeprom_packet(1, other) + sub_packet(3, 1_250_000, 0))
check("a different EEPROM resets the image: the next subpage is warm-up again",
      rd.eeprom_changes == 1 and out == [] and rd.warmup_subpages == 2)
out = rd.feed(fr.encode_eeprom_packet(2, other))
check("the same EEPROM repeated (every 5 s) changes nothing",
      rd.eeprom_changes == 1 and out == [])

dead_kv = EE.copy()
dead_kv[52] = 0
dead_kv[56] &= 0xF0FF      # every kv zero: the C driver's rescaling loop never ends
rd = fr.FrameReader()
rd.feed(fr.encode_eeprom_packet(0, dead_kv) + sub_packet(1, 1_000_000, 0))
check("a corrupt EEPROM is rejected and counted instead of hanging the reader",
      rd.eeprom_invalid == 1 and rd.params is None and rd.held_before_eeprom == 1)

# ----------------------------------------------------------------------
print("\nD. Bad-pixel correction")

BROKEN, OUTLIER = 300, 500      # interior, and not adjacent to each other
ee_bad = EE.copy()
ee_bad[64 + BROKEN] = 0
ee_bad[64 + OUTLIER] |= 0x0001
pb_, err = fr.extract_parameters(ee_bad)
check("broken (word 0) and outlier (bit 0) pixels are found in the EEPROM",
      err == 0 and pb_.broken_pixel_list() == [BROKEN] and pb_.outlier_pixel_list() == [OUTLIER])
adj = EE.copy()
adj[64 + 300] = 0
adj[64 + 333] = 0              # diagonal neighbour
check("two adjacent bad pixels are reported with the Melexis error code",
      fr.extract_parameters(adj)[1] == -fr.MLX90640_ADJACENT_BAD_PIXELS_ERROR)

# A dead pixel does not only have a zeroed EEPROM word: its readings are
# garbage too. Pixel 300 (row 9, column 12) belongs to subpage 1 in chess
# mode; make its reading stick far above its neighbours'.
dead_ram = RAM[1].copy()
dead_ram[BROKEN] = 0x2000
stream_bad = (fr.encode_eeprom_packet(0, ee_bad) + sub_packet(1, 1_000_000, 0)
              + sub_packet(2, 1_125_000, 1, ram=dead_ram))
fixed = fr.FrameReader(emissivity=1.0).feed(stream_bad)[0].frame.reshape(-1)
diag = [fixed[BROKEN - 33], fixed[BROKEN - 31], fixed[BROKEN + 31], fixed[BROKEN + 33]]
check("in chess mode the broken pixel becomes the median of its four diagonal neighbours",
      abs(fixed[BROKEN] - float(np.median(diag))) < 1e-9,
      f"{fixed[BROKEN]:.3f} C vs neighbours {np.round(diag, 3).tolist()}")
raw = fr.FrameReader(emissivity=1.0, correct_bad_pixels=False).feed(stream_bad)[0].frame.reshape(-1)
check("uncorrected, the same pixel is nowhere near its neighbours (a false target in waiting)",
      abs(raw[BROKEN] - float(np.median(diag))) > 5.0 or np.isnan(raw[BROKEN]),
      f"uncorrected {raw[BROKEN]:.1f} C")
others = np.setdiff1d(np.arange(768), [BROKEN, OUTLIER])
check("no other pixel is touched by the correction",
      np.max(np.abs(fixed[others] - TEMPS[others])) < TOL_K)

# ----------------------------------------------------------------------
print("\nE. Real Melexis frames downstream, nothing changed")

synthetic = sf.create_synthetic_frame(15.5, 11.5, rng=np.random.default_rng(1))
sub_stream = b"".join([base] + [sub_packet(i + 1, 1_000_000 + 125_000 * i, i % 2) for i in range(4)])
subs = fr.FrameReader(emissivity=1.0).feed(sub_stream)
check("Subpage.frame has the shape synthetic_frames produces",
      subs[0].frame.shape == synthetic.shape and subs[0].frame.dtype.kind == "f")

tmp = tempfile.mkdtemp(prefix="pillarb_frame_reader_")
raw_path = os.path.join(tmp, "capture.mlxraw")
with open(raw_path, "wb") as f:
    f.write(sub_stream)
from_file = list(fr.iter_file(raw_path, chunk_size=777, reader=fr.FrameReader(emissivity=1.0)))
check("replaying a saved raw capture gives the same subpages as the live feed",
      [(s.time_ms, s.subpage) for s in from_file] == [(s.time_ms, s.subpage) for s in subs]
      and all(np.array_equal(a.frame, b.frame) for a, b in zip(from_file, subs)))

dets = cd.detect_centroids(subs[-1].frame)
check("the detector finds the warm object in the Melexis example scene",
      len(dets) >= 1,
      "; ".join(f"u {d['u']:.1f} v {d['v']:.1f} n {d['n_pixels']} snr {d['snr']:.1f} "
                f"edge {d['touches_edge']}" for d in dets))

# Attitude, LiDAR and position on the same clock as the frames. In flight
# they are on the flight controller's clock, and mapping one onto the
# other is the open item in the module docstring.
orch = Orchestrator(lever_arm_b=np.zeros(3))
level = np.array([1.0, 0.0, 0.0, 0.0])
orch.on_position(0, 69.9689, 23.2717, 42.0)
for t in range(900, 1700, 20):
    orch.on_attitude(t, level)
for t in range(900, 1700, 10):
    orch.on_lidar(t, 5.0, 8000)
emitted = []
for sp in subs:
    emitted.extend(orch.on_subpage(sp.time_ms, sp.subpage, sp.frame))
check("frame_reader output drives the orchestrator unchanged: paired, located detections",
      orch.frames_paired >= 1 and len(emitted) >= 1
      and all(np.isfinite([r['north_m'], r['east_m'], r['lat_deg']]).all() for r in emitted),
      f"pairs {orch.frames_paired}, detections {len(emitted)}, "
      f"first at N {emitted[0]['north_m']:.2f} E {emitted[0]['east_m']:.2f} m" if emitted else "")

# ----------------------------------------------------------------------
print("\nF. Attitude pairs from the pillarb_stream build")

import clock_sync as cs

Q = np.array([0.9, 0.1, -0.2, 0.3824])
pkt = fr.encode_attitude_packet(7, 123_456_789, 98_765, Q, from_euler=True, wire_bytes=40)
rd = fr.FrameReader(emissivity=1.0)
ev = rd.feed_events(pkt)
check("an attitude packet decodes to both clocks, the quaternion and its flags",
      len(ev) == 1 and isinstance(ev[0], fr.AttitudePair) and ev[0].rx_us == 123_456_789
      and ev[0].time_boot_ms == 98_765 and np.allclose(ev[0].quat, Q, atol=1e-7)
      and ev[0].from_euler and ev[0].wire_bytes == 40 and rd.attitude_from_euler == 1)

CTRL_8HZ = (CTRL & ~(0x7 << 7)) | (4 << 7)
att = lambda s, us, ms: fr.encode_attitude_packet(s, us, ms, Q)
stream = (base + att(1, 900_000, 5_000) + sub_packet(1, 1_000_000, 0, control=CTRL_8HZ)
          + att(2, 1_020_000, 5_020) + att(3, 1_040_000, 5_040)
          + sub_packet(2, 1_125_000, 1, control=CTRL_8HZ) + att(4, 1_160_000, 5_160))
rd = fr.FrameReader(emissivity=1.0)
kinds = ["att" if isinstance(e, fr.AttitudePair) else "sub" for e in rd.feed_events(stream)]
check("feed_events() keeps stream order, attitude interleaved with subpages",
      kinds == ["att", "att", "att", "sub", "att"], str(kinds))
rd = fr.FrameReader(emissivity=1.0)
subs_only = rd.feed(stream)
check("feed() returns the subpages and counts the attitude it cannot return",
      len(subs_only) == 1 and rd.attitude_ignored_by_feed == 4
      and rd.counters()["packets_attitude"] == 4)
check("at the configured 8 Hz there is no refresh-rate mismatch",
      rd.refresh_rate_mismatch == 0)
rd = fr.FrameReader(emissivity=1.0)
rd.feed(base + sub_packet(1, 1_000_000, 0) + sub_packet(2, 1_125_000, 1))
check("the Melexis example, captured at 2 Hz, is counted as not matching the exposure "
      "correction's rate", rd.refresh_rate_mismatch == 2,
      f"{fr.refresh_rate_hz(CTRL):g} Hz against clock_sync.SUBPAGE_RATE_HZ "
      f"{cs.SUBPAGE_RATE_HZ:g} Hz")

# Across the micros() wrap, out of stamp order: the second subpage is
# stamped 10 ms before the wrap but sent after an attitude message parsed
# 5 ms after it, during the subpage's I2C read.
W = 1 << 32
stream = (base + att(1, W - 200_000, 5_000) + sub_packet(1, W - 135_000, 0, control=CTRL_8HZ)
          + att(2, 5_000, 5_215) + sub_packet(2, W - 10_000, 1, control=CTRL_8HZ))
evs = fr.FrameReader(emissivity=1.0).feed_events(stream)
times = [(type(e).__name__, e.rx_us if isinstance(e, fr.AttitudePair) else e.time_us) for e in evs]
check("subpages and attitude share one unwrapped clock, even sent out of stamp order "
      "across the wrap",
      times == [("AttitudePair", W - 200_000), ("AttitudePair", W + 5_000), ("Subpage", W - 10_000)],
      str(times))

rd = fr.FrameReader(emissivity=1.0)
rd.feed_events(att(1, 1_000, 10) + att(2, 21_000, 30) + att(5, 81_000, 90) + att(1, 500, 12))
check("attitude sequence gaps and restarts are counted",
      rd.attitude_lost == 2 and rd.attitude_seq_restarts == 1)

# The whole laptop path on a synthetic pillarb_stream capture: 50 Hz
# attitude pairs and 8 Hz subpages (the Melexis frames, repeated), the
# ESP32 switched on 37.25 s after the flight controller, through
# feed_events() and drive() into the orchestrator. The stream carries no
# LiDAR, so a reading is given straight to the orchestrator before each
# subpage, on the ESP32 clock.
rng = np.random.default_rng(31)
E0 = -37_250_000
WIRE = cs.wire_time_us(cs.ATTITUDE_QUATERNION_WIRE_BYTES)
parts = [(0, base)]
level = np.array([1.0, 0.0, 0.0, 0.0])
last_rx = 0
for k in range(300):                                   # 6 s of attitude
    t = 40_000_000 + 20_000 * k + rng.uniform(0, 2500)
    rx = max(int(t + E0 + WIRE + rng.exponential(2000)), last_rx + 20)
    last_rx = rx
    parts.append((rx, fr.encode_attitude_packet(k + 1, rx, int(t // 1000), level)))
t_mid = {}
for i in range(44):
    tm = 40_300_000 + 125_000 * i
    t_ready = int(tm + 62_500 + E0)
    t_mid[i] = tm
    parts.append((t_ready + 40_000, sub_packet(i + 1, t_ready, i % 2, control=CTRL_8HZ)))
capture = b"".join(p for _, p in sorted(parts, key=lambda x: x[0]))

orch = Orchestrator(lever_arm_b=np.zeros(3))
orch.on_position(40_000, 69.9689, 23.2717, 42.0)
rd = fr.FrameReader(emissivity=1.0)
found = []
for ev in rd.feed_events(capture):
    if isinstance(ev, fr.Subpage):
        orch.on_lidar_esp(ev.time_us - 50_000, 5.0, 8000)
    found.extend(fr.drive([ev], orch))
c = orch.counters()
paired = [r for r in found if r['paired']]
# The warm-up subpage (index 0) is never yielded, so a pair's subpage 0 is
# the even index whose time it carries.
errs = [min(abs(r['time_boot_ms'] - t_mid[i] / 1000.0) for i in range(0, 44, 2)) for r in paired]
check("capture -> feed_events() -> drive(): refused until the clock converges, then located "
      "on the flight controller's clock",
      c['rejected_no_clock_sync'] > 0 and len(paired) >= 5 and max(errs) <= 2.0
      and rd.attitude_yielded == 300,
      f"{c['rejected_no_clock_sync']} refused while converging, {len(paired)} pairs, "
      f"timestamps within {max(errs):.2f} ms of mid-exposure")

# ----------------------------------------------------------------------
n_fail = results.count(False)
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
if n_fail:
    sys.exit(1)
