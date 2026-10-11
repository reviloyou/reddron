"""Tests for clock_sync.py: the ESP32-to-flight-controller clock mapping.

The link is simulated the way it behaves, not the way it would be
convenient:

  * ArduPilot stamps time_boot_ms in whole milliseconds, truncated, when
    it packs the message, anywhere inside its 2.5 ms main loop.
  * The message then spends its wire time on the UART (44 bytes at
    115200 baud, 3.8 ms), then waits a random extra time: the FC's own
    transmit queue and the ESP32's polling, exponential with a 2 ms mean,
    plus up to 40 ms for a third of the messages (the ESP32 busy in an
    MLX90640 I2C read), plus up to 100 ms for one message in a hundred.
  * The UART is first in, first out: a message never overtakes the one
    before it.
  * The ESP32 and the FC run from different crystals.

A. No estimate yet: refuse and count, never a zero offset.
B. Accuracy under jitter, against the mean and the minimum.
C. Drift, constant and changing, over ten minutes.
D. Flight controller reboot: time_boot_ms back to zero mid-run.
E. ESP32 restart: micros() back to zero mid-run.
F. Every rejection path, each on its own.
G. Counter bookkeeping over a mixed run.
H. The constants the orchestrator uses.
"""

import sys

import numpy as np

import clock_sync as cs

results = []


def check(label, ok, detail=""):
    print(f"  [{'ok ' if ok else 'FAIL'}] {label}" + (f"   {detail}" if detail else ""))
    results.append(bool(ok))
    return ok


WIRE_US = cs.wire_time_us(cs.ATTITUDE_QUATERNION_WIRE_BYTES)
QUERY_LAG_US = 60_000   # a subpage's exposure midpoint, a little behind the newest pair


class Clock:
    """The ESP32's micros() as a function of true time, which here is the
    flight controller's clock in microseconds since its first boot.
    drift_ppm may be a number or a function of true time in seconds."""

    def __init__(self, esp_at_zero_us, drift_ppm, span_s=900):
        grid = np.arange(0.0, span_s * 1e6, 100_000.0)          # 0.1 s steps
        ppm = (np.array([drift_ppm(t / 1e6) for t in grid]) if callable(drift_ppm)
               else np.full(grid.size, float(drift_ppm)))
        rate = 1.0 + ppm * 1e-6
        self._t = grid
        self._esp = esp_at_zero_us + np.concatenate([[0.0], np.cumsum(rate[:-1] * 100_000.0)])

    def esp(self, t_us):
        return float(np.interp(t_us, self._t, self._esp))

    def fc(self, esp_us):
        return float(np.interp(esp_us, self._esp, self._t))


def latency_us(rng):
    lat = WIRE_US + rng.exponential(2000.0)
    if rng.random() < 1 / 3:
        lat += rng.uniform(0.0, 40_000.0)
    if rng.random() < 0.01:
        lat += rng.uniform(0.0, 100_000.0)
    return lat


def pairs(rng, clock, t_start_s, t_end_s, rate_hz=50, fc_boot_us=0.0, fifo=None,
          extra=None, rate_scale=1.0):
    """(esp_us, time_boot_ms) for every attitude message the FC sends
    between t_start_s and t_end_s of true time. fc_boot_us is when the FC
    (last) booted, so time_boot_ms counts from there. extra(t_s) adds
    latency. rate_scale multiplies the FC clock (a SITL speedup)."""
    fifo = fifo if fifo is not None else {"last": -1e18}
    out = []
    t = t_start_s * 1e6
    while t < t_end_s * 1e6:
        t_pack = t + rng.uniform(0.0, 2500.0)
        lat = latency_us(rng) + (extra(t_pack / 1e6) if extra else 0.0)
        # A UART never lets a message overtake the one before it.
        arrive = max(t_pack + lat, fifo["last"] + 20.0)
        fifo["last"] = arrive
        fc_ms = int(((t_pack - fc_boot_us) * rate_scale) // 1000)
        out.append((int(clock.esp(arrive)), fc_ms))
        t += 1e6 / rate_hz
    return out


def run(est, prs, truth_fc_us=None):
    """Feed pairs, query QUERY_LAG_US behind each one. Returns the errors
    of the served queries in microseconds, and how many were refused."""
    errs, refused = [], 0
    for esp, fc in prs:
        est.add(esp, fc)
        q = esp - QUERY_LAG_US
        r = est.to_fc_ms(q)
        if r is None:
            refused += 1
        elif truth_fc_us is not None:
            errs.append(r * 1000.0 - truth_fc_us(q))
    return np.array(errs), refused


# The ESP32 was switched on 37.25 s after the flight controller, so its
# micros() reads 37.25 s less at any instant.
BOOT_GAP_US = -37_250_000.0

# ----------------------------------------------------------------------
print("A. No estimate yet: refuse and count, never a zero offset")

rng = np.random.default_rng(1)
clk = Clock(BOOT_GAP_US, 20.0)
prs = pairs(rng, clk, 40.0, 50.0)
est = cs.ClockOffsetEstimator()
first_served = None
wrong_if_zero = []
for i, (esp, fc) in enumerate(prs):
    est.add(esp, fc)
    r = est.to_fc_ms(esp - QUERY_LAG_US)
    if r is not None and first_served is None:
        first_served = (esp - prs[0][0]) / 1e6
        first_err = r * 1000.0 - clk.fc(esp - QUERY_LAG_US)
check("the first queries are refused, not answered", est.rejected_no_estimate > 0
      and est.queries_served > 0,
      f"{est.rejected_no_estimate} refused as rejected_no_estimate, then {est.queries_served} served")
check(f"the first answer comes once {est.min_bins} bins have closed, a few seconds in",
      first_served is not None and 2.0 < first_served < 4.5,
      f"after {first_served:.2f} s of attitude")
check("and it is already right, where a zero offset would have been 37 s out",
      abs(first_err) < 3000.0,
      f"first answer off by {first_err / 1000:+.2f} ms (a zero offset: {-BOOT_GAP_US / 1e6:.2f} s)")
check("an estimator that has seen nothing refuses",
      cs.ClockOffsetEstimator().to_fc_ms(1_000_000) is None)

# ----------------------------------------------------------------------
print("\nB. Accuracy under jitter, against the mean and the minimum")

rng = np.random.default_rng(2)
clk = Clock(BOOT_GAP_US, 0.0)
prs = pairs(rng, clk, 40.0, 160.0)
est = cs.ClockOffsetEstimator()
errs, _ = run(est, prs, clk.fc)
true_offset = BOOT_GAP_US   # esp - fc, no drift
d = np.array([esp - 1000.0 * fc - WIRE_US for esp, fc in prs])
mean_bias_ms = (d.mean() - true_offset) / 1000.0
print(f"  estimator error over {len(errs)} queries: mean {errs.mean() / 1000:+.3f} ms, "
      f"std {errs.std() / 1000:.3f} ms, max |e| {np.abs(errs).max() / 1000:.3f} ms")
print(f"  offset from the mean of d instead: biased by {mean_bias_ms:+.2f} ms "
      f"(a query error of {-mean_bias_ms:+.2f} ms)")
check("every served query is within 2.5 ms of the truth", np.abs(errs).max() < 2500.0,
      f"max {np.abs(errs).max() / 1000:.2f} ms")
check("the spread is a fraction of a millisecond", errs.std() < 400.0,
      f"std {errs.std():.0f} us")
check("the mean is not a usable estimator: it carries the link's congestion",
      mean_bias_ms > 5.0 and mean_bias_ms > 4 * abs(errs.mean()) / 1000.0,
      f"mean bias {mean_bias_ms:.1f} ms against the percentile's {abs(errs.mean()) / 1000:.2f} ms")
check("the estimate always lags, never leads: latency only adds",
      errs.mean() < 0, f"mean error {errs.mean() / 1000:+.3f} ms (FC time underestimated)")

# The reason for a percentile rather than the minimum: a pair that is
# early without being impossible (a floor set 1.5 ms too high for one
# message, a corrupted byte that still passed the CRC) owns the minimum
# of its bin, but only shifts a percentile by one rank. Once a second, one
# pair is made to look as if it arrived 1.5 ms before the true offset
# allows, inside early_tolerance_us so it does reach its bin.
rng = np.random.default_rng(3)
prs = pairs(rng, clk, 40.0, 100.0)
glitched = []
for i, (esp, fc) in enumerate(prs):
    if i % 50 == 7:
        fake = int(1000 * fc + BOOT_GAP_US + WIRE_US - 1500)
        if fake > glitched[-1][0]:
            esp = fake
    glitched.append((esp, fc))
e_clean, _ = run(cs.ClockOffsetEstimator(), prs, clk.fc)
e_glitch, _ = run(cs.ClockOffsetEstimator(), glitched, clk.fc)
e_min, _ = run(cs.ClockOffsetEstimator(percentile=0.0), glitched, clk.fc)
e_min_clean, _ = run(cs.ClockOffsetEstimator(percentile=0.0), prs, clk.fc)
shift_pct = e_glitch.mean() - e_clean.mean()
shift_min = e_min.mean() - e_min_clean.mean()
check("one early glitch per second moves the 5th percentile far less than the minimum",
      abs(shift_pct) < 0.5 * abs(shift_min) and abs(shift_min) > 1000.0,
      f"percentile moved {shift_pct:+.0f} us, minimum moved {shift_min:+.0f} us")

# ----------------------------------------------------------------------
print("\nC. Drift over ten minutes")

for label, drift in [("+50 ppm", 50.0), ("-50 ppm", -50.0),
                     ("swinging +40 to -40 ppm", lambda s: 40.0 - 80.0 * min(s, 640.0) / 640.0)]:
    rng = np.random.default_rng(4)
    clk = Clock(BOOT_GAP_US, drift)
    prs = pairs(rng, clk, 40.0, 640.0)
    est = cs.ClockOffsetEstimator()
    errs, _ = run(est, prs, clk.fc)
    first_min = errs[:2500].mean()
    last_min = errs[-2500:].mean()
    check(f"{label}: within 2.5 ms for the whole ten minutes, no growth",
          np.abs(errs).max() < 2500.0 and abs(last_min - first_min) < 300.0,
          f"max {np.abs(errs).max() / 1000:.2f} ms, first minute {first_min / 1000:+.2f}, "
          f"last {last_min / 1000:+.2f} ms")

rng = np.random.default_rng(5)
clk = Clock(BOOT_GAP_US, 50.0)
prs = pairs(rng, clk, 40.0, 640.0)
est = cs.ClockOffsetEstimator()
drifts = []
frozen = None
worst_frozen = 0.0
for esp, fc in prs:
    est.add(esp, fc)
    e = est.estimate()
    if e is not None:
        drifts.append(e.drift_ppm)
        if frozen is None:
            frozen = e.offset_us      # what a boot-time calibration would keep
    if frozen is not None:
        q = esp - QUERY_LAG_US
        worst_frozen = max(worst_frozen, abs((q - frozen) - clk.fc(q)))
drifts = np.array(drifts)
check("the fitted drift matches the crystals' 50 ppm",
      abs(np.median(drifts[100:]) - 50.0) < 5.0,
      f"median {np.median(drifts[100:]):.1f} ppm, spread {drifts[100:].std():.1f} ppm")
check("an offset measured once at boot and kept would be tens of milliseconds out",
      worst_frozen > 20_000.0, f"{worst_frozen / 1000:.1f} ms by the end of ten minutes")

# ----------------------------------------------------------------------
print("\nD. Flight controller reboot")
# The FC reboots at 80 s of true time. It is silent for 3 s while it
# boots, then time_boot_ms counts from zero again. The ESP32 runs on.

rng = np.random.default_rng(6)
clk = Clock(BOOT_GAP_US, 30.0)
fifo = {"last": -1e18}
T_REBOOT = 80.0
before = pairs(rng, clk, 40.0, T_REBOOT, fifo=fifo)
after = pairs(rng, clk, T_REBOOT + 3.0, T_REBOOT + 23.0, fc_boot_us=T_REBOOT * 1e6, fifo=fifo)
est = cs.ClockOffsetEstimator()
run(est, before)
last_esp_before = before[-1][0]
epoch_before = est.epoch

# During the silence the old mapping is still right for instants before
# the reboot, and is refused once the link has been quiet too long.
early_in_silence = est.to_fc_ms(last_esp_before + 500_000 - QUERY_LAG_US)
late_in_silence = est.to_fc_ms(last_esp_before + 2_500_000)
check("through the silence: served briefly, then refused as stale",
      early_in_silence is not None and late_in_silence is None and est.rejected_stale == 1)

served_new, refused_new, errs_new = 0, 0, []
for esp, fc in after:
    est.add(esp, fc)
    q = esp - QUERY_LAG_US
    r = est.to_fc_ms(q)
    if r is None:
        refused_new += 1
    else:
        served_new += 1
        errs_new.append(r * 1000.0 - (clk.fc(q) - T_REBOOT * 1e6))
errs_new = np.array(errs_new)
check("the reboot is detected once and the estimator starts over",
      est.fc_reboots == 1 and est.epoch == epoch_before + 1)
check("it refuses while it reconverges instead of using the old offset",
      refused_new > 50 and est.rejected_no_estimate > 0,
      f"{refused_new} queries refused after the reboot")
check("every answer after the reboot is in the new epoch and within 2.5 ms",
      served_new > 500 and np.abs(errs_new).max() < 2500.0,
      f"{served_new} served, max error {np.abs(errs_new).max() / 1000:.2f} ms "
      f"(the old offset would be {T_REBOOT:.0f} s out)")
check("an instant from before the reboot can no longer be mapped",
      est.to_fc_ms(last_esp_before - 1_000_000) is None and est.rejected_before_epoch == 1)

# ----------------------------------------------------------------------
print("\nE. ESP32 restart")

rng = np.random.default_rng(7)
clk_a = Clock(BOOT_GAP_US, 15.0)
clk_b = Clock(-90_000_000.0, 15.0)     # restarted at 90 s of true time: micros() from zero
fifo = {"last": -1e18}
est = cs.ClockOffsetEstimator()
run(est, pairs(rng, clk_a, 40.0, 90.0, fifo=fifo))
errs_b, refused_b = run(est, pairs(rng, clk_b, 90.2, 110.0, fifo=fifo), clk_b.fc)
check("detected once, refused while reconverging, right afterwards",
      est.esp_restarts == 1 and refused_b > 50 and np.abs(errs_b).max() < 2500.0,
      f"{refused_b} refused, then max error {np.abs(errs_b).max() / 1000:.2f} ms")

# ----------------------------------------------------------------------
print("\nF. Every rejection path, each on its own")

rng = np.random.default_rng(8)
clk = Clock(BOOT_GAP_US, 0.0)
base = pairs(rng, clk, 40.0, 60.0)

est = cs.ClockOffsetEstimator()
run(est, base)
last_esp, last_fc = base[-1]
check("a repeated time_boot_ms is rejected and counted",
      est.add(last_esp + 100, last_fc) is False and est.pairs_rejected_out_of_order == 1)
check("so is an ESP32 timestamp going backwards",
      est.add(last_esp - 100, last_fc + 1) is False and est.pairs_rejected_out_of_order == 2)
before_est = est.estimate()
check("an impossibly early pair (negative latency) is rejected and counted",
      est.add(last_esp + 20_000 - 10_000, last_fc + 20) is False
      and est.pairs_rejected_early == 1 and est.estimate() == before_est)

# The FC clock steps 20 ms forward without going back (a clock that jumps
# is not ArduPilot's, but a replay spliced from two captures can do it).
est = cs.ClockOffsetEstimator()
stepped = base[:500] + [(esp, fc + 20) for esp, fc in base[500:]]
errs_s, _ = run(est, stepped, lambda e: clk.fc(e) + (20_000.0 if e > base[500][0] else 0.0))
check("a second full of impossibly early pairs is a step: counted, and the estimator starts over",
      est.offset_steps == 1 and est.pairs_rejected_early == est.step_early_pairs - 1,
      f"offset_steps {est.offset_steps}, early pairs rejected {est.pairs_rejected_early}")
check("and it serves the new offset once reconverged", np.abs(errs_s[-200:]).max() < 2500.0,
      f"last 200 answers within {np.abs(errs_s[-200:]).max() / 1000:.2f} ms")

# Two seconds in which every message is at least 15 ms late: a saturated
# link, or the ESP32 blocked. Bins are on the ESP32 clock, which here reads
# 37.25 s less than true time, so this covers ESP32 bin 13 completely and
# the second halves of 12 and 14. The half-jammed bins need nothing: their
# low percentile comes from their clean half.
rng = np.random.default_rng(9)
jam = lambda s: 15_000.0 + 10_000.0 * rng.random() if 50.0 <= s < 52.0 else 0.0
est = cs.ClockOffsetEstimator()
errs_j, _ = run(est, pairs(rng, clk, 40.0, 70.0, extra=jam), clk.fc)
check("a congested second is dropped from the fit, and the answers do not move",
      est.bins_rejected_congested == 1 and np.abs(errs_j).max() < 2500.0,
      f"bins_rejected_congested {est.bins_rejected_congested}, "
      f"max error {np.abs(errs_j).max() / 1000:.2f} ms")

# The link down to 5 Hz for long enough to cover one ESP32 bin completely.
rng = np.random.default_rng(10)
fifo = {"last": -1e18}
thin = (pairs(rng, clk, 40.0, 49.5, fifo=fifo) + pairs(rng, clk, 49.5, 52.0, rate_hz=5, fifo=fifo)
        + pairs(rng, clk, 52.0, 60.0, fifo=fifo))
est = cs.ClockOffsetEstimator()
run(est, thin)
check("a second with too few pairs is not used, and counted",
      est.bins_rejected_sparse >= 1, f"bins_rejected_sparse {est.bins_rejected_sparse}")

# Exactly min_bins bins, one of them congested: nothing can be dropped, so
# no fit is served until a fourth bin allows the bad one to be removed.
# True time 41.0 to 42.5 s covers ESP32 bin 4, the third bin to close.
rng = np.random.default_rng(11)
jam2 = lambda s: 20_000.0 if 41.0 <= s < 42.5 else 0.0
est = cs.ClockOffsetEstimator()
prs = pairs(rng, clk, 40.0, 50.0, extra=jam2)
early_answers = []
for esp, fc in prs:
    est.add(esp, fc)
    if est.bins_completed == 3 and est.estimate() is None:
        early_answers.append(est.to_fc_ms(esp - QUERY_LAG_US))
check("with only min_bins bins and one congested, the fit is refused rather than trusted",
      est.fits_rejected_congested >= 1 and early_answers and all(a is None for a in early_answers)
      and est.bins_rejected_congested == 1 and est.estimate() is not None,
      f"fits_rejected_congested {est.fits_rejected_congested}, then the bin was dropped")

# SITL at speedup 2: the FC clock runs at twice real time.
rng = np.random.default_rng(12)
est = cs.ClockOffsetEstimator()
_, refused = run(est, pairs(rng, clk, 40.0, 50.0, rate_scale=2.0))
check("a clock running at the wrong rate (SITL speedup 2) is refused, never served",
      est.fits_rejected_drift > 0 and est.queries_served == 0,
      f"fits_rejected_drift {est.fits_rejected_drift}, {refused} queries refused")

# 40 s of pairs against a 20 s window: the start of this epoch is long
# out of the fit, so an instant from then is too old to map, although it
# is not from a different epoch.
rng = np.random.default_rng(13)
est = cs.ClockOffsetEstimator()
long_run = pairs(rng, clk, 40.0, 80.0)
run(est, long_run)
check("a query from well before the oldest bin in the fit is refused as too old",
      est.to_fc_ms(long_run[0][0] + 5_000_000) is None and est.rejected_too_old == 1
      and est.rejected_before_epoch == 0)

# ----------------------------------------------------------------------
print("\nG. Counter bookkeeping over a mixed run")

rng = np.random.default_rng(13)
clk = Clock(BOOT_GAP_US, -25.0)
fifo = {"last": -1e18}
mixed = (pairs(rng, clk, 40.0, 70.0, fifo=fifo, extra=jam)
         + pairs(rng, clk, 73.0, 100.0, fc_boot_us=70e6, fifo=fifo))
mixed = mixed[:300] + [mixed[299]] + mixed[300:]            # a duplicate
est = cs.ClockOffsetEstimator()
run(est, mixed)
est.to_fc_ms(0)
c = est.counters()
check("every pair is accepted or rejected for exactly one counted reason",
      c["pairs_received"] == c["pairs_accepted"] + c["pairs_rejected_out_of_order"]
      + c["pairs_rejected_early"],
      f"{c['pairs_received']} = {c['pairs_accepted']} + {c['pairs_rejected_out_of_order']} "
      f"+ {c['pairs_rejected_early']}")
n_queries = len(mixed) + 1
check("every query is served or refused for exactly one counted reason",
      n_queries == c["queries_served"] + c["rejected_no_estimate"] + c["rejected_before_epoch"]
      + c["rejected_too_old"] + c["rejected_stale"],
      f"{n_queries} queries")
print("\n  Counters at the end of the mixed run:")
for k, v in c.items():
    print(f"    {k:<28} {v}")

# ----------------------------------------------------------------------
print("\nH. Constants the orchestrator uses")

check("the exposure-midpoint correction is half a subpage period at the configured rate",
      cs.EXPOSURE_MID_OFFSET_US == round(0.5e6 / cs.SUBPAGE_RATE_HZ) == 62_500,
      f"{cs.EXPOSURE_MID_OFFSET_US} us at {cs.SUBPAGE_RATE_HZ:g} Hz per subpage")
check("ATTITUDE_QUATERNION's wire time is 3.8 ms at 115200 baud, twice that at 57600",
      abs(cs.wire_time_us(44, 115200) - 3819.4) < 0.1
      and abs(cs.wire_time_us(44, 57600) - 2 * cs.wire_time_us(44, 115200)) < 1e-9,
      f"{cs.wire_time_us(44, 115200):.1f} us and {cs.wire_time_us(44, 57600):.1f} us")

# ----------------------------------------------------------------------
n_fail = results.count(False)
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
if n_fail:
    sys.exit(1)
