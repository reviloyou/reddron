"""
clock_sync.py

Maps the ESP32's clock onto the flight controller's, and says which
instant a thermal subpage's pixels describe. The two belong together
because the orchestrator needs one number from them: the flight
controller time at which to look attitude up for a subpage.

Two clocks
----------
Attitude is stamped by the flight controller. ATTITUDE_QUATERNION carries
time_boot_ms, which ArduPilot fills with AP_HAL::millis() at the moment it
packs the message for sending (GCS_Common.cpp, send_attitude_quaternion).
A thermal subpage is stamped by the ESP32: micros() when the MLX90640's
data-ready flag was seen (mlx90640_stream.cpp, t_ready_us). The two
devices boot at different moments and run from different crystals, and
nothing ties their clocks together.

What is logged, and where the conversion happens
------------------------------------------------
Every attitude message carries both clocks: its own time_boot_ms, and the
ESP32's micros() at the moment it was parsed (mavlink_request.cpp stamps
it, mlx90640_stream.cpp in the pillarb_stream build sends it as an MLXA
packet beside the subpages). An offset can be recovered afterwards from
two logged clocks; it cannot be recovered from one.

Attitude is stored against time_boot_ms, never against its arrival time.
Arrival time contains the link jitter, and writing it into the attitude
record would bake that noise in permanently. Instead, the subpage's
timestamp is converted from the ESP32 clock to the flight controller's at
lookup time, using the current estimate. The one uncertain step lives
here and nowhere else, so it can be improved later by re-running it over
a saved raw capture (frame_reader.iter_serial(save_path=...)), without
changing any log format.

The estimator
-------------
For each pair,

    d = esp_rx_us - 1000 * time_boot_ms - floor_us
      = offset + latency

where offset is the ESP32 clock minus the flight controller's at the same
instant, and latency is everything between the flight controller stamping
the message and the ESP32 parsing it, less floor_us, the part known
exactly (the message's own time on the wire, see wire_time_us). What
remains of the latency varies, and it is never negative: a message can
arrive late but never early. So the mean of d is biased by however
congested the link happens to be, while the fastest-arriving messages are
the least delayed and give the cleanest estimate of the offset. The
estimator uses a low percentile, never the mean:

  * Pairs are grouped into bins of bin_ms (one second, 50 pairs) on the
    ESP32 clock.
  * When a bin closes, the percentile-th percentile of its d is taken,
    provided it holds at least min_pairs_per_bin pairs.
  * A straight line is fitted through the bins of the last window_bins
    bin slots, so the drift between the two crystals is captured rather
    than assumed away. A constant offset measured at boot would be wrong
    by 50 ppm x 600 s = 30 ms after ten minutes at a plausible drift.

Why the line spans 20 s and not a few. Each bin's percentile scatters by
roughly 150 us. Through six one-second bins that makes the slope uncertain
by about 60 ppm, more than the 10 to 50 ppm drift it is there to capture,
and the mapping came out no better than a flat offset (simulated link:
227 us spread with the line against 200 us RMS without). Through twenty
bins the slope is good to a few ppm and the spread falls to 115 us, and a
drift swinging 80 ppm in ten minutes, far faster than a crystal warms up,
is still followed with no measurable lag (test_clock_sync.py, section C).
The percentile itself still covers one second at a time, so a congested
second is caught and dropped on its own.
  * A bin more than congestion_us above the line through the other bins
    is dropped. Latency only ever pushes d up, so a bin that sits high is
    one where even the fastest messages were late; the reverse cannot
    happen.
  * A fit whose slope is beyond max_drift_ppm is refused. Crystals are
    tens of ppm apart. Hundreds of ppm means the data is wrong (a SITL run
    at a speedup other than 1, a clock step inside the window), not that
    the crystals are that bad.

What this leaves is the low quantile of the latency above floor_us: the
truncation of time_boot_ms to whole milliseconds (0 to 1 ms, always
positive), the ESP32's polling delay, and whatever part of ArduPilot's
delay between stamping and transmitting no message ever escapes. It is a
property of the link, sub-millisecond on a quiet one, and the end-to-end
bench test in CODE_WALKTHROUGH.md section 7 is what measures it.

When there is no valid estimate
-------------------------------
to_fc_ms() returns None and counts why. The orchestrator then refuses the
subpage and counts it, exactly as it refuses a subpage whose attitude the
interpolator cannot serve. It never falls back to a zero offset, to the
last offset seen, or to a fit from too few bins: each of those produces
plausible wrong positions rather than a visible failure. This is the
state for the first few seconds after boot (until min_bins bins have
closed, about four seconds with the defaults), after the flight
controller or the ESP32 restarts, and once the attitude stream has been
silent for max_extrapolate_ms.

Discontinuities
---------------
  * Flight controller reboot: time_boot_ms steps back towards zero. Every
    pair before the step belongs to a different epoch, so the estimator
    starts over and refuses until it has converged again. Queries for
    instants before the reboot are refused (rejected_before_epoch).
  * ESP32 restart: micros() steps back. The same.
  * Many impossibly early pairs within one bin, the offset having stepped
    without either clock going back: the same, counted in offset_steps.
    A scattered early pair is rejected and counted and never enters a bin.

Units are in the names. esp_us must already be unwrapped past the 32-bit
micros() wrap (frame_reader.py does this). time_boot_ms is the uint32 from
the message, which wraps only after 49.7 days of flight controller uptime.
"""

from collections import deque, namedtuple

import numpy as np

# ----------------------------------------------------------------------
# Which instant a subpage describes
# ----------------------------------------------------------------------

# MLX90640 refresh rate, per subpage. Must match kRefreshCode in
# firmware/src/mlx90640_stream.cpp (code 4 = 8 Hz). frame_reader.py reads
# the rate back from the control register of every subpage and counts any
# disagreement in refresh_rate_mismatch, so a firmware change cannot leave
# this stale without it showing.
SUBPAGE_RATE_HZ = 8.0

# How long one subpage integrates. Assumed to be the whole subpage period:
# the sensor measures one subpage, then the other, alternating at
# SUBPAGE_RATE_HZ. The bench measurement in CODE_WALKTHROUGH.md section 7
# confirms or corrects this.
SUBPAGE_INTEGRATION_US = 1e6 / SUBPAGE_RATE_HZ

# t_ready_us marks the END of a subpage's integration (the data-ready
# flag), while the pixels describe the scene averaged over it, centred
# half an integration earlier: 62.5 ms at 8 Hz. The orchestrator subtracts
# this from every subpage timestamp before mapping it to the flight
# controller's clock.
#
# Pair-averaging in orchestrator.py does not remove it. Both halves of a
# frame are stamped at the end of their own integration, so both carry the
# same bias in the same direction. Averaging cancels the difference
# between the halves, not an error they share.
EXPOSURE_MID_OFFSET_US = int(round(0.5 * SUBPAGE_INTEGRATION_US))

# ----------------------------------------------------------------------
# The known part of the attitude latency
# ----------------------------------------------------------------------

# Must match the baud rate of the flight controller link in the firmware
# (PILLARB_FC_BAUD, and SERIALn_BAUD on the FC).
FC_LINK_BAUD = 115200

# ATTITUDE_QUATERNION on the wire in MAVLink 2: 10 header bytes, 32
# payload bytes, 2 checksum bytes. The message defines 48 payload bytes,
# but ArduPilot sends the repr_offset_q extension as zeros and MAVLink 2
# truncates trailing zeros. The firmware reports each message's actual
# length (it changes if the rates read exactly zero, or on the Euler
# fallback); this is the value used when it does not.
ATTITUDE_QUATERNION_WIRE_BYTES = 44

# A step back of at least this much on either clock is a restart, not a
# message out of order: MAVLink over a UART does not reorder by a second.
REBOOT_BACKSTEP_MS = 1000


def wire_time_us(n_bytes, baud=FC_LINK_BAUD):
    """Time a message of n_bytes spends on a UART at 8N1 (10 bits per
    byte). ArduPilot stamps time_boot_ms before the first byte leaves and
    the ESP32 cannot parse the message before the last byte arrives, so no
    message can arrive sooner than this after its stamp. Subtracting it is
    exact arithmetic, not estimation."""
    return n_bytes * 10 * 1e6 / baud


_Bin = namedtuple("_Bin", "k x_us d_us n")

Estimate = namedtuple("Estimate", "offset_us drift_ppm at_esp_us n_bins epoch")
Estimate.__doc__ = """The current fit, for telemetry and tests.

offset_us  ESP32 clock minus flight controller clock at at_esp_us.
drift_ppm  how fast that changes: positive when the ESP32 clock runs fast.
at_esp_us  where the offset is quoted (the newest bin used).
n_bins     bins behind the fit.
epoch      increments every time the estimator starts over.
"""

_COUNTER_KEYS = (
    "pairs_received", "pairs_accepted", "pairs_rejected_out_of_order",
    "pairs_rejected_early", "fc_reboots", "esp_restarts", "offset_steps", "resets",
    "bins_completed", "bins_rejected_sparse", "bins_rejected_congested",
    "fits_accepted", "fits_rejected_congested", "fits_rejected_drift",
    "queries_served", "rejected_no_estimate", "rejected_before_epoch",
    "rejected_too_old", "rejected_stale")


class ClockOffsetEstimator:
    """Low-percentile, drift-tracking map from ESP32 micros() to flight
    controller time_boot_ms. See the module docstring for the method.

    add(esp_us, time_boot_ms) takes one pair per attitude message.
    to_fc_ms(esp_us) converts, or returns None and counts why.

    Parameters
    ----------
    bin_ms : int
        Width of a bin. One second holds 50 pairs at 50 Hz.
    window_bins : int
        How many bin slots back the fit reaches: the rolling window.
    min_bins : int
        Bins needed before any conversion is served. At least 3, because a
        line through two bins always fits exactly and nothing about it can
        be checked.
    min_pairs_per_bin : int
        A bin with fewer pairs is not used (bins_rejected_sparse): its low
        percentile is just its minimum, and one early glitch would own it.
    percentile : float
        Which percentile of d a bin reports.
    congestion_us : float
        How far above the line through the other bins a bin may sit.
    max_drift_ppm : float
        Largest believable slope.
    max_extrapolate_ms : float
        How far a query may lie beyond the newest pair, or before the
        oldest bin in the fit.
    early_tolerance_us : float
        How far below the current line a pair may sit before it is
        impossible (negative latency) and rejected.
    step_early_pairs : int
        This many impossible pairs within one bin_ms mean the offset has
        stepped: at 50 Hz, 40 percent of a second's messages. Scattered
        glitches never get there.
    default_floor_us : float
        floor_us for add() calls that do not give one.
    """

    def __init__(self, bin_ms=1000, window_bins=20, min_bins=3, min_pairs_per_bin=10,
                 percentile=5.0, congestion_us=2000.0, max_drift_ppm=500.0,
                 max_extrapolate_ms=1000.0, early_tolerance_us=2000.0, step_early_pairs=20,
                 default_floor_us=wire_time_us(ATTITUDE_QUATERNION_WIRE_BYTES)):
        if min_bins < 3:
            raise ValueError("min_bins must be at least 3: a line through two bins "
                             "always fits exactly, so nothing about it can be checked")
        if window_bins < min_bins:
            raise ValueError(f"window_bins ({window_bins}) must be at least "
                             f"min_bins ({min_bins})")
        self.bin_us = int(bin_ms * 1000)
        self.window_bins = int(window_bins)
        self.min_bins = int(min_bins)
        self.min_pairs_per_bin = int(min_pairs_per_bin)
        self.percentile = float(percentile)
        self.congestion_us = float(congestion_us)
        self.max_drift_ppm = float(max_drift_ppm)
        self.max_extrapolate_us = float(max_extrapolate_ms) * 1000.0
        self.early_tolerance_us = float(early_tolerance_us)
        self.step_early_pairs = int(step_early_pairs)
        self.default_floor_us = float(default_floor_us)

        for k in _COUNTER_KEYS:
            setattr(self, k, 0)
        self.epoch = 0
        self._clear()

    # ------------------------------------------------------------------

    def _clear(self):
        self._last_fc_ms = None
        self._last_esp_us = None
        self._epoch_start_us = None
        self._open_k = None
        self._open_d = []
        self._open_x = []
        self._bins = deque()
        self._fit = None          # (x_ref_us, a_us, slope, x_first_us, n_bins)
        self._last_fit = None     # the newest fit this epoch, even if since lost
        self._recent_early = deque()

    def _start_epoch(self):
        self.epoch += 1
        self._clear()

    def reset(self):
        """Forget everything. For an owner that has learned of a
        discontinuity by other means, such as the orchestrator seeing
        time_boot_ms step back in a position message."""
        self.resets += 1
        self._start_epoch()

    @staticmethod
    def _lsq(x, d):
        """Least-squares line through (x, d), referenced to the newest x
        for conditioning. Returns (x_ref, a, slope): d = a + slope*(x - x_ref)."""
        x_ref = float(x.max())
        xs = x - x_ref
        xm = xs.mean()
        slope = float(np.sum((xs - xm) * (d - d.mean())) / np.sum((xs - xm) ** 2))
        a = float(d.mean() - slope * xm)
        return x_ref, a, slope

    def _line(self, fit, esp_us):
        x_ref, a, slope = fit[0], fit[1], fit[2]
        return a + slope * (esp_us - x_ref)

    def _close_open_bin(self):
        n = len(self._open_d)
        if n >= self.min_pairs_per_bin:
            self._bins.append(_Bin(self._open_k, float(np.mean(self._open_x)),
                                   float(np.percentile(self._open_d, self.percentile)), n))
            self.bins_completed += 1
        elif n > 0:
            self.bins_rejected_sparse += 1
        self._open_d = []
        self._open_x = []

    def _refit(self):
        self._fit = None
        if len(self._bins) < self.min_bins:
            return
        # Drift first, with a slope one bad bin cannot move. A clock running
        # at the wrong rate makes every bin look congested against the
        # others, and would otherwise be dropped a bin at a time and blamed
        # on the link. From five bins up that slope is the median of the
        # slopes between every two bins. Below five one bad bin owns the
        # median, so the smallest of them is used instead: a clock at the
        # wrong rate is steep between every two bins, while one congested
        # bin always leaves a pair of clean ones between which it is not.
        x = np.array([b.x_us for b in self._bins])
        d = np.array([b.d_us for b in self._bins])
        i, j = np.triu_indices(len(x), 1)
        slopes = np.abs((d[j] - d[i]) / (x[j] - x[i]))
        robust = np.median(slopes) if len(x) >= 5 else slopes.min()
        if robust * 1e6 > self.max_drift_ppm:
            self.fits_rejected_drift += 1
            return
        while True:
            bins = list(self._bins)
            x = np.array([b.x_us for b in bins])
            d = np.array([b.d_us for b in bins])
            # Each bin against the line through all the others, so a
            # congested bin cannot pull the line towards itself first.
            worst, worst_r = None, self.congestion_us
            for i in range(len(bins)):
                keep = np.arange(len(bins)) != i
                r = d[i] - self._line(self._lsq(x[keep], d[keep]), x[i])
                if r > worst_r:
                    worst, worst_r = i, r
            if worst is None:
                break
            if len(bins) <= self.min_bins:
                self.fits_rejected_congested += 1
                return
            self._bins.remove(bins[worst])
            self.bins_rejected_congested += 1
        x_ref, a, slope = self._lsq(x, d)
        if abs(slope) * 1e6 > self.max_drift_ppm:
            self.fits_rejected_drift += 1
            return
        self._fit = (x_ref, a, slope, float(x.min()), len(bins))
        self._last_fit = self._fit
        self.fits_accepted += 1

    def _roll_bins(self, esp_us):
        k = esp_us // self.bin_us
        if self._open_k is None:
            self._open_k = k
            return
        if k == self._open_k:
            return
        self._close_open_bin()
        self._open_k = k
        while self._bins and self._bins[0].k < k - self.window_bins:
            self._bins.popleft()
        self._refit()

    # ------------------------------------------------------------------

    def add(self, esp_us, time_boot_ms, floor_us=None):
        """One pair: the ESP32's micros() (unwrapped) when an attitude
        message was parsed, and the time_boot_ms inside it. floor_us is the
        part of its latency known exactly, normally wire_time_us() of its
        length. Returns True if the pair was used."""
        self.pairs_received += 1
        esp_us = int(esp_us)
        fc_ms = int(time_boot_ms)
        floor = self.default_floor_us if floor_us is None else float(floor_us)

        if self._last_fc_ms is not None:
            if fc_ms <= self._last_fc_ms - REBOOT_BACKSTEP_MS:
                self.fc_reboots += 1
                self._start_epoch()
            elif esp_us <= self._last_esp_us - 1000 * REBOOT_BACKSTEP_MS:
                self.esp_restarts += 1
                self._start_epoch()
            elif fc_ms <= self._last_fc_ms or esp_us < self._last_esp_us:
                self.pairs_rejected_out_of_order += 1
                return False

        self._roll_bins(esp_us)
        d = esp_us - 1000.0 * fc_ms - floor
        if self._fit is not None and d < self._line(self._fit, esp_us) - self.early_tolerance_us:
            # Counted per bin_ms rather than in an unbroken run: after a real
            # step the messages that happened to be delayed by more than the
            # step still look possible, and would keep resetting a run.
            self._recent_early.append(esp_us)
            while self._recent_early[0] < esp_us - self.bin_us:
                self._recent_early.popleft()
            if len(self._recent_early) < self.step_early_pairs:
                self.pairs_rejected_early += 1
                return False
            # Not a glitch but a new relationship between the clocks.
            self.offset_steps += 1
            self._start_epoch()
            self._roll_bins(esp_us)

        if self._epoch_start_us is None:
            self._epoch_start_us = esp_us
        self._open_d.append(d)
        self._open_x.append(esp_us)
        self._last_fc_ms = fc_ms
        self._last_esp_us = esp_us
        self.pairs_accepted += 1
        return True

    def to_fc_ms(self, esp_us):
        """Flight controller time, in milliseconds as a float, at the
        instant the ESP32's micros() read esp_us. None when there is no
        valid estimate for that instant; each reason is counted."""
        if self._fit is None:
            self.rejected_no_estimate += 1
            return None
        if esp_us < self._epoch_start_us:
            self.rejected_before_epoch += 1
            return None
        if esp_us < self._fit[3] - self.max_extrapolate_us:
            self.rejected_too_old += 1
            return None
        if esp_us > self._last_esp_us + self.max_extrapolate_us:
            self.rejected_stale += 1
            return None
        self.queries_served += 1
        return (esp_us - self._line(self._fit, esp_us)) / 1000.0

    def approx_fc_ms(self, esp_us):
        """The newest fit of this epoch applied without any validity check,
        or None if this epoch never had one. Not counted, and never for an
        attitude lookup: it is for bookkeeping that needs a clock but not a
        precise one, such as whether a queued log record is a second old
        while the attitude stream is down."""
        fit = self._fit or self._last_fit
        if fit is None:
            return None
        return (esp_us - self._line(fit, esp_us)) / 1000.0

    def estimate(self):
        """The current fit as an Estimate, or None."""
        if self._fit is None:
            return None
        x_ref, a, slope, _, n = self._fit
        return Estimate(offset_us=a, drift_ppm=slope * 1e6, at_esp_us=x_ref,
                        n_bins=n, epoch=self.epoch)

    def counters(self):
        c = {k: getattr(self, k) for k in _COUNTER_KEYS}
        c["epoch"] = self.epoch
        return c
