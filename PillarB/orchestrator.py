"""
orchestrator.py

The runtime order for Pillar B. Every other module in this package is a
library that knows how to do one thing; this file is the only place that
says in what order those things happen, and it is therefore the piece the
C++ translation actually translates.

Event driven, because that is what the ESP32 loop looks like: three
sensor streams arrive independently and none of them waits for the
others. Attitude and LiDAR readings are stored as they arrive, and the
arrival of a thermal subpage is what triggers the chain.

Chain, on each subpage:

    0. (real camera, on_subpage_esp) move the subpage's timestamp from
       the end of its exposure to the middle, then from the ESP32's clock
       to the flight controller's; refuse the frame and count it if there
       is no valid clock mapping yet
    1. evict any pending half that has timed out or been superseded
    2. look up the attitude at THIS subpage's timestamp (interpolated)
    3. reject the frame if the interpolator refuses, and count it
    4. reject the frame if the newest LiDAR reading is missing, stale,
       or outside the rangefinder's trustworthy band, and count each
       case separately
    5. project the LiDAR slant range onto the vertical
    6. detect hot regions
    7. transform each region to a ground offset

    Then hold the result, waiting for its matching subpage. When both
    halves of a frame have been processed:

    8. associate detections by nearest ground position
    9. average the paired ground positions
   10. convert to lat/lon
   11. log the complete frame

Why the attitude lookup is per subpage
--------------------------------------
The MLX90640 does not expose all 768 pixels at once. It reads two
interleaved subpages, and the refresh rate setting applies per subpage,
so an 8 Hz setting delivers each subpage at 8 Hz and complete frames at
4 Hz, with the two halves captured 125 ms apart. Using one attitude for
both halves treats pixels from two different moments as simultaneous,
which is worth about 0.33 m at 30 deg/s yaw and 5 m altitude.

Why pair-and-average, rather than emit per subpage
---------------------------------------------------
Running the detector on the full 768-pixel buffer at every subpage
produces two estimates per physical frame, not one corrected estimate.
At any instant the buffer holds one fresh half and one half from 125 ms
ago. Calling detect_centroids() on it twice gives errors in opposite
directions (one half is early, one half is late), and the ground
positions drift across consecutive subpages for a stationary target.

Averaging the two transformed ground positions cancels the first-order
error: the residual from sp0 (which used fresh attitude on a half-stale
buffer) and the residual from sp1 (fresh buffer, attitude already one
interval ahead) are equal in magnitude and opposite in sign.

Why a refusal is never replaced by the newest sample
----------------------------------------------------
When AttitudeBuffer returns None it is saying it cannot answer honestly.
Substituting the most recent attitude would usually be nearly right,
which is precisely the problem: it converts a detectable, counted
failure into an invisible bias, and it makes the interpolator's own
rejection counters meaningless. The frame is dropped and counted.

Two clocks
----------
Attitude and position are stamped by the flight controller
(time_boot_ms). The camera and the LiDAR are stamped by the ESP32
(micros()). on_subpage() takes a subpage already on the flight
controller's clock, which is what the tests and synthetic replays have.
on_subpage_esp() is the real camera's entry point. It takes frame_reader's
time_us, the ESP32 clock at the END of the subpage's exposure, and
converts it only at lookup time:

    t_mid_us = t_ready_us - EXPOSURE_MID_OFFSET_US     (clock_sync.py)
    t_fc_ms  = clock.to_fc_ms(t_mid_us)

clock is a clock_sync.ClockOffsetEstimator, fed one pair per attitude
message by on_attitude(..., esp_rx_us=...). Attitude is still stored
against its own time_boot_ms; only the query moves between clocks, so the
uncertain step stays in one place. When the clock cannot answer (the
first seconds after boot, after a restart, with the link silent) the
subpage is refused and counted in rejected_no_clock_sync. A zero or a
remembered offset is never substituted, for the same reason the newest
attitude never is. LiDAR from the ESP32 (on_lidar_esp) stays on the ESP32
clock and is aged against t_mid_us directly, with no estimate involved.

Flight controller reboot
------------------------
time_boot_ms stepping back by clock_sync.REBOOT_BACKSTEP_MS or more on an
attitude message is a reboot. Only attitude decides it: it is the one
stream continuous enough to judge by, and a false alarm costs seconds of
refused frames while the clock estimate rebuilds. A position fix that goes
back is refused and counted as before, and the reset below clears the old
fix anyway. AttitudeBuffer refuses
timestamps that go backwards, so without this every attitude sample
after a reboot would be refused as out of order until time_boot_ms passed
its pre-reboot value: an outage as long as the previous uptime. On a
reboot the orchestrator starts a new attitude buffer (carrying its
counters over), resets the clock estimator, emits the pending half
unpaired (returned by the next on_subpage call, or by close()), forgets
the position fix, and drains the log, so that no record from the old
epoch waits for an age check that can no longer compare it with anything.

Pending half eviction
---------------------
A subpage can fail to find its partner because the partner was rejected
(attitude missing, LiDAR stale) or because frames were lost in the
serial stream. PAIR_TIMEOUT_MS is the window; if the matching half does
not arrive within it, the pending half is emitted unpaired rather than
silently discarded. The same happens if the same subpage index arrives
twice in a row, which means the intervening partner was lost.

Logging
-------
One record per complete paired frame, not one per subpage. The record
stores the sp0 timestamp and sp0 attitude quaternion (the start of the
physical frame). The timestamp is always on the flight controller's
clock: for the real camera it is the converted mid-exposure time the
quaternion was looked up at, so the two fields agree. The ESP32 times and
the attitude pairs they were converted with live in the raw capture
(frame_reader.iter_serial(save_path=...)), from which a better estimate
can be recomputed. The sp1 quaternion is not stored; the layout is frozen
and cannot accommodate a second quaternion. The logged pixel buffer is
the full 768-pixel frame as it stood when sp0 completed its readout.

The chain hands each record to LogWriter.write_frame(), which only
queues it unless the writer is in auto-drain mode (see logformat.py).
On the ESP32 the drain runs in its own task on the other core, so an SD
write stall never blocks this chain. Pass log_kwargs to choose the mode;
the default is the laptop's fsync-per-record behaviour. The writer's
counters appear in counters() under a log_ prefix, and stay readable
after close() for the end-of-run summary.
"""

import math

import numpy as np

import pillar_b as pb
import centroid_detector as cd
import clock_sync as cs
import logformat as lf
from attitude_interpolator import AttitudeBuffer

# How old the newest LiDAR reading may be before a frame is treated as
# having no height source.
#
# The TFmini-S free-runs at about 100 Hz (the Phase 0 bench logs show a
# median inter-sample gap of 10 ms), so 100 ms tolerates ten consecutive
# missed readings before a frame is refused. It also bounds the error
# from using a stale height: at a 1 m/s climb or descent the height is
# wrong by at most 0.1 m, and ground error scales with that times
# tan(off-nadir), so about 2 cm at the 12.5 deg boresight and about 6 cm
# at the frame corner. Both sit below the dominant terms in the error
# budget.
LIDAR_MAX_AGE_MS = 100

# Maximum ground-plane distance between two detections on opposite
# subpages before they are considered different targets rather than
# the same target seen twice. Derived from the error budget: the
# combined localisation mean from test_pillar_b.py section E is
# 0.208 m; 0.5 m is 2.4x that, wide enough to tolerate a bad
# centroid on one side, tight enough to refuse matching two distinct
# targets 0.5 m apart. Adjust if targets are expected closer together.
PAIR_MAX_ASSOC_M = 0.5

# If the matching subpage has not arrived within this many milliseconds,
# the pending half is emitted unpaired. 250 ms is two subpage intervals
# at 8 Hz: if the partner has not come by then it almost certainly will
# not come at all (it was rejected or the frame was lost).
PAIR_TIMEOUT_MS = 250

_ATTITUDE_BUFFER_COUNTERS = ('rejected_too_old', 'rejected_too_new', 'rejected_gap',
                             'queries_served')

_LOG_COUNTER_KEYS = ('records_enqueued', 'records_pending', 'records_written',
                     'records_unsynced', 'records_dropped_queue_full',
                     'records_lost_at_close', 'batches_written', 'write_failures',
                     'bytes_written', 'file_bytes', 'card_mounted',
                     'preallocated_bytes_remaining')

# Longest a record may wait in the log writer's queue, on the subpage
# clock, before it is written even though its batch is not full. Frames
# stop reaching the log when attitude or LiDAR drops out, and without this
# the last few records before the fault, the ones that matter most, would
# sit in RAM until the next batch fills. One second, the loss bound
# logformat.py sizes SD_BATCH_RECORDS against.
LOG_MAX_PENDING_MS = 1000


class Orchestrator:
    """Owns the attitude buffer, the log writer and the detector, and
    expresses the order in which they run.

    Parameters
    ----------
    log_path : str or None
        Where to write the binary frame log. None disables logging,
        which is what the tests use.
    attitude_capacity, attitude_max_gap_ms :
        Passed straight to AttitudeBuffer.
    lidar_max_age_ms : int
        See LIDAR_MAX_AGE_MS.
    forward_tilt_deg : float
        Camera mounting tilt. The mounting matrix is computed once here
        and never recomputed in the chain, because it is constant and
        recomputing it per frame is wasted work on a microcontroller.
    lever_arm_b : array-like or None
        Camera offset from the flight controller in body FRD. Defaults
        to pillar_b.LEVER_ARM_B, which is still a placeholder awaiting
        CAD measurement.
    detector_kwargs : dict or None
        Passed through to centroid_detector.detect_centroids().
    log_kwargs : dict or None
        Passed through to logformat.LogWriter (batch_records,
        auto_drain, queue_capacity). None keeps the laptop default.
    clock : clock_sync.ClockOffsetEstimator or None
        The ESP32-to-flight-controller mapping on_subpage_esp() uses. None
        makes one with the defaults. Unused by on_subpage().
    fc_baud : int
        Baud rate of the flight controller link, for the known wire time
        of each attitude message. See clock_sync.FC_LINK_BAUD.
    exposure_mid_offset_us : int
        See clock_sync.EXPOSURE_MID_OFFSET_US. A parameter so that a bench
        measurement can replace the assumed value, and so a test can show
        what leaving it out costs; not something to tune by eye.
    """

    def __init__(self, log_path=None, attitude_capacity=128,
                 attitude_max_gap_ms=200, lidar_max_age_ms=LIDAR_MAX_AGE_MS,
                 forward_tilt_deg=pb.FORWARD_TILT_DEG, lever_arm_b=None,
                 detector_kwargs=None, log_kwargs=None, clock=None,
                 fc_baud=cs.FC_LINK_BAUD,
                 exposure_mid_offset_us=cs.EXPOSURE_MID_OFFSET_US):
        self._attitude_capacity = attitude_capacity
        self._attitude_max_gap_ms = attitude_max_gap_ms
        self.attitude = AttitudeBuffer(capacity=attitude_capacity,
                                       max_gap_ms=attitude_max_gap_ms)
        # Counters of attitude buffers retired by a flight controller
        # reboot, so the totals in counters() cover the whole run.
        self._attitude_retired = dict.fromkeys(_ATTITUDE_BUFFER_COUNTERS, 0)
        self.clock = clock if clock is not None else cs.ClockOffsetEstimator()
        self.fc_baud = fc_baud
        self.exposure_mid_offset_us = exposure_mid_offset_us
        # Newest attitude time_boot_ms, for reboot detection.
        self._fc_newest_ms = None
        self.log = (lf.LogWriter(log_path, **(log_kwargs or {}))
                    if log_path is not None else None)
        # Kept after close() so the end-of-run summary can still read the
        # writer's counters once self.log has been released.
        self._log_writer = self.log
        self.lidar_max_age_ms = lidar_max_age_ms
        self.R_bc = pb.mounting_matrix(forward_tilt_deg)
        self.lever_arm_b = (pb.LEVER_ARM_B if lever_arm_b is None
                            else np.asarray(lever_arm_b, dtype=float))
        self.detector_kwargs = dict(detector_kwargs or {})

        # Newest LiDAR reading, and which clock it is on: 'fc' through
        # on_lidar(), 'esp' through on_lidar_esp(). Set by the first
        # reading; mixing the two is a wiring error and raises.
        self._lidar_t_ms = None
        self._lidar_distance_m = None
        self._lidar_strength = None
        self._lidar_clock = None

        # Newest position fix.
        self._pos_t_ms = None
        self._lat_deg = None
        self._lon_deg = None
        self._alt_m = None

        # Pending subpage half waiting for its partner. Cleared when
        # the matching half arrives, when PAIR_TIMEOUT_MS elapses, or
        # when the same subpage index arrives again.
        #
        # Keys: t_ms, subpage, results (list of half-result dicts),
        #       height_m, quat (tuple), pixels (np.float32 array).
        self._pending_half = None
        # Results produced outside on_subpage() (a pending half flushed by
        # a flight controller reboot), handed out by the next on_subpage()
        # call or by close().
        self._carry = []

        # Counters. Every rejection path has one, so that a rising
        # rejection rate in flight is visible instead of silent, and so
        # that a wiring fault can be told apart from a threshold set too
        # tight. AttitudeBuffer exposes its own four separately.
        self.attitude_received = 0
        self.attitude_rejected_out_of_order = 0
        self.lidar_received = 0
        self.lidar_rejected_out_of_order = 0
        self.position_received = 0
        self.position_rejected_out_of_order = 0

        self.fc_reboots = 0
        self.esp_restarts = 0

        self.subpages_received = 0
        # on_subpage_esp() only: no valid ESP32-to-FC mapping for the
        # subpage's instant. clock_sync's own counters (clock_ prefix in
        # counters()) say why.
        self.rejected_no_clock_sync = 0
        self.rejected_no_attitude = 0
        self.rejected_no_lidar = 0
        self.rejected_lidar_stale = 0
        self.rejected_lidar_unusable = 0
        self.rejected_ray_not_downward = 0
        self.frames_without_position = 0
        self.frames_no_detection = 0
        self.frames_processed = 0

        # Pair-level counters
        self.frames_paired = 0
        # Paired frames the log writer accepted into its queue. Not the
        # durable count: that is log_records_written, which can be lower
        # while records wait in the queue or the device is failing.
        self.frames_logged = 0
        self.results_at_close = []
        self.halves_timed_out = 0
        self.detections_matched = 0
        self.detections_unmatched_sp0 = 0
        self.detections_unmatched_sp1 = 0
        self.detections_reported = 0
        self.detections_touching_edge = 0

    # ------------------------------------------------------------------
    # Ingest
    # ------------------------------------------------------------------

    def on_attitude(self, time_boot_ms, quat, esp_rx_us=None, wire_bytes=None):
        """Store an ATTITUDE_QUATERNION sample against its own time_boot_ms.

        esp_rx_us, when given, is the ESP32's micros() (unwrapped) at the
        moment the message was parsed, and wire_bytes its length on the
        wire. The pair feeds the clock estimator; neither changes where the
        sample is stored. Arrival time carries the link's jitter, and
        storing attitude against it would bake that jitter in for good.
        """
        self.attitude_received += 1
        self._check_fc_clock(time_boot_ms)
        if esp_rx_us is not None:
            n = cs.ATTITUDE_QUATERNION_WIRE_BYTES if wire_bytes is None else wire_bytes
            self.clock.add(esp_rx_us, time_boot_ms, floor_us=cs.wire_time_us(n, self.fc_baud))
        accepted = self.attitude.add(time_boot_ms, quat)
        if not accepted:
            self.attitude_rejected_out_of_order += 1
        return accepted

    def _set_lidar_clock(self, which):
        if self._lidar_clock not in (None, which):
            raise ValueError("LiDAR readings on both clocks: use on_lidar() with on_subpage() "
                             "and on_lidar_esp() with on_subpage_esp()")
        self._lidar_clock = which

    def on_lidar(self, time_boot_ms, distance_m, strength):
        """Store the newest rangefinder reading, on the same clock as the
        subpages given to on_subpage()."""
        self._set_lidar_clock('fc')
        self.lidar_received += 1
        if self._lidar_t_ms is not None and time_boot_ms <= self._lidar_t_ms:
            self.lidar_rejected_out_of_order += 1
            return False
        self._lidar_t_ms = time_boot_ms
        self._lidar_distance_m = float(distance_m)
        self._lidar_strength = int(strength)
        return True

    def on_lidar_esp(self, esp_us, distance_m, strength):
        """Store the newest rangefinder reading, stamped with the ESP32's
        micros() (unwrapped): the TFmini-S is on the ESP32's UART2, so its
        readings are on the same clock as the camera's and are aged against
        the subpage directly in on_subpage_esp(), with no clock estimate
        involved. A step back of a second or more is the ESP32 restarting
        (counted in esp_restarts) and starts the readings over."""
        self._set_lidar_clock('esp')
        self.lidar_received += 1
        t_ms = esp_us / 1000.0
        if self._lidar_t_ms is not None and t_ms <= self._lidar_t_ms:
            if self._lidar_t_ms - t_ms < cs.REBOOT_BACKSTEP_MS:
                self.lidar_rejected_out_of_order += 1
                return False
            self.esp_restarts += 1
        self._lidar_t_ms = t_ms
        self._lidar_distance_m = float(distance_m)
        self._lidar_strength = int(strength)
        return True

    def on_position(self, time_boot_ms, lat_deg, lon_deg, alt_m):
        """Store the newest GLOBAL_POSITION_INT fix."""
        self.position_received += 1
        if self._pos_t_ms is not None and time_boot_ms < self._pos_t_ms:
            self.position_rejected_out_of_order += 1
            return False
        self._pos_t_ms = time_boot_ms
        self._lat_deg = float(lat_deg)
        self._lon_deg = float(lon_deg)
        self._alt_m = float(alt_m)
        return True

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _check_fc_clock(self, time_boot_ms):
        """Reboot detection, on every attitude timestamp."""
        if (self._fc_newest_ms is not None
                and time_boot_ms <= self._fc_newest_ms - cs.REBOOT_BACKSTEP_MS):
            self._on_fc_reboot()
            self._fc_newest_ms = time_boot_ms
        elif self._fc_newest_ms is None or time_boot_ms > self._fc_newest_ms:
            self._fc_newest_ms = time_boot_ms

    def _on_fc_reboot(self):
        """time_boot_ms started again. See "Flight controller reboot"."""
        self.fc_reboots += 1
        # The held half was captured before the reboot: emit it with the
        # position fix it was taken under, before that fix is forgotten.
        if self._pending_half is not None:
            self.halves_timed_out += 1
            self._carry.extend(self._flush_half(self._pending_half, None))
            self._pending_half = None
        for k in self._attitude_retired:
            self._attitude_retired[k] += getattr(self.attitude, k)
        self.attitude = AttitudeBuffer(capacity=self._attitude_capacity,
                                       max_gap_ms=self._attitude_max_gap_ms)
        self.clock.reset()
        self._pos_t_ms = None
        self._lat_deg = None
        self._lon_deg = None
        self._alt_m = None
        if self._lidar_clock == 'fc':
            self._lidar_t_ms = None
        if self.log is not None:
            self.log.drain()

    def _take_carry(self):
        out, self._carry = self._carry, []
        return out

    def _housekeeping(self, t_ms, subpage_index):
        """Steps that need a clock tick but no accuracy: the log's age
        bound and the pending half's eviction. t_ms may be None when no
        mapping exists at all; then only the same-index eviction runs.
        Returns the flushed half's results, if any."""
        # Every subpage, rejected or not, is a clock tick for the log's age
        # bound. On the ESP32 the writer task's own timer does this.
        if self.log is not None and t_ms is not None:
            self.log.drain_if_older_than(t_ms, LOG_MAX_PENDING_MS)

        # Evict the pending half if it has expired or if the same
        # subpage index has arrived again (partner was lost).
        if self._pending_half is not None:
            same_index = self._pending_half['subpage'] == subpage_index
            expired = t_ms is not None and t_ms - self._pending_half['t_ms'] > PAIR_TIMEOUT_MS
            if expired or same_index:
                self.halves_timed_out += 1
                flushed = self._flush_half(self._pending_half, t_ms)
                self._pending_half = None
                return flushed
        return []

    def _lidar_height(self, now_ms, R_nb):
        """Return (height_m, reason). reason is None on success. now_ms is
        on the LiDAR's own clock: the subpage time for on_subpage(), the
        ESP32 mid-exposure time for on_subpage_esp()."""
        if self._lidar_t_ms is None:
            self.rejected_no_lidar += 1
            return None, "rejected_no_lidar"

        age_ms = now_ms - self._lidar_t_ms
        if age_ms > self.lidar_max_age_ms:
            self.rejected_lidar_stale += 1
            return None, "rejected_lidar_stale"

        cos_tilt = float(np.clip(R_nb[2, 2], -1.0, 1.0))
        tilt_rad = float(np.arccos(cos_tilt))
        if not pb.lidar_is_valid(self._lidar_distance_m, 0.0, tilt_rad):
            self.rejected_lidar_unusable += 1
            return None, "rejected_lidar_unusable"

        return self._lidar_distance_m * cos_tilt, None

    def _transform_detections(self, detections, height_m, R_nb):
        """Transform detected centroids to body-relative ground offsets.

        Returns a list of half-result dicts with north_m, east_m etc.
        lat_deg and lon_deg are not filled here; they require the
        current position fix and are added during pairing.
        """
        results = []
        for det in detections:
            try:
                north, east, off_nadir = pb.pixel_to_ground_ned(
                    det['u'], det['v'], height_m, R_nb,
                    self.lever_arm_b, self.R_bc)
            except ValueError:
                self.rejected_ray_not_downward += 1
                continue
            results.append({
                'u': det['u'], 'v': det['v'],
                'n_pixels': det['n_pixels'],
                'snr': det['snr'],
                'touches_edge': det['touches_edge'],
                'height_m': height_m,
                'north_m': north,
                'east_m': east,
                'off_nadir_deg': off_nadir,
            })
        return results

    def _latlon_for(self, north_m, east_m):
        """Return (lat, lon) from the current fix, or (nan, nan)."""
        if self._lat_deg is None:
            return float('nan'), float('nan')
        return pb.ned_to_latlon(self._lat_deg, self._lon_deg, north_m, east_m)

    def _flush_half(self, half, t_ms_now):
        """Emit a pending half as unpaired results.

        Called when the matching subpage never arrived. Discarding a
        real detection because its partner was rejected would silently
        drop a SAR contact, so unpaired results are emitted with
        paired=False. The lat/lon is computed from the current fix at
        flush time, which may be slightly different from the fix at
        capture time; that is unavoidable without storing the fix in the
        pending half.
        """
        out = []
        sp = half['subpage']
        for r in half['results']:
            lat, lon = self._latlon_for(r['north_m'], r['east_m'])
            result = dict(r)
            result.update({
                'time_boot_ms': half['t_ms'],
                'subpage': sp,
                'lat_deg': lat,
                'lon_deg': lon,
                'paired': False,
            })
            out.append(result)
            if sp == 0:
                self.detections_unmatched_sp0 += 1
            else:
                self.detections_unmatched_sp1 += 1
            self.detections_reported += 1
            if result['touches_edge']:
                self.detections_touching_edge += 1
        return out

    def _pair(self, half0, half1):
        """Pair two processed subpage halves and return a list of result dicts.

        half0 is the subpage-0 half, half1 is the subpage-1 half.
        Association is greedy nearest-neighbour in the ground plane.
        Matched pairs are averaged; unmatched detections from either
        side are emitted unpaired with paired=False.
        """
        r0_list = half0['results']
        r1_list = half1['results']

        used0 = set()
        used1 = set()
        matched = []

        for i, r0 in enumerate(r0_list):
            best_d = PAIR_MAX_ASSOC_M
            best_j = None
            for j, r1 in enumerate(r1_list):
                if j in used1:
                    continue
                d = math.hypot(r0['north_m'] - r1['north_m'],
                               r0['east_m'] - r1['east_m'])
                if d < best_d:
                    best_d = d
                    best_j = j
            if best_j is not None:
                matched.append((i, best_j))
                used0.add(i)
                used1.add(best_j)

        out = []

        for i, j in matched:
            r0, r1 = r0_list[i], r1_list[j]
            avg_n = 0.5 * (r0['north_m'] + r1['north_m'])
            avg_e = 0.5 * (r0['east_m'] + r1['east_m'])
            lat, lon = self._latlon_for(avg_n, avg_e)
            result = {
                'time_boot_ms': half0['t_ms'],
                'subpage': 0,
                'u': 0.5 * (r0['u'] + r1['u']),
                'v': 0.5 * (r0['v'] + r1['v']),
                'n_pixels': r0['n_pixels'] + r1['n_pixels'],
                'snr': 0.5 * (r0['snr'] + r1['snr']),
                'touches_edge': r0['touches_edge'] or r1['touches_edge'],
                'height_m': 0.5 * (r0['height_m'] + r1['height_m']),
                'north_m': avg_n,
                'east_m': avg_e,
                'off_nadir_deg': 0.5 * (r0['off_nadir_deg'] + r1['off_nadir_deg']),
                'lat_deg': lat,
                'lon_deg': lon,
                'paired': True,
            }
            out.append(result)
            self.detections_matched += 1
            self.detections_reported += 1
            if result['touches_edge']:
                self.detections_touching_edge += 1

        for i, r0 in enumerate(r0_list):
            if i in used0:
                continue
            lat, lon = self._latlon_for(r0['north_m'], r0['east_m'])
            result = dict(r0)
            result.update({
                'time_boot_ms': half0['t_ms'],
                'subpage': 0,
                'lat_deg': lat,
                'lon_deg': lon,
                'paired': False,
            })
            out.append(result)
            self.detections_unmatched_sp0 += 1
            self.detections_reported += 1
            if result['touches_edge']:
                self.detections_touching_edge += 1

        for j, r1 in enumerate(r1_list):
            if j in used1:
                continue
            lat, lon = self._latlon_for(r1['north_m'], r1['east_m'])
            result = dict(r1)
            result.update({
                'time_boot_ms': half0['t_ms'],
                'subpage': 1,
                'lat_deg': lat,
                'lon_deg': lon,
                'paired': False,
            })
            out.append(result)
            self.detections_unmatched_sp1 += 1
            self.detections_reported += 1
            if result['touches_edge']:
                self.detections_touching_edge += 1

        return out

    # ------------------------------------------------------------------
    # The chain
    # ------------------------------------------------------------------

    def on_subpage(self, time_boot_ms, subpage_index, pixels):
        """Run the chain for one thermal subpage.

        pixels is the full 24x32 frame buffer as it stands now.

        Returns a list of result dicts. Empty if the frame was rejected,
        or if this subpage is stored as a pending half waiting for its
        partner. Non-empty when a complete pair is emitted (and possibly
        also when a timed-out orphan is flushed at the same call).

        Each result dict has keys:
            time_boot_ms, subpage, u, v, n_pixels, snr, touches_edge,
            height_m, north_m, east_m, off_nadir_deg, lat_deg, lon_deg,
            paired (True if both subpages contributed, False if orphaned).

        time_boot_ms is on the flight controller's clock, as are the LiDAR
        readings given to on_lidar(). For the real camera, whose subpages
        are stamped by the ESP32 at the end of their exposure, use
        on_subpage_esp().
        """
        if self._lidar_clock == 'esp':
            raise ValueError("on_subpage() takes flight-controller time, but the LiDAR is on "
                             "the ESP32 clock (on_lidar_esp): use on_subpage_esp()")
        return self._chain(time_boot_ms, subpage_index, pixels, time_boot_ms)

    def on_subpage_esp(self, t_ready_us, subpage_index, pixels):
        """Run the chain for one subpage from the real camera.

        t_ready_us is frame_reader's Subpage.time_us: the ESP32's micros(),
        unwrapped, when the subpage's data-ready flag was seen. Otherwise
        the same as on_subpage(), and it returns the same result dicts,
        with time_boot_ms on the flight controller's clock.
        """
        if self._lidar_clock == 'fc':
            raise ValueError("on_subpage_esp() takes ESP32 time, but the LiDAR is on the "
                             "flight controller's clock (on_lidar): use on_lidar_esp()")
        # The data-ready flag marks the END of the subpage's exposure; the
        # pixels describe its middle. Subpage pair-averaging does not
        # remove this offset: both halves are stamped at the end of their
        # own exposure, so both carry it, in the same direction, and the
        # average carries it too. Averaging cancels the difference between
        # the halves, not an error they share. So it is subtracted here,
        # from each subpage, before anything else uses the time.
        t_mid_us = t_ready_us - self.exposure_mid_offset_us
        t_fc_ms = self.clock.to_fc_ms(t_mid_us)
        if t_fc_ms is None:
            # No valid mapping for this instant: refuse, and count it. A
            # zero or remembered offset would give plausible wrong
            # positions instead of a visible gap.
            self.subpages_received += 1
            self.rejected_no_clock_sync += 1
            return self._take_carry() + self._housekeeping(
                self.clock.approx_fc_ms(t_mid_us), subpage_index)
        # Whole milliseconds, like every other time_boot_ms. Rounding moves
        # the lookup by at most 0.5 ms, about 1 mm at 30 deg/s and 5 m.
        return self._chain(int(round(t_fc_ms)), subpage_index, pixels, t_mid_us / 1000.0)

    def _chain(self, time_boot_ms, subpage_index, pixels, lidar_now_ms):
        """Steps 1 to 11. time_boot_ms is the subpage's instant on the
        flight controller's clock; lidar_now_ms is the same instant on the
        LiDAR's clock."""
        self.subpages_received += 1
        pre_results = self._take_carry() + self._housekeeping(time_boot_ms, subpage_index)

        # --- Attitude ---
        quat = self.attitude.at(time_boot_ms)
        if quat is None:
            self.rejected_no_attitude += 1
            return pre_results

        R_nb = pb.quaternion_to_rotation(np.asarray(quat, dtype=float))

        # --- LiDAR ---
        height_m, reason = self._lidar_height(lidar_now_ms, R_nb)
        if height_m is None:
            return pre_results

        # --- Detect and transform ---
        detections = cd.detect_centroids(
            np.asarray(pixels, dtype=float), **self.detector_kwargs)
        half_results = self._transform_detections(detections, height_m, R_nb)

        self.frames_processed += 1
        if not detections:
            self.frames_no_detection += 1

        current_half = {
            't_ms': time_boot_ms,
            'subpage': subpage_index,
            'results': half_results,
            'height_m': height_m,
            'quat': tuple(np.asarray(quat, dtype=float)),
            'pixels': np.asarray(pixels, dtype=np.float32),
        }

        # --- Pair or hold ---
        if self._pending_half is not None:
            # Matching subpage arrived. Order into sp0 / sp1.
            pending = self._pending_half
            self._pending_half = None
            if pending['subpage'] == 0:
                half0, half1 = pending, current_half
            else:
                half0, half1 = current_half, pending

            paired_results = self._pair(half0, half1)
            self.frames_paired += 1

            if self._lat_deg is None:
                self.frames_without_position += 1

            if self.log is not None:
                # Log once per complete frame. Use sp0's timestamp,
                # attitude and pixel buffer. The sp1 quaternion is not
                # stored; the layout is frozen. A post-flight replay
                # should use sp0's attitude for sp0's pixels, which is
                # what this record holds.
                #
                # write_frame() returns False when the writer's queue is
                # full; the writer counts that refusal itself, and it
                # surfaces here as log_records_dropped_queue_full.
                accepted = self.log.write_frame(
                    time_boot_ms=half0['t_ms'],
                    subpage=0,
                    pixels=half0['pixels'],
                    lidar_distance_m=self._lidar_distance_m,
                    lidar_strength=(self._lidar_strength
                                    if self._lidar_strength is not None else 0),
                    quat=np.asarray(half0['quat'], dtype=np.float32),
                    lat_deg=(self._lat_deg if self._lat_deg is not None
                             else float('nan')),
                    lon_deg=(self._lon_deg if self._lon_deg is not None
                             else float('nan')),
                    alt_m=(self._alt_m if self._alt_m is not None
                           else float('nan')),
                )
                if accepted:
                    self.frames_logged += 1

            return pre_results + paired_results

        else:
            # Hold until the matching subpage arrives.
            self._pending_half = current_half
            return pre_results

    # ------------------------------------------------------------------

    def counters(self):
        """All counters as a dict, including the interpolator's own four
        (summed over every buffer a reboot retired), the clock
        estimator's, prefixed clock_, and the log writer's, prefixed log_.
        The log_ keys are always present, None when logging is disabled,
        so telemetry has one schema whether or not a log is open."""
        if self._log_writer is not None:
            log_counters = self._log_writer.counters()
        else:
            log_counters = dict.fromkeys(_LOG_COUNTER_KEYS)
        att = {k: self._attitude_retired[k] + getattr(self.attitude, k)
               for k in _ATTITUDE_BUFFER_COUNTERS}
        return {
            'attitude_received': self.attitude_received,
            'attitude_rejected_out_of_order': self.attitude_rejected_out_of_order,
            'lidar_received': self.lidar_received,
            'lidar_rejected_out_of_order': self.lidar_rejected_out_of_order,
            'position_received': self.position_received,
            'position_rejected_out_of_order': self.position_rejected_out_of_order,
            'fc_reboots': self.fc_reboots,
            'esp_restarts': self.esp_restarts,
            'subpages_received': self.subpages_received,
            'rejected_no_clock_sync': self.rejected_no_clock_sync,
            'rejected_no_attitude': self.rejected_no_attitude,
            'rejected_no_lidar': self.rejected_no_lidar,
            'rejected_lidar_stale': self.rejected_lidar_stale,
            'rejected_lidar_unusable': self.rejected_lidar_unusable,
            'rejected_ray_not_downward': self.rejected_ray_not_downward,
            'frames_without_position': self.frames_without_position,
            'frames_no_detection': self.frames_no_detection,
            'frames_processed': self.frames_processed,
            'frames_paired': self.frames_paired,
            'frames_logged': self.frames_logged,
            'halves_timed_out': self.halves_timed_out,
            'detections_matched': self.detections_matched,
            'detections_unmatched_sp0': self.detections_unmatched_sp0,
            'detections_unmatched_sp1': self.detections_unmatched_sp1,
            'detections_reported': self.detections_reported,
            'detections_touching_edge': self.detections_touching_edge,
            'attitude_rejected_too_old': att['rejected_too_old'],
            'attitude_rejected_too_new': att['rejected_too_new'],
            'attitude_rejected_gap': att['rejected_gap'],
            'attitude_queries_served': att['queries_served'],
            **{f'clock_{k}': v for k, v in self.clock.counters().items()},
            **{f'log_{k}': v for k, v in log_counters.items()},
        }

    def close(self):
        """Flush any pending half, close the log, and return the flushed
        detections (paired=False), the same way on_subpage() returns a
        timed-out half. A target seen only in the last subpage before
        shutdown is still a target. The list is also kept in
        results_at_close, so it survives a `with` block, whose __exit__
        cannot hand it back. Results a reboot flushed and no later
        on_subpage() call collected come back here too."""
        flushed = self._take_carry()
        if self._pending_half is not None:
            self.halves_timed_out += 1
            flushed += self._flush_half(self._pending_half, None)
            self._pending_half = None
        if self.log is not None:
            self.log.close()
            self.log = None
        self.results_at_close = flushed
        return flushed

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
