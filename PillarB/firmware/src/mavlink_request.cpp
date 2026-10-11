// mavlink_request.cpp
//
// Configures the flight controller's MAVLink output on the ESP32's own
// link at startup: switch off everything Pillar B does not use, then ask
// explicitly for the three messages it does.
//
// Why explicitly. ATTITUDE_QUATERNION (msg 31) is not in any of
// ArduPilot's default stream groups, so raising a stream rate parameter
// never produces it. It has to be requested by message id, with
// MAV_CMD_SET_MESSAGE_INTERVAL inside a COMMAND_LONG.
//
// Why from the ESP32, at startup, rather than as parameters set in Mission
// Planner. Parameters live on the flight controller; this request lives
// in the mission computer. The module configures its own data stream on
// any airframe it is moved to, and survives someone reflashing the FC.
// The same property means it must redo the request after every FC boot:
// message intervals set this way are not stored on the FC. poll() watches
// for that and raises needsReconfigure().
//
// Why the rate matters. The orchestrator interpolates attitude to each
// thermal subpage's timestamp, but interpolation can only be as good as
// the samples it bridges. Ground error at 5 m and 30 deg/s yaw from the
// worst-case staleness between samples: 32.7 cm at 4 Hz, 13.1 cm at
// 10 Hz, 2.6 cm at 50 Hz, against a 20.8 cm total budget. The stream rate
// is the cheapest large win available.
//
// Sequence (begin()):
//   1. Wait for the FC's HEARTBEAT, and learn its system and component id
//      from it rather than assuming 1/1.
//   2. REQUEST_DATA_STREAM with MAV_DATA_STREAM_ALL at rate 0. ArduPilot
//      (GCS_Param.cpp, handle_request_data_stream) applies this to the
//      channel it arrived on, so it zeroes the stream groups (SRn_* in
//      older firmware, MAVn_* in newer) of exactly the ESP32's port,
//      without the ESP32 needing to know which n that is. Where the
//      vehicle persists stream rates it also saves the zeros. It leaves
//      the parameter stream alone, and it is ignored if that port's
//      MAVn_OPTIONS has "ignore streamrate" set.
//      Done FIRST: zeroing a stream group re-initialises the message
//      interval of every message in it (initialise_message_intervals_for_
//      stream), and GLOBAL_POSITION_INT is in one. Zeroing after the
//      explicit requests would silently undo the position request.
//   3. SET_MESSAGE_INTERVAL for ATTITUDE_QUATERNION at 50 Hz. One command
//      outstanding at a time, because COMMAND_ACK names the command but
//      not the message id it was about, so two in flight cannot be told
//      apart. Retried on no ACK with the confirmation field incremented,
//      as the MAVLink command protocol specifies. ArduPilot converts
//      COMMAND_LONG to COMMAND_INT internally; a build without COMMAND_LONG
//      support answers MAV_RESULT_COMMAND_INT_ONLY, and the request is then
//      resent as COMMAND_INT rather than treated as refused.
//   4. Measure what actually arrives. An ACK means the request was
//      accepted, not that the link has the bandwidth for it.
//   5. If msg 31 was refused, never acknowledged, or acknowledged but
//      never arrives: fall back to ATTITUDE (msg 30, Euler angles) at the
//      same rate, and say so loudly. On the serial log, in the counters,
//      and as a STATUSTEXT the FC forwards to the ground station. The
//      fallback reintroduces the Euler conversion that quaternions were
//      meant to avoid (and its singularity at 90 deg pitch), so it must
//      never be in use without someone being able to see that it is.
//   6. GLOBAL_POSITION_INT at 25 Hz and DISTANCE_SENSOR at 10 Hz, measured
//      the same way.
//
// Bandwidth. At 115200 baud the link carries about 11,500 bytes per
// second. What is requested here is about 3.8 kB/s: ATTITUDE_QUATERNION
// 50 x ~44 B, GLOBAL_POSITION_INT 25 x ~40 B, DISTANCE_SENSOR 10 x ~26 B,
// plus heartbeats. That fits at 115200 with room, and still fits at
// 57600 (5,760 B/s) at about two thirds load. The defaults it replaces
// spend most of the link on messages Pillar B never reads.
//
// Timestamps. Every attitude sample carries two clocks: the time_boot_ms
// ArduPilot put in the message (AP_HAL::millis() when it packed it,
// GCS_Common.cpp send_attitude_quaternion) and this ESP32's micros() when
// the parser completed it. The second is taken in poll(), the moment
// mavlink_parse_char() returns a message, before anything else is done
// with it. It includes whatever time the bytes waited in the UART buffer;
// that wait is always positive, and clock_sync.py estimates the offset
// from the least-delayed messages for exactly that reason. The sample
// also carries the message's length on the wire, because its wire time
// is the part of the delay that is known exactly.
//
// Mission Planner on the same link. A GCS requests its own stream rates
// on the link it is connected to, and can override what this module set.
// With the ESP32 on its own FC UART this does not arise. During SITL
// testing it does not either, as long as the ESP32 uses its own SITL
// port; HARDWARE_GUIDE.md covers the case where it does not.

#include "mavlink_request.h"

namespace pillarb {

namespace {

const mavlink_channel_t kChan = MAVLINK_COMM_0;
const uint8_t kMyCompId = MAV_COMP_ID_ONBOARD_COMPUTER;

const float kAttitudeRateHz = 50.0f;
// The orchestrator pairs each frame with the newest position fix rather
// than interpolating, so the fix's age adds to the translation error
// directly. At 25 Hz that age is at most 40 ms, 20 ms on average: 6 cm at
// 3 m/s. The EKF position updates far faster than GPS, so this is a real
// 25 Hz, not a 5 Hz GPS repeated.
const float kPositionRateHz = 25.0f;
// Only meaningful on an airframe whose FC has its own rangefinder. On
// this one it does not (RNGFND1_TYPE = 0): the TFmini-S is wired to the
// ESP32, and height comes from there.
const float kDistanceRateHz = 10.0f;

const uint32_t kHeartbeatWaitMs = 10000;
const uint32_t kAckTimeoutMs = 500;
const uint8_t kMaxAttempts = 4;           // first try plus three retries
const uint32_t kVerifyWindowMs = 1000;
const uint32_t kLinkLossMs = 3000;        // three missed 1 Hz heartbeats
const uint32_t kAttitudeStallMs = 500;    // 25 missed samples at 50 Hz
const uint32_t kHeartbeatTxMs = 1000;

// Measured below half the requested rate is reported as a warning: the
// request was granted but the link or the FC scheduler cannot keep up.
const float kMinRateFraction = 0.5f;

// A message's length on the wire as it arrived: MAVLink 1 or 2 header,
// the payload after MAVLink 2's trailing-zero truncation, the checksum,
// and the signature when there is one.
uint8_t wireBytes(const mavlink_message_t& msg) {
  if (msg.magic == MAVLINK_STX_MAVLINK1) {
    return static_cast<uint8_t>(msg.len + 1 + MAVLINK_CORE_HEADER_MAVLINK1_LEN +
                                MAVLINK_NUM_CHECKSUM_BYTES);
  }
  uint16_t n = msg.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
  if (msg.incompat_flags & MAVLINK_IFLAG_SIGNED) n += MAVLINK_SIGNATURE_BLOCK_LEN;
  return static_cast<uint8_t>(n);
}

}  // namespace

const char* attitudeSourceName(AttitudeSource s) {
  switch (s) {
    case AttitudeSource::kQuaternion: return "ATTITUDE_QUATERNION";
    case AttitudeSource::kEulerFallback: return "ATTITUDE (Euler fallback)";
    default: return "none";
  }
}

MavlinkStreamSetup::MavlinkStreamSetup(Stream& link, Print& log)
    : link_(link), log_(log) {
  requests_[0] = {MAVLINK_MSG_ID_ATTITUDE_QUATERNION, "ATTITUDE_QUATERNION", kAttitudeRateHz, 0, -1, 0, 0};
  requests_[1] = {MAVLINK_MSG_ID_ATTITUDE, "ATTITUDE", kAttitudeRateHz, 0, -1, 0, 0};
  requests_[2] = {MAVLINK_MSG_ID_GLOBAL_POSITION_INT, "GLOBAL_POSITION_INT", kPositionRateHz, 0, -1, 0, 0};
  requests_[3] = {MAVLINK_MSG_ID_DISTANCE_SENSOR, "DISTANCE_SENSOR", kDistanceRateHz, 0, -1, 0, 0};
}

StreamRequest* MavlinkStreamSetup::findRequest(uint32_t msg_id) {
  for (uint8_t i = 0; i < kNumRequests; ++i) {
    if (requests_[i].msg_id == msg_id) return &requests_[i];
  }
  return nullptr;
}

uint32_t MavlinkStreamSetup::receivedCount(uint32_t msg_id) const {
  for (uint8_t i = 0; i < kNumRequests; ++i) {
    if (requests_[i].msg_id == msg_id) return requests_[i].received;
  }
  return 0;
}

bool MavlinkStreamSetup::begin() {
  source_ = AttitudeSource::kNone;
  verifying_fallback_ = false;
  needs_reconfigure_ = false;
  stall_flagged_ = false;
  for (uint8_t i = 0; i < kNumRequests; ++i) {
    requests_[i].attempts = 0;
    requests_[i].ack_result = -1;
    requests_[i].received = 0;
    requests_[i].measured_hz = 0.0f;
  }

  log_.println(F("[mavlink] waiting for flight controller heartbeat"));
  if (!waitForHeartbeat(kHeartbeatWaitMs)) {
    log_.println(F("[mavlink] ERROR: no heartbeat from a flight controller. "
                   "Check wiring (TX to RX), baud rate and SERIALn_PROTOCOL = 2."));
    return false;
  }
  log_.printf("[mavlink] flight controller found: sysid %u compid %u\n", fc_sysid_, fc_compid_);

  zeroStreams();

  // Attitude: quaternion first, Euler only if that fails.
  StreamRequest& quat = requests_[0];
  requestInterval(quat);
  quat.received = 0;
  measure(kVerifyWindowMs);
  if (quat.ack_result == MAV_RESULT_ACCEPTED && quat.received > 0) {
    source_ = AttitudeSource::kQuaternion;
  } else {
    counters_.fallbacks_to_euler++;
    if (quat.ack_result < 0) {
      log_.println(F("[mavlink] WARNING: ATTITUDE_QUATERNION request was never acknowledged"));
    } else if (quat.ack_result != MAV_RESULT_ACCEPTED) {
      log_.printf("[mavlink] WARNING: ATTITUDE_QUATERNION request refused, MAV_RESULT %d\n",
                  quat.ack_result);
    } else {
      log_.println(F("[mavlink] WARNING: ATTITUDE_QUATERNION accepted but none arrived"));
    }
    log_.println(F("[mavlink] WARNING: FALLING BACK to ATTITUDE (Euler angles). "
                   "The Euler conversion is back in the chain."));
    sendStatusText(MAV_SEVERITY_WARNING, "PillarB: no ATTITUDE_QUATERNION, Euler fallback");

    StreamRequest& euler = requests_[1];
    verifying_fallback_ = true;
    requestInterval(euler);
    euler.received = 0;
    measure(kVerifyWindowMs);
    verifying_fallback_ = false;
    if (euler.ack_result == MAV_RESULT_ACCEPTED && euler.received > 0) {
      source_ = AttitudeSource::kEulerFallback;
    } else {
      log_.println(F("[mavlink] ERROR: no attitude from either message. NOT READY."));
      sendStatusText(MAV_SEVERITY_CRITICAL, "PillarB: no attitude stream, not ready");
      return false;
    }
  }

  // Position and distance: requested one at a time, measured together.
  requestInterval(requests_[2]);
  requestInterval(requests_[3]);
  requests_[2].received = 0;
  requests_[3].received = 0;
  measure(kVerifyWindowMs);

  printStatus(log_);
  if (requests_[2].received == 0) {
    log_.println(F("[mavlink] WARNING: no GLOBAL_POSITION_INT. Frames will be logged "
                   "without lat/lon (orchestrator counts frames_without_position)."));
  }
  if (requests_[3].received == 0) {
    log_.println(F("[mavlink] note: no DISTANCE_SENSOR from the FC. Expected when the FC has "
                   "no rangefinder (RNGFND1_TYPE = 0); height comes from the TFmini on UART2."));
  }
  last_attitude_rx_ms_ = millis();
  return true;
}

bool MavlinkStreamSetup::waitForHeartbeat(uint32_t timeout_ms) {
  const uint32_t t0 = millis();
  const uint32_t before = counters_.heartbeats_rx;
  while (millis() - t0 < timeout_ms) {
    poll();
    if (fc_known_ && counters_.heartbeats_rx != before) return true;
    delay(1);
  }
  return false;
}

void MavlinkStreamSetup::zeroStreams() {
  // REQUEST_DATA_STREAM has no ACK, so it is sent twice, a little apart,
  // in case the first is lost on the wire. The effect shows up in
  // bytes_unused, which should fall to almost nothing.
  for (uint8_t i = 0; i < 2; ++i) {
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(fc_sysid_, kMyCompId, &msg, fc_sysid_, fc_compid_,
                                         MAV_DATA_STREAM_ALL, 0, 0);
    send(msg);
    counters_.stream_zero_requests++;
    const uint32_t t0 = millis();
    while (millis() - t0 < 50) {
      poll();
      delay(1);
    }
  }
  log_.println(F("[mavlink] requested all stream groups off on this link"));
}

int16_t MavlinkStreamSetup::requestInterval(StreamRequest& req) {
  const float interval_us = 1.0e6f / req.rate_hz;
  for (uint8_t attempt = 0; attempt < kMaxAttempts; ++attempt) {
    ack_waiting_ = true;
    ack_received_ = false;
    sendSetMessageInterval(req.msg_id, interval_us, attempt);
    req.attempts++;
    if (attempt > 0) counters_.command_retries++;

    const uint32_t t0 = millis();
    while (millis() - t0 < kAckTimeoutMs && !ack_received_) {
      poll();
      delay(1);
    }
    if (!ack_received_) continue;

    // ArduPilot built without COMMAND_LONG support refuses every one
    // with this code. That is not a refusal of the request itself: send
    // it again as COMMAND_INT, and keep using COMMAND_INT from then on.
    if (ack_result_ == MAV_RESULT_COMMAND_INT_ONLY && !use_command_int_) {
      use_command_int_ = true;
      counters_.command_int_switches++;
      log_.println(F("[mavlink] FC accepts commands only as COMMAND_INT; resending"));
      continue;
    }

    req.ack_result = ack_result_;
    if (attempt > 0) {
      // A retried command can be acknowledged twice. Swallow the late ACK
      // here, before the next command goes out and mistakes it for its own.
      const uint32_t t1 = millis();
      while (millis() - t1 < kAckTimeoutMs) {
        poll();
        delay(1);
      }
    }
    ack_waiting_ = false;
    if (ack_result_ != MAV_RESULT_ACCEPTED) counters_.commands_rejected++;
    log_.printf("[mavlink] %s at %.0f Hz: MAV_RESULT %d after %u attempt(s)\n", req.name,
                req.rate_hz, ack_result_, req.attempts);
    return req.ack_result;
  }
  ack_waiting_ = false;
  counters_.commands_unacked++;
  req.ack_result = -1;
  log_.printf("[mavlink] %s: no ACK after %u attempts\n", req.name, kMaxAttempts);
  return -1;
}

void MavlinkStreamSetup::measure(uint32_t window_ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < window_ms) {
    poll();
    delay(1);
  }
  const float seconds = window_ms / 1000.0f;
  for (uint8_t i = 0; i < kNumRequests; ++i) {
    StreamRequest& r = requests_[i];
    if (r.attempts == 0) continue;
    r.measured_hz = r.received / seconds;
    if (r.received > 0 && r.measured_hz < kMinRateFraction * r.rate_hz) {
      log_.printf("[mavlink] WARNING: %s arriving at %.1f Hz, requested %.0f Hz. "
                  "Link bandwidth or FC scheduling is limiting it.\n",
                  r.name, r.measured_hz, r.rate_hz);
    }
  }
}

void MavlinkStreamSetup::sendSetMessageInterval(uint32_t msg_id, float interval_us,
                                                uint8_t confirmation) {
  // param1 message id, param2 interval in microseconds. param3 must be 0:
  // ArduPilot denies the command otherwise.
  ack_command_ = MAV_CMD_SET_MESSAGE_INTERVAL;
  mavlink_message_t msg;
  if (use_command_int_) {
    // COMMAND_INT has no confirmation field; a retry is the same message.
    mavlink_msg_command_int_pack(fc_sysid_, kMyCompId, &msg, fc_sysid_, fc_compid_,
                                 MAV_FRAME_GLOBAL, MAV_CMD_SET_MESSAGE_INTERVAL, 0, 0,
                                 static_cast<float>(msg_id), interval_us, 0, 0, 0, 0, 0);
  } else {
    mavlink_msg_command_long_pack(fc_sysid_, kMyCompId, &msg, fc_sysid_, fc_compid_,
                                  MAV_CMD_SET_MESSAGE_INTERVAL, confirmation,
                                  static_cast<float>(msg_id), interval_us, 0, 0, 0, 0, 0);
  }
  send(msg);
  counters_.commands_sent++;
}

void MavlinkStreamSetup::markUnused(const mavlink_message_t& msg) {
  counters_.msgs_unused++;
  counters_.bytes_unused += msg.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}

void MavlinkStreamSetup::send(const mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  link_.write(buf, len);
}

void MavlinkStreamSetup::sendStatusText(uint8_t severity, const char* text) {
  if (!fc_known_) return;
  // The generated pack function may copy the full 50-byte field from the
  // pointer it is given, so a short string goes through a zeroed buffer.
  char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = {};
  strncpy(buf, text, sizeof(buf));
  mavlink_message_t msg;
  mavlink_msg_statustext_pack(fc_sysid_, kMyCompId, &msg, severity, buf, 0, 0);
  send(msg);
}

void MavlinkStreamSetup::poll() {
  mavlink_status_t* chan_status = mavlink_get_channel_status(kChan);
  while (link_.available() > 0) {
    const int c = link_.read();
    if (c < 0) break;
    counters_.bytes_rx++;
    // The parser's own error count is 8 bits and wraps; take the delta per
    // byte so the 32-bit counter here stays exact.
    const uint8_t errors_before = chan_status->parse_error;
    mavlink_message_t msg;
    mavlink_status_t status;
    if (mavlink_parse_char(kChan, static_cast<uint8_t>(c), &msg, &status)) {
      rx_us_ = micros();
      handleMessage(msg);
    }
    counters_.rx_parse_errors += static_cast<uint8_t>(chan_status->parse_error - errors_before);
  }
  serviceBackground(millis());
}

void MavlinkStreamSetup::serviceBackground(uint32_t now_ms) {
  if (!fc_known_) return;

  if (now_ms - last_heartbeat_tx_ms_ >= kHeartbeatTxMs) {
    last_heartbeat_tx_ms_ = now_ms;
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(fc_sysid_, kMyCompId, &msg, MAV_TYPE_ONBOARD_CONTROLLER,
                               MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    send(msg);
  }

  if (!link_lost_ && now_ms - last_heartbeat_ms_ > kLinkLossMs) {
    link_lost_ = true;
    counters_.link_losses++;
    log_.println(F("[mavlink] WARNING: flight controller heartbeat lost"));
  }

  // The FC is alive but attitude stopped: it rebooted (its message
  // intervals are gone) or a GCS on the same link changed the rates.
  // Either way the fix is to run the request sequence again.
  if (source_ != AttitudeSource::kNone && !link_lost_ && !stall_flagged_ &&
      now_ms - last_attitude_rx_ms_ > kAttitudeStallMs) {
    stall_flagged_ = true;
    counters_.attitude_stalls++;
    needs_reconfigure_ = true;
    log_.println(F("[mavlink] WARNING: attitude stopped while the FC is alive; reconfigure needed"));
  }
}

void MavlinkStreamSetup::deliverAttitude(const mavlink_message_t& msg, uint32_t t_ms,
                                         const float q[4], bool from_euler) {
  last_attitude_rx_ms_ = millis();
  stall_flagged_ = false;
  counters_.attitude_samples++;
  if (attitude_cb_ != nullptr) {
    AttitudeSample s;
    s.time_boot_ms = t_ms;
    s.rx_us = rx_us_;
    for (uint8_t i = 0; i < 4; ++i) s.q[i] = q[i];
    s.wire_bytes = wireBytes(msg);
    s.from_euler = from_euler;
    attitude_cb_(s);
  }
}

void MavlinkStreamSetup::handleMessage(const mavlink_message_t& msg) {
  counters_.msgs_rx++;

  if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);
    // A flight controller, not a GCS, a camera or this module's own echo.
    if (hb.autopilot == MAV_AUTOPILOT_INVALID) {
      markUnused(msg);
      return;
    }
    if (!fc_known_) {
      fc_known_ = true;
      fc_sysid_ = msg.sysid;
      fc_compid_ = msg.compid;
    }
    if (msg.sysid != fc_sysid_ || msg.compid != fc_compid_) {
      markUnused(msg);
      return;
    }
    counters_.heartbeats_rx++;
    last_heartbeat_ms_ = millis();
    if (link_lost_) {
      link_lost_ = false;
      // After a gap long enough to be a reboot, assume it was one.
      needs_reconfigure_ = true;
      log_.println(F("[mavlink] flight controller heartbeat back; reconfigure needed"));
    }
  }

  if (fc_known_ && msg.sysid == fc_sysid_ && msg.compid == fc_compid_) {
    if (have_seq_) {
      counters_.fc_seq_gaps += static_cast<uint8_t>(msg.seq - last_seq_ - 1);
    }
    have_seq_ = true;
    last_seq_ = msg.seq;
  }

  StreamRequest* req = findRequest(msg.msgid);
  if (req != nullptr) req->received++;

  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      return;

    case MAVLINK_MSG_ID_COMMAND_ACK: {
      mavlink_command_ack_t ack;
      mavlink_msg_command_ack_decode(&msg, &ack);
      const bool for_us = ack.target_system == 0 ||
                          (ack.target_system == fc_sysid_ && ack.target_component == kMyCompId);
      if (!for_us || msg.sysid != fc_sysid_) {
        markUnused(msg);
        return;
      }
      if (ack_waiting_ && !ack_received_ && ack.command == ack_command_) {
        ack_received_ = true;
        ack_result_ = ack.result;
      } else {
        counters_.duplicate_acks++;
      }
      return;
    }

    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
      if (source_ == AttitudeSource::kEulerFallback) {
        markUnused(msg);
        return;
      }
      mavlink_attitude_quaternion_t a;
      mavlink_msg_attitude_quaternion_decode(&msg, &a);
      const float q[4] = {a.q1, a.q2, a.q3, a.q4};
      deliverAttitude(msg, a.time_boot_ms, q, false);
      return;
    }

    case MAVLINK_MSG_ID_ATTITUDE: {
      if (source_ != AttitudeSource::kEulerFallback && !verifying_fallback_) {
        markUnused(msg);
        return;
      }
      mavlink_attitude_t a;
      mavlink_msg_attitude_decode(&msg, &a);
      float q[4];
      // 3-2-1 (yaw, pitch, roll), the same convention as ArduPilot and as
      // pillar_b.euler_to_rotation, so the result feeds the same chain.
      mavlink_euler_to_quaternion(a.roll, a.pitch, a.yaw, q);
      deliverAttitude(msg, a.time_boot_ms, q, true);
      return;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t p;
      mavlink_msg_global_position_int_decode(&msg, &p);
      counters_.position_samples++;
      if (position_cb_ != nullptr) {
        PositionSample s;
        s.time_boot_ms = p.time_boot_ms;
        s.lat_deg = p.lat * 1e-7;
        s.lon_deg = p.lon * 1e-7;
        s.alt_amsl_m = p.alt * 1e-3f;
        s.relative_alt_m = p.relative_alt * 1e-3f;
        position_cb_(s);
      }
      return;
    }

    case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
      mavlink_distance_sensor_t d;
      mavlink_msg_distance_sensor_decode(&msg, &d);
      counters_.distance_samples++;
      if (distance_cb_ != nullptr) {
        DistanceSample s;
        s.time_boot_ms = d.time_boot_ms;
        s.distance_m = d.current_distance * 0.01f;
        s.orientation = d.orientation;
        s.signal_quality = d.signal_quality;
        distance_cb_(s);
      }
      return;
    }

    case MAVLINK_MSG_ID_STATUSTEXT: {
      // The FC explains refusals here ("No ap_message for mavlink id"),
      // so it goes to the log rather than being discarded.
      mavlink_statustext_t t;
      mavlink_msg_statustext_decode(&msg, &t);
      char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
      memcpy(text, t.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
      log_.printf("[mavlink] FC says (severity %u): %s\n", t.severity, text);
      return;
    }

    default:
      markUnused(msg);
      return;
  }
}

void MavlinkStreamSetup::printStatus(Print& out) const {
  out.printf("[mavlink] attitude source: %s\n", attitudeSourceName(source_));
  for (uint8_t i = 0; i < kNumRequests; ++i) {
    const StreamRequest& r = requests_[i];
    if (r.attempts == 0) continue;
    out.printf("[mavlink]   %-20s requested %4.0f Hz  ack %2d  attempts %u  measured %5.1f Hz\n",
               r.name, r.rate_hz, r.ack_result, r.attempts, r.measured_hz);
  }
  const MavlinkCounters& c = counters_;
  out.printf("[mavlink]   rx %lu msgs %lu B, parse errors %lu, FC seq gaps %lu, unused %lu msgs %lu B\n",
             (unsigned long)c.msgs_rx, (unsigned long)c.bytes_rx, (unsigned long)c.rx_parse_errors,
             (unsigned long)c.fc_seq_gaps, (unsigned long)c.msgs_unused, (unsigned long)c.bytes_unused);
  out.printf("[mavlink]   commands %lu sent, %lu retries, %lu unacked, %lu rejected, %lu duplicate acks%s\n",
             (unsigned long)c.commands_sent, (unsigned long)c.command_retries,
             (unsigned long)c.commands_unacked, (unsigned long)c.commands_rejected,
             (unsigned long)c.duplicate_acks, use_command_int_ ? ", using COMMAND_INT" : "");
  out.printf("[mavlink]   fallbacks to Euler %lu, link losses %lu, attitude stalls %lu\n",
             (unsigned long)c.fallbacks_to_euler, (unsigned long)c.link_losses,
             (unsigned long)c.attitude_stalls);
}

}  // namespace pillarb
