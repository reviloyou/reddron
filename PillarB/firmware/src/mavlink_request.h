// mavlink_request.h
//
// Configures the flight controller's MAVLink output on the ESP32's own
// link at startup, then decodes what arrives. The why is in
// mavlink_request.cpp; the wiring and the SITL test in HARDWARE_GUIDE.md.

#pragma once

#include <Arduino.h>
// The ardupilotmega dialect, not common: it is a superset, and with
// common every ArduPilot-specific message would fail its CRC-extra lookup
// and be counted as a parse error, which looks exactly like a bad cable.
#include <MAVLink_ardupilotmega.h>

namespace pillarb {

enum class AttitudeSource : uint8_t {
  kNone = 0,           // no attitude obtained; the system must not report ready
  kQuaternion = 1,     // ATTITUDE_QUATERNION (31), the intended path
  kEulerFallback = 2,  // ATTITUDE (30), converted to a quaternion here
};

const char* attitudeSourceName(AttitudeSource s);

// Both clocks, on every sample. time_boot_ms says when the FC stamped the
// message; rx_us says when this ESP32 parsed it. The offset between the two
// clocks can be recovered afterwards from the pair (clock_sync.py); it
// cannot be recovered from either alone. Attitude is stored against
// time_boot_ms, never against rx_us, which carries the link's jitter.
struct AttitudeSample {
  uint32_t time_boot_ms;  // flight controller clock, taken from the message
  uint32_t rx_us;         // ESP32 micros() when the message finished parsing
  float q[4];             // w, x, y, z (Hamilton), body FRD to NED
  uint8_t wire_bytes;     // the message's length on the wire: the known part of its latency
  bool from_euler;        // true on the fallback path
};

struct PositionSample {
  uint32_t time_boot_ms;
  double lat_deg;
  double lon_deg;
  float alt_amsl_m;
  float relative_alt_m;
};

struct DistanceSample {
  uint32_t time_boot_ms;
  float distance_m;
  uint8_t orientation;     // MAV_SENSOR_ORIENTATION
  uint8_t signal_quality;  // 0 unknown, 1 worst, 100 best
};

// One entry per requested message: what was asked for and what happened.
struct StreamRequest {
  uint32_t msg_id;
  const char* name;
  float rate_hz;
  uint8_t attempts;     // COMMAND_LONG transmissions, retries included
  int16_t ack_result;   // MAV_RESULT, or -1 if no ACK ever arrived
  uint32_t received;    // messages of this id since the request was sent
  float measured_hz;    // over the verification window
};

// Every failure path has a counter, the same principle as the Python side.
// Counters are cumulative across reconfigurations.
struct MavlinkCounters {
  uint32_t bytes_rx;
  uint32_t msgs_rx;
  uint32_t rx_parse_errors;     // frames the parser rejected: bad CRC or signature, malformed header
  uint32_t fc_seq_gaps;         // FC messages that never arrived, from sequence number jumps
  uint32_t msgs_unused;         // valid, but not something Pillar B asked for
  uint32_t bytes_unused;        // their wire bytes: bandwidth the stream zeroing failed to reclaim
  uint32_t heartbeats_rx;
  uint32_t commands_sent;
  uint32_t command_retries;
  uint32_t commands_unacked;    // no ACK after every retry
  uint32_t commands_rejected;   // ACKed with anything other than MAV_RESULT_ACCEPTED
  uint32_t duplicate_acks;      // late ACKs for a retried command, discarded
  uint32_t command_int_switches;  // FC answered COMMAND_INT_ONLY, resent as COMMAND_INT
  uint32_t stream_zero_requests;
  uint32_t fallbacks_to_euler;
  uint32_t link_losses;         // no FC heartbeat for kLinkLossMs
  uint32_t attitude_stalls;     // FC alive but attitude stopped (FC reboot, or a GCS changed rates)
  uint32_t attitude_samples;
  uint32_t position_samples;
  uint32_t distance_samples;
};

class MavlinkStreamSetup {
 public:
  using AttitudeCallback = void (*)(const AttitudeSample&);
  using PositionCallback = void (*)(const PositionSample&);
  using DistanceCallback = void (*)(const DistanceSample&);

  // link is the UART (or, for SITL, the TCP client) to the flight
  // controller. log receives human-readable progress and warnings.
  MavlinkStreamSetup(Stream& link, Print& log);

  // Blocking startup sequence, a few seconds. Returns true once attitude
  // is arriving from either source. Call again whenever
  // needsReconfigure() turns true.
  bool begin();

  // Non-blocking. Call every loop iteration, also between begin() calls.
  void poll();

  bool needsReconfigure() const { return needs_reconfigure_; }
  // A flight controller heartbeat has been heard since power-up. Lets a
  // caller that must not block (the camera streamer) hold off begin(),
  // which waits for one, until an FC is actually there.
  bool fcKnown() const { return fc_known_; }
  AttitudeSource attitudeSource() const { return source_; }
  const MavlinkCounters& counters() const { return counters_; }
  uint32_t receivedCount(uint32_t msg_id) const;
  void printStatus(Print& out) const;

  void onAttitude(AttitudeCallback cb) { attitude_cb_ = cb; }
  void onPosition(PositionCallback cb) { position_cb_ = cb; }
  void onDistance(DistanceCallback cb) { distance_cb_ = cb; }

  // MAV_SEVERITY_* text, routed by the FC to the ground station.
  void sendStatusText(uint8_t severity, const char* text);

 private:
  static const uint8_t kNumRequests = 4;

  bool waitForHeartbeat(uint32_t timeout_ms);
  void zeroStreams();
  int16_t requestInterval(StreamRequest& req);
  void measure(uint32_t window_ms);
  void sendSetMessageInterval(uint32_t msg_id, float interval_us, uint8_t confirmation);
  void send(const mavlink_message_t& msg);
  void markUnused(const mavlink_message_t& msg);
  void handleMessage(const mavlink_message_t& msg);
  void deliverAttitude(const mavlink_message_t& msg, uint32_t t_ms, const float q[4],
                       bool from_euler);
  StreamRequest* findRequest(uint32_t msg_id);
  void serviceBackground(uint32_t now_ms);

  Stream& link_;
  Print& log_;

  bool fc_known_ = false;
  uint8_t fc_sysid_ = 0;
  uint8_t fc_compid_ = 0;
  bool have_seq_ = false;
  uint8_t last_seq_ = 0;
  uint32_t rx_us_ = 0;  // micros() when the message being handled finished parsing

  AttitudeSource source_ = AttitudeSource::kNone;
  bool verifying_fallback_ = false;
  bool needs_reconfigure_ = false;
  bool link_lost_ = false;
  bool stall_flagged_ = false;
  uint32_t last_heartbeat_ms_ = 0;
  uint32_t last_heartbeat_tx_ms_ = 0;
  uint32_t last_attitude_rx_ms_ = 0;

  bool use_command_int_ = false;
  bool ack_waiting_ = false;
  bool ack_received_ = false;
  uint16_t ack_command_ = 0;
  uint8_t ack_result_ = 0;

  StreamRequest requests_[kNumRequests];
  MavlinkCounters counters_ = {};

  AttitudeCallback attitude_cb_ = nullptr;
  PositionCallback position_cb_ = nullptr;
  DistanceCallback distance_cb_ = nullptr;
};

}  // namespace pillarb
