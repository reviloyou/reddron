// mlx90640_stream.cpp
//
// Streams raw MLX90640 data to the laptop over USB at 921600 baud.
// Raw ADC words, not temperatures and not CSV. The Melexis compensation
// runs on the laptop in frame_reader.py, for two reasons: the ESP32 does
// less work in its hot loop, and the raw measurement stays recoverable if
// a compensation bug turns up later. The same principle as logging raw
// LiDAR frames.
//
// What is sent, and why more than the 768 pixels
// ----------------------------------------------
// The Melexis compensation cannot run on the 768 pixel words alone. It
// also needs the 64 auxiliary RAM words (supply voltage, ambient
// temperature sensor, gain and the two compensation pixels, measured
// alongside every subpage), the control register (resolution and
// chess/interleaved mode, which decide how the words are read), the
// subpage number, and once per sensor the 832-word EEPROM calibration
// dump. So every subpage packet carries the full 832-word RAM image plus
// the status and control registers, exactly what the Melexis driver's
// MLX90640_GetFrameData collects into its 834-word frameData, and the
// EEPROM goes out in its own packet at boot and every 5 s after that, so
// a reader that attaches mid-stream can still compensate.
//
// Why 921600 baud. One subpage packet is 1702 bytes. At 8 subpages per
// second that is 13.6 kB/s, about 136 kbit/s once start and stop bits
// are counted, which does not fit at 115200 at all. At 921600 it uses
// about 15 percent of the link.
//
// Why 400 kHz I2C. Each subpage is 832 RAM words plus two registers,
// about 1.7 kB and roughly 15,000 clock cycles once addressing and ACKs
// are counted. At 100 kHz that is about 150 ms, longer than the 125 ms
// between subpages at 8 Hz, so frames arrive corrupted or not at all. At
// 400 kHz it takes about 40 ms.
//
// Refresh rate. The MLX90640's rate setting is per subpage: 8 Hz gives
// 8 subpages per second, so complete 768-pixel frames at 4 Hz, with the
// two halves captured 125 ms apart. The orchestrator pairs them.
//
// Framing
// -------
// Every packet: 4-byte magic, uint16 payload length, payload, CRC32 of
// (length + payload). All fields little-endian. The CRC is the standard
// reflected CRC-32 (polynomial 0xEDB88320), the same function as Python's
// zlib.crc32, so the reader checks it with the standard library. The
// reader scans for the magic and resyncs on anything that fails, the same
// approach main.cpp uses for the TFmini's 0x59 0x59 header.
//
//   "MLXS" subpage, payload 1692 bytes:
//     uint32 seq              subpages read from the sensor, sent or not
//     uint32 t_ready_us       ESP32 micros() when the new-data flag was seen
//     uint32 read_us          how long the RAM read took
//     uint32 i2c_errors       cumulative
//     uint32 missed_subpages  cumulative: the subpage bit did not alternate
//     uint32 tx_dropped       cumulative: packets not sent, USB buffer full
//     uint16 status           status register 0x8000 (bit 0 = subpage)
//     uint16 control          control register 0x800D
//     uint16 ram[832]         0x0400..0x073F: 768 pixels, then 64 aux words
//   "MLXE" EEPROM, payload 1668 bytes:
//     uint32 seq
//     uint16 ee[832]          0x2400..0x273F
//   "MLXT" text, payload up to 200 bytes of ASCII: status and errors,
//     sent as packets so they never corrupt the binary stream.
//   "MLXA" attitude pair, pillarb_stream build only, payload 32 bytes:
//     uint32 seq              attitude samples delivered, sent or not
//     uint32 rx_us            ESP32 micros() when the MAVLink message was parsed
//     uint32 time_boot_ms     the flight controller's stamp, from the message
//     float  q[4]             w, x, y, z
//     uint8  flags            bit 0: from the Euler fallback (ATTITUDE, msg 30)
//     uint8  wire_bytes       the message's length on the wire
//     uint16 reserved         0
//
// Timestamps. t_ready_us is taken when the new-data flag is seen, the
// arrival of the subpage at the ESP32. It is on the ESP32's clock, not
// the flight controller's, and it marks the END of the subpage's
// measurement, not its middle. Both are corrected on the laptop, at
// attitude-lookup time, never here: orchestrator.py subtracts half the
// integration time (clock_sync.EXPOSURE_MID_OFFSET_US) and maps the result
// onto the flight controller's clock with an offset estimated from the
// MLXA pairs. Nothing sent here is converted, so a better estimate can
// always be recomputed from a saved capture.
//
// The pillarb_stream build (-DPILLARB_WITH_MAVLINK) runs the MAVLink link
// of mavlink_request.cpp in this same program, so attitude and subpages
// are stamped by the same micros(). The mapping needs that: two separate
// programs on two boards would have two unrelated clocks. The link is
// polled between the I2C chunks of every read, so an attitude message
// waits at most about one chunk (3 ms) to be parsed and stamped, not a
// whole 40 ms RAM read. The blocking stream setup (begin()) runs only once
// a flight controller heartbeat has been heard, so a bench without an FC
// keeps streaming, and each run pauses the camera for a few seconds, which
// an MLXT packet reports with its duration.

#include <Arduino.h>
#include <Wire.h>

#ifdef PILLARB_WITH_MAVLINK
#include "mavlink_request.h"
#endif

namespace {

const uint8_t kAddr = 0x33;          // MLX90640 factory I2C address
const int kSdaPin = 21;
const int kSclPin = 22;
const uint32_t kI2cHz = 400000;
const uint32_t kUsbBaud = 921600;
const uint8_t kRefreshCode = 4;      // 0.5,1,2,4,8,16,32,64 Hz -> code 4 = 8 Hz per subpage
const uint32_t kEepromRepeatMs = 5000;

const uint16_t kRegStatus = 0x8000;
const uint16_t kRegControl = 0x800D;
const uint16_t kRamStart = 0x0400;
const uint16_t kRamWords = 832;
const uint16_t kEeStart = 0x2400;
const uint16_t kEeWords = 832;
// Melexis MLX90640_INIT_STATUS_VALUE: clears the new-data flag and keeps
// RAM overwrite enabled.
const uint16_t kStatusInit = 0x0030;
const uint16_t kStatusNewData = 0x0008;
// Words per I2C read. 64 words is 128 bytes, the ESP32 Wire buffer.
const uint16_t kChunkWords = 64;

const size_t kSubpagePayload = 6 * 4 + 2 * 2 + kRamWords * 2;  // 1692
const size_t kEepromPayload = 4 + kEeWords * 2;                // 1668
const size_t kTextMax = 200;
const size_t kAttitudePayload = 3 * 4 + 4 * 4 + 1 + 1 + 2;     // 32
const size_t kPacketMax = 4 + 2 + kSubpagePayload + 4;         // 1702

uint32_t crc_table[256];
// Subpage and EEPROM packets. Text and attitude packets have their own
// buffers, because in the pillarb_stream build they are sent from inside
// an I2C read, while this one may be half built.
uint8_t packet[kPacketMax];
uint8_t text_packet[4 + 2 + kTextMax + 4];
uint16_t ram[kRamWords];
uint16_t ee[kEeWords];

uint32_t subpage_seq = 0;
uint32_t eeprom_seq = 0;
uint32_t i2c_errors = 0;
uint32_t missed_subpages = 0;
uint32_t tx_dropped = 0;
bool have_last_subpage = false;
uint16_t last_subpage = 0;
uint32_t last_eeprom_ms = 0;
bool sensor_ok = false;

void crcInit() {
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    crc_table[i] = c;
  }
}

uint32_t crc32(const uint8_t* p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  while (n--) c = crc_table[(c ^ *p++) & 0xFF] ^ (c >> 8);
  return ~c;
}

void put16(uint8_t* p, uint16_t v) {
  p[0] = v & 0xFF;
  p[1] = v >> 8;
}

void put32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = (v >> (8 * i)) & 0xFF;
}

// Frames `payload_len` bytes already placed at buf + 6 and sends them.
// A full USB buffer means the laptop is not keeping up: the packet is
// dropped and counted rather than blocking, since blocking here would
// stall the sensor polling and lose subpages instead.
void sendPacket(uint8_t* buf, const char magic[4], size_t payload_len) {
  memcpy(buf, magic, 4);
  put16(buf + 4, static_cast<uint16_t>(payload_len));
  put32(buf + 6 + payload_len, crc32(buf + 4, 2 + payload_len));
  const size_t n = 6 + payload_len + 4;
  if (Serial.availableForWrite() < static_cast<int>(n)) {
    tx_dropped++;
    return;
  }
  Serial.write(buf, n);
}

void sendText(const char* text) {
  size_t len = strlen(text);
  if (len > kTextMax) len = kTextMax;
  memcpy(text_packet + 6, text, len);
  sendPacket(text_packet, "MLXT", len);
}

#ifdef PILLARB_WITH_MAVLINK
// Must equal SERIALn_BAUD on the FC port, and clock_sync.FC_LINK_BAUD on
// the laptop, which turns wire_bytes into wire time.
#ifndef PILLARB_FC_BAUD
#define PILLARB_FC_BAUD 115200
#endif
const int kFcRxPin = 32;  // ESP32 RX <- FC TX
const int kFcTxPin = 33;  // ESP32 TX -> FC RX
const uint32_t kMavRetryMs = 5000;

// mavlink_request.cpp reports progress as lines of text. Here each line
// goes out as an MLXT packet, so it never corrupts the binary stream.
class TextPacketPrint : public Print {
 public:
  size_t write(uint8_t c) override {
    if (c == '\n') {
      emitLine();
      return 1;
    }
    if (c == '\r') return 1;
    if (len_ == kTextMax) emitLine();
    line_[len_++] = static_cast<char>(c);
    return 1;
  }

 private:
  void emitLine() {
    if (len_ == 0) return;
    line_[len_] = '\0';
    sendText(line_);
    len_ = 0;
  }
  char line_[kTextMax + 1];
  size_t len_ = 0;
};

TextPacketPrint mav_log;
HardwareSerial& fc_link = Serial1;
pillarb::MavlinkStreamSetup mav(fc_link, mav_log);
bool mav_ready = false;
bool mav_attempted = false;
uint32_t mav_last_attempt_ms = 0;
uint32_t attitude_seq = 0;
uint8_t attitude_packet[4 + 2 + kAttitudePayload + 4];

void putFloat(uint8_t* p, float v) {
  uint32_t bits;
  memcpy(&bits, &v, 4);
  put32(p, bits);
}

// Every attitude sample, from either message, with both its clocks.
void onAttitude(const pillarb::AttitudeSample& s) {
  uint8_t* p = attitude_packet + 6;
  put32(p + 0, ++attitude_seq);
  put32(p + 4, s.rx_us);
  put32(p + 8, s.time_boot_ms);
  for (int i = 0; i < 4; ++i) putFloat(p + 12 + 4 * i, s.q[i]);
  p[28] = s.from_euler ? 1 : 0;
  p[29] = s.wire_bytes;
  put16(p + 30, 0);
  sendPacket(attitude_packet, "MLXA", kAttitudePayload);
}
#endif

// Called between I2C chunks and wherever the loop waits. In the
// pillarb_stream build it parses whatever MAVLink bytes have arrived, so
// each message is stamped within a few milliseconds of arriving; in the
// camera-only build it does nothing.
inline void serviceLinks() {
#ifdef PILLARB_WITH_MAVLINK
  mav.poll();
#endif
}

void waitMs(uint32_t ms) {
  const uint32_t t0 = millis();
  do {
    serviceLinks();
    delay(1);
  } while (millis() - t0 < ms);
}

bool readWords(uint16_t reg, uint16_t n, uint16_t* out) {
  while (n > 0) {
    const uint16_t chunk = n < kChunkWords ? n : kChunkWords;
    Wire.beginTransmission(kAddr);
    Wire.write(static_cast<uint8_t>(reg >> 8));
    Wire.write(static_cast<uint8_t>(reg & 0xFF));
    if (Wire.endTransmission(false) != 0) return false;  // repeated start
    const uint8_t bytes = static_cast<uint8_t>(chunk * 2);
    if (Wire.requestFrom(kAddr, bytes) != bytes) return false;
    for (uint16_t i = 0; i < chunk; ++i) {
      const uint16_t hi = Wire.read();  // big-endian on the wire
      const uint16_t lo = Wire.read();
      out[i] = static_cast<uint16_t>((hi << 8) | lo);
    }
    reg += chunk;
    out += chunk;
    n -= chunk;
    serviceLinks();
  }
  return true;
}

bool writeWord(uint16_t reg, uint16_t value) {
  Wire.beginTransmission(kAddr);
  Wire.write(static_cast<uint8_t>(reg >> 8));
  Wire.write(static_cast<uint8_t>(reg & 0xFF));
  Wire.write(static_cast<uint8_t>(value >> 8));
  Wire.write(static_cast<uint8_t>(value & 0xFF));
  return Wire.endTransmission() == 0;
}

void sendEeprom() {
  if (!readWords(kEeStart, kEeWords, ee)) {
    i2c_errors++;
    sendText("ERROR: EEPROM read failed");
    return;
  }
  uint8_t* p = packet + 6;
  put32(p, eeprom_seq++);
  for (uint16_t i = 0; i < kEeWords; ++i) put16(p + 4 + 2 * i, ee[i]);
  sendPacket(packet, "MLXE", kEepromPayload);
}

bool configureSensor() {
  uint16_t control;
  if (!readWords(kRegControl, 1, &control)) {
    sendText("ERROR: no MLX90640 at 0x33. Check SDA=GPIO21, SCL=GPIO22, 3.3 V and GND.");
    return false;
  }
  // Refresh rate lives in bits 7..9 of the control register. The write
  // is volatile: the sensor reverts to its EEPROM setting at power-up,
  // which is why it is made on every boot.
  const uint16_t wanted = static_cast<uint16_t>((control & ~(0x7u << 7)) | (kRefreshCode << 7));
  if (!writeWord(kRegControl, wanted) || !readWords(kRegControl, 1, &control) ||
      control != wanted) {
    sendText("ERROR: could not set the refresh rate");
    return false;
  }
  char msg[kTextMax];
  snprintf(msg, sizeof(msg),
           "MLX90640 ready: control 0x%04X, 8 Hz per subpage (4 Hz frames), %s mode, I2C %lu Hz",
           control, (control & 0x1000) ? "chess" : "interleaved", (unsigned long)kI2cHz);
  sendText(msg);
  return true;
}

}  // namespace

void setup() {
  crcInit();
  // Room for two subpage packets, so a write never has to block while
  // the previous packet is still going out.
  Serial.setTxBufferSize(2 * kPacketMax);
  Serial.begin(kUsbBaud);
  Wire.begin(kSdaPin, kSclPin);
  // Required, not a tuning knob: see the 400 kHz note at the top.
  Wire.setClock(kI2cHz);
  delay(100);
#ifdef PILLARB_WITH_MAVLINK
  // 50 Hz of ATTITUDE_QUATERNION is about 2.2 kB/s; the default 256-byte
  // receive buffer would overflow during a slow loop iteration. Set before
  // begin(). begin() of the stream setup itself waits in loop() for a
  // heartbeat, so a bench without a flight controller still streams.
  fc_link.setRxBufferSize(2048);
  fc_link.begin(PILLARB_FC_BAUD, SERIAL_8N1, kFcRxPin, kFcTxPin);
  mav.onAttitude(onAttitude);
  sendText("pillarb_stream: MAVLink on GPIO32/33, attitude pairs sent as MLXA packets");
#endif
  sensor_ok = configureSensor();
  if (sensor_ok) sendEeprom();
  last_eeprom_ms = millis();
}

#ifdef PILLARB_WITH_MAVLINK
// The flight controller's stream setup, once one has been heard, again
// after it reboots, and every kMavRetryMs while it keeps failing. It
// blocks for a few seconds and the camera pauses meanwhile, so the pause
// is reported with its length rather than left to show up as lost
// subpages nobody can explain.
void serviceMavlinkSetup() {
  if (!mav.fcKnown()) return;
  const bool retry_due = !mav_ready && (!mav_attempted || millis() - mav_last_attempt_ms > kMavRetryMs);
  if (!mav.needsReconfigure() && !retry_due) return;
  const uint32_t t0 = millis();
  mav_ready = mav.begin();
  mav_attempted = true;
  mav_last_attempt_ms = millis();
  char msg[kTextMax];
  snprintf(msg, sizeof(msg), "MAVLink stream setup %s; camera paused %lu ms",
           mav_ready ? "done" : "FAILED, attitude pairs may not arrive",
           (unsigned long)(mav_last_attempt_ms - t0));
  sendText(msg);
}
#endif

void loop() {
  serviceLinks();
#ifdef PILLARB_WITH_MAVLINK
  serviceMavlinkSetup();
#endif
  if (!sensor_ok) {
    waitMs(2000);
    sensor_ok = configureSensor();
    if (sensor_ok) {
      sendEeprom();
      last_eeprom_ms = millis();
    }
    return;
  }

  if (millis() - last_eeprom_ms >= kEepromRepeatMs) {
    sendEeprom();
    last_eeprom_ms = millis();
  }

  uint16_t status;
  if (!readWords(kRegStatus, 1, &status)) {
    i2c_errors++;
    waitMs(5);
    return;
  }
  if (!(status & kStatusNewData)) {
    waitMs(1);
    return;
  }

  // The same order as the Melexis driver's MLX90640_GetFrameData: note
  // the time, clear the flag, then read RAM and the control register.
  const uint32_t t_ready_us = micros();
  subpage_seq++;
  bool ok = writeWord(kRegStatus, kStatusInit);
  const uint32_t t0 = micros();
  uint16_t control = 0;
  ok = ok && readWords(kRamStart, kRamWords, ram) && readWords(kRegControl, 1, &control);
  const uint32_t read_us = micros() - t0;
  if (!ok) {
    // No packet. seq has already moved on, so the reader sees the gap
    // and this counter says it happened on the I2C side.
    i2c_errors++;
    return;
  }

  const uint16_t subpage = status & 0x0001;
  if (have_last_subpage && subpage == last_subpage) missed_subpages++;
  have_last_subpage = true;
  last_subpage = subpage;

  uint8_t* p = packet + 6;
  put32(p + 0, subpage_seq);
  put32(p + 4, t_ready_us);
  put32(p + 8, read_us);
  put32(p + 12, i2c_errors);
  put32(p + 16, missed_subpages);
  put32(p + 20, tx_dropped);
  put16(p + 24, status);
  put16(p + 26, control);
  for (uint16_t i = 0; i < kRamWords; ++i) put16(p + 28 + 2 * i, ram[i]);
  sendPacket(packet, "MLXS", kSubpagePayload);
}
