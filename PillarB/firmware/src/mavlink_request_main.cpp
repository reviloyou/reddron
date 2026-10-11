// mavlink_request_main.cpp
//
// Bench and SITL harness for mavlink_request.cpp. Runs the startup
// sequence, then once a second prints the measured rate of every
// requested message, the latest attitude, and the link counters over USB.
// HARDWARE_GUIDE.md shows the output to expect.
//
// Two builds of the same code:
//   env:mavlink_request       real flight controller on UART1
//                             ESP32 GPIO32 (RX) <- FC TELEM TX
//                             ESP32 GPIO33 (TX) -> FC TELEM RX
//   env:mavlink_request_sitl  ArduPilot SITL over WiFi TCP. Needs
//                             include/secrets.h (copy secrets.example.h).

#include <Arduino.h>

#include "mavlink_request.h"

#ifdef PILLARB_SITL_TCP
#include <WiFi.h>
#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy include/secrets.example.h to include/secrets.h and fill it in (HARDWARE_GUIDE.md, SITL section)."
// Placeholders so the line above is the only error the compiler reports.
#define PILLARB_WIFI_SSID ""
#define PILLARB_WIFI_PASSWORD ""
#define PILLARB_SITL_HOST ""
#define PILLARB_SITL_PORT 0
#endif
static WiFiClient fc_link;
#else
// Must equal SERIALn_BAUD on the FC port the ESP32 is wired to, where 115
// means 115200. The parameter dump for this airframe has the telemetry
// ports at 57 (57600); either change the FC or build with
// -DPILLARB_FC_BAUD=57600.
#ifndef PILLARB_FC_BAUD
#define PILLARB_FC_BAUD 115200
#endif
static const int kFcRxPin = 32;  // ESP32 RX <- FC TX
static const int kFcTxPin = 33;  // ESP32 TX -> FC RX
static HardwareSerial& fc_link = Serial1;
#endif

static pillarb::MavlinkStreamSetup mav(fc_link, Serial);
static bool configured = false;
static uint32_t last_attempt_ms = 0;
static uint32_t last_print_ms = 0;
static uint32_t last_counts[4] = {};
static const uint32_t kIds[4] = {MAVLINK_MSG_ID_ATTITUDE_QUATERNION, MAVLINK_MSG_ID_ATTITUDE,
                                 MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAVLINK_MSG_ID_DISTANCE_SENSOR};
static const char* const kShort[4] = {"ATT_Q", "ATT", "POS", "DIST"};

static volatile bool have_attitude = false;
static pillarb::AttitudeSample latest_attitude;

static void onAttitude(const pillarb::AttitudeSample& s) {
  latest_attitude = s;
  have_attitude = true;
}

#ifdef PILLARB_SITL_TCP
static void connectSitl() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("[sitl] joining WiFi '%s'", PILLARB_WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(PILLARB_WIFI_SSID, PILLARB_WIFI_PASSWORD);
    const uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
      delay(500);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("[sitl] ERROR: WiFi did not connect. Check SSID and password (2.4 GHz only)."));
      return;
    }
    Serial.printf("[sitl] WiFi up, ESP32 address %s\n", WiFi.localIP().toString().c_str());
  }
  Serial.printf("[sitl] connecting to %s:%d\n", PILLARB_SITL_HOST, PILLARB_SITL_PORT);
  if (!fc_link.connect(PILLARB_SITL_HOST, PILLARB_SITL_PORT)) {
    Serial.println(F("[sitl] ERROR: TCP connect failed. Is SITL running, is the port right, "
                     "and does Windows Firewall allow it?"));
    return;
  }
  fc_link.setNoDelay(true);
  Serial.println(F("[sitl] TCP connected"));
}
#endif

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("Pillar B mavlink_request harness"));
#ifdef PILLARB_SITL_TCP
  connectSitl();
#else
  // 50 Hz of ATTITUDE_QUATERNION alone is about 2.2 kB/s. The default
  // 256-byte receive buffer covers only about 100 ms of it, so a slow
  // loop iteration would drop bytes and punch holes in the attitude
  // buffer. Must be set before begin().
  fc_link.setRxBufferSize(2048);
  fc_link.begin(PILLARB_FC_BAUD, SERIAL_8N1, kFcRxPin, kFcTxPin);
  Serial.printf("[uart] FC link on GPIO%d/GPIO%d at %d baud\n", kFcRxPin, kFcTxPin, PILLARB_FC_BAUD);
#endif
  mav.onAttitude(onAttitude);
  configured = mav.begin();
  last_attempt_ms = millis();
}

void loop() {
#ifdef PILLARB_SITL_TCP
  if (!fc_link.connected()) {
    configured = false;
    delay(2000);
    connectSitl();
    if (fc_link.connected()) {
      configured = mav.begin();
      last_attempt_ms = millis();
    }
    return;
  }
#endif

  mav.poll();

  if (mav.needsReconfigure() || (!configured && millis() - last_attempt_ms > 5000)) {
    Serial.println(F("[main] running the request sequence again"));
    configured = mav.begin();
    last_attempt_ms = millis();
  }

  const uint32_t now = millis();
  if (now - last_print_ms >= 1000) {
    const float dt = (now - last_print_ms) / 1000.0f;
    last_print_ms = now;
    Serial.print(configured ? "[ok]  " : "[NOT READY] ");
    for (uint8_t i = 0; i < 4; ++i) {
      const uint32_t c = mav.receivedCount(kIds[i]);
      Serial.printf("%s %5.1f Hz  ", kShort[i], (c - last_counts[i]) / dt);
      last_counts[i] = c;
    }
    if (have_attitude) {
      const pillarb::AttitudeSample a = latest_attitude;
      Serial.printf("| t=%lu q=(%.4f %.4f %.4f %.4f)%s", (unsigned long)a.time_boot_ms, a.q[0],
                    a.q[1], a.q[2], a.q[3], a.from_euler ? " EULER" : "");
    }
    const pillarb::MavlinkCounters& k = mav.counters();
    Serial.printf(" | parse_err %lu seq_gaps %lu unused %lu B\n", (unsigned long)k.rx_parse_errors,
                  (unsigned long)k.fc_seq_gaps, (unsigned long)k.bytes_unused);
  }
}
