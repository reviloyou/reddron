#include <Arduino.h>

// TFmini-S (UART version) characterization logger for Phase 0.
//
// Wiring:
//   Sensor Vcc -> VIN (5V)
//   Sensor GND -> GND
//   Sensor TX  -> GPIO16   (ESP32 RX2)
//   Sensor RX  -> GPIO17   (ESP32 TX2)
//
// Frame (9 bytes, 115200 baud):
//   0x59 0x59  dist_L dist_H  str_L str_H  temp_L temp_H  checksum
//   checksum = low byte of sum of first 8 bytes.
//   distance in cm, strength 0..65535, temp_c = temp_raw/8 - 256.
//   temp_c is the LiDAR's OWN internal chip temperature (drift compensation);
//   it self-heats above ambient in use. Not a scene or air temperature.
//
// No filtering: every checksum-valid frame is emitted, dropouts included.
// Classification (surface vs bottom vs specular vs dropout) happens in Python.
//
// The firmware does NOT count to 200. It free-runs. The 200-samples-per-point
// rule lives in the Python capture script, which owns the per-point metadata
// (angle, rig height, condition). Firmware stays identical across all points.
//
// Onboard LED (GPIO2 on a standard WROOM-32 devkit) = LINK-ALIVE indicator:
//   fast flash (~8 Hz) = valid frames arriving (sensor alive and talking)
//   off                = no valid frame for LED_TIMEOUT_MS (real fault)
// It tracks the serial link, NOT return quality. Over water a valid "no return"
// is still a frame, so the LED keeps flashing and the CSV reports the dropout.
// A dark LED therefore always means a fault, never a valid measurement result.

static const int      PIN_RX          = 16;      // <- sensor TX
static const int      PIN_TX          = 17;      // -> sensor RX
static const int      PIN_LED         = 2;       // onboard LED on most WROOM-32 devkits
static const uint32_t SENSOR_BAUD     = 115200;
static const uint32_t USB_BAUD        = 115200;
static const uint32_t LED_TIMEOUT_MS  = 250;     // no frame in this window -> LED off
static const uint32_t LED_BLINK_MS    = 60;      // toggle period -> ~8 Hz fast flash

uint8_t       frame[9];
int           idx          = 0;
unsigned long lastFrameMs  = 0;
unsigned long lastToggleMs = 0;
bool          ledOn        = false;
bool          hasFrame     = false;              // stay dark until the first real frame

void setup() {
  Serial.begin(USB_BAUD);
  Serial2.begin(SENSOR_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  delay(100);
  Serial.println("t_ms,distance_cm,strength,temp_c");
}

void loop() {
  // --- read and emit frames ---
  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (idx == 0) {
      if (b == 0x59) frame[idx++] = b;
    } else if (idx == 1) {
      if (b == 0x59) frame[idx++] = b;
      else           idx = 0;            // second header byte wrong -> resync
    } else {
      frame[idx++] = b;
      if (idx == 9) {
        idx = 0;
        uint8_t sum = 0;
        for (int i = 0; i < 8; i++) sum += frame[i];
        if (sum == frame[8]) {
          uint16_t dist     = frame[2] | (frame[3] << 8);
          uint16_t strength = frame[4] | (frame[5] << 8);
          uint16_t temp_raw = frame[6] | (frame[7] << 8);
          float    temp_c   = temp_raw / 8.0f - 256.0f;

          lastFrameMs = millis();        // link is alive
          hasFrame    = true;

          Serial.print(millis());
          Serial.print(',');
          Serial.print(dist);
          Serial.print(',');
          Serial.print(strength);
          Serial.print(',');
          Serial.println(temp_c, 1);
        }
        // bad checksum: drop frame, resync on next 0x59 0x59
      }
    }
  }

  // --- LED: fast flash while frames arrive, off otherwise ---
  // Runs every loop iteration, so the LED goes dark on its own when data stops.
  unsigned long now = millis();
  if (hasFrame && (now - lastFrameMs < LED_TIMEOUT_MS)) {
    if (now - lastToggleMs >= LED_BLINK_MS) {
      lastToggleMs = now;
      ledOn = !ledOn;
      digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
    }
  } else if (ledOn) {
    ledOn = false;
    digitalWrite(PIN_LED, LOW);
  }
}
