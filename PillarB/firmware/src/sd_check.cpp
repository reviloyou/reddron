// sd_check.cpp
//
// Verifies the microSD card before anything relies on it. Mounts it with
// SdFat, reports what it is, preallocates a file the way the flight logger
// will, writes 1 MB in log-sized batches with a sync after each (as
// logformat.py's drain does) while timing every one, reads it all back,
// and cleans up. HARDWARE_GUIDE.md shows the expected output.
//
// The number to look at is the worst batch time. The flight logger's RAM
// queue (logformat.DEFAULT_QUEUE_CAPACITY, 16 records) holds 4 s of log at
// 4 records per second; a card whose worst stall approaches that is not
// fit to fly.
//
// Wiring, Adafruit 254 breakout on VSPI:
//   3V  <- ESP32 3V3   (the 3V pin, not 5V: 3.3 V into the 5V pin leaves
//                       the breakout's regulator below dropout)
//   GND <- GND
//   CLK <- GPIO18, DO -> GPIO19, DI <- GPIO23, CS <- GPIO5

#include <Arduino.h>
#include <SdFat.h>

namespace {

const uint8_t kCsPin = 5;
// Conservative for a level-shifted breakout on jumper wires. Raise it once
// the check passes reliably.
const uint32_t kSck = SD_SCK_MHZ(16);
const char* kPath = "/pillarb_sd_check.bin";
const uint32_t kRecordBytes = 3132;      // logformat.RECORD_SIZE
const uint32_t kBatchRecords = 4;        // logformat.SD_BATCH_RECORDS
const uint32_t kTotalBytes = 1u << 20;   // 1 MB, about 80 s of flight log
const uint64_t kPrealloc = 4ull << 20;   // the flight logger preallocates 64 MB
const uint32_t kSlowUs = 50000;          // a stall worth counting

SdFs sd;
FsFile file;
uint8_t batch[kRecordBytes * kBatchRecords];

uint8_t pattern(uint32_t offset) {
  return static_cast<uint8_t>((offset * 31u + 7u) ^ (offset >> 11));
}

const char* fsName(uint8_t type) {
  switch (type) {
    case FAT_TYPE_EXFAT: return "exFAT";
    case FAT_TYPE_FAT32: return "FAT32";
    case FAT_TYPE_FAT16: return "FAT16";
    default: return "unknown";
  }
}

void fail(const char* what) {
  Serial.printf("SD CHECK FAILED: %s (sdErrorCode 0x%02X, sdErrorData 0x%02X)\n", what,
                sd.sdErrorCode(), sd.sdErrorData());
  for (;;) delay(1000);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("Pillar B SD card check"));

  if (!sd.begin(SdSpiConfig(kCsPin, DEDICATED_SPI, kSck))) {
    fail("mount. Check the card is inserted, the wiring, and that 3.3 V goes to the 3V pin");
  }
  const double card_gb = sd.card()->sectorCount() * 512.0 / 1e9;
  Serial.printf("card: %.1f GB, filesystem %s\n", card_gb, fsName(sd.fatType()));
  Serial.println(F("counting free space (a few seconds on a large card)"));
  const double free_gb =
      static_cast<double>(sd.freeClusterCount()) * sd.sectorsPerCluster() * 512.0 / 1e9;
  Serial.printf("free: %.1f GB\n", free_gb);

  sd.remove(kPath);
  if (!file.open(kPath, O_RDWR | O_CREAT | O_TRUNC)) fail("create the test file");
  if (!file.preAllocate(kPrealloc)) fail("preAllocate");
  Sector_t first = 0;
  Sector_t last = 0;
  const bool contiguous = file.contiguousRange(&first, &last);
  Serial.printf("preallocated %lu MB, contiguous: %s (sectors %lu to %lu)\n",
                (unsigned long)(kPrealloc >> 20), contiguous ? "yes" : "NO",
                (unsigned long)first, (unsigned long)last);

  uint32_t n = 0;
  uint32_t slow = 0;
  uint32_t worst_us = 0;
  uint64_t total_us = 0;
  uint32_t written = 0;
  while (written < kTotalBytes) {
    for (uint32_t i = 0; i < sizeof(batch); ++i) batch[i] = pattern(written + i);
    const uint32_t t0 = micros();
    if (file.write(batch, sizeof(batch)) != sizeof(batch)) fail("write");
    if (!file.sync()) fail("sync");
    const uint32_t dt = micros() - t0;
    total_us += dt;
    worst_us = max(worst_us, dt);
    if (dt > kSlowUs) slow++;
    n++;
    written += sizeof(batch);
  }
  Serial.printf("wrote %lu batches of %u bytes with a sync each: mean %lu us, worst %lu us, "
                "%lu slower than %lu ms\n",
                (unsigned long)n, (unsigned)sizeof(batch), (unsigned long)(total_us / n),
                (unsigned long)worst_us, (unsigned long)slow, (unsigned long)(kSlowUs / 1000));

  bool match = file.seekSet(0);
  uint32_t checked = 0;
  while (match && checked < written) {
    const int got = file.read(batch, sizeof(batch));
    if (got <= 0) {
      match = false;
      break;
    }
    for (int i = 0; i < got; ++i) {
      if (batch[i] != pattern(checked + i)) {
        match = false;
        break;
      }
    }
    checked += got;
  }
  Serial.printf("read back %lu bytes: %s\n", (unsigned long)checked,
                match ? "all match" : "MISMATCH");

  const bool cleaned = file.truncate(written) && file.close() && sd.remove(kPath);
  Serial.println(match && cleaned ? "SD CHECK PASSED" : "SD CHECK FAILED");
}

void loop() {}
