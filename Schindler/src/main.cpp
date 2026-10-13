// ============================================================================
// Schindler Car Node — MPU6500 + DS3231 RTC + SD Card + WiFi UDP Telemetry
// ============================================================================
//
// IMU:  Raw register-level I²C for the MPU6500  (unchanged from original)
// RTC:  DS3231 on the ESP32's *second* I²C bus  (Wire1, GPIO 16/17)
//       → both MPU6500 and DS3231 have address 0x68, so they MUST be on
//         separate buses.
// SD:   Default VSPI — CS on GPIO 5
// WiFi: ESP32 creates a soft-AP; laptop connects to it and receives live
//       CSV data over UDP broadcast.
//
// WIRING
// ──────────────────────────────────────────────────
//  MPU6500   →  ESP32          (I²C bus 0 — Wire)
//    VCC        3V3
//    GND        GND
//    SDA        GPIO 21
//    SCL        GPIO 22
//
//  DS3231    →  ESP32          (I²C bus 1 — Wire1)
//    VCC        3V3
//    GND        GND
//    SDA        GPIO 16
//    SCL        GPIO 17
//
//  SD Card   →  ESP32          (SPI — VSPI)
//    VCC        3V3
//    GND        GND
//    CS         GPIO 5
//    MOSI       GPIO 23
//    MISO       GPIO 19
//    SCK        GPIO 18
// ──────────────────────────────────────────────────
//
// VIEWING LIVE DATA ON YOUR LAPTOP
//   1) Connect laptop WiFi to "Schindler_CarNode" (password: schindler5000)
//   2) Run:  python3 tools/udp_receiver.py
//      Data streams in real-time as CSV.
//
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ---- WiFi Access Point ----
const char* AP_SSID = "Schindler_CarNode";
const char* AP_PASS = "schindler5000";      // min 8 chars for WPA2
#define UDP_PORT 4210

// ---- Pin map ----
#define SDA_PIN     21   // IMU I²C (bus 0)
#define SCL_PIN     22
#define RTC_SDA_PIN 16   // RTC I²C (bus 1) — separate bus, same 0x68 address
#define RTC_SCL_PIN 17
#define SD_CS_PIN    5
#define MPU_ADDR  0x68

// ---- IMU calibration (unchanged) ----
const float ACCEL_OFFSET[3] = {0.0, 0.0, 0.0};  // m/s^2
const float GYRO_OFFSET[3]  = {0.0, 0.0, 0.0};  // rad/s

// ---- Peripherals ----
TwoWire rtcWire(1);          // second I²C peripheral for the RTC
RTC_DS3231 rtc;
WiFiUDP udp;
File logFile;

const char* LOG_FILENAME = "/car_data.csv";
unsigned long sampleCount = 0;
bool sdReady  = false;
bool rtcReady = false;

// ---- IMU register write (unchanged) ----
uint8_t writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Schindler Car Node ===");

  // --- WiFi Access Point ---
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);                       // let the AP settle
  Serial.printf("WiFi AP : %s  (pass: %s)\n", AP_SSID, AP_PASS);
  Serial.printf("AP IP   : %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("UDP port: %d\n", UDP_PORT);
  udp.begin(UDP_PORT);

  // --- IMU I²C bus (unchanged) ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz fast-mode I2C for fluid plotting

  while (writeReg(0x6B, 0x01) != 0) {
    Serial.println("MPU not responding, check wiring...");
    delay(1000);
  }
  delay(100);
  writeReg(0x1A, 0x03);  // DLPF ~42 Hz for cleaner curves without lag
  writeReg(0x1B, 0x08);  // +-500 deg/s -> 65.5 LSB/(deg/s)
  writeReg(0x1C, 0x08);  // +-4 g        -> 8192 LSB/g
  Serial.println("MPU6500 ready.");

  // --- RTC on second I²C bus (avoids 0x68 clash with MPU) ---
  rtcWire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  if (!rtc.begin(&rtcWire)) {
    Serial.println("DS3231 RTC not found!  (check wiring on GPIO 16/17)");
  } else {
    rtcReady = true;
    if (rtc.lostPower()) {
      Serial.println("RTC lost power — setting to compile time.");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    DateTime now = rtc.now();
    Serial.printf("RTC ready: %04d-%02d-%02d %02d:%02d:%02d\n",
                  now.year(), now.month(), now.day(),
                  now.hour(), now.minute(), now.second());
  }

  // --- SD card ---
  // NOTE: The ESP32 SD library only supports FAT16 / FAT32.
  //       64 GB cards ship as exFAT — you MUST reformat to FAT32 first!
  //       Linux:   sudo mkfs.vfat -F 32 /dev/sdX1
  //       Windows: use "SD Card Formatter" app, or guiformat.exe for >32 GB
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("!!! SD card init FAILED !!!");
    Serial.println("    → Is the card inserted?");
    Serial.println("    → Is it formatted as FAT32?  (exFAT won't work!)");
  } else {
    // Print diagnostics
    uint8_t cardType = SD.cardType();
    const char* typeStr = "UNKNOWN";
    if (cardType == CARD_MMC)  typeStr = "MMC";
    if (cardType == CARD_SD)   typeStr = "SD";
    if (cardType == CARD_SDHC) typeStr = "SDHC";
    Serial.printf("SD card detected: %s, %.1f GB\n",
                  typeStr, SD.cardSize() / (1024.0 * 1024 * 1024));
    Serial.printf("Total space: %.1f MB, Used: %.1f MB\n",
                  SD.totalBytes() / (1024.0 * 1024),
                  SD.usedBytes()  / (1024.0 * 1024));

    // Test write
    logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (logFile) {
      logFile.println("sample,timestamp,millis,ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads");
      logFile.flush();
      logFile.close();
      // Verify the file exists
      if (SD.exists(LOG_FILENAME)) {
        sdReady = true;
        Serial.printf("SD logging to %s  ✓\n", LOG_FILENAME);
      } else {
        Serial.println("!!! SD write verification FAILED — file not found after write");
        Serial.println("    → Card is probably exFAT.  Reformat as FAT32!");
      }
    } else {
      Serial.println("!!! SD file open FAILED");
      Serial.println("    → Card is probably exFAT.  Reformat as FAT32!");
    }
  }

  Serial.println("\nSetup complete — logging + UDP telemetry active.");
  Serial.println("──────────────────────────────────────────────────");
  delay(3000);  // pause so you can read the messages above
}

void loop() {
  // ================================================================
  // IMU read — everything from here to the Serial.printf is UNCHANGED
  // ================================================================
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14) != 14) {
    return;
  }

  int16_t raw[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    raw[i] = (int16_t)((hi << 8) | Wire.read());
  }

  // Unit conversions
  float ax = (raw[0] / 8192.0 * 9.80665) - ACCEL_OFFSET[0];
  float ay = (raw[1] / 8192.0 * 9.80665) - ACCEL_OFFSET[1];
  float az = (raw[2] / 8192.0 * 9.80665) - ACCEL_OFFSET[2];
  float gx = (raw[4] / 65.5 * DEG_TO_RAD) - GYRO_OFFSET[0];
  float gy = (raw[5] / 65.5 * DEG_TO_RAD) - GYRO_OFFSET[1];
  float gz = (raw[6] / 65.5 * DEG_TO_RAD) - GYRO_OFFSET[2];

  // Serial Plotter format: "Label1:Value1,Label2:Value2,..."
  // Pinning min/max prevents the auto-scaler from snapping erratically
  Serial.printf("min:-15.0,max:15.0,ax:%.2f,ay:%.2f,az:%.2f,gx:%.2f,gy:%.2f,gz:%.2f\n",
                ax, ay, az, gx, gy, gz);
  // ================================================================
  // END of original IMU code
  // ================================================================

  // --- Timestamp ---
  unsigned long ms = millis();
  char tsBuf[24];
  if (rtcReady) {
    DateTime now = rtc.now();
    snprintf(tsBuf, sizeof(tsBuf), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
  } else {
    strncpy(tsBuf, "NO_RTC", sizeof(tsBuf));
  }

  // --- Build CSV row ---
  sampleCount++;
  char csvRow[160];
  snprintf(csvRow, sizeof(csvRow),
           "%lu,%s,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
           sampleCount, tsBuf, ms,
           ax, ay, az, gx, gy, gz);

  // --- SD card logging (with auto-retry if card was inserted late) ---
  static unsigned long lastSdRetry = 0;
  if (!sdReady && (ms - lastSdRetry > 3000)) {
    lastSdRetry = ms;
    if (SD.begin(SD_CS_PIN)) {
      logFile = SD.open(LOG_FILENAME, FILE_APPEND);
      if (logFile) {
        sdReady = true;
        Serial.println("\n>>> [SD CARD DETECTED & LOGGING STARTED] <<<");
      }
    }
  }

  if (sdReady) {
    if (!logFile) {
      logFile = SD.open(LOG_FILENAME, FILE_APPEND);
    }
    if (logFile) {
      logFile.println(csvRow);
      // Flush every 50 samples (~1 second) to ensure data is saved to flash
      if (sampleCount % 50 == 0) {
        logFile.flush();
      }
    } else {
      sdReady = false; // file write failed, card possibly removed
    }
  }

  // --- UDP broadcast to laptop ---
  udp.beginPacket(IPAddress(192, 168, 4, 255), UDP_PORT);
  udp.println(csvRow);
  udp.endPacket();

  delay(20); // 50 Hz refresh rate for smooth animations
}