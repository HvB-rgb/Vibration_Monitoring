// ============================================================================
// Schindler Car Node — Dual-Core FreeRTOS Architecture
// ============================================================================
//
// CORE 1 (IMU & Sampling):
//   - Samples MPU6500 (raw register I²C, untouched original code)
//   - Fast Serial Plotter output
//   - DS3231 RTC timestamping (Wire1, GPIO 16/17)
//   - Pushes measurements into a lock-free FreeRTOS Queue (non-blocking)
//   - Runs at a strict, uninterrupted 50 Hz!
//
// CORE 0 (SD Card Logging & WiFi Telemetry):
//   - Drains the FreeRTOS Queue
//   - Writes CSV lines to SD card and flushes periodically
//   - Broadcasts live UDP packets to laptop
//   - Handles SD card retries without ever stalling the IMU!
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
//    VCC        5V / VIN  (or 3V3 if module has no regulator)
//    GND        GND
//    CS         GPIO 5
//    MOSI       GPIO 23
//    MISO       GPIO 19
//    SCK        GPIO 18
// ──────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ---- WiFi Access Point ----
const char* AP_SSID = "Schindler_CarNode";
const char* AP_PASS = "schindler5000";
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

// ---- FreeRTOS Queue for Core 0 <-> Core 1 Decoupling ----
struct ImuRecord {
  unsigned long sample;
  char timestamp[22];
  unsigned long ms;
  float ax, ay, az;
  float gx, gy, gz;
};

#define QUEUE_CAPACITY 150  // 150 items = 3 seconds buffer at 50 Hz
QueueHandle_t dataQueue;

// ---- Peripherals ----
TwoWire rtcWire(1);          // second I²C peripheral for the RTC
RTC_DS3231 rtc;
WiFiUDP udp;
File logFile;

const char* LOG_FILENAME = "/car_data.csv";
unsigned long sampleCount = 0;
volatile bool sdReady  = false;
bool rtcReady = false;

// ---- IMU register write (unchanged) ----
uint8_t writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission();
}

// ============================================================================
// CORE 0 TASK: SD Card Writer & WiFi Telemetry Worker
// ============================================================================
void sdLoggingTask(void *pvParameters) {
  // Initialize SD card on Core 0
  SPI.begin(18, 19, 23, SD_CS_PIN);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  bool sdOk = SD.begin(SD_CS_PIN, SPI, 4000000);
  if (!sdOk) {
    vTaskDelay(pdMS_TO_TICKS(100));
    sdOk = SD.begin(SD_CS_PIN, SPI, 1000000);
  }

  if (sdOk) {
    logFile = SD.open(LOG_FILENAME, FILE_APPEND);
    if (logFile) {
      // If new file, write CSV header
      if (logFile.size() == 0) {
        logFile.println("sample,timestamp,millis,ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads");
        logFile.flush();
      }
      sdReady = true;
      Serial.println(">>> Core 0: SD card logging active ✓ <<<");
    }
  } else {
    Serial.println(">>> Core 0: SD card not ready yet. Retrying in background... <<<");
  }

  ImuRecord rec;
  char csvRow[160];
  unsigned long lastRetry = 0;

  while (true) {
    // Wait for sample from Core 1 queue (blocking wait, zero CPU usage when empty)
    if (xQueueReceive(dataQueue, &rec, pdMS_TO_TICKS(100)) == pdTRUE) {
      snprintf(csvRow, sizeof(csvRow),
               "%lu,%s,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
               rec.sample, rec.timestamp, rec.ms,
               rec.ax, rec.ay, rec.az, rec.gx, rec.gy, rec.gz);

      // --- Write to SD Card ---
      if (sdReady && logFile) {
        logFile.println(csvRow);
        // Flush every 50 samples (~1s) to commit NAND flash blocks
        if (rec.sample % 50 == 0) {
          logFile.flush();
        }
      }

      // --- Broadcast via UDP to Laptop ---
      udp.beginPacket(IPAddress(192, 168, 4, 255), UDP_PORT);
      udp.println(csvRow);
      udp.endPacket();
    }

    // --- Background SD Reconnect if disconnected ---
    unsigned long now = millis();
    if (!sdReady && (now - lastRetry > 3000)) {
      lastRetry = now;
      if (SD.begin(SD_CS_PIN, SPI, 1000000)) {
        logFile = SD.open(LOG_FILENAME, FILE_APPEND);
        if (logFile) {
          if (logFile.size() == 0) {
            logFile.println("sample,timestamp,millis,ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads");
          }
          sdReady = true;
          Serial.println("\n>>> Core 0: [SD CARD DETECTED & LOGGING STARTED] <<<");
        }
      }
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Schindler Car Node (Dual-Core) ===");

  // Create queue: 150 items capacity (~3 seconds buffer for SD flash lag)
  dataQueue = xQueueCreate(QUEUE_CAPACITY, sizeof(ImuRecord));
  if (dataQueue == NULL) {
    Serial.println("Error creating FreeRTOS queue!");
  }

  // --- WiFi Access Point ---
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
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

  // --- Launch SD / WiFi Worker Task on CORE 0 ---
  xTaskCreatePinnedToCore(
    sdLoggingTask,       /* Function to implement the task */
    "SD_Worker",         /* Name of the task */
    8192,                /* Stack size in bytes */
    NULL,                /* Task input parameter */
    1,                   /* Priority of the task */
    NULL,                /* Task handle */
    0                    /* Core where task should run (Core 0) */
  );

  Serial.println("\nCore 1 dedicated to high-speed IMU sampling.");
  Serial.println("Core 0 handling SD logging + UDP telemetry.");
  Serial.println("──────────────────────────────────────────────────");
  delay(1500);
}

// ============================================================================
// CORE 1 LOOP: Dedicated IMU Sampling & Plotting
// ============================================================================
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
  char tsBuf[22];
  if (rtcReady) {
    DateTime now = rtc.now();
    snprintf(tsBuf, sizeof(tsBuf), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
  } else {
    strncpy(tsBuf, "NO_RTC", sizeof(tsBuf));
  }

  // --- Push to Core 0 FreeRTOS Queue (non-blocking, timeout 0) ---
  ImuRecord rec;
  rec.sample = ++sampleCount;
  strncpy(rec.timestamp, tsBuf, sizeof(rec.timestamp));
  rec.ms = ms;
  rec.ax = ax; rec.ay = ay; rec.az = az;
  rec.gx = gx; rec.gy = gy; rec.gz = gz;

  // xQueueSend with 0 timeout never waits; if queue is full during extreme lag,
  // it gracefully skips without delaying IMU sampling!
  xQueueSend(dataQueue, &rec, 0);

  delay(20); // 50 Hz refresh rate for smooth animations
}