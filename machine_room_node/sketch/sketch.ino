// Machine-room Node Simulation — ESP32 + current-clamp stand-in (potentiometer)
// + RTC + SD card + sync button
//
// Mirrors the machine-room node from your design addendum: samples the drive
// current channel, timestamps every sample, logs continuously to SD, and
// shares the same sync-button behavior as the car node so the two nodes'
// data can be lined up later.
//
// NOTE ON HC-12: Wokwi has no simulator model for the HC-12 radio, so the
// receive side (car node -> machine-room node over UART) can't actually be
// exercised in this simulation. The UART read code below is written the way
// it will work on real hardware, but in the simulator it will simply never
// receive anything - that's expected, not a bug. Everything else (current
// sampling, RTC timestamping, SD logging, sync button) is fully testable here.
//
// Libraries needed (same as the car node, minus the MPU6050 ones):
//   - RTClib

#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>

// ---- Pin map (matches diagram.json) ----
#define SD_CS_PIN        5
#define SYNC_BUTTON_PIN  4
#define CURRENT_ADC_PIN  34   // stand-in for the bias-network output from the SCT-013-030

// ---- Sampling ----
const unsigned long SAMPLE_INTERVAL_MS = 100;  // 10 Hz demo rate, matches car node
unsigned long lastSampleTime = 0;

// ---- Calibration (placeholder - replace with real bench-calibrated values) ----
const float ADC_MAX_VOLTS = 3.3;
const int   ADC_RESOLUTION = 4095;  // ESP32 ADC is 12-bit
const float ADC_MIDPOINT_VOLTS = 1.65;
const float ADC_TO_AMPS = 15.0;     // volts-above-midpoint -> amps, placeholder scale

RTC_DS1307 rtc;              // stand-in for DS3231 — swap to RTC_DS3231 on real hardware
File logFile;
const char* LOG_FILENAME = "/current_log.csv";

bool lastButtonState = HIGH;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Machine-room node booting...");

  pinMode(SYNC_BUTTON_PIN, INPUT_PULLUP);
  analogReadResolution(12);

  Wire.begin();

  if (!rtc.begin()) {
    Serial.println("RTC not found!");
  } else {
    if (!rtc.isrunning()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    Serial.println("RTC ready.");
  }

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card init failed!");
  } else {
    Serial.println("SD card ready.");
    logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (logFile) {
      logFile.println("timestamp,adc_raw,voltage,current_est_A,event");
      logFile.close();
    }
  }

  Serial.println("Setup complete. Logging started.");
}

void logRow(const DateTime &ts, int adcRaw, float voltage, float current, const char* event) {
  Serial.printf("%04d-%02d-%02d %02d:%02d:%02d, adc=%d, %.3fV, %.3fA, %s\n",
                ts.year(), ts.month(), ts.day(),
                ts.hour(), ts.minute(), ts.second(),
                adcRaw, voltage, current, event);

  logFile = SD.open(LOG_FILENAME, FILE_APPEND);
  if (logFile) {
    logFile.printf("%04d-%02d-%02d %02d:%02d:%02d,%d,%.3f,%.3f,%s\n",
                    ts.year(), ts.month(), ts.day(),
                    ts.hour(), ts.minute(), ts.second(),
                    adcRaw, voltage, current, event);
    logFile.close();
  }
}

void loop() {
  unsigned long now = millis();

  // --- Sync button ---
  bool buttonState = digitalRead(SYNC_BUTTON_PIN);
  if (lastButtonState == HIGH && buttonState == LOW) {
    DateTime ts = rtc.now();
    int adcRaw = analogRead(CURRENT_ADC_PIN);
    float voltage = (adcRaw / (float)ADC_RESOLUTION) * ADC_MAX_VOLTS;
    float current = (voltage - ADC_MIDPOINT_VOLTS) * ADC_TO_AMPS;
    Serial.println(">>> SYNC EVENT (button pressed) <<<");
    logRow(ts, adcRaw, voltage, current, "SYNC");
  }
  lastButtonState = buttonState;

  // --- Regular timed sampling ---
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    int adcRaw = analogRead(CURRENT_ADC_PIN);
    float voltage = (adcRaw / (float)ADC_RESOLUTION) * ADC_MAX_VOLTS;
    float current = (voltage - ADC_MIDPOINT_VOLTS) * ADC_TO_AMPS;
    DateTime ts = rtc.now();

    logRow(ts, adcRaw, voltage, current, "");
  }

  // --- HC-12 receive stub (inert in simulation, real on hardware) ---
  // if (Serial2.available()) {
  //   String packet = Serial2.readStringUntil('\n');
  //   // parse car-node summary, merge with current data, forward via MQTT
  // }
}
