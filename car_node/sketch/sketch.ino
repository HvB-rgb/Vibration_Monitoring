// Car Node Simulation — ESP32 + MPU6050 (accelerometer stand-in) + RTC + SD card + sync button
// This mirrors your design addendum's car-node role for Phases 1-3:
// sample the accelerometer, timestamp every sample, log continuously to SD,
// and let a push button mark a manual sync/fiducial event.
//
// UPDATE: added a synthetic vibration generator so you can validate the
// full acquisition + FFT pipeline without real lift data. This mirrors
// Phase 7 of your design addendum: "feed a known-frequency signal... confirm
// the sampling rate and FFT bins correctly recover that known frequency."
// With USE_SIMULATED_VIBRATION = 1, every logged row is a known sine wave
// (VIB_FREQ_HZ) plus noise instead of the MPU6050's real (flat) simulator
// output — so you can run FFT on the resulting CSV and check you get
// VIB_FREQ_HZ back out. Set it to 0 to go back to reading the real
// (simulated-stationary) MPU6050.
//
// Libraries needed (install via Wokwi's Library Manager, or they'll be
// auto-suggested when you run the simulation):
//   - Adafruit MPU6050
//   - Adafruit Unified Sensor
//   - Adafruit BusIO
//   - RTClib

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>

// ---- Pin map (matches diagram.json) ----
#define SD_CS_PIN   5
#define SYNC_BUTTON_PIN 4

// ---- Sampling ----
const unsigned long SAMPLE_INTERVAL_MS = 100;  // 10 Hz demo rate
unsigned long lastSampleTime = 0;

// ---- Synthetic vibration generator ----
#define USE_SIMULATED_VIBRATION 1   // 1 = fake known-frequency signal, 0 = real MPU6050 reading

const float VIB_FREQ_HZ      = 5.0;   // the "true" frequency your FFT should recover
const float VIB_AMPLITUDE    = 2.0;   // m/s^2, superimposed on gravity (Z axis)
const float NOISE_AMPLITUDE  = 0.15;  // m/s^2, random noise sprinkled on every sample

Adafruit_MPU6050 mpu;
RTC_DS1307 rtc;              // stand-in for DS3231 — same RTClib interface,
                              // swap to RTC_DS3231 rtc; on real hardware
File logFile;
const char* LOG_FILENAME = "/run1.csv";

bool lastButtonState = HIGH;

void getAcceleration(float &ax, float &ay, float &az) {
#if USE_SIMULATED_VIBRATION
  float t = millis() / 1000.0;
  float noise = (((float)random(0, 1000) / 1000.0) - 0.5) * 2.0 * NOISE_AMPLITUDE;
  ax = 0.10 * sin(2 * PI * VIB_FREQ_HZ * t) + noise;
  ay = 0.10 * sin(2 * PI * VIB_FREQ_HZ * t + 1.2) + noise;
  az = 9.81 + VIB_AMPLITUDE * sin(2 * PI * VIB_FREQ_HZ * t) + noise;
#else
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
#endif
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Car node booting...");
#if USE_SIMULATED_VIBRATION
  Serial.printf("Using SIMULATED vibration: %.2f Hz + noise\n", VIB_FREQ_HZ);
#endif

  pinMode(SYNC_BUTTON_PIN, INPUT_PULLUP);
  randomSeed(analogRead(0));

  Wire.begin();

  // --- RTC init ---
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
  } else {
    if (!rtc.isrunning()) {
      // Set RTC to the compile time (only matters in simulation)
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    Serial.println("RTC ready.");
  }

  // --- Accelerometer init (still initialized even in simulated mode,
  //     harmless, and needed if you flip USE_SIMULATED_VIBRATION to 0) ---
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 ready.");
  }

  // --- SD init ---
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card init failed!");
  } else {
    Serial.println("SD card ready.");
    logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (logFile) {
      logFile.println("timestamp,ax,ay,az,event");
      logFile.close();
    }
  }

  Serial.println("Setup complete. Logging started.");
}

void logRow(const DateTime &ts, float ax, float ay, float az, const char* event) {
  // Print to Serial so you can watch it live in the simulator
  Serial.printf("%04d-%02d-%02d %02d:%02d:%02d, %.3f, %.3f, %.3f, %s\n",
                ts.year(), ts.month(), ts.day(),
                ts.hour(), ts.minute(), ts.second(),
                ax, ay, az, event);

  // Append to SD card
  logFile = SD.open(LOG_FILENAME, FILE_APPEND);
  if (logFile) {
    logFile.printf("%04d-%02d-%02d %02d:%02d:%02d,%.3f,%.3f,%.3f,%s\n",
                    ts.year(), ts.month(), ts.day(),
                    ts.hour(), ts.minute(), ts.second(),
                    ax, ay, az, event);
    logFile.close();
  }
}

void loop() {
  unsigned long now = millis();

  // --- Sync button: falling edge = pressed (INPUT_PULLUP) ---
  bool buttonState = digitalRead(SYNC_BUTTON_PIN);
  if (lastButtonState == HIGH && buttonState == LOW) {
    DateTime ts = rtc.now();
    float ax, ay, az;
    getAcceleration(ax, ay, az);
    Serial.println(">>> SYNC EVENT (button pressed) <<<");
    logRow(ts, ax, ay, az, "SYNC");
  }
  lastButtonState = buttonState;

  // --- Regular timed sampling ---
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    float ax, ay, az;
    getAcceleration(ax, ay, az);
    DateTime ts = rtc.now();

    logRow(ts, ax, ay, az, "");
  }
}
