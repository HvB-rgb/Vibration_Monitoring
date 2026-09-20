# Vibration & Load Monitoring Prototype — Wokwi Simulation

Two-node ESP32 firmware simulation for the Schindler 5000 vibration monitoring
project. Each folder is an independent Wokwi project (its own diagram.json,
wokwi.toml, and sketch/) matching the design addendum's two-node topology.

## Folders

- `car_node/` — rides in the lift car. Accelerometer (MPU6050 stand-in for
  the real ADXL355) + DS1307 RTC (stand-in for DS3231) + microSD + sync
  button. Includes a synthetic vibration generator (`USE_SIMULATED_VIBRATION`
  in sketch.ino) so the sampling/logging/FFT pipeline can be validated
  without real hardware — set to 0 to switch back to reading the real
  MPU6050 sensor model instead.

- `machine_room_node/` — sits at the drive in the machine room. Current-clamp
  stand-in (potentiometer on an ADC pin) + DS1307 RTC + microSD + sync
  button. The HC-12 receive side is written but inert in simulation — Wokwi
  has no HC-12 model, so that part only becomes testable on real hardware.

## Compiling a node

From inside either folder:

```
arduino-cli compile --fqbn esp32:esp32:esp32 --output-dir build sketch
```

This produces `build/sketch.ino.bin` and `build/sketch.ino.elf`, which is
what each folder's `wokwi.toml` points at for the Wokwi VS Code extension.

## Running the simulation

Open the folder (`car_node` or `machine_room_node`) as the VS Code workspace
root, then run "Wokwi: Start Simulator" from the command palette. Watch the
"Wokwi Terminal" tab for the serial log output.

## Libraries required

- Car node: Adafruit MPU6050, Adafruit Unified Sensor, Adafruit BusIO, RTClib
- Machine-room node: RTClib

Install with:
```
arduino-cli lib install "Adafruit MPU6050" "Adafruit Unified Sensor" "Adafruit BusIO" "RTClib"
```

## Board package required (one-time setup)

```
arduino-cli config set board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

## Known simulation limitations

- No HC-12 model in Wokwi — the wireless link between the two nodes can't
  actually be exercised here, only on real hardware.
- No DS3231 model in Wokwi — DS1307 is used instead (same RTClib interface
  and I2C address, safe to swap to RTC_DS3231 on real hardware).
- The car node's accelerometer output is synthetic by default (see above),
  since Wokwi's MPU6050 model reports a flat, stationary reading with no way
  to inject real shake data.
