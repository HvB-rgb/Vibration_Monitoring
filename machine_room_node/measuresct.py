import numpy as np
import sounddevice as sd
import time

# --- Audio Configuration ---
SAMPLE_RATE = 44100
BLOCK_SIZE = 44100     # 1-second sample window
CHANNELS = 1

# --- SCT-013-030 Specifics ---
# Sensor ratio: 30A produces 1V RMS
# Laptop sound cards clip at full scale (~1.0 normalized float)
SENSOR_RATED_AMPS = 30.0

# Initial multiplier. Run calibration step below to adjust this value.
CALIBRATION_FACTOR = 1.0  
MAINS_VOLTAGE = 230.0     # Change to 120.0 if you are in North America

def audio_callback(indata, frames, time_info, status):
    if status:
        print(f"ALSA Status: {status}", flush=True)

    # Convert to 1D float array
    raw = indata[:, 0].astype(np.float64)

    # 1. Strip DC bias offset added by sound card microphone bias voltage
    signal = raw - np.mean(raw)

    # 2. Calculate RMS value of the audio waveform (range 0.0 to 1.0)
    rms_raw = np.sqrt(np.mean(signal**2))

    # Low-signal noise gate (suppresses line hiss when nothing is on)
    if rms_raw < 0.002:
        current = 0.0
    else:
        current = rms_raw * SENSOR_RATED_AMPS * CALIBRATION_FACTOR

    power = current * MAINS_VOLTAGE

    print(f"Raw Audio RMS: {rms_raw:7.4f} | Current: {current:5.2f} A | Power: {power:6.1f} W")

def main():
    print("Reading SCT-013-030 on default mic input. Press Ctrl+C to stop.\n")
    try:
        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            blocksize=BLOCK_SIZE,
            channels=CHANNELS,
            callback=audio_callback
        ):
            while True:
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nMeasurement stopped.")

if __name__ == "__main__":
    main()