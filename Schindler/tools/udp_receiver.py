#!/usr/bin/env python3
"""
Schindler Car Node — UDP Data Receiver
=======================================

Connect your laptop to the "Schindler_CarNode" WiFi network
(password: schindler5000), then run this script.

Usage:
    python3 udp_receiver.py              # print to terminal
    python3 udp_receiver.py -o data.csv  # also save to CSV file
    python3 udp_receiver.py --plot       # live matplotlib plot (requires matplotlib)
"""

import socket
import argparse
import sys
import signal
from datetime import datetime

UDP_PORT = 4210
HEADER = "sample,timestamp,millis,ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads"


def main():
    parser = argparse.ArgumentParser(description="Receive live data from Schindler Car Node")
    parser.add_argument("-o", "--output", help="Save received data to a CSV file")
    parser.add_argument("--port", type=int, default=UDP_PORT, help=f"UDP port (default: {UDP_PORT})")
    parser.add_argument("--plot", action="store_true", help="Live plot (requires matplotlib)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(1.0)  # 1s timeout so Ctrl-C is responsive

    print(f"Listening for Car Node UDP packets on port {args.port} ...")
    print(f"Make sure you're connected to WiFi: {repr('Schindler_CarNode')}")
    print(HEADER)
    print("-" * len(HEADER))

    csv_file = None
    if args.output:
        csv_file = open(args.output, "w")
        csv_file.write(HEADER + "\n")
        print(f"  → Also saving to {args.output}")

    if args.plot:
        live_plot(sock, csv_file)
        return

    # Simple text mode
    running = True

    def stop(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)

    count = 0
    while running:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            continue
        except OSError:
            break

        line = data.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        count += 1
        print(line)

        if csv_file:
            csv_file.write(line + "\n")
            if count % 50 == 0:          # flush every ~1 second at 50 Hz
                csv_file.flush()

    if csv_file:
        csv_file.close()
        print(f"\nSaved {count} rows to {args.output}")

    sock.close()
    print("Done.")


def live_plot(sock, csv_file):
    """Real-time scrolling plot of accelerometer + gyro data."""
    try:
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
    except ImportError:
        print("ERROR: matplotlib is required for --plot.  Install with:")
        print("       pip install matplotlib")
        sys.exit(1)

    from collections import deque

    N = 500  # number of points on screen (~10 seconds at 50 Hz)
    t   = deque(maxlen=N)
    axd = deque(maxlen=N)
    ayd = deque(maxlen=N)
    azd = deque(maxlen=N)
    gxd = deque(maxlen=N)
    gyd = deque(maxlen=N)
    gzd = deque(maxlen=N)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle("Schindler Car Node — Live IMU Data")

    ax1.set_ylabel("Acceleration (m/s²)")
    ax2.set_ylabel("Gyroscope (rad/s)")
    ax2.set_xlabel("Sample #")

    line_ax, = ax1.plot([], [], label="ax", linewidth=0.8)
    line_ay, = ax1.plot([], [], label="ay", linewidth=0.8)
    line_az, = ax1.plot([], [], label="az", linewidth=0.8)
    ax1.legend(loc="upper left")

    line_gx, = ax2.plot([], [], label="gx", linewidth=0.8)
    line_gy, = ax2.plot([], [], label="gy", linewidth=0.8)
    line_gz, = ax2.plot([], [], label="gz", linewidth=0.8)
    ax2.legend(loc="upper left")

    count = [0]

    def update(frame):
        # Drain all pending packets (non-blocking)
        sock.setblocking(False)
        while True:
            try:
                data, _ = sock.recvfrom(1024)
            except BlockingIOError:
                break
            except OSError:
                break

            line = data.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if csv_file:
                csv_file.write(line + "\n")
                count[0] += 1
                if count[0] % 50 == 0:
                    csv_file.flush()

            parts = line.split(",")
            if len(parts) < 9:
                continue
            try:
                sample = int(parts[0])
                ax_v = float(parts[3])
                ay_v = float(parts[4])
                az_v = float(parts[5])
                gx_v = float(parts[6])
                gy_v = float(parts[7])
                gz_v = float(parts[8])
            except ValueError:
                continue

            t.append(sample)
            axd.append(ax_v)
            ayd.append(ay_v)
            azd.append(az_v)
            gxd.append(gx_v)
            gyd.append(gy_v)
            gzd.append(gz_v)

        if len(t) > 1:
            tl = list(t)
            line_ax.set_data(tl, list(axd))
            line_ay.set_data(tl, list(ayd))
            line_az.set_data(tl, list(azd))
            line_gx.set_data(tl, list(gxd))
            line_gy.set_data(tl, list(gyd))
            line_gz.set_data(tl, list(gzd))

            ax1.set_xlim(tl[0], tl[-1])
            ax1.relim()
            ax1.autoscale_view(scalex=False)
            ax2.set_xlim(tl[0], tl[-1])
            ax2.relim()
            ax2.autoscale_view(scalex=False)

        return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

    if csv_file:
        csv_file.close()
    sock.close()


if __name__ == "__main__":
    main()
