#!/usr/bin/env python3
"""Connects to one FYP-IMU board over BLE UART and reports update rate, packet loss and drift.

Usage: ble_imu_test.py [seconds] [name] [csv]   (defaults: 20, FYP-IMU-1, no csv)
Keep the board still during the run so the yaw/pitch/roll change shows drift.
If csv is given, every sample is saved as t,seq,roll,pitch,yaw (t in seconds from the first sample).
"""
import asyncio
import math
import sys
import time

from bleak import BleakClient, BleakScanner

NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # board -> host notifications


def quat_to_rpy(w, x, y, z):
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


async def main(seconds, name, csv_path=None):
    print(f"Scanning for {name}...")
    device = await BleakScanner.find_device_by_name(name, timeout=15)
    if device is None:
        sys.exit(f"{name} not found")

    buf = ""
    samples = []  # (arrival time, seq, roll, pitch, yaw)

    def on_data(_, data):
        nonlocal buf
        buf += data.decode(errors="replace")
        *lines, buf = buf.split("\n")
        now = time.monotonic()
        for line in lines:
            parts = line.split(",")
            if len(parts) != 5:
                continue
            seq = int(parts[0])
            samples.append((now, seq, *quat_to_rpy(*map(float, parts[1:]))))

    async with BleakClient(device) as client:
        print(f"Connected, MTU {client.mtu_size}. Recording {seconds}s...")
        await client.start_notify(NUS_TX, on_data)
        await asyncio.sleep(seconds)
        await client.stop_notify(NUS_TX)

    if len(samples) < 2:
        sys.exit("No data received")
    t = [s[0] for s in samples]
    if csv_path:
        with open(csv_path, "w") as f:
            f.write("t,seq,roll,pitch,yaw\n")
            for s in samples:
                f.write(f"{s[0] - t[0]:.3f},{s[1]},{s[2]:.2f},{s[3]:.2f},{s[4]:.2f}\n")
        print(f"Saved {csv_path}")
    gaps = [b - a for a, b in zip(t, t[1:])]
    expected = samples[-1][1] - samples[0][1] + 1
    print(f"Samples: {len(samples)} in {t[-1] - t[0]:.1f}s -> {(len(samples) - 1) / (t[-1] - t[0]):.1f} Hz")
    print(f"Lost packets: {expected - len(samples)} of {expected}")
    print(f"Inter-arrival ms: mean {1000 * sum(gaps) / len(gaps):.1f}, max {1000 * max(gaps):.1f}")
    for i, axis in enumerate(("roll", "pitch", "yaw"), start=2):
        d = samples[-1][i] - samples[0][i]
        d = (d + 180) % 360 - 180
        print(f"{axis} drift: {d:+.2f} deg")


if __name__ == "__main__":
    asyncio.run(main(float(sys.argv[1]) if len(sys.argv) > 1 else 20,
                     sys.argv[2] if len(sys.argv) > 2 else "FYP-IMU-1",
                     sys.argv[3] if len(sys.argv) > 3 else None))
