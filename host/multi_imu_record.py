#!/usr/bin/env python3
"""Streams orientation from several FYP-IMU boards at once and saves it to one CSV.

Usage: multi_imu_record.py <seconds> <csv> [names...]   (names default to FYP-IMU-1..3)
Boards are connected one after another (BlueZ dislikes parallel connects), with retries,
then all stream together. CSV columns: t, board, seq, qw, qx, qy, qz (t in seconds from start).
"""
import asyncio
import subprocess
import sys

from bleak import BleakClient, BleakScanner

NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
CONNECT_ATTEMPTS = 3


def drop_stale_connections():
    # A recorder that was killed leaves BlueZ connected to the boards; connected boards stop
    # advertising, so a new run would not find them.
    out = subprocess.run(["bluetoothctl", "devices", "Connected"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) == 3 and parts[2].startswith("FYP-IMU"):
            subprocess.run(["bluetoothctl", "disconnect", parts[1]], capture_output=True)
            print(f"Dropped stale connection to {parts[2]}")


async def connect(name):
    err = None
    for _ in range(CONNECT_ATTEMPTS):
        try:
            device = await BleakScanner.find_device_by_name(name, timeout=10)
            if device is None:
                err = "not found"
                continue
            client = BleakClient(device, timeout=20)
            await client.connect()
            return client
        except Exception as e:
            err = repr(e)
    sys.exit(f"{name}: could not connect ({err})")


async def main(seconds, csv_path, names):
    loop = asyncio.get_running_loop()
    rows = []
    clients = []
    recording = False  # only keep samples inside the common window, not during (dis)connects

    def handler(name):
        buf = ""

        def on_data(_, data):
            nonlocal buf
            buf += data.decode(errors="replace")
            *lines, buf = buf.split("\n")
            if not recording:
                return
            now = loop.time()
            for line in lines:
                parts = line.split(",")
                if len(parts) == 5:
                    rows.append((now, name, int(parts[0]), *map(float, parts[1:])))
        return on_data

    drop_stale_connections()
    try:
        for name in names:
            client = await connect(name)
            clients.append(client)
            print(f"{name} connected")
        for name, client in zip(names, clients):
            await client.start_notify(NUS_TX, handler(name))
        await asyncio.sleep(1)  # let every stream settle before the window opens
        recording = True
        print(f"Recording {seconds}s...")
        await asyncio.sleep(seconds)
        recording = False
    finally:
        for client in clients:
            await client.disconnect()

    if not rows:
        sys.exit("No data received")
    t0 = min(r[0] for r in rows)
    with open(csv_path, "w") as f:
        f.write("t,board,seq,qw,qx,qy,qz\n")
        for r in rows:
            f.write(f"{r[0] - t0:.3f},{r[1]},{r[2]},{r[3]:.4f},{r[4]:.4f},{r[5]:.4f},{r[6]:.4f}\n")
    for name in names:
        mine = [r for r in rows if r[1] == name]
        if len(mine) < 2:
            print(f"{name}: no data")
            continue
        expected = mine[-1][2] - mine[0][2] + 1
        span = mine[-1][0] - mine[0][0]
        print(f"{name}: {len(mine)} samples, {(len(mine) - 1) / span:.1f} Hz, {expected - len(mine)} lost")
    print(f"Saved {csv_path}")


if __name__ == "__main__":
    asyncio.run(main(float(sys.argv[1]), sys.argv[2],
                     sys.argv[3:] or ["FYP-IMU-1", "FYP-IMU-2", "FYP-IMU-3"]))
