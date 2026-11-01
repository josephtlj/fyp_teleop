#!/usr/bin/env python3
"""Streams orientation from several FYP-IMU boards at once and saves it to one CSV.

Usage: multi_imu_record.py <seconds> <csv> [names...]   (names default to FYP-IMU-1..3)
Boards are connected one after another (BlueZ dislikes parallel connects), with retries,
then all stream together. CSV columns: t, board, seq, qw, qx, qy, qz (t in seconds from start).
"""
import asyncio
import logging
import sys

from ble_common import NUS_TX, connect, drop_stale_connections, line_splitter


async def main(seconds, csv_path, names):
    loop = asyncio.get_running_loop()
    rows = []
    clients = []
    recording = False  # only keep samples inside the common window, not while connecting

    def handler(name):
        def on_line(line):
            if not recording:
                return
            parts = line.split(",")
            if len(parts) == 5:
                rows.append((loop.time(), name, int(parts[0]), *map(float, parts[1:])))
        return line_splitter(on_line)

    drop_stale_connections()
    try:
        for name in names:
            client = await connect(name)
            if client is None:
                sys.exit(f"{name}: could not connect")
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
    logging.basicConfig(format="%(message)s", level=logging.INFO)
    asyncio.run(main(float(sys.argv[1]), sys.argv[2],
                     sys.argv[3:] or ["FYP-IMU-1", "FYP-IMU-2", "FYP-IMU-3"]))
