#!/usr/bin/env python3
"""Records three filters' output side by side from one FYP-IMU board.

Usage: filter_compare.py <seconds> <csv> [--name FYP-IMU-1] [--filters a,b,c] [--beta name=value ...]
Filters: madgwick, mahony, kalman, madgwick_b, madgwick_c (default madgwick,mahony,kalman).
--beta sets a Madgwick filter's beta first, e.g. --beta madgwick_b=0.25.
All filters run on the board on the same samples, so the comparison is fair.
CSV columns: t, seq, then roll/pitch/yaw in degrees for each filter.
"""
import argparse
import asyncio
import sys

from bleak import BleakClient, BleakScanner

NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
CHUNK = 20  # the board's RX characteristic takes at most 20 bytes per write


async def send(client, text):
    data = (text + "\n").encode()
    for i in range(0, len(data), CHUNK):
        await client.write_gatt_char(NUS_RX, data[i:i + CHUNK])


async def main(args):
    filters = args.filters.split(",")
    labels = [f + (f"({b})" if b else "") for f in filters
              for b in [dict(x.split("=") for x in args.beta).get(f)]]
    print(f"Scanning for {args.name}...")
    device = await BleakScanner.find_device_by_name(args.name, timeout=15)
    if device is None:
        sys.exit(f"{args.name} not found")

    buf = ""
    rows = []
    loop = asyncio.get_running_loop()

    def on_data(_, data):
        nonlocal buf
        buf += data.decode(errors="replace")
        *lines, buf = buf.split("\n")
        for line in lines:
            parts = line.split(",")
            if len(parts) == 1 + 3 * len(filters):
                rows.append((loop.time(), int(parts[0]), *(int(v) / 10 for v in parts[1:])))

    async with BleakClient(device) as client:
        await client.start_notify(NUS_TX, on_data)
        for b in args.beta:
            name, value = b.split("=")
            await send(client, f"beta,{name},{value}")
        await send(client, "compare," + ",".join(filters))
        print(f"Recording {args.seconds}s of {', '.join(labels)}...")
        await asyncio.sleep(args.seconds)
        await send(client, "compare")

    if not rows:
        sys.exit("No data received")
    header = ["t", "seq"] + [f"{f}_{a}" for f in labels for a in ("roll", "pitch", "yaw")]
    with open(args.csv, "w") as f:
        f.write(",".join(header) + "\n")
        for r in rows:
            f.write(f"{r[0] - rows[0][0]:.3f},{r[1]}," + ",".join(f"{v:.1f}" for v in r[2:]) + "\n")
    expected = rows[-1][1] - rows[0][1] + 1
    print(f"Saved {len(rows)} rows ({expected - len(rows)} lost) to {args.csv}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("seconds", type=float)
    p.add_argument("csv")
    p.add_argument("--name", default="FYP-IMU-1")
    p.add_argument("--filters", default="madgwick,mahony,kalman")
    p.add_argument("--beta", action="append", default=[])
    asyncio.run(main(p.parse_args()))
