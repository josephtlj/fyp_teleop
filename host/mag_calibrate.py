#!/usr/bin/env python3
"""Calibrates one FYP-IMU board's magnetometer and stores the result on the board.

Usage: mag_calibrate.py [seconds] [name] [csv]   (defaults: 60, FYP-IMU-1, no csv)
       mag_calibrate.py <recording.csv> [name]   (fit an earlier recording instead of recording)
While it records, slowly turn the board through every direction (figure-8s while rolling it over).

The first SKIP_SECONDS of samples are left out of the fit: that is when the board is still
being unplugged next to the laptop, whose field throws the whole fit off.

Fit: per-axis min/max. Offset (hard iron) = midpoint, scale (soft iron, axis-aligned) = mean
half-range / axis half-range. The fit is only sent to the board if every axis was well covered.
"""
import asyncio
import statistics
import sys

from bleak import BleakClient, BleakScanner

NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # host -> board
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # board -> host
MIN_RANGE_RATIO = 0.7  # each axis's range must be at least this fraction of the largest
MAX_SPREAD_PCT = 6.0  # corrected field strength must vary less than this; board 1 reached 2.8%
SKIP_SECONDS = 10
SAMPLE_HZ = 50  # raw mode send rate on the board
CHUNK = 20  # the board's RX characteristic takes at most 20 bytes per write


async def send(client, text):
    data = (text + "\n").encode()
    for i in range(0, len(data), CHUNK):
        await client.write_gatt_char(NUS_RX, data[i:i + CHUNK])


def magnitude_spread(samples, offset, scale):
    mags = [sum(((v - o) * s) ** 2 for v, o, s in zip(m, offset, scale)) ** 0.5 for m in samples]
    mean = statistics.mean(mags)
    return mean, 100 * statistics.pstdev(mags) / mean


async def main(seconds, name, csv_path=None, samples=None):
    recorded = samples is not None
    print(f"Scanning for {name}...")
    device = await BleakScanner.find_device_by_name(name, timeout=15)
    if device is None:
        sys.exit(f"{name} not found")

    buf = ""
    samples = samples or []
    replies = []
    raw = False

    def on_data(_, data):
        nonlocal buf
        buf += data.decode(errors="replace")
        *lines, buf = buf.split("\n")
        for line in lines:
            parts = line.split(",")
            if parts[0] == "cal":
                replies.append(line)
            elif raw and len(parts) == 4:
                samples.append(tuple(map(float, parts[1:])))

    async with BleakClient(device) as client:
        await client.start_notify(NUS_TX, on_data)
        if not recorded:
            await send(client, "raw")
            await asyncio.sleep(0.5)
            raw = True
            print(f"Recording {seconds}s (the first {SKIP_SECONDS}s are not used). "
                  "Turn the board slowly through every direction now.")
            for remaining in range(int(seconds), 0, -10):
                await asyncio.sleep(min(10, remaining))
                print(f"  {len(samples)} samples, {max(remaining - 10, 0)}s left")
            raw = False
            await send(client, "raw")
            await asyncio.sleep(0.5)

        if csv_path and not recorded:
            with open(csv_path, "w") as f:
                f.write("mx,my,mz\n")
                for m in samples:
                    f.write(f"{m[0]:.2f},{m[1]:.2f},{m[2]:.2f}\n")
            print(f"Saved {csv_path}")

        samples = samples[SKIP_SECONDS * SAMPLE_HZ:]
        if len(samples) < 100:
            sys.exit("Too few samples")
        lo = [min(m[i] for m in samples) for i in range(3)]
        hi = [max(m[i] for m in samples) for i in range(3)]
        half = [(h - l) / 2 for h, l in zip(hi, lo)]
        offset = [(h + l) / 2 for h, l in zip(hi, lo)]
        scale = [statistics.mean(half) / r for r in half]

        for axis, l, h in zip("xyz", lo, hi):
            print(f"{axis}: {l:7.1f} .. {h:7.1f} uT (range {h - l:5.1f})")
        mean_before, spread_before = magnitude_spread(samples, [0, 0, 0], [1, 1, 1])
        mean_after, spread_after = magnitude_spread(samples, offset, scale)
        print(f"Field strength before: {mean_before:.1f} uT +/- {spread_before:.1f}%")
        print(f"Field strength after:  {mean_after:.1f} uT +/- {spread_after:.1f}%")

        if min(half) < MIN_RANGE_RATIO * max(half):
            sys.exit("Coverage too low on at least one axis; not saving. Rotate more fully and retry.")
        if spread_after > MAX_SPREAD_PCT:
            sys.exit(f"Fit too poor (+/- {spread_after:.1f}% > {MAX_SPREAD_PCT}%); not saving. Check for "
                     "anything moving relative to the board or nearby metal, and retry.")

        cmd = "cal," + ",".join(f"{v:.3f}" for v in offset) + "," + ",".join(f"{v:.4f}" for v in scale)
        await send(client, cmd)
        await asyncio.sleep(1)
        print(f"Board replied: {replies[-1] if replies else 'nothing'}")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1].endswith(".csv"):
        with open(sys.argv[1]) as f:
            next(f)
            rows = [tuple(map(float, line.split(","))) for line in f if line.strip()]
        asyncio.run(main(0, sys.argv[2] if len(sys.argv) > 2 else "FYP-IMU-1", samples=rows))
    else:
        asyncio.run(main(float(sys.argv[1]) if len(sys.argv) > 1 else 60,
                         sys.argv[2] if len(sys.argv) > 2 else "FYP-IMU-1",
                         sys.argv[3] if len(sys.argv) > 3 else None))
