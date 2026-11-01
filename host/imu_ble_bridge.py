#!/usr/bin/env python3
"""Streams every FYP-IMU board to the ROS2 node in the container over UDP on localhost.

Usage: imu_ble_bridge.py [--port 9870] [names...]   (names default to FYP-IMU-1..3)
Runs on the Pi host, because BlueZ is only reachable from the host. The container shares the
host network, so its fyp_imu_bridge node receives the datagrams on 127.0.0.1.

Each board keeps reconnecting on its own if it drops out. Datagrams are JSON:
  {"type": "imu", "board": 1, "seq": 12, "t": <host unix time>, "q": [w, x, y, z]}
  {"type": "status", "board": 1, "connected": true, "battery_mv": 3950, "lost": 0, "received": 500}
"""
import argparse
import asyncio
import json
import logging
import signal
import socket
import time

from ble_common import NUS_RX, NUS_TX, connect, drop_stale_connections, line_splitter

STATUS_PERIOD_S = 1.0
BATTERY_PERIOD_S = 30.0
RECONNECT_DELAY_S = 2.0
SILENCE_LIMIT_S = 3.0  # a connected board that sends nothing for this long counts as dropped
WATCH_PERIOD_S = 0.5
STOP_TIMEOUT_S = 8.0

log = logging.getLogger("imu_ble_bridge")


class Board:
    def __init__(self, name, sock, addr):
        self.name = name
        self.id = int(name.rsplit("-", 1)[1])
        self.sock = sock
        self.addr = addr
        self.connected = False
        self.battery_mv = None
        self.last_seq = None
        self.last_rx = 0.0
        self.lost = 0
        self.received = 0
        self.link = None  # Link of the current connection

    def send(self, msg):
        self.sock.sendto(json.dumps(msg).encode(), self.addr)

    def on_line(self, line):
        t = time.time()
        self.last_rx = t
        parts = line.split(",")
        if parts[0] == "bat" and len(parts) == 2:
            self.battery_mv = int(parts[1])
        elif len(parts) == 5:
            seq = int(parts[0])
            # A board restart resets seq to 0, which is not a loss.
            if self.last_seq is not None and seq > self.last_seq + 1:
                self.lost += seq - self.last_seq - 1
            self.last_seq = seq
            self.received += 1
            self.send({"type": "imu", "board": self.id, "seq": seq, "t": t,
                       "q": [float(v) for v in parts[1:]]})

    def send_status(self):
        self.send({"type": "status", "board": self.id, "connected": self.connected,
                   "battery_mv": self.battery_mv, "lost": self.lost, "received": self.received})


class Link:
    """Disconnect tracking for one connection, so a late callback from an old connection can
    never mark a newer one as dropped."""

    def __init__(self, loop):
        self.loop = loop
        self.dropped = asyncio.Event()
        self.armed = False

    def on_disconnect(self, _):
        # Bleak also calls this for each failed internal attempt while a connect is still in
        # progress, so only a disconnect after the connection is up counts.
        if self.armed:
            self.loop.call_soon_threadsafe(self.dropped.set)


async def run_board(board, connect_lock, stopping):
    loop = asyncio.get_running_loop()
    while not stopping.is_set():
        link = Link(loop)
        board.link = link
        # BlueZ handles one connect at a time badly, so boards take turns connecting.
        async with connect_lock:
            client = await connect(board.name, disconnected_callback=link.on_disconnect)
        if client is None:
            await asyncio.sleep(RECONNECT_DELAY_S)
            continue
        dropped = link.dropped
        try:
            link.armed = True
            board.last_seq = None
            board.last_rx = time.time()
            await client.start_notify(NUS_TX, line_splitter(board.on_line))
            board.connected = True
            log.info(f"{board.name} connected")
            next_battery = 0.0
            while not dropped.is_set() and not stopping.is_set() and client.is_connected:
                if time.time() - board.last_rx > SILENCE_LIMIT_S:
                    log.info(f"{board.name}: no data for {SILENCE_LIMIT_S:.0f}s")
                    break
                if time.time() >= next_battery:
                    await client.write_gatt_char(NUS_RX, b"bat?\n")
                    next_battery = time.time() + BATTERY_PERIOD_S
                try:
                    await asyncio.wait_for(dropped.wait(), WATCH_PERIOD_S)
                except asyncio.TimeoutError:
                    pass
        except Exception as e:
            log.info(f"{board.name}: {e!r}")
        finally:
            board.connected = False
            try:
                await client.disconnect()
            except Exception:
                pass
            log.info(f"{board.name} disconnected" + ("" if stopping.is_set() else ", reconnecting"))
        if not stopping.is_set():
            await asyncio.sleep(RECONNECT_DELAY_S)


async def report_status(boards, stopping):
    while not stopping.is_set():
        for board in boards:
            board.send_status()
        await asyncio.sleep(STATUS_PERIOD_S)


async def main(args):
    drop_stale_connections()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = ("127.0.0.1", args.port)
    boards = [Board(name, sock, addr) for name in args.names]
    connect_lock = asyncio.Lock()
    stopping = asyncio.Event()

    def stop():
        # Each board's loop sees this, finishes and disconnects its board, so the boards
        # advertise again for the next run.
        stopping.set()
        for board in boards:
            if board.link:
                board.link.dropped.set()

    asyncio.get_running_loop().add_signal_handler(signal.SIGTERM, stop)
    asyncio.get_running_loop().add_signal_handler(signal.SIGINT, stop)
    log.info(f"Sending to udp://{addr[0]}:{addr[1]}")
    status_task = asyncio.create_task(report_status(boards, stopping))
    board_tasks = [asyncio.create_task(run_board(b, connect_lock, stopping)) for b in boards]
    await stopping.wait()
    # A board in the middle of a slow connect attempt is cancelled rather than waited for.
    _, pending = await asyncio.wait(board_tasks, timeout=STOP_TIMEOUT_S)
    for task in pending:
        task.cancel()
    await asyncio.gather(status_task, *pending, return_exceptions=True)
    log.info("Stopped")


if __name__ == "__main__":
    logging.basicConfig(format="%(asctime)s %(message)s", level=logging.INFO)
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=9870)
    p.add_argument("names", nargs="*", default=["FYP-IMU-1", "FYP-IMU-2", "FYP-IMU-3"])
    asyncio.run(main(p.parse_args()))
