"""BLE helpers shared by the scripts that talk to more than one FYP-IMU board."""
import logging
import subprocess

from bleak import BleakClient, BleakScanner

log = logging.getLogger("ble_common")

NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # host -> board
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # board -> host
CONNECT_ATTEMPTS = 3


def drop_stale_connections():
    # A script that was killed leaves BlueZ connected to the boards. Connected boards stop
    # advertising, so a new run would not find them.
    out = subprocess.run(["bluetoothctl", "devices", "Connected"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) == 3 and parts[2].startswith("FYP-IMU"):
            subprocess.run(["bluetoothctl", "disconnect", parts[1]], capture_output=True)
            log.info(f"Dropped stale connection to {parts[2]}")


async def connect(name, attempts=CONNECT_ATTEMPTS, disconnected_callback=None):
    """Finds and connects to a board, retrying because BlueZ connects fail now and then.
    Returns None if every attempt fails."""
    err = None
    for _ in range(attempts):
        try:
            device = await BleakScanner.find_device_by_name(name, timeout=10)
            if device is None:
                err = "not found"
                continue
            client = BleakClient(device, timeout=20, disconnected_callback=disconnected_callback)
            await client.connect()
            return client
        except Exception as e:
            err = repr(e)
    log.warning(f"{name}: could not connect ({err})")
    return None


def line_splitter(on_line):
    """Returns a notification handler that reassembles newline-terminated lines, since one
    line can arrive split over several BLE packets."""
    buf = ""

    def on_data(_, data):
        nonlocal buf
        buf += data.decode(errors="replace")
        *lines, buf = buf.split("\n")
        for line in lines:
            on_line(line)
    return on_data
