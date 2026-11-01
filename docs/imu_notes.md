# IMU notes

Notes on the three Adafruit Feather nRF52840 Sense boards: firmware, calibration, and test results from 22-23 Oct 2026. Raw recordings are in `~/fyp_data/imu/` on the Pi (backed up to `~/fyp_backup/fyp_data/imu/` on the Mac).

## Hardware

- 3 × Feather nRF52840 Sense, each with a 490 mAh LiPo fixed to the board with blu-tack.
- Accel/gyro: LSM6DS33 (board 1 confirmed. The firmware also supports the LSM6DS3TR-C used on newer revisions). Magnetometer: LIS3MDL.
- Boards advertise over BLE as `FYP-IMU-1`, `FYP-IMU-2`, `FYP-IMU-3`.

## Firmware (`firmware/imu_stream`)

- Sensors read at 100 Hz, fused with Madgwick (β = 0.5), sent at 50 Hz over BLE UART (Nordic UART Service) as `seq,qw,qx,qy,qz\n`.
- Gyro bias is measured at boot over 200 samples, so boards should be still for about 1 s after powering on.
- Gyro range 2000 dps, accel ±4 g, magnetometer ±4 gauss.
- Mahony, the NXP Kalman filter and two extra Madgwick instances run on the same samples, so filters can be switched or compared without reflashing.
- Magnetometer calibration (offset + per-axis scale) is stored in internal flash and survives reflashing and power loss.
- LEDs: red LED + blue NeoPixel while recording calibration data. NeoPixel red when the battery is below 3.5 V (clears above 3.6 V, not yet tested on a real low battery).
- An I2C bus recovery runs at boot. Without it, reflashing while the battery kept the sensors powered could leave the bus stuck and the board hung at sensor init.

BLE commands (one per line. The board's RX characteristic takes at most 20 bytes per write, so longer commands must be sent in chunks):

| Command | Effect |
|---|---|
| `raw` | Toggle raw magnetometer stream `seq,mx,my,mz` (µT) |
| `cal,ox,oy,oz,sx,sy,sz` | Store calibration: corrected = (raw − o) × s |
| `cal?` | Reply `cal,...` with the stored calibration |
| `bat?` | Reply `bat,<mV>` |
| `filter,<name>` | Stream from `madgwick`, `mahony`, `kalman`, `madgwick_b` or `madgwick_c` |
| `beta,<name>,<value>` | Set β of a Madgwick instance |
| `compare[,a,b,c]` | Toggle compare mode: roll/pitch/yaw of three filters in tenths of a degree, 25 Hz |

Build and flash (on the Mac). `LC_ALL` must be UTF-8 or Adafruit's packaging tool fails:

```sh
export LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
arduino-cli compile -b adafruit:nrf52:feather52840sense \
  --build-property "compiler.cpp.extra_flags=-DIMU_ID=2" --build-path /tmp/build2 firmware/imu_stream
arduino-cli upload -b adafruit:nrf52:feather52840sense -p /dev/cu.usbmodem1101 \
  --input-dir /tmp/build2 firmware/imu_stream
```

Libraries: Adafruit LSM6DS, LIS3MDL, AHRS, Unified Sensor, NeoPixel. Board package `adafruit:nrf52` 1.7.0.

## Pi scripts (`host/`)

Run on the Pi host (not in the container) with the venv at `~/fyp_venv` (`python3 -m venv ~/fyp_venv && ~/fyp_venv/bin/pip install bleak`).

| Script | Purpose |
|---|---|
| `ble_imu_test.py` | One board: rate, packet loss, drift, optional CSV |
| `mag_calibrate.py` | Record raw magnetometer, fit, check quality, store on the board, can refit an earlier CSV |
| `filter_compare.py` | Record three filters side by side from one board |
| `multi_imu_record.py` | Record all boards together in a common time window |

## Magnetometer calibration

Procedure: start `mag_calibrate.py`. When the blue light comes on, unplug the USB, move away from the laptop, and keep turning the board continuously (tip, roll, flip, spin) until the light goes off. The first 10 s are discarded. A fit is stored only if every axis is well covered and the corrected field strength varies by less than ±6 %.

Results (spread of corrected field strength, lower is better):

| Board | Fit | Offset (µT) |
|---|---|---|
| 1 | ±2.8 % | 37.5, −39.1, −19.4 |
| 2 | ±3.0 % | 27.9, −24.9, −15.5 |
| 3 | ±2.2 % | −1.5, −8.4, 9.6 |

A full ellipsoid fit gave no improvement over the per-axis model (board 1: 2.7 % vs 2.8 %).

Lessons from failed attempts:

- The battery must be fixed to the board. A loose battery changes the distortion as the board turns, and no single correction fits.
- Calibration needs continuous rotation through all directions. Holding a few poses, or moving the board through space without changing its orientation, does not work.
- The laptop's field near the USB port (≈49-62 µT vs ≈37 µT elsewhere) ruins the fit if those samples are included, hence the 10 s discard.
- Dropping a board can shift the battery and invalidate its calibration.

## Test results

### Single board, before vs after calibration (board 1, Madgwick β = 0.1)

| | Before | After |
|---|---|---|
| Heading change while tilting forward | 9° | 0.3° |
| Creep after a 90° turn | 20° | 4.5° |
| Overshoot turning back | 18° | none |

Still for 2 min: < 0.3° drift on all axes. BLE: 50 Hz, 0 lost packets in 30 s.

### Filter comparison (board 1, same samples, library defaults)

| | Madgwick | Mahony | NXP Kalman |
|---|---|---|---|
| Heading drift while still | 0.2-0.6° | 0.2-1.0° | 2.2-3.7° |
| Creep after slow turn | 0.2° | 2.8° | 1.4° |
| Heading settle after waving | 6.8 s | 8.5 s | 18.5 s |
| Tilt settle after waving | 4.0 s | 8.2 s | 17.8 s |
| Max tilt error after waving | 4° | 5° | 25° |

All three filters together take 2.9 ms per 100 Hz update. With default settings Madgwick performed best. The Kalman filter's defaults are tuned for NXP's sensors and would need sensor-specific tuning to compete.

### Madgwick β (board 1, same samples)

| β | Heading error after waving | Settle to 1° | Sample-to-sample jitter | 90 s wander |
|---|---|---|---|---|
| 0.1 | 27° | 6.3 s | 0.02° | 1.2° |
| 0.25 | 16° | 2.1 s | 0.03° | 1.5° |
| 0.5 | 8° | 0.6 s | 0.04° | 1.7° |

β = 0.5 chosen. Raising the gyro range from 500 to 2000 dps did not change settle time (6.9 s vs 6.3 s). The error after fast waving comes from gyro scale error accumulating, not saturation.

### Three-board consistency (boards packed rigidly in a padded box, 4 min)

Disagreement between boards' relative orientation, by rotation speed:

| Rotation speed | Mean | 95th percentile |
|---|---|---|
| Still (< 5°/s) | 0.5° | 0.9° |
| 5-50°/s | 2.2° | 5.5° |
| 50-150°/s | 4.3° | 8.7° |
| 150-250°/s | 4.9° | 9.4° |

After ~90 s of vigorous motion the boards returned to within ~1° of each other. Shifting the boards' timelines against each other did not reduce the disagreement, so it is not a timing artefact. A likely cause is β = 0.5 weighting the accelerometer heavily while each board feels different linear acceleration during fast rotation. An adaptive β is the planned fix if it matters for teleop. 1 lost packet out of ~36,000.

## Open items

- Accuracy against a ground truth is untested: strap a board to the DOFBOT and command known servo angles.
- Heading accuracy near the arm's motors is untested.
- Heading shifts ≈4-5° when a board is tilted (residual magnetometer error).
- Low-battery warning not yet seen on a real low battery.
- The Pi's BLE connect occasionally needs a retry. The scripts in `host/` retry up to 3 times per attempt.
