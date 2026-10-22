# fyp_teleop

NTU Final Year Project CCDS26-0299: 6-DOF Robotic ARM based system #2.

An assistive reach system: a Yahboom DOFBOT 6-DOF arm teleoperated through three wearable IMU sensors (Adafruit Feather nRF52840 Sense) streaming over Bluetooth LE.

## Platform

- Raspberry Pi 5 running Yahboom's image `dofbot_pi5_20260702` (Debian 12 host)
- ROS2 Humble inside Yahboom's `dofbot_ai:3.0.2` container, `ROS_DOMAIN_ID=62`
- The container is reused across boots (see `~/Docker_ROS.sh` on the Pi)

Yahboom's own code (APP_DOFBOT, their example packages, Arm_Lib) is not part of this repository.

## Layout

| Folder | Contents | Runs on |
|---|---|---|
| `ros2/` | ROS2 packages (IMU bridge, teleop controller) | Pi, inside the container |
| `host/` | Bluetooth LE IMU reader | Pi host |
| `firmware/` | Feather nRF52840 Sense firmware | IMU boards, built on the Mac |
| `tools/` | Setup and backup scripts | varies |
| `docs/` | Notes and test results | |

## Notes

- Everything installed inside the container is recorded in `tools/container_setup.sh`, so the container can be rebuilt.
- The `host/` scripts run on the Pi host in the venv `~/fyp_venv` (needs `bleak`). IMU firmware, calibration and test results are in `docs/imu_notes.md`.
- Recordings and other large data live in `~/fyp_data` on the Pi, outside git. Back them up from the Mac with `tools/backup_from_pi.sh`.
- Files created inside the container are owned by root. If git or an editor can't modify them, run `sudo chown -R pi:pi ~/colcon_ws/src/fyp_teleop` on the Pi.
