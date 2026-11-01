# ROS2 IMU bridge

Gets the three IMU boards' orientation into ROS2 on the Pi. Results are from 1-2 Nov 2026.

## Design

BlueZ (Bluetooth) is only reachable from the Pi host, and ROS2 Humble only exists inside the `dofbot` container. The container runs with `--net=host`, so the two halves talk over UDP on localhost without changing Yahboom's container.

```
boards --BLE--> host/imu_ble_bridge.py --UDP 127.0.0.1:9870--> fyp_imu_bridge node (container) --> ROS2 topics
```

- `host/imu_ble_bridge.py` runs in `~/fyp_venv` on the host. It keeps a connection to each board, reconnects on its own when a board drops, polls battery voltage every 30 s and forwards everything as JSON datagrams.
- `ros2/fyp_imu_bridge` is an `ament_python` package. Its `imu_udp_node` turns the datagrams into ROS2 messages.

## Topics (`ROS_DOMAIN_ID=62`, same as Yahboom's nodes)

| Topic | Type | Content |
|---|---|---|
| `/imu/upper_arm` | `sensor_msgs/Imu` | Board 1 orientation, 50 Hz |
| `/imu/forearm` | `sensor_msgs/Imu` | Board 2 orientation, 50 Hz |
| `/imu/hand` | `sensor_msgs/Imu` | Board 3 orientation, 50 Hz |
| `/imu/status` | `diagnostic_msgs/DiagnosticArray` | Per board: streaming or not, rate, bridge latency, battery, lost packets, 1 Hz |

- Only `orientation` is filled. Angular velocity and linear acceleration are marked unavailable (covariance[0] = -1) until the firmware sends them.
- `header.stamp` is the host time the sample arrived over BLE. `frame_id` is `imu_<segment>`.
- Orientation covariance is 0.0012 rad^2 per axis, about (2 deg)^2, from the three-board consistency test.
- The IMU topics use the sensor data QoS (best effort). Use `--qos-reliability best_effort` with `ros2 topic echo`.
- Which board is which segment is set in `ros2/fyp_imu_bridge/config/imu_bridge.yaml`.

## Running it (on the Pi)

Build once after changes (in the container):

```sh
docker exec dofbot bash -c "source /opt/ros/humble/setup.bash && cd /root/colcon_ws && colcon build --packages-select fyp_imu_bridge"
```

Start, check and stop both halves:

```sh
tools/imu_bridge.sh start    # boards connect within about a minute
tools/imu_bridge.sh status
tools/imu_bridge.sh stop     # disconnects the boards cleanly
```

Watch the data (in the container, with `ROS_DOMAIN_ID=62`):

```sh
ros2 topic hz /imu/forearm
ros2 topic echo /imu/status
ros2 bag record -o <name> /imu/upper_arm /imu/forearm /imu/hand /imu/status
```

The container only mounts `~/colcon_ws` and `~/APP_DOFBOT`, so bags recorded there land in `~/colcon_ws`. Move them to `~/fyp_data/bags` afterwards (they are owned by root).

## Test results

10 minute recording of all three boards (`~/fyp_data/bags/bridge_stability_1`):

| Board | Messages | Rate | Lost | Longest gap |
|---|---|---|---|---|
| 1 (upper arm) | 29,980 | 50.00 Hz | 1 | 145 ms (4 gaps over 100 ms) |
| 2 (forearm) | 29,980 | 50.00 Hz | 0 | 97 ms |
| 3 (hand) | 29,981 | 50.00 Hz | 0 | 97 ms |

- No disconnects in 10 minutes. Gaps up to about 100 ms are BLE delivering samples in short bursts.
- Bridge latency (BLE arrival on the host to ROS2 publish) is 0.2 to 0.5 ms.
- Reconnect: pulling board 2's battery and plugging it back in, the bridge reported it disconnected and reconnected it on its own. Boards 1 and 3 kept streaming with no lost packets.
- Start-up: boards connect one at a time and all three were up 35 s to 2.5 min after `start`. Board 1 sometimes needs a few timed-out attempts first.
- Stop: all three boards disconnect within 0.1 s of each other before the bridge exits.

## Problems found and fixed

- Bleak calls the disconnect callback for every failed internal attempt while a connection is still being set up. The bridge first treated these as real drops and kept disconnecting boards it had just connected. It now ignores the callback until the connection is up, and also treats 3 s of silence from a connected board as a drop.
- One malformed datagram (an integer where ROS2 needs a float) killed the node's receive thread while the node kept running and looked healthy. The receive loop now drops bad datagrams with a warning instead.
- A bridge that is killed without cleanup leaves BlueZ connected to the boards, and connected boards stop advertising. On SIGTERM or Ctrl+C the bridge now ends each board's loop, waits for every board to disconnect and only then exits. A first version cancelled everything at once and exited before the disconnects finished. Every script also clears stale connections on start.
