#!/usr/bin/env bash
# Starts or stops both halves of the IMU bridge on the Pi:
#   host/imu_ble_bridge.py on the host (BLE -> UDP) and the fyp_imu_bridge node in the container
#   (UDP -> ROS2 topics). Logs go to ~/fyp_data/imu_ble_bridge.log and /tmp/imu_bridge.log in
#   the container.
# Usage (on the Pi): tools/imu_bridge.sh start|stop|status
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
CONTAINER=dofbot
HOST_CMD="$HOME/fyp_venv/bin/python -u imu_ble_bridge.py"
ROS_ENV="source /opt/ros/humble/setup.bash && source /root/colcon_ws/install/setup.bash && export ROS_DOMAIN_ID=62"

host_running() { pgrep -f "^$HOST_CMD" >/dev/null; }
node_running() { docker exec "$CONTAINER" pgrep -f "lib/fyp_imu_bridge/imu_udp_node" >/dev/null; }

case "${1:-}" in
  start)
    if node_running; then
      echo "ROS2 node already running"
    else
      docker exec -d "$CONTAINER" bash -c "$ROS_ENV && ros2 launch fyp_imu_bridge imu_bridge.launch.py > /tmp/imu_bridge.log 2>&1"
      echo "ROS2 node started"
    fi
    if host_running; then
      echo "BLE bridge already running"
    else
      mkdir -p "$HOME/fyp_data"
      cd "$REPO/host"
      setsid nohup $HOST_CMD > "$HOME/fyp_data/imu_ble_bridge.log" 2>&1 < /dev/null &
      echo "BLE bridge started, boards connect within about a minute"
    fi
    ;;
  stop)
    # SIGTERM lets the BLE bridge disconnect the boards cleanly.
    pkill -TERM -f "^$HOST_CMD" && echo "BLE bridge stopped" || echo "BLE bridge was not running"
    docker exec "$CONTAINER" pkill -INT -f "ros2 launch fyp_imu_bridge" && echo "ROS2 node stopping" \
      || echo "ROS2 node was not running"
    # Both take a moment to shut down, so wait so that status straight after is accurate.
    for _ in $(seq 20); do
      host_running || node_running || break
      sleep 0.5
    done
    ;;
  status)
    host_running && echo "BLE bridge: running" || echo "BLE bridge: stopped"
    node_running && echo "ROS2 node: running" || echo "ROS2 node: stopped"
    grep -v "not found" "$HOME/fyp_data/imu_ble_bridge.log" 2>/dev/null | tail -5 || true
    ;;
  *)
    echo "Usage: $0 start|stop|status" >&2
    exit 1
    ;;
esac
