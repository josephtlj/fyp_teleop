"""Publishes the FYP-IMU boards' orientation as sensor_msgs/Imu, one topic per body segment.

host/imu_ble_bridge.py on the Pi host sends the boards' data to this node over UDP, because
BlueZ is only reachable from the host. The container shares the host network.

Topics:
  /imu/<segment>  sensor_msgs/Imu, orientation only, stamped with the host's receive time
  /imu/status     diagnostic_msgs/DiagnosticArray at 1 Hz, one entry per board
"""
import json
import socket
import threading
import time

import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def to_stamp(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


class BoardState:
    def __init__(self):
        self.last_rx = None  # time.time() of the last sample
        self.count = 0  # samples since the last status report
        self.latency_sum = 0.0  # host receive -> published, summed over count
        self.status = {}


class ImuUdpNode(Node):
    def __init__(self):
        super().__init__("imu_udp_node")
        self.port = self.declare_parameter("port", 9870).value
        self.segments = self.declare_parameter("segments", ["upper_arm", "forearm", "hand"]).value
        self.variance = self.declare_parameter("orientation_variance", 0.0012).value
        self.stale_after = self.declare_parameter("stale_after_s", 0.5).value

        self.imu_pubs = {
            board: self.create_publisher(Imu, f"/imu/{segment}", qos_profile_sensor_data)
            for board, segment in enumerate(self.segments, start=1)
        }
        self.status_pub = self.create_publisher(DiagnosticArray, "/imu/status", 10)
        self.boards = {board: BoardState() for board in self.imu_pubs}
        self.lock = threading.Lock()
        self.last_report = time.time()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", self.port))
        self.sock.settimeout(0.5)
        self.running = True
        self.rx_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.rx_thread.start()
        self.create_timer(1.0, self.report_status)
        self.get_logger().info(f"Listening on udp://127.0.0.1:{self.port} for boards {self.segments}")

    def receive_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            # One malformed datagram must not stop the receiver, or the node would look alive
            # while publishing nothing.
            try:
                msg = json.loads(data)
                board = msg.get("board")
                if board not in self.boards:
                    continue
                if msg.get("type") == "imu":
                    self.publish_imu(board, msg)
                elif msg.get("type") == "status":
                    with self.lock:
                        self.boards[board].status = msg
            except Exception as e:
                self.get_logger().warn(f"Dropped bad datagram: {e!r}", throttle_duration_sec=5.0)

    def publish_imu(self, board, msg):
        imu = Imu()
        imu.header.stamp = to_stamp(msg["t"])
        imu.header.frame_id = f"imu_{self.segments[board - 1]}"
        w, x, y, z = (float(v) for v in msg["q"])
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = w, x, y, z
        imu.orientation_covariance = [self.variance, 0.0, 0.0,
                                      0.0, self.variance, 0.0,
                                      0.0, 0.0, self.variance]
        # The boards do not send rates or accelerations yet. -1 marks a field as unavailable.
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.imu_pubs[board].publish(imu)
        now = time.time()
        with self.lock:
            state = self.boards[board]
            state.last_rx = now
            state.count += 1
            state.latency_sum += now - msg["t"]

    def report_status(self):
        now = time.time()
        span = now - self.last_report
        self.last_report = now
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            for board, state in self.boards.items():
                rate = state.count / span if span > 0 else 0.0
                latency_ms = 1000 * state.latency_sum / state.count if state.count else float("nan")
                bridge = state.status
                fresh = state.last_rx is not None and now - state.last_rx < self.stale_after
                entry = DiagnosticStatus()
                entry.name = f"FYP-IMU-{board} ({self.segments[board - 1]})"
                entry.hardware_id = f"FYP-IMU-{board}"
                if fresh:
                    entry.level, entry.message = DiagnosticStatus.OK, "streaming"
                elif bridge.get("connected"):
                    entry.level, entry.message = DiagnosticStatus.WARN, "connected but no data"
                else:
                    entry.level, entry.message = DiagnosticStatus.ERROR, "not connected"
                entry.values = [
                    KeyValue(key="rate_hz", value=f"{rate:.1f}"),
                    KeyValue(key="bridge_latency_ms", value=f"{latency_ms:.2f}"),
                    KeyValue(key="battery_mv", value=str(bridge.get("battery_mv"))),
                    KeyValue(key="lost", value=str(bridge.get("lost"))),
                    KeyValue(key="received", value=str(bridge.get("received"))),
                ]
                array.status.append(entry)
                state.count = 0
                state.latency_sum = 0.0
        self.status_pub.publish(array)

    def destroy_node(self):
        self.running = False
        self.rx_thread.join(timeout=1.0)
        self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ImuUdpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
