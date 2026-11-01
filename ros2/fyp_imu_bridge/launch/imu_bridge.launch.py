import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("fyp_imu_bridge"), "config", "imu_bridge.yaml")
    return LaunchDescription([
        Node(package="fyp_imu_bridge", executable="imu_udp_node", name="imu_udp_node",
             parameters=[config], output="screen"),
    ])
