from glob import glob

from setuptools import setup

package_name = "fyp_imu_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Joseph",
    maintainer_email="167979343+josephtlj@users.noreply.github.com",
    description="Publishes the FYP-IMU boards' orientation as ROS2 topics.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "imu_udp_node = fyp_imu_bridge.imu_udp_node:main",
        ],
    },
)
