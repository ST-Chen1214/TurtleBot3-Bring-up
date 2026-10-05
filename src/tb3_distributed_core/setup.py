from glob import glob
import os
from setuptools import find_packages, setup

package_name = "tb3_distributed_core"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Steven Chen",
    maintainer_email="maintainer@example.com",
    description="Distributed ROS 2 teleoperation, safety, telemetry, camera, and communication bridges for TurtleBot3",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "safety_supervisor = tb3_distributed_core.safety_supervisor:main",
            "system_coordinator = tb3_distributed_core.system_coordinator:main",
            "status_aggregator = tb3_distributed_core.status_aggregator:main",
            "sensor_health = tb3_distributed_core.sensor_health:main",
            "snapshot_server = tb3_distributed_core.snapshot_server:main",
            "udp_bridge = tb3_distributed_core.udp_bridge:main",
            "tcp_bridge = tb3_distributed_core.tcp_bridge:main",
            "zmq_bridge = tb3_distributed_core.zmq_bridge:main",
            "bluetooth_bridge = tb3_distributed_core.bluetooth_bridge:main",
            "rest_api = tb3_distributed_core.rest_api:main",
        ],
    },
)
