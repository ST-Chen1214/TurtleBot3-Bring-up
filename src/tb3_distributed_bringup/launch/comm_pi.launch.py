import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("tb3_distributed_core"), "config", "comm.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    def node(exe, name, condition):
        return Node(
            package="tb3_distributed_core",
            executable=exe,
            name=name,
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration(condition)),
        )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_udp", default_value="true"),
        DeclareLaunchArgument("enable_tcp", default_value="true"),
        DeclareLaunchArgument("enable_zmq", default_value="true"),
        DeclareLaunchArgument("enable_rest", default_value="true"),
        # Keep Bluetooth opt-in because desktop/simulation machines may not expose RFCOMM.
        DeclareLaunchArgument("enable_bluetooth", default_value="false"),
        node("udp_bridge", "udp_command_bridge", "enable_udp"),
        node("tcp_bridge", "tcp_reliable_command_bridge", "enable_tcp"),
        node("zmq_bridge", "zmq_telemetry_bridge", "enable_zmq"),
        node("rest_api", "rest_api_bridge", "enable_rest"),
        node("bluetooth_bridge", "bluetooth_maintenance_bridge", "enable_bluetooth"),
    ])
