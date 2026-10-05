import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("tb3_distributed_core"), "config", "safety.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    common = [config, {"use_sim_time": use_sim_time}]
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="tb3_distributed_core",
            executable="system_coordinator",
            name="system_coordinator",
            output="screen",
            parameters=common,
        ),
        Node(
            package="tb3_distributed_core",
            executable="safety_supervisor",
            name="safety_supervisor",
            output="screen",
            parameters=common,
        ),
        Node(
            package="tb3_distributed_core",
            executable="status_aggregator",
            name="status_aggregator",
            output="screen",
            parameters=common,
        ),
    ])
