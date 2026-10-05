import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    model = LaunchConfiguration("model").perform(context)
    world = LaunchConfiguration("world").perform(context)
    start_comm = LaunchConfiguration("start_comm").perform(context)
    enable_camera_helpers = LaunchConfiguration("enable_camera_helpers").perform(context)
    camera_topic = LaunchConfiguration("camera_topic")

    if model not in {"burger", "waffle", "waffle_pi"}:
        raise RuntimeError(f"Unsupported TurtleBot3 model: {model}")
    world_map = {
        "empty": "empty_world.launch.py",
        "world": "turtlebot3_world.launch.py",
        "house": "turtlebot3_house.launch.py",
    }
    if world not in world_map:
        raise RuntimeError("world must be empty, world, or house")

    os.environ["TURTLEBOT3_MODEL"] = model
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("turtlebot3_gazebo"),
                "launch",
                world_map[world],
            )
        )
    )
    coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tb3_distributed_bringup"),
                "launch",
                "coordinator_pi.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    sensor_config = os.path.join(
        get_package_share_directory("tb3_distributed_core"), "config", "sensors.yaml"
    )
    sensor_health = Node(
        package="tb3_distributed_core",
        executable="sensor_health",
        name="sensor_health",
        output="screen",
        parameters=[sensor_config, {"use_sim_time": True, "camera_expected": True, "image_topic": camera_topic}],
    )
    actions = [gazebo, coordinator, sensor_health]

    if enable_camera_helpers.lower() in {"1", "true", "yes", "on"}:
        actions.append(
            Node(
                package="tb3_distributed_core",
                executable="snapshot_server",
                name="snapshot_server",
                output="screen",
                parameters=[sensor_config, {"use_sim_time": True, "image_topic": camera_topic}],
            )
        )

    if start_comm.lower() in {"1", "true", "yes", "on"}:
        comm = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("tb3_distributed_bringup"),
                    "launch",
                    "comm_pi.launch.py",
                )
            ),
            launch_arguments={
                "use_sim_time": "true",
                "enable_bluetooth": "false",
            }.items(),
        )
        actions.append(comm)
    return actions


def generate_launch_description():
    return LaunchDescription([
        # Waffle Pi is the most useful default for the full pipeline because it
        # includes a camera in addition to LiDAR/IMU in TurtleBot3 simulation.
        DeclareLaunchArgument("model", default_value="waffle_pi"),
        DeclareLaunchArgument("world", default_value="world"),
        DeclareLaunchArgument("start_comm", default_value="true"),
        DeclareLaunchArgument("enable_camera_helpers", default_value="true"),
        DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw"),
        OpaqueFunction(function=_setup),
    ])
