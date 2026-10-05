import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context):
    lds_model = LaunchConfiguration("lds_model").perform(context)
    lidar_port = LaunchConfiguration("lidar_port").perform(context)
    camera_topic = LaunchConfiguration("camera_topic")

    if lds_model == "LDS-01":
        pkg = "hls_lfcd_lds_driver"
        launch_file = "hlds_laser.launch.py"
    elif lds_model == "LDS-02":
        pkg = "ld08_driver"
        launch_file = "ld08.launch.py"
    elif lds_model == "LDS-03":
        pkg = "coin_d4_driver"
        launch_file = "single_lidar_node.launch.py"
    else:
        raise RuntimeError("lds_model must be LDS-01, LDS-02, or LDS-03")

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), "launch", launch_file)
        ),
        launch_arguments={
            "port": lidar_port,
            "frame_id": "base_scan",
            "namespace": "",
        }.items(),
    )

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="camera",
        name="v4l2_camera",
        output="screen",
        parameters=[{"output_encoding": "bgr8"}],
        condition=IfCondition(LaunchConfiguration("enable_camera")),
    )

    sensor_config = os.path.join(
        get_package_share_directory("tb3_distributed_core"), "config", "sensors.yaml"
    )
    helpers = [
        Node(
            package="tb3_distributed_core",
            executable="sensor_health",
            name="sensor_health",
            output="screen",
            parameters=[
                sensor_config,
                {
                    "camera_expected": ParameterValue(
                        LaunchConfiguration("enable_camera"), value_type=bool
                    ),
                    "image_topic": camera_topic,
                },
            ],
        ),
        Node(
            package="tb3_distributed_core",
            executable="snapshot_server",
            name="snapshot_server",
            output="screen",
            parameters=[sensor_config, {"image_topic": camera_topic}],
            condition=IfCondition(LaunchConfiguration("enable_camera")),
        ),
    ]
    return [lidar, camera, *helpers]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("lds_model", default_value="LDS-02"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw"),
        OpaqueFunction(function=_setup),
    ])
