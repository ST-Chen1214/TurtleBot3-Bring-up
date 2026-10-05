import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    model = LaunchConfiguration("model").perform(context)
    usb_port = LaunchConfiguration("usb_port").perform(context)
    if model not in {"burger", "waffle", "waffle_pi"}:
        raise RuntimeError(f"Unsupported TurtleBot3 model: {model}")

    os.environ["TURTLEBOT3_MODEL"] = model
    bringup_share = get_package_share_directory("turtlebot3_bringup")
    param_file = os.path.join(bringup_share, "param", f"{model}.yaml")
    state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "turtlebot3_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": "false", "namespace": ""}.items(),
    )
    base = Node(
        package="turtlebot3_node",
        executable="turtlebot3_ros",
        name="turtlebot3_node",
        output="screen",
        parameters=[param_file, {"namespace": ""}],
        arguments=["-i", usb_port],
    )
    return [state_pub, base]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="burger"),
        DeclareLaunchArgument("usb_port", default_value="/dev/ttyACM0"),
        OpaqueFunction(function=_setup),
    ])
