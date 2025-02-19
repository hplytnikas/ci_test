import launch
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    package = get_package_share_directory("mpc_controller")

    mpc_mode = LaunchConfiguration("mpc_mode").perform(context)

    safe_config = os.path.join(package, "config", "mpc_safe_config.yaml")
    risky_config = os.path.join(package, "config", "mpc_risky_config.yaml")
    rain_config = os.path.join(package, "config", "mpc_rain_config.yaml")
    skidpad_config = os.path.join(package, "config", "mpc_skidpad_config.yaml")

    if mpc_mode == "rain":
        selected_config = rain_config
    elif mpc_mode == "risky":
        selected_config = risky_config
    elif mpc_mode == "skidpad":
        selected_config = skidpad_config
    else:
        selected_config = safe_config

    node = Node(
        package="mpc_controller",
        executable="mpc_controller",
        output="screen",
        parameters=[selected_config, {"mpc_mode": mpc_mode}],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return [node]


def generate_launch_description():
    declare_mpc_mode_cmd = DeclareLaunchArgument(
        "mpc_mode",
        default_value="safe",
        description="MPC mode: 'safe', 'risky', or 'rain' (default: 'safe')",
    )

    return LaunchDescription(
        [declare_mpc_mode_cmd, OpaqueFunction(function=launch_setup)]
    )
