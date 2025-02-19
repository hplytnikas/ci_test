import launch
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo,
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os


def launch_setup(context, *args, **kwargs):
    package = get_package_share_directory("global_planner")

    controller_mode = LaunchConfiguration("controller_mode").perform(context)

    mpc_safe_config = os.path.join(package, "config", "global_planner_config_safe.yaml")
    mpc_risky_config = os.path.join(
        package, "config", "global_planner_config_risky.yaml"
    )
    mpc_rain_config = os.path.join(package, "config", "global_planner_config_rain.yaml")
    pp_config = os.path.join(package, "config", "global_planner_config_pp.yaml")

    if controller_mode == "mpc_safe":
        selected_config = mpc_safe_config
    elif controller_mode == "mpc_risky":
        selected_config = mpc_risky_config
    elif controller_mode == "mpc_rain":
        selected_config = mpc_rain_config
    elif controller_mode == "pure_pursuit":
        selected_config = pp_config
    else:
        raise ValueError(f"Invalid controller mode: {controller_mode}")

    # Create a LogInfo action to log the selected configuration
    # Just for debug, remove it if not nedeed
    log_selected_config = LogInfo(msg=f"Selected configuration file: {selected_config}")

    node_planner = Node(
        package="global_planner",
        executable="global_planner",
        output="screen",
        parameters=[selected_config],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return [log_selected_config, node_planner]


def generate_launch_description():
    declare_controller_mode_cmd = DeclareLaunchArgument(
        "controller_mode",
        default_value="mpc_safe",
        description="Controller mode: 'mpc_safe' or 'mpc_risky' or 'pure_pursuit' or 'mpc_rain' (default: ERROR)",
    )

    # Set the environment variable for colorized output
    set_colorized_output = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    return LaunchDescription(
        [
            set_colorized_output,
            declare_controller_mode_cmd,
            OpaqueFunction(function=launch_setup),
        ]
    )
