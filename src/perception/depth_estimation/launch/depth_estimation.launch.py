from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    car_mode = LaunchConfiguration("car_mode")
    config = os.path.join(
        get_package_share_directory("depth_estimation"), "config", "default.yaml"
    )

    declare_car_mode_cmd = DeclareLaunchArgument(
        "car_mode",
        default_value="dufour",
        description="Type of car to launch: dufour, castor",
    )

    depth_estimation_node = Node(
        package="depth_estimation",
        executable="depth_estimation",
        name="depth_estimation",
        parameters=[config, {"car_mode": car_mode}],
    )

    return LaunchDescription([declare_car_mode_cmd, depth_estimation_node])
