from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    lidar_mode = LaunchConfiguration("lidar_mode")
    car_mode = LaunchConfiguration("car_mode")

    declare_lidar_mode_cmd = DeclareLaunchArgument(
        "lidar_mode",
        default_value="hesai",
        description="Type of lidar to launch: ouster, hesai",
    )

    declare_car_mode_cmd = DeclareLaunchArgument(
        "car_mode",
        default_value="dufour",
        description="Type of car to launch: dufour, castor",
    )

    config = os.path.join(
        get_package_share_directory("sensor_fusion"), "config", "default.yaml"
    )

    sensor_fusion_node = Node(
        package="sensor_fusion",
        executable="sensor_fusion",
        name="sensor_fusion",
        parameters=[config, {"lidar_mode": lidar_mode, "car_mode": car_mode}],
    )

    return LaunchDescription(
        [declare_lidar_mode_cmd, declare_car_mode_cmd, sensor_fusion_node]
    )
