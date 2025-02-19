from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    perception_mode = LaunchConfiguration("perception_mode")

    declare_perception_mode_cmd = DeclareLaunchArgument(
        "perception_mode",
        default_value="lidar_only",
        description="Mode to launch: camera_only, lidar_only or sensor_fusion",
    )

    config = os.path.join(
        get_package_share_directory("cone_fusion"), "config", "default.yaml"
    )

    cone_fusion_node = Node(
        package="cone_fusion",
        executable="cone_fusion",
        name="cone_fusion",
        parameters=[config, {"perception_mode": perception_mode}],
    )

    return LaunchDescription([declare_perception_mode_cmd, cone_fusion_node])
