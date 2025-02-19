import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package = get_package_share_directory("acceleration_path_planner_basic")
    default_config = os.path.join(
        package, "config", "acceleration_path_planner_config.yaml"
    )
    node = Node(
        package="acceleration_path_planner_basic",
        executable="acceleration_path_planner_basic_node",
        output="screen",
        parameters=[default_config],
    )
    return LaunchDescription([node])
