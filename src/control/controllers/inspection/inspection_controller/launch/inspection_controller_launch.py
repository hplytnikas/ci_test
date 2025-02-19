import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package = get_package_share_directory("inspection_controller")
    default_config = os.path.join(package, "config", "inspection_config.yaml")
    node = Node(
        package="inspection_controller",
        executable="inspection_controller",
        output="screen",
        parameters=[default_config],
    )
    return LaunchDescription([node])
