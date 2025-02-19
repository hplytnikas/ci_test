import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package = get_package_share_directory("autocross_handler")
    default_config = os.path.join(package, "config", "autocross_handler_config.yaml")
    node = Node(
        package="autocross_handler",
        executable="autocross_handler_node",
        output="screen",
        parameters=[default_config],
    )
    return LaunchDescription([node])
