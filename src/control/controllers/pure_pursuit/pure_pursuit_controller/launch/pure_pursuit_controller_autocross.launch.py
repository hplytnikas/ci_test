import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package = get_package_share_directory("pure_pursuit_controller")
    default_config = os.path.join(
        package, "config", "pure_pursuit_config_autocross.yaml"
    )
    node = Node(
        package="pure_pursuit_controller",
        executable="pure_pursuit_controller",
        output="screen",
        parameters=[default_config],
    )
    return LaunchDescription([node])
