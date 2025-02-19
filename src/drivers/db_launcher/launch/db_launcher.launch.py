import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package = get_package_share_directory("db_launcher")
    node = Node(package="db_launcher", executable="db_launcher_node", output="screen")
    return LaunchDescription([node])
