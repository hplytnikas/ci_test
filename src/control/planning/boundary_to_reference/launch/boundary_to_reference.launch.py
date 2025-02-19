import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("boundary_to_reference"),
        "config",
        "boundary_to_reference_config.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="boundary_to_reference",
                executable="boundary_to_reference_node",
                name="boundary_to_reference_node",
                output="screen",
                parameters=[config],
            )
        ]
    )
