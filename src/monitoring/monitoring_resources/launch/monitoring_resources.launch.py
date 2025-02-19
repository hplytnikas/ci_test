from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory("monitoring_resources"),
        "config",
        "monitoring_resources_params.yaml",
    )
    print(f"Using parameter file: {parameter_file}")

    return LaunchDescription(
        [
            Node(
                package="monitoring_resources",
                executable="monitoring_resources_node",
                name="monitoring_resources_node",
                output="screen",
                parameters=[parameter_file],
            )
        ]
    )
