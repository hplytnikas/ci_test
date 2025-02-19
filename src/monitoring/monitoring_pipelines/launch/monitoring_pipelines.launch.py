import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("monitoring_pipelines"),
        "config",
        "monitoring_pipelines.yaml",
    )

    # Print the path to the parameter file
    print(f"Using parameter file: {config}")

    return LaunchDescription(
        [
            Node(
                package="monitoring_pipelines",
                executable="monitoring_pipelines_node",
                name="monitoring_pipelines_node",
                parameters=[config],
                output="screen",
            )
        ]
    )
