from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = "slam_frontend"  # Define the package name
    executable_name = "slam_frontend"  # Make sure this matches the name of your executable defined in CMakeLists.txt
    node_name = "slam"

    # Define the path to the parameter files
    config_slam = os.path.join(
        get_package_share_directory("slam_frontend"), "config", "params_default.yaml"
    )
    config_visualizer = os.path.join(
        get_package_share_directory("slam_frontend"),
        "config",
        "visualization_params_default.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package=package_name,
                executable=executable_name,
                name=node_name,
                output="screen",
                parameters=[config_slam, config_visualizer],
            ),
        ]
    )
