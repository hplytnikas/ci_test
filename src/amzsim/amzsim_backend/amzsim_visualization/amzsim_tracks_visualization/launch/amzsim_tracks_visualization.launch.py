import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Define the path to the track file
    track_path_default = os.path.join(
        get_package_share_path("amzsim_tracks"),
        "tracks_csv/standard/skidpad/Skidpad.csv",
    )

    # Define launch arguments
    track_path_arg = DeclareLaunchArgument(
        "track_path",
        default_value=track_path_default,
        description="Path to the track file",
    )

    # Define the node for track visualization
    track_visualization_node = Node(
        package="amzsim_tracks_visualization",
        executable="tracks_visualization_node",
        name="tracks_visualization",
        output="screen",
        parameters=[{"track_path": LaunchConfiguration("track_path")}],
    )

    # Create and return the launch description
    return LaunchDescription([track_path_arg, track_visualization_node])
