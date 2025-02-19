import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Camera launch file
    pylon_camera_launch = os.path.join(
        get_package_share_directory("pylon_ros2_camera_wrapper"),
        "launch",
        "pylon_ros2_camera.launch.py",
    )

    # Camera configuration file
    config_file_path = os.path.join(
        get_package_share_directory("camera_launcher"), "config", "default.yaml"
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file",
        default_value=config_file_path,
        description="Path to the configuration file for the Pylon camera",
    )

    declare_camera_id_cmd = DeclareLaunchArgument(
        "camera_id",
        default_value="forward_camera",
        description="Id of the camera. Used as node namespace.",
    )

    # Launch description
    launch_description = [
        declare_config_file_cmd,
        declare_camera_id_cmd,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pylon_camera_launch),
        ),
    ]

    return LaunchDescription(launch_description)
