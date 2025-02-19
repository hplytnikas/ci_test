from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lidar_mode = LaunchConfiguration("lidar_mode")

    hesai_config = os.path.join(
        get_package_share_directory("lidar_cone_detector"),
        "config",
        "lidar_cone_detector_parameters_hesai_accelaration.yaml",
    )
    ouster_config = os.path.join(
        get_package_share_directory("lidar_cone_detector"),
        "config",
        "lidar_cone_detector_parameters_ouster.yaml",
    )

    # Declare the lidar_mode launch argument
    declare_lidar_mode_cmd = DeclareLaunchArgument(
        "lidar_mode", default_value="hesai", description="Lidar mode: ouster or hesai"
    )

    # Configure the node based on the lidar_mode parameter
    lidar_pipeline_node = Node(
        package="lidar_cone_detector",
        executable="lidar_cone_detector",
        name="lidar_pipeline",
        parameters=[hesai_config, ouster_config, {"lidar_mode": lidar_mode}],
    )

    return LaunchDescription([declare_lidar_mode_cmd, lidar_pipeline_node])
