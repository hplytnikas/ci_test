# AMZ Driverless Project
#
# Copyright (c) 2023 - 2024  Authors:
# - Lorenzo Codeluppi <lcodeluppi@student.ethz.ch>
#
# All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory("boundary_estimation")

    perception_mode = LaunchConfiguration("perception_mode")

    default_path_file = "boundary_estimation_default.yaml"
    camera_path_file = "camera_only_parameters.yaml"
    lidar_path_file = "lidar_only_parameters.yaml"
    sensor_fusion_path_file = "sensor_fusion_parameters.yaml"

    default_file_parameters = os.path.join(package, "config", default_path_file)
    camera_file_parameters = os.path.join(package, "config", camera_path_file)
    lidar_file_parameters = os.path.join(package, "config", lidar_path_file)
    sensor_fusion_file_parameters = os.path.join(
        package, "config", sensor_fusion_path_file
    )

    # Declare the lidar_mode launch argument
    declare_perception_mode_cmd = DeclareLaunchArgument(
        "perception_mdoe",
        default_value="sensor_fusion",
        description="Perception mode (Sensor Fusion, Lidar only, Camera only)",
    )

    # Define nodes
    boundary_estimation_node = Node(
        package="boundary_estimation",
        executable="boundary_estimation",
        name="delaunay_search",
        output="screen",
        parameters=[
            default_file_parameters,
            camera_file_parameters,
            lidar_file_parameters,
            sensor_fusion_file_parameters,
            {"perception_mode": perception_mode},
        ],
    )

    return LaunchDescription([declare_perception_mode_cmd, boundary_estimation_node])
