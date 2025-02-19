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
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package = get_package_share_directory("boundary_estimation")

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
        ],
    )

    # Foxglove visualization Node
    visualization = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch/foxglove_bridge_launch.xml",
            )
        )
    )

    return LaunchDescription([boundary_estimation_node, visualization])
