#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("tf_publisher"), "config", "default.yaml"
    )
    visualization = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch/foxglove_bridge_launch.xml",
            )
        )
    )
    tf_publisher_node = Node(
        package="tf_publisher",
        executable="tf_publisher",
        name="tf_publisher",
        output="screen",
        parameters=[config],
    )

    ld.add_action(visualization)
    ld.add_action(tf_publisher_node)
    return ld
