from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("lap_counter"), "config", "param.yaml"
    )

    ld = LaunchDescription()

    lap_counter_node = Node(
        package="lap_counter",
        executable="lap_counter",
        parameters=[config],
        output="screen",
    )
    ld.add_action(lap_counter_node)
    return ld
