from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("vcu_comm_interface"), "config", "default.yaml"
    )
    sender_node = Node(
        package="vcu_comm_interface",
        executable="sender_node",
        name="sender_node",
        parameters=[config],
    )
    receiver_node = Node(
        package="vcu_comm_interface",
        executable="receiver_node",
        name="receiver_node",
        parameters=[config],
    )

    return LaunchDescription([sender_node, receiver_node])
