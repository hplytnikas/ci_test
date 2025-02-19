from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_control = Node(
        package="amzsim_fakecontrol", executable="talker", name="control_publisher"
    )

    return LaunchDescription(
        [
            publish_control,
        ]
    )
