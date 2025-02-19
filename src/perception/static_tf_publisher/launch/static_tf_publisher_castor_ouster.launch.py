from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Path to the configuration files
    forward_camera_to_lidar_path = (
        "package://static_tf_publisher/config/castor_2024/forward_camera_to_ouster.yaml"
    )
    lidar_to_base_link_path = (
        "package://static_tf_publisher/config/castor_2024/ouster_to_base_link.yaml"
    )
    forward_camera_to_base_link_path = "package://static_tf_publisher/config/castor_2024/forward_camera_to_base_link.yaml"

    perception_tf_static_node = Node(
        package="static_tf_publisher",
        executable="static_tf_publisher",
        name="perception_tf_static",
        arguments=[forward_camera_to_lidar_path, lidar_to_base_link_path],
    )

    return LaunchDescription([perception_tf_static_node])
