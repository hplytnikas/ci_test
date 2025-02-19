from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("yolo_camera_detector"), "config", "default.yaml"
    )
    yolo_camera_detector_node = Node(
        package="yolo_camera_detector",
        executable="yolo_camera_detector",
        name="yolo_camera_detector",
        parameters=[config],
    )

    return LaunchDescription([yolo_camera_detector_node])
