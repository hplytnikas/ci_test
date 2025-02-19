import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

# Launches inspection_controller inspection_controller_launch.py and records rosbag


def generate_launch_description():
    # Share directory of the packages
    inspection_controller_pkg_share_dir = get_package_share_directory(
        "inspection_controller"
    )

    # Get the path to the included launch file
    inspection_controller_launch = os.path.join(
        inspection_controller_pkg_share_dir, "launch", "inspection_controller_launch.py"
    )

    return LaunchDescription(
        [
            # Launch the inspection_controller_launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(inspection_controller_launch),
            ),
            TimerAction(
                period=10.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "bag",
                            "record",
                            "-a",
                            "-s",
                            "mcap",
                            # "-x",
                            # "(/lidar_points|/forward_camera/pylon_ros2_camera_node/image_raw)",
                        ],
                        output="screen",
                    ),
                ],
            ),
        ]
    )
