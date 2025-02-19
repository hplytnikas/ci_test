import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

"""
To make it start on the power on place this file on /etc/systemd/system/
db_launcher.service:

[Unit]
Description=ROS2 Dashboard Launcher
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/bash -c '. /opt/ros/humble/setup.bash; . install/setup.bash; ros2 launch launcher/driver_and_db_launcher.launch.py'
Restart=always
User=amz
WorkingDirectory=/home/amz/autonomous_2024

[Install]
WantedBy=multi-user.target


Then run
sudo systemctl daemon-reload
sudo systemctl enable db_launcher.service #To start at boot
sudo systemctl start db_launcher.service #To start now
sudo systemctl status db_launcher.service #To check status
"""

# ros2 launch db_launcher db_launcher.launch.py
# ros2 launch vcu_comm_interface vcu_comm_interface.launch.py


def generate_launch_description():
    # Share directory of the packages
    db_launcher_pkg_share_dir = get_package_share_directory("db_launcher")
    vcu_comm_interface_pkg_share_dir = get_package_share_directory("vcu_comm_interface")

    # Get the path to the included launch file
    db_launcher_launch = os.path.join(
        db_launcher_pkg_share_dir, "launch", "db_launcher.launch.py"
    )
    vcu_comm_interface_launch = os.path.join(
        vcu_comm_interface_pkg_share_dir, "launch", "vcu_comm_interface.launch.py"
    )

    return LaunchDescription(
        [
            # Launch the db_launcher.launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(db_launcher_launch),
            ),
            # Launch the vcu_comm_interface.launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vcu_comm_interface_launch),
            ),
        ]
    )
