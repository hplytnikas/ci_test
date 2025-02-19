#!/bin/bash

#Loop until the VCU ip is available (192.168.1.111)

while ! ping -c 1 -W 1 192.168.1.111; do
    echo "Waiting for VCU to be on..."
done

# Wait a bit after it is on
sleep 10

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/usr/local/share/amz/acados/lib"
export ACADOS_SOURCE_DIR="/usr/local/share/amz/acados"

source ~//HesaiLidar_ROS_2.0_PointCloud_Filter/install/setup.bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch launcher/driver_and_db_launcher.launch.py
