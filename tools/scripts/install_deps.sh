#!/bin/bash

SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT_DIR="$( realpath $SCRIPTS_DIR/../..)"
ROSDEP_DIR="$REPO_ROOT_DIR/tools/rosdep"

# Create necessary directories
sudo mkdir -p /etc/ros/rosdep/amz/
sudo mkdir -p /usr/local/share/amz
sudo mkdir -p /etc/ros/rosdep/sources.list.d

# Add the GTSAM repository
sudo add-apt-repository -y ppa:borglab/gtsam-develop

# Add your custom APT repository
echo "deb [trusted=yes] http://amz-cpu1.ethz.ch:8081 stable main" | sudo tee /etc/apt/sources.list.d/custom-repo.list #129.132.161.48

# Update APT package list
sudo apt update

# Copy rosdep manifest and lists
echo "Rosdep directory in: $ROSDEP_DIR"
sudo cp $ROSDEP_DIR/amz.yaml /etc/ros/rosdep/amz/
sudo cp $ROSDEP_DIR/amz.list /etc/ros/rosdep/sources.list.d/

# Setup local shared directory
sudo mkdir -p /usr/local/share/amz
ln -sfn /usr/local/share/amz ~/.amz

# Source ROS setup
source /opt/ros/humble/setup.bash

# Update rosdep
rosdep update

# Install dependencies
rosdep install -y -r --ignore-src --from-paths $REPO_ROOT_DIR/ "$@"

# Ensure setup script is sourced
echo "source $REPO_ROOT_DIR/tools/scripts/setup.sh" >> ~/.bashrc
source $REPO_ROOT_DIR/tools/scripts/setup.sh
