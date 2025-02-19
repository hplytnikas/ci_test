#!/bin/bash

SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT_DIR="$( realpath $SCRIPTS_DIR/../..)"
ROSDEP_DIR="$REPO_ROOT_DIR/tools/rosdep"

sudo mkdir -p /etc/ros/rosdep/amz/
sudo mkdir -p /etc/ros/rosdep/amz/rdmanifests
sudo mkdir -p /usr/local/share/amz
sudo mkdir -p /etc/ros/rosdep/sources.list.d # needed at the later step, sometimes this directory is not generated on ros install

sudo add-apt-repository -y ppa:borglab/gtsam-develop

sudo cp $ROSDEP_DIR/amz.yaml /etc/ros/rosdep/amz/ # cp overrides with the new pulled one
sudo cp -r $ROSDEP_DIR/rdmanifests /etc/ros/rosdep/amz/ # cp overrides
sudo cp $ROSDEP_DIR/amz.list /etc/ros/rosdep/sources.list.d/ # cp overrides
sudo cp $ROSDEP_DIR/empty.tar /etc/ros/rosdep/amz/

sudo mkdir -p /usr/local/share/amz
ln -sfn /usr/local/share/amz ~/.amz

source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y -r --ignore-src --from-paths $REPO_ROOT_DIR/ "$@"

echo "source $REPO_ROOT_DIR/tools/scripts/setup.sh" >> ~/.bashrc
source $REPO_ROOT_DIR/tools/scripts/setup.sh
