#!/bin/bash

SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT_DIR="$( realpath $SCRIPTS_DIR/../..)"

source /opt/ros/humble/setup.bash
source $SCRIPTS_DIR/setup.sh

cd $REPO_ROOT_DIR
colcon build "$@"
