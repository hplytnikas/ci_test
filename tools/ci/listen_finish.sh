#!/bin/bash

SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT_DIR="$( realpath $SCRIPTS_DIR/../..)"

# This script is used in CI. The first argument gives the number of seconds that the GitHub runner should wait
# for completing a mission in simulation. Exit code 0 is returned if mission succeeds in this time. Otherwise
# exit code 1 is returned.
source $REPO_ROOT_DIR/install/setup.bash

timeout=$1

# Wait for roscore to initialize
while ! ros2 topic list >/dev/null 2>&1; do
    echo "Waiting for roscore to be initialized"
    sleep 1
done

echo "Roscore initialized. Launching timeout."

# Wait for mission to finish
while ! ros2 topic echo --once /vcu_msgs/mission_finished >/dev/null 2>&1; do
    echo "Waiting for /vcu_msgs/mission_finished"
done

exit 0
