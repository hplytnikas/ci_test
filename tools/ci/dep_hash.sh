#!/bin/bash

# This script generates a hash from the dependencies of our project.

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$(realpath "$SCRIPT_DIR/../..")"
TOOLS_DIR="$REPO_ROOT/tools"
ROSDEP_DIR="$TOOLS_DIR/rosdep"
# Prepare a variable that the dependency files will be hashed into
HASH=""

# simulate install, get output

simulate_install_output=$($TOOLS_DIR/scripts/install_deps.sh -s 2>&1)
# find all the calls that will be made to install stuff, apt pip rosdep-source, etc.
installation_calls=$(echo "$simulate_install_output" | grep -E "pip. install|apt-get install|rosdep-source install")
# sort the calls, and remove white space to ensure proper hashing
hashable_string=$(echo "$installation_calls" | sort | tr -d "[:space:]")

HASH=$(echo $hashable_string$HASH | sha1sum)

# Next, hash in the installation scripts
SCRIPT_LIST=$(ls $ROSDEP_DIR/rdmanifests)
for script in $SCRIPT_LIST
do
    HASH=$(echo $(cat $ROSDEP_DIR/rdmanifests/$script)$HASH | sha1sum)
done

# Only return the first 16 characters of the hash. Should be enough for our
# purposes. If you ever find a collision, I'll visit Infinity racing unannounced
# and ask for a tour.
PREFIX=${HASH:0:16}
echo $PREFIX
