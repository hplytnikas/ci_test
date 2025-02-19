#!/bin/bash

SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
VIRTUALIZATION_DIR="$(dirname "$SCRIPTS_DIR")"

source $SCRIPTS_DIR/gh_agent_setup.sh

# XDG_RUNTIME_DIR related comments are for sharing the wayland socket
# DISPLAY is for setting which wayland/x11 display docker should be using (the one we are currently using)
# .X11-unix for sharing the x11 socket
# adding the files directory shared volume for file sharing between docker and host
# sharing the SSH_AUTH_SOCK for the same github access rights for docker as we have on host
docker run \
	-e "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
	-e "WAYLAND_DISPLAY=$WAYLAND_DISPLAY" \
	-v "$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY" \
	-e "DISPLAY=$DISPLAY" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix" \
	-v "$VIRTUALIZATION_DIR/files:/home/amz/files" \
  -v "$SSH_AUTH_SOCK:/ssh-agent" \
  -v "$VIRTUALIZATION_DIR/dotfiles:/home/amz/.config" \
  -e "SSH_AUTH_SOCK=/ssh-agent" \
  -p 9090:9090 \
	-d \
	--name virtualization \
	virtualization \
	tail -f /dev/null 2>&1 | $SCRIPTS_DIR/prettify.sh "Running Docker"

xhost +local:docker
