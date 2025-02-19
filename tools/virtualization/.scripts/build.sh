#!/bin/bash

SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
VIRTUALIZATION_DIR="$(dirname "$SCRIPTS_DIR")"

IMAGE_NAME="$(cat $SCRIPTS_DIR/constants/image_name)"

source $SCRIPTS_DIR/gh_agent_setup.sh
echo $SSH_AUTH_SOCK

BRANCH="$(git branch --show-current)"
echo $BRANCH

DOCKER_BUILDKIT=1 docker build \
  --ssh default \
  -t virtualization \
  --build-arg BRANCH=$BRANCH \
  "$@" \
  $VIRTUALIZATION_DIR
