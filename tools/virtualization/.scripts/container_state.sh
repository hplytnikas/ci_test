#!/bin/bash

SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
image_name="$(cat $SCRIPTS_DIR/constants/image_name)"

# already running
running="$(docker ps --filter "ancestor=$image_name" --format '{{.ID}}')"
if [[ -n "${running// /}" ]]; then
  echo "running"
  exit 0
fi

# container exists but not running
exists="$(docker container ls -a --filter "ancestor=$image_name" --format '{{.ID}}')"
if [[ -n "${exists// /}" ]]; then
  echo "container exists"
  exit 0
fi

# image exists but container doesnt
image_exists="$(docker images | grep -w "$image_name" | awk '{print $3}')"
if [[ -n "${image_exists// /}" ]]; then
  echo "only image exists"
  exit 0
fi
echo "image doesnt exist"
