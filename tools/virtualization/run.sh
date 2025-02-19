#!/bin/bash

# Directories
VIRTUALIZATION_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SCRIPTS_DIR="$VIRTUALIZATION_DIR/.scripts"
REPO_SCRIPTS_DIR="$(realpath $VIRTUALIZATION_DIR/../scripts)"

IMAGE_NAME="$(cat $SCRIPTS_DIR/constants/image_name)"
CONTAINER_NAME="$(cat $SCRIPTS_DIR/constants/container_name)"

container_state="$($SCRIPTS_DIR/container_state.sh)"

# Initialize a flag variable
found_f=false
# Loop through all arguments
for arg in "$@"; do
    if [[ "$arg" == "-f" ]]; then
        found_f=true
        shift
        break # Exit the loop since we found what we were looking for
    fi
done

prettify_output() {
    while IFS= read -r line; do
      echo -e "\033[31m$line\033[0m"
    done
}

show_help() {
cat << EOF
Usage:
${0##*/} COMMAND
${0##*/} -f COMMAND   Force a command (rm, rmi)

Commands:
b|build       execute build process
r|run         execute run process
s|stop        stops the container
rm|remove          removes the container ! BE CAREFUL TO NOT LOOSE PROGRESS !
rmi|remove_image         removes the image
ba|build_autonomous
help        display this help and exit
EOF
}

build() {
  exists="$(docker container ls -a --filter "ancestor=$IMAGE_NAME" --format '{{.ID}}')"
  if [[ -n "${exists// /}" ]]; then
    echo "Building the image will make the current container outdated, you can choose to remove it (Y), or not (N)." 2>&1 | prettify_output
    remove
  fi
  echo Building the image...
  $SCRIPTS_DIR/build.sh "$@"
  echo Container built
}

run() {
  echo "Attempting to run..."

  case $container_state in
    "running")
      echo "The container is already running !"
      ;;
    "container exists")
      $SCRIPTS_DIR/gh_agent_setup.sh
      docker start $CONTAINER_NAME
      ;;
    "only image exists")
      $SCRIPTS_DIR/run.sh "$@"
      ;;
    "image doesnt exist")
      build "$@"
      $SCRIPTS_DIR/run.sh "$@"
      ;;
    *)
      echo "Something went horribly wrong contact Bartosz"
  esac

  docker exec -it $IMAGE_NAME bash
}

stop() {
  echo "Stopping the container..."
  docker stop $CONTAINER_NAME
}

start() {
  echo "Starting the container..."
  run
}

remove() {
  choice  "echo \"CONFIRM REMOVING THE CONTAINER (all changes will be lost)  y/N\"" \
          "echo \"Stopping and removing the container...\"
           docker stop $CONTAINER_NAME
           docker rm $CONTAINER_NAME" \
          "echo \"Not removing anything...\""  2>&1 | prettify_output
}

remove_image() {
  choice  "echo \"CONFIRM REMOVING THE IMAGE AND CONTAINER (all changes will be lost) y/N\"" \
          "echo \"Stopping and removing the container and the image...\"
           docker stop $CONTAINER_NAME
           docker rm $CONTAINER_NAME
           docker rmi $IMAGE_NAME" \
          "echo \"Not removing anything...\""  2>&1 | prettify_output
}

build_autonomous() {
  docker exec "virtualization" bash -c "$REPO_SCRIPTS_DIR/build.sh" 2>&1 | $SCRIPTS_DIR/prettify.sh "Building the autonomous repo"
}

choice() {
  local pre_choice="$1"
  local eval_y="$2"
  local eval_n="$3"
  if [[ $found_f == true ]]; then
    eval "$eval_y"
    return
  fi

  eval "$pre_choice"
  read choice
  if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
    eval "$eval_y"
  else
    eval "$eval_n"
  fi
}


command="$1"
shift # Remove the first argument, the rest are parameters for the command


# Options
case "$command" in
  "")
    run "$@"
    ;;
  b|build)
    build "$@"
    ;;
  r|run)
    run "$@"
    ;;
  help|--help|-h)
    show_help
    ;;
  s|stop)
    stop "$@"
    ;;
  rm|remove)
    remove "$@"
    ;;
  rmi|remove_image)
    remove_image "$@"
    ;;
  ba|build_autonomous)
    build_autonomous "$@"
    ;;
  start)
    start "$@"
    ;;
  restart)
    stop "$@"
    container_state="$($SCRIPTS_DIR/container_state.sh)"
    run "$@"
    ;;
  *)
    echo "Error: Unknown command '$command'" >&2
    show_help >&2
    exit 1
    ;;
esac
