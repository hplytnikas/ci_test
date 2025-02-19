#!/bin/bash

# Initialize variables
dep_name=""
dep_type=""
actual_dep_name=""
dep_types=("custom" "apt" "pip")
dep_types_possible_custom_name=("apt" "pip")
rosdep_directory="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
templates_directory="$rosdep_directory/.script_stuff"


show_help() {
cat << EOF
Usage:
${0##*/} dep_name dep_type [actual_dep_name]
${0##*/} -h   Returns this message

Generate a custom dependency !
EOF
}
# Loop through all the command line arguments
for arg in "$@"; do
  if [[ "$arg" == "-h" || "$arg" == "--help" ]]; then
    show_help
    exit 0
  fi
done


# Function to set dependency name
set_dep_name() {
  # our name of the dependency from script arguments
  dep_name=$1
  if [ -z "$dep_name" ]; then
    read -p "Enter dependency name: " dep_name
  fi
}

# Function to set dependency type
set_dep_type() {
  # dependency type as given in dep_types
  dep_type=$1
  if [ -z "$dep_type" ] || [[ ! " ${dep_types[*]} " =~ " ${dep_type} " ]]; then
    dep_types=("custom" "apt" "pip")

    echo "Please choose a dependency type:"
    select dep_type in "${dep_types[@]}"; do
      if [ -n "$dep_type" ]; then
        echo "You selected $dep_type"
        break
      else
        echo "Invalid selection. Please try again."
      fi
    done
  fi
}

# Function to set the actual dependency name
set_actual_dep_name() {
  # the actual name of the dependency from script arguments
  actual_dep_name=$1
  # if not specified as an argument to the script then ask for it
  if [ -z "$actual_dep_name" ]; then
    # Prompt for actual dependency name, with option to press Enter to use the same name
    read -p "Enter dependency name (actual apt or pip name), or press Enter for the same name as previous ($dep_name): " actual_dep_name

    # If the user does not enter a new name, use the previous name
    if [ -z "$actual_dep_name" ]; then
      actual_dep_name=$dep_name
    fi

    echo "Dependency name to use: $actual_dep_name"
  fi
  # if nothing then simply use the previously given name
  if [ -z "$actual_dep_name" ]; then
    actual_dep_name=$dep_name
  fi
}

# Function to update amz.yaml
update_amz_yaml() {
  # replace the names with the ones specified by the user
  new_text=$(
    cat "$templates_directory/yaml_template_$dep_type" | \
    sed "s/\$dep_name/$dep_name/g" | \
    sed "s/\$actual_dep_name/$actual_dep_name/g"
  )

  # update the file
  {
    echo ""
    printf '%s\n' "$new_text"
  } >> "$rosdep_directory/amz.yaml"
}

# Function to create rdmanifest
create_rdmanifest() {
  # new folder + file from template
  cat "$templates_directory/rdmanifest_template" >> "$rosdep_directory/rdmanifests/$dep_name.rdmanifest"
}

# Main script execution

# setting the variables
set_dep_name "$1"
if [ -z "$dep_name" ]; then
  echo "name for the dependency not specified"
  exit 1
fi

set_dep_type "$2"
if [ -z "$dep_type" ]; then
  echo "type for the dependency not specified"
  exit 1
fi
if [[ " ${dep_types_possible_custom_name[*]} " =~ " ${dep_type} " ]]; then
  set_actual_dep_name "$3"
  if [ -z "$actual_dep_name" ]; then
    echo "actual dependency name not specified"
    exit 1
  fi
fi

# update the amz yaml
update_amz_yaml
if [ $? -eq 0 ]; then
  echo "updated amz.yaml"
else
  echo "failed updating amz.yaml"
  exit 1
fi

# create rdmanifest
if [ "$dep_type" = "${dep_types[0]}" ]; then # if custom
  create_rdmanifest
  if [ $? -eq 0 ]; then
    echo "created $dep_name.rdmanifest"
    echo "now please update the $dep_name.rdmanifest"
  else
    echo "failed creating $dep_name.rdmanifest"
    exit 1
  fi
fi
