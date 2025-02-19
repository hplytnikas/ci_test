# Managing Custom Dependencies in ROS Projects

## Introduction

This guide outlines how to manage custom dependencies for ROS projects, particularly those not listed on [ROS Index](https://index.ros.org/). Custom dependencies are essential for projects requiring libraries or tools not available through the standard `rosdep install` process.

Before proceeding with manual dependency management, please ensure the required dependency is not available on ROS Index to avoid unnecessary work.

## Directory Structure Overview
```
├── README.md
├── amz.list
├── amz.yaml
├── empty.tar
├── generate_dep.sh
├── rdmanifests
│   └── onnxruntime.rdmanifest
└── .script_stuff
    ├── package_xml_template
    ├── rdmanifest_template
    ├── yaml_template_apt
    ├── yaml_template_custom
    └── yaml_template_pip
```

The ROS dependency directory includes several important files and directories:

- `README.md`: Provides an overview and documentation.
- `generate_dep.sh`: A script to simplify the addition of new dependencies.
- `amz.yaml`: The main file where custom dependencies are listed.
- `rdmanifests`: A directory containing `.rdmanifest` files, which define how to install non-standard dependencies.
- `.script_stuff`: A directory holding templates for creating `amz.yaml` entries and `.rdmanifest` files. This directory is primarily for internal use by scripts.

Notable Files:
- `empty.tar`: A placeholder required for `.rdmanifest` files. It's necessary but not used directly.

## Adding New Dependencies

### Using the `generate_dep.sh` Script

For ease of use, the `generate_dep.sh` script automates the process of adding new dependencies. It's recommended to use this script to ensure consistency and reduce manual errors. This script assists in adding entries to `amz.yaml` and creating `.rdmanifest` files as needed.
This can be called both with `./generate_dep.sh` for a tui, or with parameters `./generate_dep.sh dep_name dep_type [actual_dep_name]`

### Understanding Dependency Types

There are three types of dependencies you can add:

1. **apt**: Dependencies installable via the `apt` package manager.
2. **pip**: Python packages installable via `pip`.
3. **rdmanifest**: For dependencies that cannot be installed through `apt` or `pip`, `.rdmanifest` files allow you to define custom installation scripts.

### The Role of `.rdmanifest` Files

`.rdmanifest` files are essentially shell scripts that define custom installation procedures. For example, installing a library from a compressed file hosted on GitHub:

```bash
uri: 'https://raw.githubusercontent.com/TheSquake/amz_rosdep/master/empty.tar'
md5sum: '1276481102f218c981e0324180bafd9f'
install-script: |
  #!/bin/bash
  wget "https://github.com/microsoft/onnxruntime/releases/download/v1.12.0/onnxruntime-linux-x64-1.12.0.tgz" -O /tmp/onnxruntime.tgz
  mkdir -p ~/.amz/libs
  tar zxvf /tmp/onnxruntime.tgz -C $HOME/.amz/libs
  rm /tmp/onnxruntime.tgz
check-presence-script: |
  #!/bin/bash
  if [ -d ~/.amz/libs/onnxruntime-linux-x64-1.12.0 ]; then
    exit 0;
  else
    exit 1;
  fi
```

These scripts are executed by `rosdep` to install the custom dependency.

### Manual Creation of `.rdmanifest` Files

If you prefer not to use the `generate_dep.sh` script, you can manually create `.rdmanifest` files in the `rdmanifests` directory. Afterwards, you must add an entry to `amz.yaml` pointing to the new `.rdmanifest` file. For example:

```yaml
onnxruntime:
  ubuntu:
    source:
      uri: "file:///etc/ros/rosdep/amz/rdmanifests/onnxruntime.rdmanifest"
```
yes, this gets copied to /etc/ros/rosdep/amz/, this happens automatically when calling `/tools/scripts/install_deps.sh`
