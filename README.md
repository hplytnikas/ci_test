# Autonomous 2024

Autonomous pipeline for season 2023/2024

## Installing Dependencies Guide

To install dependencies successfully, you must execute the installation script with elevated permissions. Follow the steps outlined below to ensure a smooth installation process:

1. **Run the Installation Script**: Begin by executing the `install_deps.sh` script with `sudo` to grant it the necessary permissions:

   ```
   sudo ./tools/scripts/install_deps.sh
   ```

   This script is designed to perform several critical actions to set up your environment:

   - **Copy Required Files**: It transfers essential files needed by `rosdep` to specific directories outside the standard user space, ensuring `rosdep` can operate correctly.
   - **Update System**: The script updates your system's package list to ensure you have the latest versions of software available.
   - **Install Dependencies**: Finally, it proceeds to install the dependencies required for your project.

2. **Learn More About the Process**: For detailed information on the installation process, including specifics about how dependencies are managed and installed, refer to the README files located in the `rosdep` and `virtualization` directories. These documents offer valuable insights into the workings of the script and the overall setup.

## How to add code to this repository

For new features, create a branch `feature/MOD-my_new_feature` in your local copy of
the repository: `git checkout -b feature/MOD-my_new_feature`. You can add changes,
commit them then push it to the repository. You can then create a merge request.

For fixes use the word `fix` instead of `feature`.

You'll need to pass all the builds and tests before you can merge, as well as
receive approvals from your colleagues. Once you've received the necessary
approvals and tests pass you can merge your code.

If you are writing a new ROS package, make sure that it contains a README which should contain:

1. Short and concise description of what your package does.
2. List of topics the package/node subscribes to (with short explanations why if possible).
3. List of topics the package published to (with short explanation of what that is or maybe what depends on it).

## How to initialize the submodules of this repository

To ensures that all the submodules defined in the repository, along with their respective nested submodules are correctly cloned and checked out to the specific commits that your repository tracks, please run the following command:
```
git submodule update --init --recursive
```

## Pre-commit checks

In order to set-up Git hooks for pre-commit checks in your repository, please run the following commands:
```
pip3 install pre-commit
pip3 install cpplint
# In the root of the repository
pre-commit install
```

Now every time you make a commit to the repository, your C++ changes will be validated (and automatically formatted) against [Google Style Guidelines](https://google.github.io/styleguide/cppguide.html). It will also format your Python code, check the correctness of your XML, JSON and YAML files and automatically format other aesthetic things (like empty lines at end of files).

In order to run the pre-commit checks on all files in the repository (not just the ones editted in current commit) use:
```
pre-commit run --all-files
```
This could come handy if you started implementing changes before installing the pre-commit hooks.

## Planned source code structure

<pre>
src
├── perception
|   ├── yolo_camera_detector
|   ├── static_tf_publisher
|   ├── depth_estimation
|   ├── sensor_fusion
|   ├── lidar_cone_detector
|   └── perception_meta
├── estimation
|   ├── boundary_estimation
|   ├── tf_publisher
|   ├── slam
|   └── estimation_meta
├── control
|   ├── acceleration
|   ├── skidpad
|   ├── inspection
|   ├── llc_tuning
|   ├── mpc
|   |   ├── controller
|   |   |       ├── mpc_controller
|   |   |       └── mpc_node
|   |   ├── solvers
|   |   |       ├── code_gen
|   |   |       └── solver_common
|   |   └── utils
|   |           ├── rosparam_handler
|   |           └── splines
|   └── control_meta
├── amzsim
|   ├──amzsim_backend
|   ├──mazsim_frontend
|   ├──amzsim_interface
|   └──amzsim_msgs
├── drivers
|   ├── vcu_msgs
|   ├── vcu_comm_interface
|   ├── HesaiLidar_ROS_2
|   ├── pylon_ros_camera
└── common
    ├── alglib
    ├── parameter_handler
    ├── rosbag_record
    ├── visualisation_plugins
    ├── autonomous_msgs
    └── autonomous_meta
</pre>
