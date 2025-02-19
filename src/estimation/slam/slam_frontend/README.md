# SLAM Frontend Package

The SLAM Frontend Package is a critical component of a SLAM system, designed to interface directly with perception publishing and manage the flow of data through the SLAM process. This package acts as the intermediary between the cone observations and the mapping and localization backend, handling observation preprocessing, data association, and the invocation of optimization routines. Built upon the foundational `slam_backend` and `slam_common` packages, it encapsulates the main algorithm responsible for producing and maintaining an accurate map of the environment and the car's position within it.

## Features

- **Observation Preprocessing**: Processes cone observations to prepare them for data association and mapping.
- **Data Association**: Matches incoming cone observations with existing landmarks in the map, facilitating accurate map updates and pose estimations.
- **Optimization Invocation**: Utilizes the `slam_backend` package to perform graph optimization, improving the accuracy of the map and the car's estimated trajectory.
- **Pose and Landmark Publishing**: After optimization, publishes the updated poses and landmarks to be consumed by other components or for visualization purposes.
- **map->odom Transform Correction**: Maintains and corrects the transformation between the map and odometry frames, ensuring consistent and accurate localization.

## Dependencies

This package depends on both the `slam_backend` and `slam_common` packages. Ensure these dependencies are correctly installed and configured in your environment:

- **slam_backend**: Provides the optimization algorithms and graph-based SLAM infrastructure.
- **slam_common**: Supplies common SLAM functionalities including the node implementation, pose and landmark representations, and utility functions.

## Build

1. **Install Dependencies**: Follow the installation instructions for each dependency package, ensuring that all required libraries and ROS packages are installed.
2. **Build the Package**: `colcon build`.
3. **Source Your Workspace**: `source install/setup.bash`.

## Usage

To use the SLAM Frontend Package, ensure that all dependencies are running and that the ROS workspace has been sourced. You can then launch the SLAM system, which will start the frontend node (`slam`), process incoming observations, and manage the SLAM process.

`ros2 launch slam_frontend_launch_default.py`

or `ros2 launch slam_frontend_launch_default_with_foxglove.py` for visualization using Foxglove Studio.

These launch files are configured to start the frontend node, subscribe to the appropriate topics (e.g., `/perception/cone_array`), and publish the optimized map and poses to the `/estimation/online_map` topic, among others.

Parameters can be modified in the `params_default.yaml` and `visualization_params_default.yaml`.
