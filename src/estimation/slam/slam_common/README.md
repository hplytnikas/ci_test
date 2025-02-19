# SLAM Common Package

SLAM Common Package is a foundational library designed to support the development of SLAM systems. It encompasses a comprehensive set of functionalities including representation of poses and landmarks, utility functions for map transformations, SLAM node implementation, and a visualization component. This package serves as the core upon which SLAM applications can be built, emphasizing modularity, ease of use, and integration flexibility.

## Features

- **Pose and Landmark Representation**: Provides classes for representing 2D poses (positions and orientations) and landmarks within the environment, facilitating easy manipulation and access.
- **Map Transformations Utilities**: Includes a suite of utility functions for performing transformations and manipulations on map data, enhancing the versatility of map handling operations.
- **SLAM Node Implementation**: Implements the core SLAM node that can be used to fetch and broadcast tf transforms, receive perception cone messages, and publish the map.
- **Visualization**: Features a visualizer component that enables real-time visualization of the SLAM process, including the global map, poses, raw graph before optimization, and data associations between observed landmarks.

## Broadcast Transformation

**map -> odom**

The package can be used to broadcast the static transformation between the `map` frame and the `odom` frame.

## Subscriptions and Publications

**This package subscribes to the following topic:**

- `/perception/cone_array`: Receives an array of detected cones (landmarks) from the perception module.

**This package publishes to the following topic:**

- `/estimation/online_map`: Publishes the online map constructed and updated by the SLAM system.

**And to the following visualization topics:**

- `/estimation/viz/optimized_global_map`: Visualizes the global map after every optimization step.
- `/estimation/viz/optimized_poses`: Visualizes the poses after every optimization step.
- `/estimation/viz/raw_graph`: Visualizes the raw graph before optimization.
- `/estimation/viz/association`: Visualizes the associations between detected cones.

## Dependencies

This package relies on several external libraries:

- **GTSAM**: A library designed for robust pose estimation and SLAM. It can be installed from [GTSAM's GitHub repository](https://github.com/borglab/gtsam) or using package managers.
- **Eigen3**: A high-level C++ library for linear algebra, matrix and vector operations, geometrical transformations, numerical solvers, and related algorithms. Install it with the command:
  `sudo apt install libeigen3-dev`

## Build

1. **Install Dependencies**: Before building the library, make sure all dependencies are installed. For GTSAM, follow the instructions provided in its official repository. For Eigen3, use the command provided above.
2. **Build the Library**: `colcon build`.
3. **Source Your Workspace**: `source install/setup.bash`.

Ensure that your build system (e.g., CMake) is correctly set up to find and link against GTSAM and Eigen3.
