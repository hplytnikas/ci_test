# SLAM Backend Package

SLAM Backend Package is a robust library designed to facilitate the implementation of graph-based SLAM and optimization using the GTSAM library.

## Features

- **Graph SLAM**: Utilizes graph-based approaches to SLAM to build and optimize maps. Offers the ability to construct a graph of 2D poses and landmarks (cones) and optimize it. The graph edges can be set to a maximum and pruning is handled automatically.
- **Optimization with GTSAM**: Leverages the Generic Factor Graph Solving and Optimization library (GTSAM) for robust pose estimation and map optimization, ensuring high precision and reliability.

## Dependencies

This package depends on the following external libraries:

- **GTSAM**: A library designed for robust pose estimation and SLAM. It can be installed from [GTSAM's GitHub repository](https://github.com/borglab/gtsam) or using package managers.
- **Eigen3**: A high-level C++ library for linear algebra, matrix and vector operations, geometrical transformations, numerical solvers, and related algorithms. Install it with the command:
  `sudo apt install libeigen3-dev`
- **SLAM Common Package**: A set of common utilities and interfaces for SLAM systems. Ensure this package is included in your project as a dependency.

## Build

1. **Install Dependencies**: Before building the library, make sure all dependencies are installed. For GTSAM, follow the instructions provided in its official repository. For Eigen3, use the command provided above.
2. **Build the Library**: `colcon build`.
3. **Source Your Workspace**: `source install/setup.bash`.

Ensure that your build system (e.g., CMake) is correctly set up to find and link against GTSAM, Eigen3, and the SLAM Common package.
