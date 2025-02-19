# Boundary Estimation

Boundary Estimation is the package responsible for computing the boundary of the track and the correspondig correct path within it.

It uses as input the SLAM map and compute the result based on the Delaunay triangulation.

## Publisher and Subscriber

**This package subscribes to the following topic**

`/estimation/online_map`
Where it receives the global map of the cones from SLAM

**This package publishes to the following topics**

`/estimation/bounded_path`
Where it publishes the boundary of the track and the middle points of the path

`/estimation/be_visual_delaunay`
Where it publishes the visualization data, this topic is used for debugging and can be disabled

`/estimation/be_runtime`
Where it publishes the runtime of the algorithm, this topic is used for debugging and can be disabled

## Package dependencies

This package depends on two packages that can be installed with **apt**

 - CGAL: Computation Geometry lib, that is used to compute the delaunay triangulation.
 It can be installed with the following comand
 `sudo apt install libcgal-dev`
 - Eigen: Linear Algebra lib used to perform any matrix operations.
 It can be installed with the following comand
 `sudo apt install libeigen3-dev`

## Build and Run

To build and run this package run the following commands:

First we need to build the package:
`colcon build`

Then we can source the installation:
`source ./install/setup.bash`

Finally we can run the node, and this can be done via the following commands:

1. If you want to use a launch file you can do the following:
`ros2 launch boundary_estimation boundary_estimation_launch.py`
**N.B.** There are different launch files, please check them in the `/launch` folder and use the most appropriate one.
2. If you want to simply run the node, you can use:
`ros2 run boundary_estimation boundary_estimation --ros-args --params-file ./src/estimation/boundary_estimation/config/boundary_estimation_default.yaml --params-file ./src/estimation/boundary_estimation/config/camera_only_parameters.yaml --params-file ./src/estimation/boundary_estimation/config/lidar_only_parameters.yaml --params-file ./src/estimation/boundary_estimation/config/sensor_fusion_parameters.yaml`
