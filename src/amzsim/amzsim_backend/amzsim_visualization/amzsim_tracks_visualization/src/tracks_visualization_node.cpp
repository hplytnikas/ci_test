/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023 Authors:
 *   - Jonas Grütter <jgruette@ethz.ch>
 *   - Bartosz Mila <bamila@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "amzsim_tracks_visualization/tracks_visualization.hpp"

// using tracks_visualization::TracksVisualizer::loadCones;
// using tracks_visualization::TracksVisualizer::publishMarkers;

int main(int argc, char **argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<tracks_visualization::TracksVisualizer>(rclcpp::NodeOptions());

  // Loads the cones position from the desired track
  node->tracks_visualization::TracksVisualizer::loadCones();

  // Publishes the markers (i.e. cones in this case) into Rviz
  node->tracks_visualization::TracksVisualizer::publishMarkers();

  // Spin the node
  rclcpp::spin(node);

  // Shutdown ROS2
  rclcpp::shutdown();

  return 0;
}
