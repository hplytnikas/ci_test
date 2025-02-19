/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Romir Damle <rdamle@ethz.ch>
 *   - Emil Fahrig <efahrig@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "amzsim_perception/perception_faked.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("faked_perception");

  string track_path;
  node->declare_parameter("track_path", "default_path");
  if (!node->get_parameter("track_path", track_path)) {
    RCLCPP_INFO(node->get_logger(), "Could not find param!");
    rclcpp::shutdown();
  }

  perception_faked::Perception_Faker perception_fake(node, track_path);

  perception_fake.publish_cones_in_fov();

  rclcpp::shutdown();
  return 0;
}
