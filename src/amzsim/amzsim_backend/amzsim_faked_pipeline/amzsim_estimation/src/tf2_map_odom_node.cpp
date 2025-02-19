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

#include "amzsim_estimation/tf_ego_vehicle.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("faked_estimation");
  tf_ego_vehicle::Listener_Publisher estimation_fake(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
