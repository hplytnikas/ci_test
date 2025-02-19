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

#include "amzsim_simulator/simulator.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("simulator");
  Simulator simulator(node);

  rclcpp::Rate rate(1000);

  while (rclcpp::ok()) {
    simulator.publish_odom();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  return 0;
}
