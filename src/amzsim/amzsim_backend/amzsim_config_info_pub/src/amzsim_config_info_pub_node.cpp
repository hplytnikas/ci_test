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

#include "amzsim_config_info_pub/amzsim_config_info_pub.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("amzsim_config");
  ConfigInfoPub config_pub(node);
  config_pub.publishConfigInfo();

  rclcpp::shutdown();
  return 0;
}
