/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Diego Garcia Soto <digarcia@ethz.ch>
 *   - Hironobu Akiyama <hakiyama@ethz.ch>
 *   - Jonas Ohnemus <johnemus@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "db_launcher/db_launcher_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[], char *envp[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DBLauncher>(envp);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
