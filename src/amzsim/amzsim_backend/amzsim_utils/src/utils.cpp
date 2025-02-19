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

#include "amzsim_utils/utils.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

namespace utils {
std::string getRosPackageDir(const std::string &packageName) {
  auto directory = ament_index_cpp::get_package_share_directory(packageName);
  RCLCPP_INFO(rclcpp::get_logger("utils"), "Directory path: %s", directory.c_str());
  return directory;
}
} // namespace utils
