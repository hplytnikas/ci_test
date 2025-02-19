/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023 - 2024  Authors:
 *   - Lorenzo Codeluppi <lcodeluppi@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#include <cstring>
#include <ctime>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "boundary_estimation_frontend.hpp"
#include "node_handle.hpp"
#include <easy/profiler.h>

const char profiler_dir[] = "profiler/";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // this allow to load all the values from yaml without
  // specifically declare all the variables in the node
  const rclcpp::NodeOptions &options = (rclcpp::NodeOptions()
                                            // .allow_undeclared_parameters(true)
                                            .automatically_declare_parameters_from_overrides(true));

  auto node = std::make_shared<boundary_estimation::NodeHandle>("delaunay_search", "/", options);

  bool profiling_enabled;
  node->get_parameter("logging.enabled", profiling_enabled);

  if (profiling_enabled) {
    EASY_PROFILER_ENABLE; // Enable profiling
  }

  boundary_estimation::BoundaryEstimationFrontend delaunay_search_handle(node);

  // Get current time
  std::time_t now = std::time(nullptr);
  // Format time to a string "YYYYMMDD_HHMM"
  char buf[19];
  std::strftime(buf, sizeof(buf), "%Y_%m_%d-%H_%M", std::localtime(&now));

  rclcpp::spin(node);
  rclcpp::shutdown();

  if (profiling_enabled) {
    std::string filename = std::string("be_") + buf + ".prof";
    std::string profiler_file_path = profiler_dir + filename;
    profiler::dumpBlocksToFile(profiler_file_path.c_str()); // Store profiling stats
    RCLCPP_INFO_STREAM(node->get_logger(), "Boundary Estimation: Profiling data saved to " << profiler_file_path);
  }

  return 0;
}
