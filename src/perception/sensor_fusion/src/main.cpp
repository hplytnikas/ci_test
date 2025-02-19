/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - Matteo Mazzonelli <m.mazzonelli@gmail.com>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include <rclcpp/rclcpp.hpp>

#include "fusion.hpp"

int main(int argc, char **argv) {
  EASY_PROFILER_ENABLE;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_fusion_baseline::SensorFusion>();
  bool profiling_enabled;
  node->get_parameter("profiling_enabled", profiling_enabled);

  // Get current time
  std::time_t now = std::time(nullptr);
  // Format time to a string "YYYYMMDD_HHMM"
  char buf[19];
  std::strftime(buf, sizeof(buf), "%Y_%m_%d-%H_%M", std::localtime(&now));

  rclcpp::spin(node);
  rclcpp::shutdown();

  if (profiling_enabled) {
    std::string filename = std::string("sensor_fusion_") + buf + ".prof";
    std::string profiler_file_path = "profiler/" + filename;
    profiler::dumpBlocksToFile(profiler_file_path.c_str()); // Store profiling stats
    RCLCPP_INFO_STREAM(node->get_logger(), "Sensor fusion: Profiling data saved to " << profiler_file_path);
  }
  return 0;
}
