/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - Chenhao Sun    <chensun@student.ethz.ch>
 *   - Paul Gregoire  <gregoirep@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "easy/profiler.h"
#include "lidar_cone_detector_frontend.hpp"
#include "lidar_cone_detector_node.hpp"
#include "lidar_cone_detector_structs.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // this allow to load all the values from yaml without
  // specifically declare all the variables in the node
  const rclcpp::NodeOptions &options = (rclcpp::NodeOptions()
                                            // .allow_undeclared_parameters(true)
                                            .automatically_declare_parameters_from_overrides(true));

  auto node = std::make_shared<lidar_cone_detector::NodeHandle>("lidar_cone_detector", "/", options);

  std::string lidar_mode;
  bool profiling_enabled;
  node->get_parameter("lidar_mode", lidar_mode);
  node->get_parameter(lidar_mode + ".profiling.enabled", profiling_enabled);
  // Print lidar mode
  RCLCPP_INFO_STREAM(node->get_logger(), "Lidar Cone detection: Lidar mode is " << lidar_mode);
  if (profiling_enabled) EASY_PROFILER_ENABLE;

  std::shared_ptr<lidar_cone_detector::LidarConeDetectorFrontendBase> lidar_cone_detector_frontend;
  if (lidar_mode == constants::kHesaiLidar) {
    lidar_cone_detector_frontend =
        std::make_shared<lidar_cone_detector::LidarConeDetectorFrontend<lidar_cone_detector::LidarPointHesai>>(node);
  } else if (lidar_mode == constants::kOusterLidar) {
    lidar_cone_detector_frontend =
        std::make_shared<lidar_cone_detector::LidarConeDetectorFrontend<lidar_cone_detector::LidarPointOuster>>(node);
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Lidar Cone detection: Invalid lidar mode " << lidar_mode << ". Exiting.");
    rclcpp::shutdown();
    return 1;
  }
  // Get current time
  std::time_t now = std::time(nullptr);
  // Format time to a string "YYYYMMDD_HHMM"
  char buf[19];
  std::strftime(buf, sizeof(buf), "%Y_%m_%d-%H_%M", std::localtime(&now));

  rclcpp::spin(node);
  rclcpp::shutdown();

  if (profiling_enabled) {
    std::string filename = std::string("lc_") + buf + ".prof";
    std::string profiler_file_path = "profiler/" + filename;
    profiler::dumpBlocksToFile(profiler_file_path.c_str()); // Store profiling stats
    RCLCPP_INFO_STREAM(node->get_logger(), "Lidar Cone detection: Profiling data saved to " << profiler_file_path);
  }

  return 0;
}
