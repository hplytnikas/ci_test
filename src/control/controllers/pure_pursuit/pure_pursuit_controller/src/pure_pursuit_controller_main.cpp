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

#include "pure_pursuit_controller/pure_pursuit_controller.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<ControllerNode> controller_node = std::make_shared<ControllerNode>("pure_pursuit_controller_node");
  std::shared_ptr<PurePursuitVisualizationNode> pure_pursuit_visualization_node =
      std::make_shared<PurePursuitVisualizationNode>("pure_pursuit_visualization_node");
  std::shared_ptr<pure_pursuit_controller_params::ParamListener> param_listener =
      std::make_shared<pure_pursuit_controller_params::ParamListener>(controller_node);
  PurePursuitController pure_pursuit_controller(controller_node, pure_pursuit_visualization_node, param_listener);

  bool profiling_enabled = controller_node->get_parameter("profiling_enabled").as_bool();
  if (profiling_enabled) {
    EASY_PROFILER_ENABLE;
  }

  // Get current time
  std::time_t now = std::time(nullptr);
  // Format time to a string "YYYY_MM_DD-HH_MM"
  char buf[19];
  std::strftime(buf, sizeof(buf), "%Y_%m_%d-%H_%M", std::localtime(&now));
  std::string timestamp = std::string(buf);

  // Execute both nodes at the same time
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_node);
  executor.add_node(pure_pursuit_visualization_node);
  executor.spin();

  rclcpp::shutdown();
  if (profiling_enabled) {
    std::string prof_filename = "./profiler/pure_pursuit_" + timestamp + ".prof";
    profiler::dumpBlocksToFile(prof_filename.c_str());
    RCLCPP_INFO_STREAM(controller_node->get_logger(), "Profiling data saved to " << prof_filename);
  }
  return 0;
}
