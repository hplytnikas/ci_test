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

#include "inspection_controller/inspection_controller.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<ControllerNode> controller_node = std::make_shared<ControllerNode>("inspection_controller_node");
  std::shared_ptr<InspectionVisualizationNode> inspection_visualization_node =
      std::make_shared<InspectionVisualizationNode>("inspection_visualization_node");
  std::shared_ptr<inspection_controller_params::ParamListener> param_listener =
      std::make_shared<inspection_controller_params::ParamListener>(controller_node);
  InspectionController inspection_controller(controller_node, inspection_visualization_node, param_listener);

  // Execute both nodes at the same time
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_node);
  executor.add_node(inspection_visualization_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
