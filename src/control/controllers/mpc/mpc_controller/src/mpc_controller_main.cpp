/*******************************************************************************
 * AMZ Driverless Project                                                      *
 * Copyright (c) 2023-2024                                                     *
 * Authors:                                                                    *
 *   - Diego Garcia Soto <digarcia@ethz.ch>                                    *
 *   - Hironobu Akiyama <hakiyama@ethz.ch>                                     *
 *   - Jonas Ohnemus <johnemus@ethz.ch>                                        *
 *                                                                             *
 * All rights reserved.                                                        *
 *                                                                             *
 * Unauthorized copying of this file, via any medium, is strictly prohibited.  *
 * Proprietary and confidential.                                               *
 ******************************************************************************/

#include "mpc_controller/mpc_controller.hpp"

std::string GetFilnameString() {
  // Get current time
  std::time_t now = std::time(nullptr);
  // Format time to a string "YYYY_MM_DD-HH_MM"
  char buf[19];
  std::strftime(buf, sizeof(buf), "%Y_%m_%d-%H_%M", std::localtime(&now));
  return "profiler/MPC_ProfilerLog_" + std::string(buf) + ".prof";
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Initialises the controller node
  std::shared_ptr<ControllerNode> controller_node = std::make_shared<ControllerNode>("mpc_controller_node");

  controller_node->declare_parameter("enable_profiling", rclcpp::ParameterValue(false));
  bool enable_profiling = controller_node->get_parameter("enable_profiling").get_value<bool>();

  if (enable_profiling) {
    EASY_PROFILER_ENABLE; // Enable profiling
  }

  // Initialises the visualization node
  std::shared_ptr<MpcVisualisationNode> mpc_visualisation_node =
      std::make_shared<MpcVisualisationNode>("mpc_visualisation_node");

  // Initialises the parameter listener
  std::shared_ptr<mpc_controller_params::ParamListener> param_listener =
      std::make_shared<mpc_controller_params::ParamListener>(controller_node);

  // Initialises the MPC object
  MpcController mpc_controller(controller_node, mpc_visualisation_node, param_listener);

  // Executes both nodes at the same time
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_node);
  executor.add_node(mpc_visualisation_node);
  executor.spin();

  rclcpp::shutdown();

  if (enable_profiling) {
    // write to a new file every time, where filename has the date and time
    profiler::dumpBlocksToFile(GetFilnameString().c_str());
  }

  return 0;
}
