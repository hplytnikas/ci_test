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

#include "local_planner/local_planner_node.hpp"

std::string GetFilnameString() {
  // Get current time
  std::time_t now = std::time(nullptr);
  // Format time to a string "YYYY_MM_DD-HH_MM"
  char buf[19];
  std::strftime(buf, sizeof(buf), "%Y_%m_%d-%H_%M", std::localtime(&now));
  return "profiler/LocalPlanner_ProfilerLog_" + std::string(buf) + ".prof";
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Initialises the local planner node
  std::shared_ptr<LocalPlannerNode> local_planner_node = std::make_shared<LocalPlannerNode>("local_planner_node");

  bool enable_profiling = local_planner_node->get_parameter("enable_profiler").get_value<bool>();

  if (enable_profiling) {
    EASY_PROFILER_ENABLE;
  }

  // Executes the node
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(local_planner_node);
  executor.spin();

  rclcpp::shutdown();

  if (enable_profiling) {
    // write to a new file every time, where filename has the date and time
    profiler::dumpBlocksToFile(GetFilnameString().c_str());
  }

  return 0;
}
