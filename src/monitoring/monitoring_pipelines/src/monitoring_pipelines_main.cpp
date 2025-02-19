/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Jonas Ohnemus <johnemus@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "monitoring_pipelines/monitoring_pipelines_node.hpp"

int main(int argc, char *argv[]) {
  // create ros2 node
  rclcpp::init(argc, argv);

  auto pipeline_monitor = std::make_shared<monitoring_pipelines::PipelineMonitor>();
  pipeline_monitor->RunMonitoring();

  rclcpp::shutdown();

  return 0;
}
