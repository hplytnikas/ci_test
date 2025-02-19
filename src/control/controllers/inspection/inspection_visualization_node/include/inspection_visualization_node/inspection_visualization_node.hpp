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

#pragma once

#include <autonomous_msgs/msg/bool_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

/*
 * NOTE: Technically publishing is not visualization. It should probably be done
 * with a 'Discipline Handler'. However, we haven't defined how we want to do
 * the Discipline Handlers yet so we'll just go with this for now.
 */

class InspectionVisualizationNode : public rclcpp::Node {
public:
  explicit InspectionVisualizationNode(std::string node_name);
  ~InspectionVisualizationNode() override;

  void publishMissionFinished(bool finished);

private:
  std::shared_ptr<rclcpp::Publisher<autonomous_msgs::msg::BoolStamped>> mission_finished_publisher_;

  std::string mission_finished_topic_;
};
