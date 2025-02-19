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

#include "inspection_visualization_node/inspection_visualization_node.hpp"

InspectionVisualizationNode::InspectionVisualizationNode(std::string node_name) : Node(node_name) {
  this->declare_parameter("mission_finished_topic", "/vcu_msgs/mission_finished");
  mission_finished_topic_ = this->get_parameter("mission_finished_topic").as_string();
  mission_finished_publisher_ = this->create_publisher<autonomous_msgs::msg::BoolStamped>(mission_finished_topic_, 1);
}

InspectionVisualizationNode::~InspectionVisualizationNode() {}

void InspectionVisualizationNode::publishMissionFinished(bool finished) {
  autonomous_msgs::msg::BoolStamped mission_finished_msg;
  mission_finished_msg.header.stamp = this->now();
  mission_finished_msg.data = finished;
  mission_finished_publisher_->publish(mission_finished_msg);
}
