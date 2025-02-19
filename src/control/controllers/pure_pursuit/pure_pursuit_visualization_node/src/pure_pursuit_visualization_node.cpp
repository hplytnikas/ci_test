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

#include "pure_pursuit_visualization_node/pure_pursuit_visualization_node.hpp"

PurePursuitVisualizationNode::PurePursuitVisualizationNode(std::string node_name) : Node(node_name) {
  lookahead_point_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>("/control/viz/lookahead_point", 10);
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/control/viz/shifted_path", 10);

  pid_longitudinal_state_publisher_ =
      this->create_publisher<control_msgs::msg::PidState>("/control/viz/pid_longitudinal", 10);
  pid_lateral_state_publisher_ = this->create_publisher<control_msgs::msg::PidState>("/control/viz/pid_lateral", 10);
  lateral_deviation_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/control/viz/lateral_deviation", 10);

  accel_map_created_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "/estimation/acceleration_map_status", 1,
      std::bind(&PurePursuitVisualizationNode::CallbackAccelMapCreated, this, std::placeholders::_1));

  accel_map_created_ = false;
}

void PurePursuitVisualizationNode::shareLookaheadPoint(const ControllerNode::ReferenceState &point) {
  EASY_FUNCTION(profiler::colors::Green);
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = this->now(); // I should not use now() here TODO(digarcia)
  marker.ns = "lookahead_point";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  lookahead_point_publisher_->publish(marker);
}

void PurePursuitVisualizationNode::shareReference(const ControllerNode::Reference &reference) {
  EASY_FUNCTION(profiler::colors::Green);
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "base_link";
  path_msg.header.stamp = this->now(); // I should not use now() here TODO(digarcia)
  path_msg.poses.resize(reference.rows());
  for (size_t i = 0; i < reference.rows(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = reference[i].x;
    pose.pose.position.y = reference[i].y;
    pose.pose.position.z = 0;
    path_msg.poses[i] = pose;
  }
  path_publisher_->publish(path_msg);
}

void PurePursuitVisualizationNode::shareLongitudinalPidState(const control_msgs::msg::PidState &pid_state) {
  EASY_FUNCTION(profiler::colors::Green);
  pid_longitudinal_state_publisher_->publish(pid_state);
}

void PurePursuitVisualizationNode::shareLateralPidState(const control_msgs::msg::PidState &pid_state) {
  EASY_FUNCTION(profiler::colors::Green);
  pid_lateral_state_publisher_->publish(pid_state);
}

void PurePursuitVisualizationNode::shareLateralDeviation(float lateral_deviation) {
  EASY_FUNCTION(profiler::colors::Green);
  std_msgs::msg::Float32 msg;
  msg.data = lateral_deviation;
  lateral_deviation_publisher_->publish(msg);
}

void PurePursuitVisualizationNode::CallbackAccelMapCreated(const std_msgs::msg::Bool::SharedPtr msg) {
  accel_map_created_ = msg->data;
}

bool PurePursuitVisualizationNode::getAccelMapCreated() { return accel_map_created_; }
