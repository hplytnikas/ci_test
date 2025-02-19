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

/*
 *
 * TODO(digarcia) lookahead point topic should be a parameter
 *
 * PurePursuitControllerNode class
 * This class inherits from AbstractControllerNode and PurePursuitController.
 * It is the node that runs the pure pursuit controller.
 * It subscribes to and publishes to everything AbstractControllerNode does.
 *
 * Parameters:
 *  TODO(digarcia): Add parameters
 *
 * Subscribes to:
 *
 * Publishes to:
 *    - lookahead_point (geometry_msgs::msg::PointStamped)
 */

#pragma once

#include "controller_node/controller_node.hpp"
#include <autonomous_msgs/msg/boundary.hpp>
#include <control_msgs/msg/pid_state.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <easy/profiler.h>
#include <memory>
#include <string>

class PurePursuitVisualizationNode : public rclcpp::Node {
public:
  ~PurePursuitVisualizationNode() {}
  explicit PurePursuitVisualizationNode(std::string node_name);

  void shareLookaheadPoint(const ControllerNode::ReferenceState &lookahead_point);

  void shareReference(const ControllerNode::Reference &reference);

  void shareLongitudinalPidState(const control_msgs::msg::PidState &pid_state);
  void shareLateralPidState(const control_msgs::msg::PidState &pid_state);

  void shareLateralDeviation(float lateral_deviation);

  bool getAccelMapCreated();

protected:
  // SUBSCRIBERS
  // any subscriber you want to add

  // Only used for accel. It is not great that this is done here, but it is a quick fix
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr accel_map_created_subscriber_;

  // PUBLISHERS
  // any publisher you want to add
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> lookahead_point_publisher_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> path_publisher_;

  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::PidState>> pid_longitudinal_state_publisher_;
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::PidState>> pid_lateral_state_publisher_;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> lateral_deviation_publisher_;

  // CALLBACKS
  // any callback you want to add
  void CallbackAccelMapCreated(const std_msgs::msg::Bool::SharedPtr msg);

  bool accel_map_created_;
};
