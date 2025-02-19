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

#include <acceleration_path_planner_params.hpp>
#include <rclcpp/rclcpp.hpp>

#include <control_msgs/msg/controller_ref.hpp>
#include <control_msgs/msg/reference_state.hpp>

#include <memory>
#include <string>

/**
 * @brief Class that does path planning for acceleration
 *
 * The acceleration handler does the detection of the end of the track and
 * publishes the target/max velocity for the discipline.
 *
 * This node just publishes the path which is just a straight line.
 */

class AccelerationPathPlanner : public rclcpp::Node {
public:
  explicit AccelerationPathPlanner(std::string node_name);
  ~AccelerationPathPlanner();

private:
  // Publishers
  rclcpp::Publisher<control_msgs::msg::ControllerRef>::SharedPtr reference_pub_;

  // Params
  std::shared_ptr<acceleration_path_planner_params::ParamListener> param_listener_;
  acceleration_path_planner_params::Params params_;

  // Timers
  rclcpp::TimerBase::SharedPtr reference_timer_;
  void publishReference();
};
