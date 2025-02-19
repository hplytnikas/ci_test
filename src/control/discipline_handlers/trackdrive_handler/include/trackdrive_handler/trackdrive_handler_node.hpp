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

#include <autonomous_msgs/msg/bool_stamped.hpp>   // control/brake_controller, vcu_msgs/mission_finished
#include <autonomous_msgs/msg/double_stamped.hpp> // control/max_velocity
#include <control_msgs/msg/controller_ref.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp> // lap_count
#include <trackdrive_handler_params.hpp>
#include <vcu_msgs/msg/velocity_estimation.hpp> // vcu_msgs/velocity_estimation

#include <memory>
#include <string>

/**
 * @brief Class that handles the controller for trackdrive.
 *
 * It listens to /lap_count and publishes /control/brake_controller when the lap count of 10 is reached.
 * Then when /vcu_msgs/velocity_estimation is low enough, it publishes /vcu_msgs/mission_finished.
 *
 * It also takes care of the switch from local planning to localizing in the globally optimized map.
 */

class TrackdriveHandler : public rclcpp::Node {
public:
  explicit TrackdriveHandler(std::string node_name);
  ~TrackdriveHandler();

private:
  // Publishers
  rclcpp::Publisher<autonomous_msgs::msg::BoolStamped>::SharedPtr brake_controller_pub_;
  rclcpp::Publisher<autonomous_msgs::msg::BoolStamped>::SharedPtr mission_finished_pub_;
  rclcpp::Publisher<control_msgs::msg::ControllerRef>::SharedPtr controller_ref_pub_;
  rclcpp::Publisher<autonomous_msgs::msg::DoubleStamped>::SharedPtr max_velocity_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lap_count_sub_;
  rclcpp::Subscription<vcu_msgs::msg::VelocityEstimation>::SharedPtr velocity_estimation_sub_;
  rclcpp::Subscription<control_msgs::msg::ControllerRef>::SharedPtr local_ref_sub_;
  rclcpp::Subscription<control_msgs::msg::ControllerRef>::SharedPtr global_ref_sub_;

  void lapCountCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void velocityEstimationCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg);
  void localRefCallback(const control_msgs::msg::ControllerRef::SharedPtr msg);
  void globalRefCallback(const control_msgs::msg::ControllerRef::SharedPtr msg);

  // Params
  std::shared_ptr<trackdrive_handler_params::ParamListener> param_listener_;
  trackdrive_handler_params::Params params_;

  // Timers
  rclcpp::TimerBase::SharedPtr max_velocity_timer_;
  void publishMaxVelocity();

  // variables
  bool mission_finished_detected_;
  bool global_ref_received_;
};
