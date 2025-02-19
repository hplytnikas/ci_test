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

#include "autocross_handler/autocross_handler_node.hpp"

AutocrossHandler::AutocrossHandler(std::string node_name) : Node(node_name), mission_finished_detected_(false) {
  // Get parameters
  param_listener_ = std::make_shared<autocross_handler_params::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // Publishers
  brake_controller_pub_ = this->create_publisher<autonomous_msgs::msg::BoolStamped>(params_.brake_controller_topic, 1);
  max_velocity_pub_ = this->create_publisher<autonomous_msgs::msg::DoubleStamped>(params_.max_velocity_topic, 1);
  mission_finished_pub_ = this->create_publisher<autonomous_msgs::msg::BoolStamped>(params_.mission_finished_topic, 1);
  controller_ref_pub_ = this->create_publisher<control_msgs::msg::ControllerRef>(params_.reference_topic, 1);

  // Subscribers
  lap_count_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      params_.lap_count_topic, 1, std::bind(&AutocrossHandler::lapCountCallback, this, std::placeholders::_1));
  velocity_estimation_sub_ = this->create_subscription<vcu_msgs::msg::VelocityEstimation>(
      params_.velocity_estimation_topic, 1,
      std::bind(&AutocrossHandler::velocityEstimationCallback, this, std::placeholders::_1));

  local_ref_sub_ = this->create_subscription<control_msgs::msg::ControllerRef>(
      params_.local_reference_topic, 1, std::bind(&AutocrossHandler::localRefCallback, this, std::placeholders::_1));

  // Timers
  max_velocity_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1 / params_.max_velocity_publish_frequency),
                              std::bind(&AutocrossHandler::publishMaxVelocity, this));
}

AutocrossHandler::~AutocrossHandler() {}

void AutocrossHandler::localRefCallback(const control_msgs::msg::ControllerRef::SharedPtr msg) {
  controller_ref_pub_->publish(*msg);
}

void AutocrossHandler::lapCountCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }
  if (msg->data < params_.num_laps) {
    return;
  }
  autonomous_msgs::msg::BoolStamped brake_controller_msg;
  brake_controller_msg.header.stamp = this->now();
  brake_controller_msg.data = true;
  brake_controller_pub_->publish(brake_controller_msg);
  mission_finished_detected_ = true;
}

void AutocrossHandler::velocityEstimationCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg) {
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }
  if (!mission_finished_detected_) {
    // publishMissionFinished(false);
    return;
  }
  if (std::hypot(msg->vel.x, msg->vel.y) > params_.mission_finished_velocity_threshold) {
    // publishMissionFinished(false);
    return;
  }
  publishMissionFinished(true);
}

void AutocrossHandler::publishMaxVelocity() {
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }
  autonomous_msgs::msg::DoubleStamped max_velocity_msg;
  max_velocity_msg.header.stamp = this->get_clock()->now();
  max_velocity_msg.data = params_.max_velocity;
  max_velocity_pub_->publish(max_velocity_msg);
}

void AutocrossHandler::publishMissionFinished(bool finished) {
  autonomous_msgs::msg::BoolStamped mission_finished_msg;
  mission_finished_msg.header.stamp = this->now();
  mission_finished_msg.data = finished;
  mission_finished_pub_->publish(mission_finished_msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<AutocrossHandler> autocross_handler = std::make_shared<AutocrossHandler>("autocross_handler_node");
  rclcpp::spin(autocross_handler);
  rclcpp::shutdown();
  return 0;
}
