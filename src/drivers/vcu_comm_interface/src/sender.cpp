/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Stanislaw Piasecki <stanislaw.piasecki@inf.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "vcu_comm_interface/sender.h"

#include <boost/algorithm/clamp.hpp>

namespace vcu_comm_interface {

Sender::Sender() : rclcpp::Node("sender_node") {
  // Get the port that we send the messages from
  unsigned int sender_port = this->declare_parameter("ports.sender_port", 1044);
  if (!this->get_parameter("ports.sender_port", sender_port)) {
    RCLCPP_FATAL(get_logger(), "Did not specify CB port used for sending messages from VCU!");
    rclcpp::shutdown();
  }

  // Initialize the socket for sending messages
  send_socket_ = std::make_unique<Socket>((uint16_t)sender_port, this);

  // Initialize flags and counters
  lap_counter_ = 0;
  cones_count_actual_ = 0;
  cones_count_all_ = 0;
  block_throttle_commands_ = false;

  // Subscribe to topics
  cone_count_actual_sub_ = this->create_subscription<autonomous_msgs::msg::ConeArray>(
      "/perception/cone_array", 1, std::bind(&Sender::ParseActualCountMessage, this, std::placeholders::_1));
  cone_count_all_sub_ = this->create_subscription<autonomous_msgs::msg::ConeArray>(
      "/estimation/online_map", 1, std::bind(&Sender::ParseAllCountMessage, this, std::placeholders::_1));
  lap_counter_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/lap_count", 1, std::bind(&Sender::ParseLapCountMessage, this, std::placeholders::_1));
  car_command_sub_ = this->create_subscription<vcu_msgs::msg::CarCommand>(
      "/control/car_command", 1, std::bind(&Sender::ParseCarCommandMessage, this, std::placeholders::_1));
  mission_finished_sub_ = this->create_subscription<autonomous_msgs::msg::BoolStamped>(
      "/vcu_msgs/mission_finished", 1, std::bind(&Sender::ParseMissionFinishedMessage, this, std::placeholders::_1));
}

void Sender::ParseActualCountMessage(const autonomous_msgs::msg::ConeArray::SharedPtr msg) {
  cones_count_actual_ = (uint16_t)msg->cones.size();
  this->ParseAndSendFsgDataloggerMsg();
}

void Sender::ParseAllCountMessage(const autonomous_msgs::msg::ConeArray::SharedPtr msg) {
  cones_count_all_ = (uint16_t)msg->cones.size();
  this->ParseAndSendFsgDataloggerMsg();
}

void Sender::ParseLapCountMessage(const std_msgs::msg::Int32::SharedPtr msg) {
  lap_counter_ = (uint8_t)msg->data;
  RCLCPP_INFO(get_logger(), "Sending lap counter: %d", lap_counter_);
  this->ParseAndSendFsgDataloggerMsg();
}

void Sender::ParseCarCommandMessage(const vcu_msgs::msg::CarCommand::SharedPtr msg) {
  CarCommandMsg car_command_msg{};

  // Prevent from sending throttle if block_throttle_commands_ is true!
  if (!block_throttle_commands_) {
    for (unsigned int i = 0; i < 3; ++i) {
      // car_command_msg.steering_angle[i] = boost::algorithm::clamp((int16_t)msg->steering_angle[i] * 1e4, -0.5, 0.5);
      // car_command_msg.a_x[i] = boost::algorithm::clamp((int16_t)msg->a_x[i] * 1000, -30.0, 30.0);
      // car_command_msg.yaw_rate[i] = boost::algorithm::clamp((int16_t)msg->yaw_rate[i] * 1e4, -4.0, 4.0);
      car_command_msg.steering_angle[i] = (int16_t)(boost::algorithm::clamp(msg->steering_angle[i], -0.5, 0.5) * 1e4);
      car_command_msg.a_x[i] = (int16_t)(boost::algorithm::clamp(msg->a_x[i], -30.0, 30.0) * 1000);
      car_command_msg.yaw_rate[i] = (int16_t)(boost::algorithm::clamp(msg->yaw_rate[i], -4.0, 4.0) * 1e4);
    }
  }

  this->SendMessage(reinterpret_cast<char *>(&car_command_msg), sizeof(CarCommandMsg));
}

void Sender::ParseMissionFinishedMessage(const autonomous_msgs::msg::BoolStamped::SharedPtr msg) {
  MissionFinishedMsg mission_finished_msg;
  if (msg->data) {
    block_throttle_commands_ = true;
    RCLCPP_WARN(get_logger(), "Sending zero throttle!");
  }
  mission_finished_msg.mission_finished = msg->data;

  this->SendMessage(reinterpret_cast<char *>(&mission_finished_msg), sizeof(MissionFinishedMsg));
}

void Sender::ParseAndSendFsgDataloggerMsg() {
  FsgDataloggerMsg fsg_datalogger_msg;
  fsg_datalogger_msg.lap_counter = lap_counter_;
  fsg_datalogger_msg.cones_count_actual = cones_count_actual_;
  fsg_datalogger_msg.cones_count_all = cones_count_all_;

  this->SendMessage(reinterpret_cast<char *>(&fsg_datalogger_msg), sizeof(FsgDataloggerMsg));
}

void Sender::SendMessage(char data[], size_t data_len) {
  bool success = send_socket_->SendMsg((const char *)data, data_len);
  if (!success) {
    RCLCPP_WARN(get_logger(), "Failed to send UDP message with ID: %d", (unsigned int)data[0]);
  }
}

} // namespace vcu_comm_interface
