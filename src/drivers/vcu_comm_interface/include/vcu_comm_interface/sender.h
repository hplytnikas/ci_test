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

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>

#include <autonomous_msgs/msg/bool_stamped.hpp>
#include <autonomous_msgs/msg/cone_array.hpp>
// #include <autonomous_msgs/msg/int_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vcu_msgs/msg/car_command.hpp>

#include "sender_msgs.h"
#include "socket.h"

namespace vcu_comm_interface {

class Sender : public rclcpp::Node {
public:
  Sender();
  ~Sender() {}

private:
  // Socket
  std::unique_ptr<Socket> send_socket_;

  // Flags and counters
  bool block_throttle_commands_;
  uint8_t lap_counter_;
  uint8_t cones_count_actual_;
  uint16_t cones_count_all_;
  uint8_t mission_id_;

  // Subscribers
  rclcpp::Subscription<autonomous_msgs::msg::ConeArray>::SharedPtr cone_count_actual_sub_;
  rclcpp::Subscription<autonomous_msgs::msg::ConeArray>::SharedPtr cone_count_all_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lap_counter_sub_;
  rclcpp::Subscription<vcu_msgs::msg::CarCommand>::SharedPtr car_command_sub_;
  rclcpp::Subscription<autonomous_msgs::msg::BoolStamped>::SharedPtr mission_finished_sub_;

  // Callback methods (used for parsing and sending messages)
  void ParseActualCountMessage(const autonomous_msgs::msg::ConeArray::SharedPtr msg);
  void ParseAllCountMessage(const autonomous_msgs::msg::ConeArray::SharedPtr msg);
  void ParseLapCountMessage(const std_msgs::msg::Int32::SharedPtr msg);
  void ParseCarCommandMessage(const vcu_msgs::msg::CarCommand::SharedPtr msg);
  void ParseMissionFinishedMessage(const autonomous_msgs::msg::BoolStamped::SharedPtr msg);

  // Helper function
  void ParseAndSendFsgDataloggerMsg();

  // Send message function
  void SendMessage(char data[], size_t data_len);
};
} // namespace vcu_comm_interface
