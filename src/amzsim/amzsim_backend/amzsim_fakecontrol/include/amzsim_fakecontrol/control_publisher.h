/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Romir Damle <rdamle@ethz.ch>
 *   - Emil Fahrig <efahrig@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include "amzsim_msgs/msg/res_state.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vcu_msgs/msg/car_command.hpp"
#include <memory>

// using namespace std::chrono_literals;

class ControlPublisher : public rclcpp::Node {
public:
  ControlPublisher();
  virtual ~ControlPublisher() = default;

private:
  void publishControl();

  rclcpp::Publisher<amzsim_msgs::msg::ResState>::SharedPtr publisher_res_state_;
  rclcpp::Publisher<vcu_msgs::msg::CarCommand>::SharedPtr publisher_car_command_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  rclcpp::Clock clock;
};
