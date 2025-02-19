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

#include "amzsim_msgs/msg/config_info.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>

using std::string;

class ConfigInfoPub {
public:
  explicit ConfigInfoPub(rclcpp::Node::SharedPtr nh);
  virtual ~ConfigInfoPub() = default;
  void publishConfigInfo();

private:
  // publish config info message every second
  double frequency = 1.0;
  rclcpp::Rate rate_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<amzsim_msgs::msg::ConfigInfo>::SharedPtr pub_info_;
};
