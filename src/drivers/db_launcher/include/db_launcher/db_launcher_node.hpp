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

#include <autonomous_msgs/msg/bool_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include <vcu_msgs/msg/mission_select.hpp>

class DBLauncher : public rclcpp::Node {
public:
  explicit DBLauncher(char **envp);

private:
  void missionSelectCallback(const vcu_msgs::msg::MissionSelect::SharedPtr msg);
  void missionFinishedCallback(const autonomous_msgs::msg::BoolStamped::SharedPtr msg);
  void launchMission(const std::string &command);
  void killMission();

  char **split(const std::string &s);

  rclcpp::Subscription<vcu_msgs::msg::MissionSelect>::SharedPtr mission_select_sub_;
  rclcpp::Subscription<autonomous_msgs::msg::BoolStamped>::SharedPtr mission_finished_sub_;

  std::string current_mission_;
  pid_t current_pid_;
  char **envp_;
};
