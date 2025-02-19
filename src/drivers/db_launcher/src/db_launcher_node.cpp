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

#include "db_launcher/db_launcher_node.hpp"
#include <cstdlib>

DBLauncher::DBLauncher(char **envp) : Node("db_launcher"), current_mission_(""), current_pid_(-1), envp_(envp) {
  mission_select_sub_ = this->create_subscription<vcu_msgs::msg::MissionSelect>(
      "/vcu_msgs/mission_select", 10, std::bind(&DBLauncher::missionSelectCallback, this, std::placeholders::_1));

  mission_finished_sub_ = this->create_subscription<autonomous_msgs::msg::BoolStamped>(
      "/vcu_msgs/mission_finished", 10, std::bind(&DBLauncher::missionFinishedCallback, this, std::placeholders::_1));
}

void DBLauncher::missionSelectCallback(const vcu_msgs::msg::MissionSelect::SharedPtr msg) {
  if (msg->mission_name != current_mission_) {
    killMission();
    current_mission_ = msg->mission_name;

    if (current_mission_ == "inspection") {
      launchMission("ros2 launch launcher/nodes_launcher_inspection.launch.py");
    } else if (current_mission_ == "acceleration") {
      launchMission("ros2 launch launcher/nodes_launcher_acceleration.launch.py perception_mode:=lidar_only "
                    "lidar_mode:=hesai controller_mode:=pure_pursuit car_mode:=dufour");
    } else if (current_mission_ == "skidpad") {
      launchMission("ros2 launch launcher/nodes_launcher_skidpad.launch.py perception_mode:=lidar_only "
                    "lidar_mode:=hesai controller_mode:=pure_pursuit car_mode:=dufour");
    } else if (current_mission_ == "autocross") {
      launchMission("ros2 launch launcher/nodes_launcher_autocross.launch.py perception_mode:=sensor_fusion "
                    "lidar_mode:=hesai controller_mode:=mpc_safe car_mode:=dufour");
    } else if (current_mission_ == "trackdrive") {
      launchMission("ros2 launch launcher/nodes_launcher_trackdrive.launch.py perception_mode:=sensor_fusion "
                    "lidar_mode:=hesai controller_mode:=mpc_safe car_mode:=dufour");
    } else if (current_mission_ == "ebs_test") {
      launchMission("ros2 launch launcher/nodes_launcher_acceleration.launch.py perception_mode:=lidar_only "
                    "lidar_mode:=hesai controller_mode:=pure_pursuit car_mode:=dufour");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown mission: %s", current_mission_.c_str());
    }
  }
}

void DBLauncher::missionFinishedCallback(const autonomous_msgs::msg::BoolStamped::SharedPtr msg) {
  if (msg->data) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    killMission();
  }
}

void DBLauncher::launchMission(const std::string &command) {
  current_pid_ = fork();
  if (current_pid_ == 0) {
    // In child process
    execvpe("ros2", split(command), envp_);
    RCLCPP_ERROR(this->get_logger(), "Failed to launch mission: %s", command.c_str());
    _exit(EXIT_FAILURE); // If execl fails
  }
  RCLCPP_INFO(this->get_logger(), "Launching mission: %s with PID: %d", command.c_str(), current_pid_);
}

void DBLauncher::killMission() {
  RCLCPP_INFO(this->get_logger(), "Killing mission with PID: %d", current_pid_);
  if (current_pid_ > 0) {
    if (kill(current_pid_, 0) == 0) {
      RCLCPP_INFO(this->get_logger(), "PID %d exists", current_pid_);
    }
    if (kill(current_pid_, SIGINT) == 0) {
      RCLCPP_INFO(this->get_logger(), "Killed PID %d", current_pid_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to kill PID %d, %d", current_pid_);
    }
    current_pid_ = -1;
  }
}

/*
 * Split a string into an array of strings using spaces as delimiters
 */
char **DBLauncher::split(const std::string &s) {
  std::vector<std::string> tokens;

  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, ' ')) {
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }

  char **argv = new char *[tokens.size() + 1];
  for (size_t i = 0; i < tokens.size(); i++) {
    argv[i] = new char[tokens[i].size() + 1];
    std::snprintf(argv[i], tokens[i].size() + 1, "%s", tokens[i].c_str());
  }
  argv[tokens.size()] = NULL;

  return argv;
}
