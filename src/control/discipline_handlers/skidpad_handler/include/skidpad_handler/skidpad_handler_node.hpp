
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
#include <autonomous_msgs/msg/double_stamped.hpp>
#include <control_msgs/msg/controller_ref.hpp>
#include <control_msgs/msg/reference_state.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <skidpad_handler_params.hpp>
#include <vcu_msgs/msg/velocity_estimation.hpp> // vcu_msgs/velocity_estimation

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/utils.h> //tf2::doTransform
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

// CSV parser from https://github.com/ben-strasser/fast-cpp-csv-parser
#define CSV_IO_NO_THREAD
#include "csv.h"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

/*
 * SkidpadHandler Node
 *
 * This node is responsible for handling the skidpad mission.
 * It reads the path from a file, and publishes the next few points of the path, the maximum velocity for the controller
 * given some velocity profile in the config.
 *
 * TODO(digarcia): It should publish a boundary with the path, but it is not implemented yet.
 * This is not important for pure pursuit, but it will be important for MPC.
 * The plan is to run pure pursuit, but the idea is to make this controller independent.
 */

class SkidpadHandler : public rclcpp::Node {
public:
  explicit SkidpadHandler(const std::string node_name);
  ~SkidpadHandler();

private:
  // Publishers
  rclcpp::Publisher<autonomous_msgs::msg::BoolStamped>::SharedPtr brake_controller_pub_;
  rclcpp::Publisher<autonomous_msgs::msg::DoubleStamped>::SharedPtr max_velocity_pub_;
  rclcpp::Publisher<autonomous_msgs::msg::BoolStamped>::SharedPtr mission_finished_pub_;
  rclcpp::Publisher<control_msgs::msg::ControllerRef>::SharedPtr reference_pub_;

  // Subscribers
  rclcpp::Subscription<vcu_msgs::msg::VelocityEstimation>::SharedPtr velocity_estimation_sub_;

  void velocityEstimationCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg);

  // Params
  std::shared_ptr<skidpad_handler_params::ParamListener> param_listener_;
  skidpad_handler_params::Params params_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  void iteration();

  // variables
  bool mission_finished_detected_;

  // Inizializes the path, boundaries, etc... at the Constructor
  void initialize();

  // tf2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // To store the path
  std::vector<geometry_msgs::msg::Point32> path_;
  // To store the next few points of the path to publish
  std::vector<geometry_msgs::msg::Point32> next_points_;
  // To store the index of the point where the car is
  int current_point_index_;

  // Other
  void publishMissionFinished(bool finished);
  tf2::Stamped<tf2::Transform> getCurrentPosition();
  double getVelocityFromIndex(int index);
};
