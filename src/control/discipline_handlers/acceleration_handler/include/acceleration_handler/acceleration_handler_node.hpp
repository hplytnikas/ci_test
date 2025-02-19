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

#include <acceleration_handler_params.hpp>
#include <autonomous_msgs/msg/bool_stamped.hpp> // control/brake_controller, vcu_msgs/mission_finished
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>             // distance_traveled
#include <std_msgs/msg/int32.hpp>               // lap_count
#include <vcu_msgs/msg/velocity_estimation.hpp> // vcu_msgs/velocity_estimation

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/utils.h> //tf2::doTransform
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

/**
 * @brief Class that handles the controller for acceleration.
 *
 * It can detect the end of the trac either by the itnegration of velocity estimation, or by the possition given by
 * slam. This can be choosen with the parameter distance_measurment_method /control/brake_controller when the lap count
 * is reached. Then when /vcu_msgs/velocity_estimation is low enough, it publishes /vcu_msgs/mission_finished.
 *
 */

class AccelerationHandler : public rclcpp::Node {
public:
  explicit AccelerationHandler(const std::string node_name);
  ~AccelerationHandler();

private:
  // Publishers
  rclcpp::Publisher<autonomous_msgs::msg::BoolStamped>::SharedPtr brake_controller_pub_;
  rclcpp::Publisher<autonomous_msgs::msg::BoolStamped>::SharedPtr mission_finished_pub_;

  // Subscribers
  rclcpp::Subscription<vcu_msgs::msg::VelocityEstimation>::SharedPtr velocity_estimation_sub_;
  void velocityEstimationCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg);

  // Params
  std::shared_ptr<acceleration_handler_params::ParamListener> param_listener_;
  acceleration_handler_params::Params params_;

  rclcpp::TimerBase::SharedPtr check_distance_timer_;
  void checkDistance();

  // tf2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // variables
  bool mission_finished_detected_;

  // Other
  void publishMissionFinished(bool finished);
  void publishBrakeController();
  double getYawFromQuaternion(const tf2::Quaternion &tf2_quat);
  tf2::Stamped<tf2::Transform> getCurrentPosition();
};
