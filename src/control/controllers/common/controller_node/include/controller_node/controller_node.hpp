/*******************************************************************************
 * AMZ Driverless Project                                                      *
 * Copyright (c) 2023-2024                                                     *
 * Authors:                                                                    *
 *   - Diego Garcia Soto <digarcia@ethz.ch>                                    *
 *   - Hironobu Akiyama <hakiyama@ethz.ch>                                     *
 *   - Jonas Ohnemus <johnemus@ethz.ch>                                        *
 *                                                                             *
 * All rights reserved.                                                        *
 *                                                                             *
 * Unauthorized copying of this file, via any medium, is strictly prohibited.  *
 * Proprietary and confidential.                                               *
 ******************************************************************************/

#pragma once

#include <Eigen/Dense>
#include <easy/arbitrary_value.h>
#include <easy/profiler.h>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h" //tf2::doTransform
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "autonomous_msgs/msg/bool_stamped.hpp"
#include "autonomous_msgs/msg/double_stamped.hpp"
#include "control_msgs/msg/controller_ref.hpp"
#include "control_msgs/msg/reference_state.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "vcu_msgs/msg/car_command.hpp"
#include "vcu_msgs/msg/res_state.hpp"
#include "vcu_msgs/msg/velocity_estimation.hpp"

/*
 * ControllerNode class
 * This class is an abstract class that inherits from rclcpp::Node
 *
 * Parameters:
 * - velocity_estimation_topic
 * - steering_topic
 * - reference_topic
 * - max_velocity_topic
 * - car_command_topic
 * - brake_controller_topic
 * - res_state_topic
 * - controller_frequency
 * - controller_timeout_reset
 * - controller_timeout_fatal
 * - braking_acceleration
 * - has_res_go_been_pressed

 *
 * Subscribes to:
 * - velocity_estimation_topic:   /vcu_msgs/velocity_estimation
 * - steering_topic:              /vcu_msgs/steering_feedback
 * - reference_topic:             /planning/reference
 * - max_velocity_topic:          /control/max_velocity
 * - brake_controller_topic:      /control/brake_controller
 * - res_state_topic:             /vcu_msgs/res_state
 *
 * Publishes to:
 * - car_command_topic:           /control/car_command
 */

class ControllerNode : public rclcpp::Node {
public:
  // State Struct needed for all feedback controllers
  struct State {
    double dx{0.0};      // x velocity
    double dy{0.0};      // y velocity
    double dpsi{0.0};    // yaw rate
    double ddx{0.0};     // x acceleration
    double ddy{0.0};     // y acceleration
    double delta_s{0.0}; // steering angle
  };

  struct ReferenceState {
    double x{0.0};
    double y{0.0};
    double bound_lateral_left{0.0};
    double bound_lateral_right{0.0};
    double velocity{0.0};
  };

  typedef Eigen::Matrix<ReferenceState, Eigen::Dynamic, 1> Reference;

  ~ControllerNode() {}
  explicit ControllerNode(const std::string node_name);

  // Velocity is from velocity estimation
  // topic and steering angle from the steering angle topic
  const State GetState();

  // Path and boundaries from the target trajectory topic
  const Reference &GetReference();

  // Max velocity from the max velocity topic
  double GetMaxVelocity();

  // Returns the current time in seconds
  double GetTime() { return this->get_clock()->now().seconds(); }

  // Function to set the update function of the controller
  // accepts a function that returns a CarCommand
  void SetUpdateFunction(std::function<vcu_msgs::msg::CarCommand()> f);

  // Function to set the reset function of the controller
  // accepts a function that returns void
  void SetResetFunction(std::function<void()> f);

  double GetControllerFrequency() { return controller_frequency_; }

  // These are defined so the controller can log without having to get the
  // logger or the node
  template <typename... Args> void LogDebug(const char *format, Args... args) {
    RCLCPP_DEBUG(this->get_logger(), format, args...);
  }

  template <typename... Args> void LogInfo(const char *format, Args... args) {
    RCLCPP_INFO(this->get_logger(), format, args...);
  }

  template <typename... Args> void LogWarn(const char *format, Args... args) {
    RCLCPP_WARN(this->get_logger(), format, args...);
  }

  template <typename... Args> void LogError(const char *format, Args... args) {
    RCLCPP_ERROR(this->get_logger(), format, args...);
  }

  template <typename... Args> void LogFatal(const char *format, Args... args) {
    RCLCPP_FATAL(this->get_logger(), format, args...);
  }

  // This exception is thrown when the controller step fails

  // The node will catch this exception and not publish the car command
  // The error policy is that if there the controller has failed for longer than
  // controller_timeout_reset_ the controller will be reset and
  // controller_timeout_fatal_ the controller will be stopped
  class ControllerStepException : public std::runtime_error {
  public:
    explicit ControllerStepException(const std::string &what_arg) : std::runtime_error(what_arg) {}
  };

private:
  // Topic names
  std::string velocity_estimation_topic_;
  std::string steering_topic_;
  std::string reference_topic_;
  std::string max_velocity_topic_;
  std::string car_command_topic_;
  std::string brake_controller_topic_;
  std::string res_state_topic_;

  // Controller reference
  Reference reference_;

  // Others
  double controller_frequency_;
  unsigned int controller_timeout_reset_;
  unsigned int controller_timeout_fatal_;
  double braking_acceleration_;
  bool has_res_go_been_pressed_;

  // PUBLISHERS
  rclcpp::Publisher<vcu_msgs::msg::CarCommand>::SharedPtr car_command_publisher_;

  // SUBSCRIBERS
  rclcpp::Subscription<vcu_msgs::msg::VelocityEstimation>::SharedPtr velocity_estimation_subscriber_;
  rclcpp::Subscription<autonomous_msgs::msg::DoubleStamped>::SharedPtr steering_subscriber_;
  rclcpp::Subscription<control_msgs::msg::ControllerRef>::SharedPtr reference_subscriber_;
  rclcpp::Subscription<autonomous_msgs::msg::DoubleStamped>::SharedPtr max_velocity_subscriber_;
  rclcpp::Subscription<autonomous_msgs::msg::BoolStamped>::SharedPtr brake_controller_subscriber_;
  rclcpp::Subscription<vcu_msgs::msg::ResState>::SharedPtr res_state_subscriber_;

  // Callback functions
  void CallbackVelocityEstimation(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg);
  void CallbackSteering(const autonomous_msgs::msg::DoubleStamped::SharedPtr msg);
  void CallbackReference(const control_msgs::msg::ControllerRef::SharedPtr msg);
  void CallbackMaxVelocity(const autonomous_msgs::msg::DoubleStamped::SharedPtr msg);
  void CallbackBrakeController(const autonomous_msgs::msg::BoolStamped::SharedPtr msg);
  void CallbackResState(const vcu_msgs::msg::ResState::SharedPtr msg);

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // TIMERS
  // Timer to run the controller step at specified frequency
  rclcpp::TimerBase::SharedPtr timer_;

  // Controller step called by the timer at the specified frequency
  void RunControllerStep();

  // LAST MESSAGES BUFFER
  vcu_msgs::msg::VelocityEstimation latest_velocity_estimation_msg_;
  autonomous_msgs::msg::DoubleStamped latest_steering_msg_;
  control_msgs::msg::ControllerRef latest_reference_msg_;
  Reference updated_reference_;
  autonomous_msgs::msg::DoubleStamped latest_max_velocity_msg_;
  bool latest_velocity_estimation_msg_received_ = false;
  bool latest_steering_msg_received_ = false;
  bool latest_reference_msg_received_ = false;
  bool latest_max_velocity_msg_received_ = false;

  // Update function
  // Given by the controller with setUpdateFunction
  std::function<vcu_msgs::msg::CarCommand()> update_function_;

  // Reset function
  // Given by the controller with setResetFunction
  std::function<void()> reset_function_;

  // Number of consectutive controller step failures
  // Used to check if the controller has failed for too long
  unsigned int controller_step_failures_ = 0;

  // Has the controller been braked / commanded to stop
  bool controller_braking_ = false;
};
