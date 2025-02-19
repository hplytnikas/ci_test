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

#include "controller_node/controller_node.hpp"

ControllerNode::ControllerNode(const std::string node_name) : Node(node_name) {
  // PARAMETERS
  // Topic names
  this->declare_parameter("velocity_estimation_topic", "/vcu_msgs/velocity_estimation");
  this->declare_parameter("steering_topic", "/vcu_msgs/steering_feedback");
  this->declare_parameter("reference_topic", "/planning/reference");
  this->declare_parameter("car_command_topic", "/control/car_command");
  this->declare_parameter("max_velocity_topic", "/control/max_velocity");
  this->declare_parameter("brake_controller_topic", "/control/brake_controller");
  this->declare_parameter("res_state_topic", "/vcu_msgs/res_state");
  // Others
  this->declare_parameter("controller_frequency", 40.0);
  this->declare_parameter("controller_timeout_reset", 100);
  this->declare_parameter("controller_timeout_fatal", 400);
  this->declare_parameter("braking_acceleration", 10.0);
  this->declare_parameter("profiling_enabled", false);

  // Load parameters
  velocity_estimation_topic_ = this->get_parameter("velocity_estimation_topic").as_string();
  steering_topic_ = this->get_parameter("steering_topic").as_string();
  reference_topic_ = this->get_parameter("reference_topic").as_string();
  car_command_topic_ = this->get_parameter("car_command_topic").as_string();
  max_velocity_topic_ = this->get_parameter("max_velocity_topic").as_string();
  brake_controller_topic_ = this->get_parameter("brake_controller_topic").as_string();
  res_state_topic_ = this->get_parameter("res_state_topic").as_string();
  controller_frequency_ = this->get_parameter("controller_frequency").as_double();
  braking_acceleration_ = this->get_parameter("braking_acceleration").as_double();

  controller_timeout_reset_ = this->get_parameter("controller_timeout_reset").as_int();
  controller_timeout_fatal_ = this->get_parameter("controller_timeout_fatal").as_int();

  if (controller_frequency_ <= 0) {
    throw std::runtime_error("Controller frequency must be positive");
  }

  // SUBSCRIBERS
  velocity_estimation_subscriber_ = this->create_subscription<vcu_msgs::msg::VelocityEstimation>(
      velocity_estimation_topic_, 1,
      std::bind(&ControllerNode::CallbackVelocityEstimation, this, std::placeholders::_1));
  steering_subscriber_ = this->create_subscription<autonomous_msgs::msg::DoubleStamped>(
      steering_topic_, 1, std::bind(&ControllerNode::CallbackSteering, this, std::placeholders::_1));

  reference_subscriber_ = this->create_subscription<control_msgs::msg::ControllerRef>(
      reference_topic_, 1, std::bind(&ControllerNode::CallbackReference, this, std::placeholders::_1));

  max_velocity_subscriber_ = this->create_subscription<autonomous_msgs::msg::DoubleStamped>(
      max_velocity_topic_, 1, std::bind(&ControllerNode::CallbackMaxVelocity, this, std::placeholders::_1));
  brake_controller_subscriber_ = this->create_subscription<autonomous_msgs::msg::BoolStamped>(
      brake_controller_topic_, 1, std::bind(&ControllerNode::CallbackBrakeController, this, std::placeholders::_1));
  res_state_subscriber_ = this->create_subscription<vcu_msgs::msg::ResState>(
      res_state_topic_, 1, std::bind(&ControllerNode::CallbackResState, this, std::placeholders::_1));

  // PUBLISHERS
  car_command_publisher_ = this->create_publisher<vcu_msgs::msg::CarCommand>(car_command_topic_, 1);

  // TF2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // TIMER
  timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / controller_frequency_),
                                   std::bind(&ControllerNode::RunControllerStep, this));

  // Initialize watchdog
  controller_step_failures_ = 0;

  // Other
  has_res_go_been_pressed_ = false;
}

double ControllerNode::GetMaxVelocity() {
  EASY_FUNCTION();

  if (!latest_max_velocity_msg_received_) {
    throw ControllerNode::ControllerStepException("Executed GetMaxVelocity without max velocity message");
  }
  return latest_max_velocity_msg_.data;
}

const ControllerNode::State ControllerNode::GetState() {
  EASY_FUNCTION(profiler::colors::Green);
  if (!latest_velocity_estimation_msg_received_) {
    throw ControllerNode::ControllerStepException("Executed GetState without velocity estimation message");
  }
  if (!latest_steering_msg_received_) {
    throw ControllerNode::ControllerStepException("Executed GetState without steering message");
  }
  // state in vehicle frame
  ControllerNode::State state;
  state.dx = latest_velocity_estimation_msg_.vel.x;
  state.dy = latest_velocity_estimation_msg_.vel.y;
  state.dpsi = latest_velocity_estimation_msg_.vel.theta;
  state.ddx = latest_velocity_estimation_msg_.acc.x;
  state.ddy = latest_velocity_estimation_msg_.acc.y;
  state.delta_s = latest_steering_msg_.data;

  return state;
}

const ControllerNode::Reference &ControllerNode::GetReference() {
  EASY_FUNCTION(profiler::colors::Red);

  if (!latest_reference_msg_received_) {
    throw ControllerNode::ControllerStepException("Executed getTrajectory without reference message");
  }
  // Retrieve the reference frame and time
  tf2::TimePoint reference_time = tf2_ros::fromMsg(latest_reference_msg_.header.stamp);
  const std::string reference_frame = latest_reference_msg_.header.frame_id;

  // If the reference is in the map frame, we need to transform it to the vehicle frame
  // Otherwise transform_map is the identity
  geometry_msgs::msg::TransformStamped transform_map_msg;
  tf2::Stamped<tf2::Transform> transform_map;
  if (reference_frame == "map") {
    try {
      EASY_BLOCK("lookupTransform");
      transform_map_msg =
          tf_buffer_->lookupTransform("base_link", tf2::TimePointZero, "map", tf2::TimePointZero, "map");
      tf2::fromMsg(transform_map_msg, transform_map);
      // reference_time is updated so when we look up the base_link transform we get the correct one
      reference_time = tf2_ros::fromMsg(transform_map_msg.header.stamp);
      EASY_END_BLOCK;
    } catch (tf2::TransformException &ex) {
      throw ControllerNode::ControllerStepException("Could not transform reference to vehicle frame: " +
                                                    std::string(ex.what()));
    }
  } else {
    transform_map.setIdentity();
  }

  // Transform from base_link at reference_time to latest base_link
  geometry_msgs::msg::TransformStamped transform_base_link_msg;
  tf2::Stamped<tf2::Transform> transform_base_link;
  try {
    EASY_BLOCK("lookupTransform");
    transform_base_link_msg =
        tf_buffer_->lookupTransform("base_link", tf2::TimePointZero, "base_link", reference_time, "odom");
    tf2::fromMsg(transform_base_link_msg, transform_base_link);
    EASY_END_BLOCK;
  } catch (tf2::TransformException &ex) {
    throw ControllerNode::ControllerStepException("Could not transform reference to vehicle frame: " +
                                                  std::string(ex.what()));
  }

  // Combine the two transforms: map to base_link and base_link to latest base_link
  tf2::Stamped<tf2::Transform> transform_combined;
  transform_combined.setData(transform_base_link * transform_map);
  geometry_msgs::msg::TransformStamped transform_combined_msg;
  tf2::toMsg(transform_combined, transform_combined_msg.transform);

  // Transform the reference to latest base_link
  EASY_BLOCK("transformReference");
  updated_reference_.resize(0);
  updated_reference_.resize(latest_reference_msg_.reference_trajectory.size());

  for (int i = 0; i < latest_reference_msg_.reference_trajectory.size(); i++) {
    const auto &reference_state = latest_reference_msg_.reference_trajectory[i];
    geometry_msgs::msg::Point32 new_point;
    tf2::doTransform(reference_state.position, new_point, transform_combined_msg);
    updated_reference_[i].x = new_point.x;
    updated_reference_[i].y = new_point.y;
    updated_reference_[i].bound_lateral_left = reference_state.boundary_left;
    updated_reference_[i].bound_lateral_right = reference_state.boundary_right;
    updated_reference_[i].velocity = reference_state.vx_ref;
  }
  EASY_END_BLOCK;
  return updated_reference_;
}

void ControllerNode::CallbackVelocityEstimation(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg) {
  EASY_FUNCTION();
  latest_velocity_estimation_msg_ = *msg;
  latest_velocity_estimation_msg_received_ = true;
}

void ControllerNode::CallbackSteering(const autonomous_msgs::msg::DoubleStamped::SharedPtr msg) {
  EASY_FUNCTION();
  latest_steering_msg_ = *msg;
  latest_steering_msg_received_ = true;
}

void ControllerNode::CallbackReference(const control_msgs::msg::ControllerRef::SharedPtr msg) {
  latest_reference_msg_ = *msg;
  latest_reference_msg_received_ = true;
}

void ControllerNode::CallbackMaxVelocity(const autonomous_msgs::msg::DoubleStamped::SharedPtr msg) {
  EASY_FUNCTION();
  latest_max_velocity_msg_ = *msg;
  latest_max_velocity_msg_received_ = true;
}

void ControllerNode::CallbackBrakeController(const autonomous_msgs::msg::BoolStamped::SharedPtr msg) {
  EASY_FUNCTION();
  if (msg->data) controller_braking_ = true;
}

void ControllerNode::CallbackResState(const vcu_msgs::msg::ResState::SharedPtr msg) {
  if (msg->push_button && !has_res_go_been_pressed_) {
    LogInfo("Res go has been pressed, starting controller.");
    has_res_go_been_pressed_ = true;
  }
}

void ControllerNode::RunControllerStep() {
  EASY_FUNCTION();

  // car_command_publisher_->publish(updateControlCommand());
  // This function does this but with error handling
  if (!has_res_go_been_pressed_) {
    return;
  }
  vcu_msgs::msg::CarCommand command;
  try {
    // Use the update function, which is defined in a controller
    command = update_function_();
    // However, if the controller is braking, we should brake. This may be due to a timeout or a critical error
    if (controller_braking_) {
      command.a_x[0] = command.a_x[1] = command.a_x[2] = -braking_acceleration_;
    }
    car_command_publisher_->publish(command);
    controller_step_failures_ = 0;
  } catch (const ControllerNode::ControllerStepException &ex) {
    controller_step_failures_++;
    RCLCPP_WARN(this->get_logger(), "%s\n", ex.what());
    if (controller_step_failures_ > controller_timeout_fatal_) {
      RCLCPP_FATAL(this->get_logger(), "Controller has failed for longer than controller_timeout_fatal_");
      // Maybe there is a way to do an emergency brake here
      controller_braking_ = true;
    } else if (controller_step_failures_ > controller_timeout_reset_) {
      RCLCPP_ERROR(this->get_logger(), "Controller has failed for longer than "
                                       "controller_timeout_reset_. Resetting controller.");
      reset_function_();
    }
  }
}

void ControllerNode::SetUpdateFunction(std::function<vcu_msgs::msg::CarCommand()> f) { update_function_ = f; }

void ControllerNode::SetResetFunction(std::function<void()> f) { reset_function_ = f; }
