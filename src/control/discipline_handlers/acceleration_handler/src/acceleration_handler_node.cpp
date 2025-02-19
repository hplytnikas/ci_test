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

#include "acceleration_handler/acceleration_handler_node.hpp"

AccelerationHandler::AccelerationHandler(const std::string node_name)
    : Node(node_name), mission_finished_detected_(false) {
  // Get parameters
  param_listener_ = std::make_shared<acceleration_handler_params::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // Publishers
  brake_controller_pub_ = this->create_publisher<autonomous_msgs::msg::BoolStamped>(params_.brake_controller_topic, 10);
  mission_finished_pub_ = this->create_publisher<autonomous_msgs::msg::BoolStamped>(params_.mission_finished_topic, 10);

  // Subscribers
  velocity_estimation_sub_ = this->create_subscription<vcu_msgs::msg::VelocityEstimation>(
      params_.velocity_estimation_topic, 1,
      std::bind(&AccelerationHandler::velocityEstimationCallback, this, std::placeholders::_1));

  // Timers
  check_distance_timer_ = this->create_wall_timer(std::chrono::duration<double>(1 / params_.check_distance_frequency),
                                                  std::bind(&AccelerationHandler::checkDistance, this));

  // tf2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

AccelerationHandler::~AccelerationHandler() {}

double AccelerationHandler::getYawFromQuaternion(const tf2::Quaternion &tf2_quat) {
  tf2::Matrix3x3 m(tf2_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

void AccelerationHandler::publishBrakeController() {
  autonomous_msgs::msg::BoolStamped brake_controller_msg;
  brake_controller_msg.header.stamp = this->now();
  brake_controller_msg.data = true;
  brake_controller_pub_->publish(brake_controller_msg);
}

void AccelerationHandler::checkDistance() {
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  tf2::Stamped<tf2::Transform> current_position_tf = getCurrentPosition();

  geometry_msgs::msg::Point32 current_position;
  current_position.x = current_position_tf.getOrigin().x();
  current_position.y = current_position_tf.getOrigin().y();
  const double current_yaw = getYawFromQuaternion(current_position_tf.getRotation());

  // Check if the current position is within the distance threshold
  if (std::hypot(current_position.x, current_position.y) > params_.distance_threshold) {
    publishBrakeController(); // Brake as the finish line is reached
    mission_finished_detected_ = true;
    return;
  }

  // Check if the lateral deviation (y axis) is greater than 6 meters (left or right)
  if (std::abs(current_position.y) > params_.lateral_deviation_threshold_emergency) {
    RCLCPP_FATAL(this->get_logger(), "Lateral deviation is too high (%f). Emergency braking.", current_position.y);
    publishBrakeController(); // Emergency braking due to lateral deviation
    return;
  }

  // Check if the current yaw is within the yaw threshold
  if (std::abs(current_yaw) > params_.yaw_threshold_emergency) {
    RCLCPP_FATAL(this->get_logger(), "Yaw deviation is too high (%f). Emergency braking.", current_yaw);
    publishBrakeController(); // Emergency braking due to yaw deviation
    return;
  }

  return;
}

tf2::Stamped<tf2::Transform> AccelerationHandler::getCurrentPosition() {
  geometry_msgs::msg::TransformStamped transform_map_msg, transform_base_link_msg;
  tf2::Stamped<tf2::Transform> transform_map, transform_base_link, transform_combined;

  // Fetch the initial transform from map to base_link
  try {
    transform_map_msg = tf_buffer_->lookupTransform("base_link", tf2::TimePointZero, "map", tf2::TimePointZero, "map");
    tf2::fromMsg(transform_map_msg, transform_map);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform lookup (map to base_link) failed: %s", ex.what());
    // throw std::runtime_error("Transform lookup (map to base_link) failed");
  }

  // Fetch the latest transform from base_link at the time of transform_map to the current base_link
  try {
    transform_base_link_msg =
        tf_buffer_->lookupTransform("base_link", tf2::TimePointZero, "base_link", transform_map.stamp_, "odom");
    tf2::fromMsg(transform_base_link_msg, transform_base_link);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform lookup (base_link to base_link) failed: %s", ex.what());
    // throw std::runtime_error("Transform lookup (base_link to base_link) failed");
  }

  // Combine the two transforms to get an accurate map to base_link transform
  transform_combined.setData(transform_base_link * transform_map);
  // transform_combined.stamp_ = ...

  // Invert to get possition not transform
  transform_combined.setData(transform_combined.inverse());

  return transform_combined;
}

void AccelerationHandler::velocityEstimationCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg) {
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }
  if (!mission_finished_detected_) {
    // publishMissionFinished(false);
    return;
  }
  if (std::hypot(msg->vel.x, msg->vel.y) > params_.mission_finished_velocity_threshold) {
    // publishMissionFinished(false);
    return;
  }
  publishMissionFinished(true);
}

void AccelerationHandler::publishMissionFinished(bool finished) {
  autonomous_msgs::msg::BoolStamped mission_finished_msg;
  mission_finished_msg.header.stamp = this->now();
  mission_finished_msg.data = finished;
  mission_finished_pub_->publish(mission_finished_msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<AccelerationHandler> acceleration_handler =
      std::make_shared<AccelerationHandler>("acceleration_handler_node");
  rclcpp::spin(acceleration_handler);
  rclcpp::shutdown();
  return 0;
}
