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

#include "skidpad_handler/skidpad_handler_node.hpp"

SkidpadHandler::SkidpadHandler(const std::string node_name)
    : Node(node_name), mission_finished_detected_(false), current_point_index_(0) {
  // Get parameters
  param_listener_ = std::make_shared<skidpad_handler_params::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // Publishers
  brake_controller_pub_ = this->create_publisher<autonomous_msgs::msg::BoolStamped>(params_.brake_controller_topic, 1);
  max_velocity_pub_ = this->create_publisher<autonomous_msgs::msg::DoubleStamped>(params_.max_velocity_topic, 1);
  mission_finished_pub_ = this->create_publisher<autonomous_msgs::msg::BoolStamped>(params_.mission_finished_topic, 1);
  reference_pub_ = this->create_publisher<control_msgs::msg::ControllerRef>(params_.reference_topic, 1);

  // Subscribers
  velocity_estimation_sub_ = this->create_subscription<vcu_msgs::msg::VelocityEstimation>(
      params_.velocity_estimation_topic, 1,
      std::bind(&SkidpadHandler::velocityEstimationCallback, this, std::placeholders::_1));

  // Timers
  timer_ = this->create_wall_timer(std::chrono::duration<double>(1 / params_.frequency),
                                   std::bind(&SkidpadHandler::iteration, this));

  // tf2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize the path, boundaries, etc...
  initialize();

  // Checks
  if (params_.velocity_profile.size() != params_.velocity_profile_indices.size()) {
    throw std::runtime_error("Velocity profile and velocity profile indices must have the same size");
  }
  if (params_.velocity_profile_indices.size() == 0) {
    throw std::runtime_error("Velocity profile indices must have at least one element");
  }
  if (params_.velocity_profile_indices[0] != 0) {
    throw std::runtime_error("First element of velocity profile indices must be 0");
  }
  if (!std::is_sorted(params_.velocity_profile_indices.begin(), params_.velocity_profile_indices.end())) {
    throw std::runtime_error("Velocity profile indices must be sorted");
  }
}

void SkidpadHandler::initialize() {
  // Load path_ with all the points in the path file
  const std::string file_path = ament_index_cpp::get_package_share_directory("skidpad_handler") + params_.path_file;
  io::CSVReader<2> in(file_path);
  in.read_header(io::ignore_extra_column, "x", "y");
  double x, y;
  while (in.read_row(x, y)) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    path_.push_back(point);
  }
  // Load next_points_ with the first few points of the path
  for (int i = 0; i < params_.num_points_published; i++) {
    next_points_.push_back(path_[i]);
  }
}

SkidpadHandler::~SkidpadHandler() {}

void SkidpadHandler::velocityEstimationCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg) {
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

void SkidpadHandler::iteration() {
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }
  // Step 1: find point in next_points_ closest to current possition
  //    Get possition of the car
  tf2::Stamped<tf2::Transform> current_position_tf = getCurrentPosition();
  geometry_msgs::msg::Point32 current_position;
  current_position.x = current_position_tf.getOrigin().x();
  current_position.y = current_position_tf.getOrigin().y();
  current_position.z = current_position_tf.getOrigin().z();
  // RCLCPP_INFO(this->get_logger(), "Current position: (%f, %f)", current_position.x, current_position.y);

  //    Find the point in next_points_ that is closest to the car
  double min_distance = std::numeric_limits<double>::max();
  //    Index of the point in next_points_ that is closest to the car
  int min_distance_index = 0;
  for (int i = 0; i < next_points_.size(); i++) {
    const double distance = std::hypot(next_points_[i].x - current_position.x, next_points_[i].y - current_position.y);
    if (distance < min_distance) {
      min_distance = distance;
      min_distance_index = i;
    }
  }

  //    Index of the point in path_ that is closest to the car
  current_point_index_ += min_distance_index;
  RCLCPP_INFO(this->get_logger(), "Current point index: %d", current_point_index_);
  //    Brake controller if the car is at the end of the path
  if (current_point_index_ > params_.index_mission_finished) {
    autonomous_msgs::msg::BoolStamped brake_controller_msg;
    brake_controller_msg.header.stamp = this->now();
    brake_controller_msg.data = true;
    brake_controller_pub_->publish(brake_controller_msg);
    mission_finished_detected_ = true;
  }
  // Step 2: update next_points_ with the next few points of the path
  next_points_.clear();
  for (int i = current_point_index_; i < current_point_index_ + params_.num_points_published && i < path_.size(); i++) {
    next_points_.push_back(path_[i]);
  }
  // Step 3: publish next_points_
  control_msgs::msg::ControllerRef reference_msg;
  reference_msg.header.frame_id = "map";
  reference_msg.header.stamp = tf2_ros::toMsg(current_position_tf.stamp_);
  for (int i = 0; i < next_points_.size(); i++) {
    control_msgs::msg::ReferenceState new_point;
    new_point.position.x = next_points_[i].x;
    new_point.position.y = next_points_[i].y;
    new_point.boundary_left = params_.lane_width_left;
    new_point.boundary_right = params_.lane_width_right;
    new_point.vx_ref = getVelocityFromIndex(current_point_index_ + i);
    reference_msg.reference_trajectory.push_back(new_point);
  }
  reference_pub_->publish(reference_msg);
}

double SkidpadHandler::getVelocityFromIndex(int index) {
  double result = 0;
  for (int i = 0; i < params_.velocity_profile_indices.size(); i++) {
    if (index > params_.velocity_profile_indices[i]) {
      if (i == params_.velocity_profile_indices.size() - 1) {
        result = params_.velocity_profile[i];
      } else {
        const float interpolation_factor =
            static_cast<float>(current_point_index_ - params_.velocity_profile_indices[i]) /
            (params_.velocity_profile_indices[i + 1] - params_.velocity_profile_indices[i]);
        result = params_.velocity_profile[i] +
                 interpolation_factor * (params_.velocity_profile[i + 1] - params_.velocity_profile[i]);
      }
    }
  }
  return result;
}

void SkidpadHandler::publishMissionFinished(bool finished) {
  autonomous_msgs::msg::BoolStamped mission_finished_msg;
  mission_finished_msg.header.stamp = this->now();
  mission_finished_msg.data = finished;
  mission_finished_pub_->publish(mission_finished_msg);
}

tf2::Stamped<tf2::Transform> SkidpadHandler::getCurrentPosition() {
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
  // Carefull, usually here you would want to use transform_base_link.stamp_
  // but in this case we want the map one as later we
  // publish the path in the map frame with this timestamp
  transform_combined.stamp_ = transform_map.stamp_;

  // Invert to get possition not transform
  transform_combined.setData(transform_combined.inverse());

  return transform_combined;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<SkidpadHandler> skidpad_handler = std::make_shared<SkidpadHandler>("skidpad_handler_node");
  rclcpp::spin(skidpad_handler);
  rclcpp::shutdown();
  return 0;
}
