/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023 - 2024  Authors:
 *   - Lorenzo Codeluppi <lcodeluppi@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#include "node_handle.hpp"

namespace boundary_estimation {

NodeHandle::NodeHandle(const std::string &node_name, const std::string &node_namespace,
                       const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, node_namespace, options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {}

void NodeHandle::CreateOnlineMapSubscriber(const std::string &topic_name, size_t queue_size,
                                           std::function<void(const autonomous_msgs::msg::ConeArray)> callback) {
  this->online_map_subscriber_ =
      this->create_subscription<autonomous_msgs::msg::ConeArray>(topic_name, queue_size, callback);
}
void NodeHandle::CreateLapCounterSubscriber(const std::string &topic_name, size_t queue_size,
                                            std::function<void(const std_msgs::msg::Int32)> callback) {
  this->lap_counter_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(topic_name, queue_size, callback);
}

void NodeHandle::CreateGlobalMapFeedbackSubscriber(const std::string &topic_name, size_t queue_size,
                                                   std::function<void(const std_msgs::msg::Bool)> callback) {
  this->global_map_feedback_subscriber_ =
      this->create_subscription<std_msgs::msg::Bool>(topic_name, queue_size, callback);
}

void NodeHandle::CreateBoundaryPublisher(const std::string &topic_name, size_t queue_size) {
  this->boundary_publisher_ = this->create_publisher<autonomous_msgs::msg::Boundary>(topic_name, queue_size);
}

void NodeHandle::CreateVisualizerPublisher(const std::string &topic_name, size_t queue_size) {
  this->visualizer_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, queue_size);
}

void NodeHandle::CreateFinalPathPublisher(const std::string &topic_name, size_t queue_size) {
  this->final_path_publisher_ = this->create_publisher<autonomous_msgs::msg::Boundary>(topic_name, queue_size);
}

void NodeHandle::CreateFinalPathVisualPublisher(const std::string &topic_name, size_t queue_size) {
  this->final_path_visual_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, queue_size);
}

void NodeHandle::PublishBoundary(const autonomous_msgs::msg::Boundary &message) {
  if (boundary_publisher_) {
    boundary_publisher_->publish(message);
  } else {
    // Handle error, publisher not created
    RCLCPP_ERROR(this->get_logger(), "Boundary publisher not initialized");
  }
}

void NodeHandle::PublishFinalBoundary(const autonomous_msgs::msg::Boundary &message) {
  if (final_path_publisher_) {
    final_path_publisher_->publish(message);
  } else {
    // Handle error, publisher not created
    RCLCPP_ERROR(this->get_logger(), "Final boundary publisher not initialized");
  }
}

void NodeHandle::PublishFinalPathVisual(const visualization_msgs::msg::MarkerArray &message) {
  if (final_path_visual_publisher_) {
    final_path_visual_publisher_->publish(message);
  } else {
    // Handle error, publisher not created
    RCLCPP_ERROR(this->get_logger(), "Final path visual publisher not initialized");
  }
}

void NodeHandle::PublishVisualizer(const visualization_msgs::msg::MarkerArray &message) {
  if (visualizer_publisher_) {
    visualizer_publisher_->publish(message);
  } else {
    // Handle error, publisher not created
    RCLCPP_ERROR(this->get_logger(), "Visualizer publisher not initialized");
  }
}

std::optional<geometry_msgs::msg::TransformStamped>
NodeHandle::GetTransform(const std::string &source_frame, const std::string &target_frame, const tf2::TimePoint &time) {
  geometry_msgs::msg::TransformStamped transform;
  tf2::Duration timeout = tf2::durationFromSec(0.01); // 0.01 second
  try {
    transform = this->tf_buffer_.lookupTransform(target_frame, source_frame, time, timeout);
    return transform;
  } catch (const tf2::LookupException &e) {
    RCLCPP_ERROR(this->get_logger(), "Lookup transform failed: %s", e.what());
  } catch (const tf2::ConnectivityException &e) {
    RCLCPP_ERROR(this->get_logger(), "Connectivity error during transform lookup: %s", e.what());
  } catch (const tf2::ExtrapolationException &e) {
    RCLCPP_ERROR(this->get_logger(), "Extrapolation error during transform lookup: %s", e.what());
  }
  return std::nullopt;
}

std::optional<geometry_msgs::msg::TransformStamped> NodeHandle::GetTransformWithRetry(const std::string &source_frame,
                                                                                      const std::string &target_frame,
                                                                                      const tf2::TimePoint &time) {
  std::optional<geometry_msgs::msg::TransformStamped> transform = GetTransform(source_frame, target_frame, time);

  if (!transform.has_value()) {
    RCLCPP_WARN(this->get_logger(), "Initial transform lookup failed, retrying with tf2::TimePointZero");
    transform = GetTransform(source_frame, target_frame, tf2::TimePointZero);
  }

  return transform;
}

std::optional<geometry_msgs::msg::TransformStamped>
NodeHandle::GetTransformBetweenTimestamps(const std::string &source_frame, const tf2::TimePoint &source_time,
                                          const std::string &target_frame, const tf2::TimePoint &target_time,
                                          const std::string &fixed_frame) {
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = this->tf_buffer_.lookupTransform(source_frame, source_time, target_frame, target_time, fixed_frame);
    return transform;
  } catch (const tf2::LookupException &e) {
    RCLCPP_ERROR(this->get_logger(), "Lookup transform failed: %s", e.what());
  } catch (const tf2::ConnectivityException &e) {
    RCLCPP_ERROR(this->get_logger(), "Connectivity error during transform lookup: %s", e.what());
  } catch (const tf2::ExtrapolationException &e) {
    RCLCPP_ERROR(this->get_logger(), "Extrapolation error during transform lookup: %s", e.what());
  }
  return std::nullopt;
}
} // namespace boundary_estimation
