/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - Chenhao Sun    <chensun@student.ethz.ch>
 *   - Paul Gregoire  <gregoirep@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "lidar_cone_detector_node.hpp"

namespace lidar_cone_detector {

NodeHandle::NodeHandle(const std::string &node_name, const std::string &node_namespace,
                       const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, node_namespace, options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {}

void NodeHandle::CreatePointCloudSubscriber(
    const std::string &topic_name, size_t queue_size,
    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> callback) {
  this->point_cloud_subscriber_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, queue_size, callback);
}

void NodeHandle::CreatePointCloudCompensatedPublisher(const std::string &topic_name, size_t queue_size) {
  this->point_cloud_compensated_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, queue_size);
}

void NodeHandle::CreateConeArrayPublisher(const std::string &topic_name, size_t queue_size) {
  this->cone_array_publisher_ = this->create_publisher<autonomous_msgs::msg::ConeArray>(topic_name, queue_size);
}

void NodeHandle::CreateFilteredPointCloudPublisher(const std::string &topic_name, size_t queue_size) {
  this->filtered_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, queue_size);
}

void NodeHandle::CreateCentroidPublisher(const std::string &topic_name, size_t queue_size) {
  this->centroid_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, queue_size);
}

void NodeHandle::PublishConeArray(const autonomous_msgs::msg::ConeArray &message) {
  if (cone_array_publisher_) {
    cone_array_publisher_->publish(message);
  } else {
    // Handle error, publisher not created
    RCLCPP_ERROR(this->get_logger(), "Cone array publisher not initialized");
  }
}

// General function to publish the point cloud
// Will call the specific publisher function based on the point cloud type
void NodeHandle::PublishPointCloud(const sensor_msgs::msg::PointCloud2 &message, const PointCloudType &type) {
  switch (type) {
  case PointCloudType::COMPENSATED:
    PublishPointCloudCompensated(message);
    break;
  case PointCloudType::FILTERED:
    PublishFilteredPointCloud(message);
    break;
  default:
    // Handle error, unknown point cloud type
    RCLCPP_ERROR(this->get_logger(), "Unknown point cloud type");
    break;
  }
}

void NodeHandle::PublishPointCloudCompensated(const sensor_msgs::msg::PointCloud2 &message) {
  if (point_cloud_compensated_publisher_) {
    point_cloud_compensated_publisher_->publish(message);
  } else {
    // Handle error, publisher not created
    RCLCPP_ERROR(this->get_logger(), "Point cloud compensate publisher not initialized");
  }
}

void NodeHandle::PublishFilteredPointCloud(const sensor_msgs::msg::PointCloud2 &message) {
  if (filtered_point_cloud_publisher_) {
    filtered_point_cloud_publisher_->publish(message);
  } else {
    // Handle error, publisher not created
    RCLCPP_ERROR(this->get_logger(), "Filtered point cloud publisher not initialized");
  }
}

void NodeHandle::PublishCentroid(const visualization_msgs::msg::MarkerArray &message) {
  if (centroid_publisher_) {
    centroid_publisher_->publish(message);
  } else {
    // Handle error, publisher not created
    RCLCPP_ERROR(this->get_logger(), "Centroid publisher not initialized");
  }
}

std::optional<geometry_msgs::msg::TransformStamped>
NodeHandle::GetTransform(const std::string &target_frame, const std::string &source_frame, const tf2::TimePoint &time) {
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = this->tf_buffer_.lookupTransform(target_frame, source_frame, time);
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

std::optional<geometry_msgs::msg::TransformStamped>
NodeHandle::GetTransformBetweenTimestamps(const std::string &target_frame, const tf2::TimePoint &target_time,
                                          const std::string &source_frame, const tf2::TimePoint &source_time,
                                          const std::string &fixed_frame) {
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = this->tf_buffer_.lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
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

} // namespace lidar_cone_detector
