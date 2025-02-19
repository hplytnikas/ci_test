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

#pragma once
#define PCL_NO_PRECOMPILE
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "autonomous_msgs/msg/cone.hpp"
#include "autonomous_msgs/msg/cone_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "lidar_cone_detector_structs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace lidar_cone_detector {

class NodeHandle : public rclcpp::Node {
public:
  // Constructor
  NodeHandle(const std::string &, const std::string &, const rclcpp::NodeOptions &);

  // Create subscriber and publisher
  void CreatePointCloudSubscriber(const std::string &topic_name, size_t queue_size,
                                  std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> callback);

  void CreatePointCloudCompensatedPublisher(const std::string &topic_name, size_t queue_size);

  void CreateConeArrayPublisher(const std::string &topic_name, size_t queue_size);

  void CreateFilteredPointCloudPublisher(const std::string &topic_name, size_t queue_size);

  void CreateCentroidPublisher(const std::string &topic_name, size_t queue_size);

  // Create publisher functions
  void PublishPointCloud(const sensor_msgs::msg::PointCloud2 &message, const PointCloudType &type);

  void PublishPointCloudCompensated(const sensor_msgs::msg::PointCloud2 &message);

  void PublishFilteredPointCloud(const sensor_msgs::msg::PointCloud2 &message);

  void PublishConeArray(const autonomous_msgs::msg::ConeArray &message);

  void PublishCentroid(const visualization_msgs::msg::MarkerArray &message);

  std::optional<geometry_msgs::msg::TransformStamped>
  GetTransform(const std::string &target_frame, const std::string &source_frame, const tf2::TimePoint &time);

  std::optional<geometry_msgs::msg::TransformStamped> GetTransformBetweenTimestamps(const std::string &target_frame,
                                                                                    const tf2::TimePoint &target_time,
                                                                                    const std::string &source_frame,
                                                                                    const tf2::TimePoint &source_time,
                                                                                    const std::string &fixed_frame);

private:
  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_compensated_publisher_;
  rclcpp::Publisher<autonomous_msgs::msg::ConeArray>::SharedPtr cone_array_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_point_cloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centroid_publisher_;

  // Buffer
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

} // namespace lidar_cone_detector
