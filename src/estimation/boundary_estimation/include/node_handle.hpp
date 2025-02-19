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
#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "autonomous_msgs/msg/boundary.hpp"
#include "autonomous_msgs/msg/cone_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace boundary_estimation {

class NodeHandle : public rclcpp::Node {
public:
  NodeHandle(const std::string &, const std::string &, const rclcpp::NodeOptions &);

  void CreateOnlineMapSubscriber(const std::string &topic_name, size_t queue_size,
                                 std::function<void(const autonomous_msgs::msg::ConeArray)> callback);

  void CreateLapCounterSubscriber(const std::string &topic_name, size_t queue_size,
                                  std::function<void(const std_msgs::msg::Int32)> callback);

  void CreateGlobalMapFeedbackSubscriber(const std::string &topic_name, size_t queue_size,
                                         std::function<void(const std_msgs::msg::Bool)> callback);

  void CreateBoundaryPublisher(const std::string &topic_name, size_t queue_size);

  void CreateVisualizerPublisher(const std::string &topic_name, size_t queue_size);

  void CreateFinalPathVisualPublisher(const std::string &topic_name, size_t queue_size);

  void CreateFinalPathPublisher(const std::string &topic_name, size_t queue_size);

  void PublishBoundary(const autonomous_msgs::msg::Boundary &message);
  void PublishFinalBoundary(const autonomous_msgs::msg::Boundary &message);
  void PublishVisualizer(const visualization_msgs::msg::MarkerArray &message);
  void PublishFinalPathVisual(const visualization_msgs::msg::MarkerArray &message);

  std::optional<geometry_msgs::msg::TransformStamped>
  GetTransform(const std::string &source_frame, const std::string &target_frame, const tf2::TimePoint &time);

  std::optional<geometry_msgs::msg::TransformStamped>
  GetTransformWithRetry(const std::string &source_frame, const std::string &target_frame, const tf2::TimePoint &time);

  std::optional<geometry_msgs::msg::TransformStamped> GetTransformBetweenTimestamps(const std::string &source_frame,
                                                                                    const tf2::TimePoint &source_time,
                                                                                    const std::string &target_frame,
                                                                                    const tf2::TimePoint &target_time,
                                                                                    const std::string &fixed_frame);

private:
  // Subscriber
  rclcpp::Subscription<autonomous_msgs::msg::ConeArray>::SharedPtr online_map_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lap_counter_subscriber_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr transforms_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_map_feedback_subscriber_;

  // Publisher
  rclcpp::Publisher<autonomous_msgs::msg::Boundary>::SharedPtr boundary_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualizer_publisher_;
  rclcpp::Publisher<autonomous_msgs::msg::Boundary>::SharedPtr final_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr final_path_visual_publisher_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::MessageFilter<autonomous_msgs::msg::ConeArray>> online_map_filter_;
};
} // namespace boundary_estimation
