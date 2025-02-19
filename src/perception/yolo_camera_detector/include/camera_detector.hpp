/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - Matteo Mazzonelli     <m.mazzonelli@gmail.com>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include "yolo_detector.hpp"
#include <onnxruntime_cxx_api.h>
#include <perception_msgs/msg/bounding_box.hpp>
#include <perception_msgs/msg/box_array.hpp>
#include <perception_msgs/msg/pipeline_runtime.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <easy/arbitrary_value.h>
#include <easy/profiler.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>

#include "utils.hpp"

namespace camera_detector {

using Pose = std::pair<float, float>;

class CameraDetector : public rclcpp::Node {
public:
  CameraDetector();
  YOLODetector detector{nullptr};

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; // TF2 Listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                       // TF2 Buffer

  bool debug_;         // Debugging mode
  bool use_gpu_;       // GPU or CPU
  bool enable_orange_; // Predict small/big orange cones
  bool night_mode_;    // Night mode

  int buffer_size_; // Buffer size

  int image_width_;         // Image Inference Width
  int image_height_;        // Image Inference Height
  int scaleFactor_;         // Downsampling factor
  float confThreshold_;     // Yolo object detection confidence threshold
  float iouThreshold_;      // NMS
  float small_cone_height_; // Cone height (m)
  float large_cone_height_;
  int bounding_box_filter_height_; // Bounding box height filter(pixels)
  int brightness_;
  float contrast_;

  int cone_counter_;                   // Cone counter
  std::string path_to_model_weights_;  // yolov5 Model Path
  std::string intrinsics_server_name_; // Intrinsics Server

  // Functions
  void loadParameters();
  void enablePTPSynchronization();
  void subscribeTopics();
  void advertiseTopics();
  void DetectCones(const sensor_msgs::msg::Image::SharedPtr fw_image);
  void SetBoxesMessage(const std::vector<Detection> &detections, perception_msgs::msg::BoxArray &boxes);

  // Camera Status
  bool status_cone_array_;

  // Topic Names
  std::string topic_fw_cam_;

  std::string topic_runtime_;
  std::string topic_fw_bbox_;
  std::string topic_fw_overlay_;
  std::string topic_cone_array_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_fw_cam_;

  // Publishers
  rclcpp::Publisher<perception_msgs::msg::PipelineRuntime>::SharedPtr pub_runtime_;
  rclcpp::Publisher<perception_msgs::msg::BoxArray>::SharedPtr pub_fw_bbox_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_fw_overlay_;
  rclcpp::Publisher<perception_msgs::msg::BoundingBox>::SharedPtr pub_cone_array_;

  // Services for camera ptp
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ptp_client_;
};
} // namespace camera_detector
