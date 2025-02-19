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

#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <autonomous_msgs/msg/cone_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <perception_msgs/msg/bounding_box.hpp>
#include <perception_msgs/msg/box_array.hpp>
#include <perception_msgs/msg/pipeline_runtime.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <easy/arbitrary_value.h>
#include <easy/profiler.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>

namespace depth_estimation {

using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<perception_msgs::msg::BoxArray, sensor_msgs::msg::Image>;
using Sync = message_filters::Synchronizer<SyncPolicy>;

using Pose = std::pair<double, double>;

class DepthEstimation : public rclcpp::Node {
public:
  DepthEstimation();
  KeypointDetector detector{nullptr};

private:
  std::shared_ptr<Sync> sync_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; // TF2 Listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                       // TF2 Buffer

  std::string weights_path_;

  // Pipeline Status
  bool use_gpu_;
  bool status_cone_array_; // publish camera cone array
  bool debug_;             // Debugging mode
  bool mde_on_;

  // Frame IDs
  std::string camera_frame_id_;
  std::string base_link_frame_id_;

  // Topics
  std::string topic_bbox_depth_;
  std::string topic_cone_array_;
  std::string intrinsics_server_name_;
  std::string topic_fw_cam_;
  std::string topic_fw_bbox_;
  std::string topic_img_overlay_;

  // Intrinsics
  std::string intrinsics_file_;

  // Subscriber
  message_filters::Subscriber<perception_msgs::msg::BoxArray> sub_bbox_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;

  // Publishers
  rclcpp::Publisher<perception_msgs::msg::BoxArray>::SharedPtr pub_bbox_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_overlay_;
  rclcpp::Publisher<autonomous_msgs::msg::ConeArray>::SharedPtr pub_cone_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cone_markers_;

  // Intrinsics & Extrinscs (MRH2Cam) for the active camera
  cv::Mat K_mat_ = cv::Mat(3, 3, CV_32F);
  cv::Mat D_mat_ = cv::Mat(1, 14, CV_32F);
  cv::Mat transform_camera_to_base_ = cv::Mat(4, 4, CV_32F);

  // Functions
  void loadParameters();
  void advertiseTopics();
  void subscribeTopics();
  void loadCalibrationParameters();

  // Parameters
  int image_width_;
  int image_height_;
  float small_cone_width_;
  float small_cone_height_;
  float big_cone_width_;
  float big_cone_height_;
  float small_cone_width_mde_;
  float small_cone_height_mde_;
  float big_cone_width_mde_;
  float big_cone_height_mde_;
  int distance_threshold_;
  float scaling_;

  void BboxDepth(const perception_msgs::msg::BoxArray::SharedPtr bbox_msg,
                 const sensor_msgs::msg::Image::SharedPtr img_msg);
  Pose DistanceEstimate(cv::Mat &img, const std::vector<int> &bounding_box, const bool is_big);
  void FillCones(const std::vector<perception_msgs::msg::BoundingBox> &boxes, const std::vector<Pose> &pose_list,
                 autonomous_msgs::msg::ConeArray &cones);
  void DepthFromBoxHeight(const std::vector<int> &bounding_box, Pose &pose, double &distance, const bool is_big);
  void PublishConeMarkers(const autonomous_msgs::msg::ConeArray &cone_array);
};

} // namespace depth_estimation
