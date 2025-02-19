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

#include "camera_detector.hpp"

#include "utils.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/viz/types.hpp>
#include <string>

namespace camera_detector {

void CameraDetector::SetBoxesMessage(const std::vector<Detection> &detections, perception_msgs::msg::BoxArray &boxes) {
  for (int i = 0; i < detections.size(); i++) {
    auto box = detections[i].box;

    perception_msgs::msg::BoundingBox output_box;
    output_box.box_x_min = box.x;
    output_box.box_x_max = box.x + box.width;
    output_box.box_y_min = box.y;
    output_box.box_y_max = box.y + box.height;
    output_box.prob_cone = detections[i].conf;
    output_box.label = detections[i].classId;
    output_box.prob_type.blue = detections[i].b_conf;
    output_box.prob_type.yellow = detections[i].y_conf;
    output_box.prob_type.orange = detections[i].os_conf;
    output_box.prob_type.orange_big = detections[i].ob_conf;
    output_box.depth = 0.0; // Depth will be assgined downstream in the depth_estimation node.
    boxes.boxes.push_back(output_box);
  }
}

void CameraDetector::loadParameters() {
  // Subscriber Topic Names
  this->declare_parameter<std::string>("topics.sub.fw_cam", "/sensors/forward_camera/image_color");

  // Publisher Topic Names
  this->declare_parameter<std::string>("topics.pub.runtime", "/perception/runtimes");
  this->declare_parameter<std::string>("topics.pub.fw_bbox", "/perception/yolo_camera_detector/forward_bbox");
  this->declare_parameter<std::string>("topics.pub.debug.fw_overlay",
                                       "/perception/yolo_camera_detector/debug/forward_overlay");

  // Other Parameters
  this->declare_parameter<bool>("use_gpu", false);
  this->declare_parameter<bool>("debug", true);
  this->declare_parameter<bool>("enable_orange_cones", true);
  this->declare_parameter<bool>("night_mode", false);
  this->declare_parameter<int>("buffer_size", 1);

  // YOLO Parameters
  this->declare_parameter<int>("image_width", 2592);
  this->declare_parameter<int>("image_height", 352);
  this->declare_parameter<float>("confThreshold", 0.25);
  this->declare_parameter<float>("iouThreshold", 0.7);
  this->declare_parameter<int>("scaleFactor", 1);

  this->declare_parameter<float>("small_cone_height", 0.329);
  this->declare_parameter<float>("large_cone_height", 0.505);
  this->declare_parameter<int>("bounding_box_filter_height", 1000);
  this->declare_parameter<int>("brightness", 140);
  this->declare_parameter<float>("contrast", 1.0);
  this->declare_parameter<std::string>("weights", "/.amz/best_2560s.onnx");

  this->get_parameter("topics.sub.fw_cam", topic_fw_cam_);

  this->get_parameter("topics.pub.runtime", topic_runtime_);
  this->get_parameter("topics.pub.fw_bbox", topic_fw_bbox_);
  this->get_parameter("topics.pub.debug.fw_overlay", topic_fw_overlay_);

  this->get_parameter("use_gpu", use_gpu_);
  this->get_parameter("debug", debug_);
  this->get_parameter("enable_orange_cones", enable_orange_);
  this->get_parameter("night_mode", night_mode_);
  this->get_parameter("buffer_size", buffer_size_);

  this->get_parameter("image_width", image_width_);
  this->get_parameter("image_height", image_height_);
  this->get_parameter("confThreshold", confThreshold_);
  this->get_parameter("iouThreshold", iouThreshold_);
  this->get_parameter("scaleFactor", scaleFactor_);

  this->get_parameter("small_cone_height", small_cone_height_);
  this->get_parameter("large_cone_height", large_cone_height_);
  this->get_parameter("bounding_box_filter_height", bounding_box_filter_height_);
  this->get_parameter("brightness", brightness_);
  this->get_parameter("contrast", contrast_);
  path_to_model_weights_ = std::string(std::getenv("HOME")) + this->get_parameter("weights").as_string();
}

// Subscribe to camera image topics
void CameraDetector::subscribeTopics() {
  sub_fw_cam_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_fw_cam_, buffer_size_, std::bind(&CameraDetector::DetectCones, this, std::placeholders::_1));
}

// Advertise yolo predictions - debugging purposes
void CameraDetector::advertiseTopics() {
  pub_runtime_ = this->create_publisher<perception_msgs::msg::PipelineRuntime>(topic_runtime_, 1);
  pub_fw_bbox_ = this->create_publisher<perception_msgs::msg::BoxArray>(topic_fw_bbox_, 1);
  pub_fw_overlay_ = this->create_publisher<sensor_msgs::msg::Image>(topic_fw_overlay_, 1);
}

void CameraDetector::enablePTPSynchronization() {
  ptp_client_ = this->create_client<std_srvs::srv::SetBool>("/forward_camera/pylon_ros2_camera_node/enable_ptp");

  while (!ptp_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the PTP service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for the PTP service to be available...");
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Successfully enabled PTP.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to enable PTP.");
    }
  };

  auto future_result = ptp_client_->async_send_request(request, response_received_callback);
}

} // namespace camera_detector
