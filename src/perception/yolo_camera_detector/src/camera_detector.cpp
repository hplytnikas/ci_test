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

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "utils.hpp"
#include "yolo_detector.hpp"

namespace camera_detector {

CameraDetector::CameraDetector() : Node("yolo_camera_detector") {
  loadParameters();
  enablePTPSynchronization();

  detector = YOLODetector(path_to_model_weights_, use_gpu_, debug_, cv::Size(image_width_, image_height_));
  RCLCPP_INFO(get_logger(), "YOLO Model has initialized");

  subscribeTopics();
  advertiseTopics();
}

void CameraDetector::DetectCones(const sensor_msgs::msg::Image::SharedPtr img_msg) {
  EASY_FUNCTION(profiler::colors::Red); // Profiler function
  rclcpp::Time img_msg_time = img_msg->header.stamp;
  rclcpp::Time end_time = this->get_clock()->now();
  EASY_VALUE("Latency", (end_time - img_msg_time).nanoseconds() / 1e6);

  // Creates a copy of img_msg (is a ROS type image - sensors::msgs::Image) and
  // converts it to img_mat (is an OpenCV type image - cv::Mat). We do the
  // conversion to feed it to the detector.
  cv::Mat img_mat = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  // FYI - image tensor format [B, C, H , W]
  // To test different input shape -> Export yolov5s_amz.pt again using
  // ultralitics script and
  // --img-size x y --dynamic --include onnx

  cv::Scalar img_brightness = cv::mean(img_mat);
  float required_brightness = brightness_ - ((img_brightness[0] + img_brightness[1] + img_brightness[2]) / 3);

  std::vector<Detection> detections;
  detections = detector.detect(img_mat, cv::Size(image_width_, image_height_), confThreshold_, iouThreshold_,
                               scaleFactor_, required_brightness, contrast_, enable_orange_, night_mode_);
  perception_msgs::msg::BoxArray boxes;
  boxes.header = img_msg->header;
  SetBoxesMessage(detections, boxes);

  pub_fw_bbox_->publish(boxes);

  // PublishRuntime(img_msg->header.stamp, this->get_clock()->now());

  if (debug_) {
    // Draw bounding boxes to image frame
    img_mat.convertTo(img_mat, -1, contrast_, required_brightness);
    utils::drawBoundingBoxes(img_mat, detections, enable_orange_);
    // convert CV image back to ROS image
    auto overlay_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_mat).toImageMsg();
    pub_fw_overlay_->publish(*overlay_img);
  }
}
} // namespace camera_detector
