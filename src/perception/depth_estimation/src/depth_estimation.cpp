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

#include "depth_estimation.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

namespace depth_estimation {
DepthEstimation::DepthEstimation() : Node("depth_estimation") {
  loadParameters();
  loadCalibrationParameters();

  detector = KeypointDetector(weights_path_, use_gpu_);

  advertiseTopics();
  subscribeTopics();
  RCLCPP_INFO(this->get_logger(), "Depth Estimation has initialized. Thanks for waiting :)");
}

void DepthEstimation::BboxDepth(const perception_msgs::msg::BoxArray::SharedPtr bbox_msg,
                                const sensor_msgs::msg::Image::SharedPtr img_msg) {
  EASY_FUNCTION(profiler::colors::Red);
  rclcpp::Time img_msg_time = img_msg->header.stamp;
  rclcpp::Time end_time = this->get_clock()->now();
  EASY_VALUE("Latency", (end_time - img_msg_time).nanoseconds() / 1e6);

  perception_msgs::msg::BoxArray boxes_with_depth;
  std::vector<perception_msgs::msg::BoundingBox> boxes = bbox_msg->boxes;
  cv::Mat image_mat = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  std::vector<Pose> poses;

  for (const auto &box : boxes) {
    std::vector<int> boxLimits{box.box_x_min, box.box_y_min, box.box_x_max, box.box_y_max};
    bool is_big = (box.label == 3) ? true : false;

    // Skipping cones for which bounding box is close to the image bottom border
    if (box.box_y_max >= (image_height_ - 5)) {
      continue;
    }

    perception_msgs::msg::BoundingBox output_box = box;
    Pose pose;
    double distance;
    if (mde_on_) {
      pose = DistanceEstimate(image_mat, boxLimits, is_big);
      distance = std::sqrt((pose.first * pose.first) + (pose.second * pose.second));
    } else {
      DepthFromBoxHeight(boxLimits, pose, distance, is_big);
    }
    output_box.depth = distance;
    boxes_with_depth.boxes.push_back(output_box);
    poses.push_back(pose);
  }

  boxes_with_depth.header = bbox_msg->header;
  pub_bbox_depth_->publish(boxes_with_depth);

  if (debug_) {
    auto overlay_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_mat).toImageMsg();
    pub_img_overlay_->publish(*overlay_img);
  }

  if (status_cone_array_) {
    autonomous_msgs::msg::ConeArray cones;
    cones.header = bbox_msg->header;
    cones.header.frame_id = base_link_frame_id_;
    FillCones(boxes_with_depth.boxes, poses, cones);
    pub_cone_array_->publish(cones);
    DepthEstimation::PublishConeMarkers(cones);
  }
}

void DepthEstimation::PublishConeMarkers(const autonomous_msgs::msg::ConeArray &cone_array) {
  visualization_msgs::msg::MarkerArray markerArray;
  int markerId = 0;

  for (const auto &centroid : cone_array.cones) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link"; // Change to your point cloud's frame ID
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "cones";
    marker.id = markerId++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = centroid.position.x;
    marker.pose.position.y = centroid.position.y;
    marker.pose.position.z = centroid.position.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2; // Specify the size of the marker
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0; // Color the marker red
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // Make the marker opaque

    markerArray.markers.push_back(marker);
  }

  pub_cone_markers_->publish(markerArray);
}

Pose DepthEstimation::DistanceEstimate(cv::Mat &img, const std::vector<int> &bounding_box, const bool is_big = false) {
  EASY_FUNCTION(profiler::colors::Green);
  std::vector<double> bbox_limits{static_cast<double>(std::max(bounding_box[0], 0)),
                                  static_cast<double>(std::max(bounding_box[1], 0)),
                                  static_cast<double>(std::min(bounding_box[2], image_width_ - 1)),
                                  static_cast<double>(std::min(bounding_box[3], image_height_ - 1))};

  int upmiddle_x = static_cast<int>((bbox_limits[0] + bbox_limits[2]) / 2);
  cv::Mat imagePoints(7, 2, CV_64F);
  // the seven points are the following
  float cone_width = small_cone_width_mde_;
  float cone_height = small_cone_height_mde_;

  if ((bbox_limits[3] - bbox_limits[1]) * (bbox_limits[2] - bbox_limits[0]) > distance_threshold_) {
    cv::Mat roi =
        img(cv::Rect(bbox_limits[0], bbox_limits[1], bbox_limits[2] - bbox_limits[0], bbox_limits[3] - bbox_limits[1]))
            .clone();
    std::vector<float> keypoints = detector.PredictPoints(roi);
    for (int i = 0; i < 7; i++) {
      imagePoints.at<double>(i, 0) = static_cast<int>(((keypoints[2 * i] / 80) * roi.cols) + bbox_limits[0]);
      imagePoints.at<double>(i, 1) = static_cast<int>(((keypoints[(2 * i) + 1] / 80) * roi.rows) + bbox_limits[1]);
    }

    if (is_big) {
      cone_width = big_cone_width_mde_;
      cone_height = big_cone_height_mde_;
    }
  } else {
    imagePoints =
        (cv::Mat_<double>(7, 2) << upmiddle_x, bbox_limits[1],
         static_cast<int>(bbox_limits[0] + 2 * (upmiddle_x - bbox_limits[0]) / 3),
         static_cast<int>(bbox_limits[3] + 2 * (bbox_limits[1] - bbox_limits[3]) / 3),
         static_cast<int>(bbox_limits[0] + (upmiddle_x - bbox_limits[0]) / 3),
         static_cast<int>(bbox_limits[3] + (bbox_limits[1] - bbox_limits[3]) / 3), bbox_limits[0], bbox_limits[3],
         static_cast<int>(bbox_limits[2] - 2 * (upmiddle_x - bbox_limits[0]) / 3),
         static_cast<int>(bbox_limits[3] + 2 * (bbox_limits[1] - bbox_limits[3]) / 3),
         static_cast<int>(bbox_limits[2] - (upmiddle_x - bbox_limits[0]) / 3),
         static_cast<int>(bbox_limits[3] + (bbox_limits[1] - bbox_limits[3]) / 3), bbox_limits[2], bbox_limits[3]);

    if (is_big) {
      cone_width = big_cone_width_ * scaling_;
      cone_height = big_cone_height_ * scaling_;
    } else {
      cone_width = small_cone_width_ * scaling_;
      cone_height = small_cone_height_ * scaling_;
    }
  }

  cv::Mat objectPoints = (cv::Mat_<double>(7, 3) << 0, 0, cone_height, 0, cone_width / 6, 2 * cone_height / 3, 0,
                          cone_width / 3, cone_height / 3, 0, cone_width / 2, 0, 0, -cone_width / 6,
                          2 * cone_height / 3, 0, -cone_width / 3, cone_height / 3, 0, -cone_width / 2, 0);

  cv::Mat rvec, tvec;
  cv::solvePnP(objectPoints, imagePoints, K_mat_, D_mat_, rvec, tvec,
               false); // tvec is the translation vector of the base of the cone
                       // w.r.t. cam frame origin

  if (debug_) {
    cv::Mat cone_base = cv::Mat(1, 1, CV_64FC3, cv::Scalar(0.0, 0.0, 0.0));
    cv::projectPoints(cone_base, rvec, tvec, K_mat_, D_mat_, cone_base);
    cv::Point2f cone_base_point(cone_base.at<double>(0, 0), cone_base.at<double>(0, 1));
    cv::circle(img, cone_base_point, 2, cv::Scalar(0, 0, 0), -1);
  }

  cv::Mat tvec_cam = cv::Mat::zeros(4, 1, CV_64F);
  tvec.row(0).copyTo(tvec_cam.row(0));
  tvec.row(1).copyTo(tvec_cam.row(1));
  tvec.row(2).copyTo(tvec_cam.row(2));
  tvec_cam.at<double>(3, 0) = 1.0;
  cv::Mat tvec_base = transform_camera_to_base_ * tvec_cam;

  Pose cone_pose = std::make_pair(tvec_base.at<double>(0, 0), tvec_base.at<double>(1, 0));
  double distance = std::sqrt((cone_pose.first * cone_pose.first) + (cone_pose.second * cone_pose.second));

  int colour = 0;
  if ((bbox_limits[3] - bbox_limits[1]) * (bbox_limits[2] - bbox_limits[0]) > distance_threshold_) {
    colour = 255;
  }

  if (debug_) {
    cv::rectangle(img, cv::Point(bbox_limits[0], bbox_limits[1]), cv::Point(bbox_limits[2], bbox_limits[3]),
                  cv::Scalar(255, 0, 0), 1);
    for (int i = 0; i < imagePoints.rows; i++) {
      cv::circle(img, cv::Point(imagePoints.at<double>(i, 0), imagePoints.at<double>(i, 1)), 2,
                 cv::Scalar(0, 0, colour), -1);
    }
    cv::putText(img, std::to_string(distance), cv::Point(bbox_limits[0], bbox_limits[1] - 10), cv::FONT_HERSHEY_SIMPLEX,
                0.5, cv::Scalar(255, 0, 0), 2);
  }

  return cone_pose;
}

} // namespace depth_estimation
