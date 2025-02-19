/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - Matteo Mazzonelli <m.mazzonelli@gmail.com>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "fusion.hpp"

namespace sensor_fusion_baseline {
SensorFusion::SensorFusion() : Node("sensor_fusion") {
  LoadParameters();
  LoadCalibrationParameters();
  AdvertiseTopics();
  SubscribeTopics();
}

/**
 * Callback function of all the topics. The message_filters package calls this
 * function when all the messages arrive at (approximately) the same time.
 *
 * @param pc_msg
 * @param bbox_msg
 * @param img_msg
 */
void SensorFusion::DetectCones(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg,
                               const perception_msgs::msg::BoxArray::SharedPtr bbox_msg,
                               const sensor_msgs::msg::Image::SharedPtr img_msg) {
  EASY_FUNCTION(profiler::colors::Red); // Profiler function
  rclcpp::Time img_msg_time = img_msg->header.stamp;
  rclcpp::Time pc_msg_time = pc_msg->header.stamp;
  rclcpp::Time end_time = this->get_clock()->now();
  EASY_VALUE("Latency camera", (end_time - img_msg_time).nanoseconds() / 1e6);
  EASY_VALUE("Latency lidar", (end_time - pc_msg_time).nanoseconds() / 1e6);
  // ROS_INFO("[SF] Callback called ");
  // print received timestamps in nanoseconds

  std::vector<perception_msgs::msg::BoundingBox> boxes = bbox_msg->boxes;
  cv::Mat image_mat = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  perception_msgs::msg::BoxArrayDebug sf_boxes;
  // Lidar points are filtered, motion compensated and in [os_sensor] frame at
  // their own timestamp
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*pc_msg, *pcl_cloud);
  // Adding a single dummy point at (0,0,0) so that SF does not die
  // even in the case that the motion compensated pointcloud is empty
  if (pcl_cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty pointcloud received");
    return;
  }

  // Transform the pointcloud to the timestamp of the camera image
  pcl_cloud = TransformCloud2CamStamp(pcl_cloud, bbox_msg->header.stamp);
  std::vector<cv::Point3f> cloud;
  cloud.reserve(pc_msg->height * pc_msg->width);
  cloud.emplace_back(cv::Point3f(0.0f, 0.0f, 0.0f));

  for (const auto &point : pcl_cloud->points) {
    cloud.emplace_back(cv::Point3f(point.x, point.y, point.z));
  }
  std::vector<cv::Point2f> points_cam;
  cv::projectPoints(cloud, rvecR_, t_mat_, K_mat_, D_mat_, points_cam);
  if (debug_) {
    for (const auto &point : points_cam) {
      cv::circle(image_mat, point, 2, cv::viz::Color::red(), -1);
    }
  }

  // Estimating Cone Array and publishing the message
  autonomous_msgs::msg::ConeArray cone_array_msg;
  EstimateConePosition(&image_mat, boxes, points_cam, &cloud, &cone_array_msg, bbox_msg->header.stamp, sf_boxes);
  cone_array_msg.header.stamp = bbox_msg->header.stamp;
  cone_array_msg.header.frame_id = base_link_frame_id_;

  // Cones are in the [base_link] frame at the camera (bbox_msg) timestamp
  // Need to transform the cone array to the LiDAR timestamp in the [base_link]
  // frame and then without mathematically transforming anything change the
  if (!SensorFusion::TransformCones(base_link_frame_id_, pc_msg->header.stamp, cone_array_msg)) {
    RCLCPP_WARN(this->get_logger(), "Couldn't transform the SF cone array "
                                    "to the lidar timestmap");
  }
  pub_cone_array_->publish(cone_array_msg);
  SensorFusion::PublishConeMarkers(cone_array_msg);
  if (debug_) {
    for (const auto &box : boxes) {
      // Draw BBoxes on the image
      auto rect = cv::Rect(box.box_x_min, box.box_y_min, box.box_x_max - box.box_x_min, box.box_y_max - box.box_y_min);
      cv::rectangle(image_mat, rect, cv::Scalar(255, 255, 0), 4, 8, 0);
    }
    auto img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_mat).toImageMsg();
    pub_img_overlay_->publish(*img);

    sf_boxes.header = bbox_msg->header;
    pub_bbox_fusion_->publish(sf_boxes);
  }
}
} // namespace sensor_fusion_baseline
