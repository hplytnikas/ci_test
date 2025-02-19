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

#pragma once
#define PCL_NO_RECOMPILE

#include "visualization_msgs/msg/marker_array.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autonomous_msgs/msg/cone_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/PointIndices.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <perception_msgs/msg/bounding_box.hpp>
#include <perception_msgs/msg/bounding_box_debug.hpp>
#include <perception_msgs/msg/box_array.hpp>
#include <perception_msgs/msg/box_array_debug.hpp>
#include <perception_msgs/msg/pipeline_runtime.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <easy/arbitrary_value.h>
#include <easy/profiler.h>
#include <memory>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz/types.hpp>
#include <string>
#include <vector>

namespace sensor_fusion_baseline {
// shorthands for message_filters synchronization
// policy (we use approximate time policy) types
using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, perception_msgs::msg::BoxArray,
                                                    sensor_msgs::msg::Image>;
using Sync = message_filters::Synchronizer<SyncPolicy>;

class SensorFusion : public rclcpp::Node {
public:
  SensorFusion();

private:
  std::shared_ptr<Sync> sync_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // TF2 Listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;              // TF2 Buffer

  // Pipeline Status
  bool debug_; // Debugging mode

  // Parameters related to time sync of messages
  int max_sync_offset_;   // Currenlty not used
  int min_inter_message_; // Currently not used

  // Frame IDs
  std::string camera_frame_id_;
  std::string lidar_frame_id_;
  std::string base_link_frame_id_;
  std::string reference_frame_id_;

  // Image resolution
  int image_width_;
  int image_height_;

  // Filter Tuning Parameters
  float min_cone_distance_;
  float max_cone_distance_;
  int min_lidar_points_;
  int image_bottom_border_;
  float max_dist_camera_depth_;
  float max_dist_pixels_;
  float averaging_depth_offset_;

  // Topic Names
  std::string topic_runtime_;
  std::string topic_points_compensated_;
  std::string topic_fw_bbox_;
  std::string topic_fw_cam_;

  // Intrinsics
  std::string intrinsics_file_;

  std::string topic_img_overlay_;
  std::string topic_cone_array_;
  std::string topic_bbox_fusion_;

  // Subscribers
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pointcloud_;
  message_filters::Subscriber<perception_msgs::msg::BoxArray> sub_bbox_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;

  // Publishers
  rclcpp::Publisher<perception_msgs::msg::PipelineRuntime>::SharedPtr pub_runtime_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_overlay_;
  rclcpp::Publisher<autonomous_msgs::msg::ConeArray>::SharedPtr pub_cone_array_;
  rclcpp::Publisher<perception_msgs::msg::BoxArrayDebug>::SharedPtr pub_bbox_fusion_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cone_markers_;

  // Intrinsics & Extrinscs (MRH2Cam) for the active camera
  cv::Mat K_mat_ = cv::Mat(3, 3, CV_32F);
  cv::Mat D_mat_ = cv::Mat(1, 14, CV_32F);
  cv::Mat t_mat_ = cv::Mat(1, 3, CV_32F);
  cv::Mat rvecR_ = cv::Mat(3, 1, CV_32F);

  void LoadParameters();
  void LoadCalibrationParameters();
  void SubscribeTopics();
  void AdvertiseTopics();
  void LoadIntrinsics();
  bool LoadStaticTransforms();

  /**
   * Callback function for the synchronized messages
   * @param pc_msg Point cloud message
   * @param bbox_msg Bounding box message
   * @param img_msg Image message
   */
  void DetectCones(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg,
                   const perception_msgs::msg::BoxArray::SharedPtr bbox_msg,
                   const sensor_msgs::msg::Image::SharedPtr img_msg);

  /**
   * Creates an autonomous_msgs::Cone out of a detected cone and its color label.
   * Code is copied from monocamera to stay consistent
   * @param cone_position Location of the cone in x,y,z base_link frame
   * @param box used for probability and color
   */
  autonomous_msgs::msg::Cone CreateConeMsg(const cv::Point3f &cone_position,
                                           const perception_msgs::msg::BoundingBox box);

  /**
   * Given bounding boxes, projected points, and a point cloud, it extracts the
   * cone position.
   *
   * The bounding boxes come from the camera pipeline, opencv rectangles.
   * Then for each bounding box we filter all the projected LiDAR points
   * (points_cam).
   *
   *
   * To estimate the position of the cone, the median point of the LiDAR points
   * (LiDAR points from cloud as they have x,y,z coordinates) are taken. The
   * median is more resilient against outliers than the mean. The median is with
   * respect to the distance from the egomotion frame
   *
   * @param image_mat The original image
   * @param boxes The bounding boxes from the camera pipeline. Must be
   * the same camera than the points_cam
   * @param points_cam Projected LiDAR points on a camera. Must be the same camera
   * than the bounding_boxes
   * @param cloud Motion compensated LiDAR point cloud (x,y,z) points in MRH LiDAR frame and properly time transformed
   * @param cone_array ConeArray message that will be published
   * @param time_stamp The time stamp used to take the correct motion compensated
   * transformation, camera timestamp in our case
   * @param sf_boxes Bounding boxes for debug
   */
  void EstimateConePosition(const cv::Mat *image_mat, const std::vector<perception_msgs::msg::BoundingBox> &boxes,
                            const std::vector<cv::Point2f> &points_cam, const std::vector<cv::Point3f> *cloud,
                            autonomous_msgs::msg::ConeArray *cone_array, const rclcpp::Time time_stamp,
                            perception_msgs::msg::BoxArrayDebug &sf_boxes);

  /**
   * Fetch the transformation between two frames at a given time stamp
   * @param src_frame The source frame
   * @param dest_frame The destination frame
   * @param time_stamp The time stamp
   * @param transform The returned transformation matrix
   * @return True if the fetch was successful
   */
  bool GetOpenCVTransform(const std::string &src_frame, const std::string &dest_frame, const rclcpp::Time &time_stamp,
                          cv::Mat *transform);

  /**
   * Get TF transform, and it returns the transformation matrix
   * @param tf The transform message
   * @param matrix The returned transformation matrix
   */
  void Tf2matrix(geometry_msgs::msg::TransformStamped tf, Eigen::Matrix4f &matrix);

  /**
   * Transform point cloud to camera timestamp
   * @param cloud The point cloud
   * @param destination_time_stamp The destination time stamp
   * @return The transformed point cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr TransformCloud2CamStamp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                              const rclcpp::Time destination_time_stamp);

  /**
   * Transform cones to the destination frame
   * @param destination_frame The destination frame
   * @param destination_time The destination time
   * @param cone_array The cone array to be transformed
   * @return True if the transformation was successful
   */
  bool TransformCones(const std::string &destination_frame, const rclcpp::Time destination_time,
                      autonomous_msgs::msg::ConeArray &cone_array);

  /**
   * Publishes the cone markers for better visualization in rviz2
   * @param cone_array The array of cones to be visualized
   */
  void PublishConeMarkers(const autonomous_msgs::msg::ConeArray &cone_array);

  /**
   * Resolves the filename from a 'package://' format to the actual file path in the package directory
   * Inspired by "CameraInfoManager::getPackageFileName" in
   * http://docs.ros.org/en/hydro/api/camera_info_manager/html/camera__info__manager_8cpp_source.html
   * @param filename The filename in 'package://' format
   * @return The actual file path
   */
  std::string GetPackageFilename(const std::string &filename);
};
} // namespace sensor_fusion_baseline
