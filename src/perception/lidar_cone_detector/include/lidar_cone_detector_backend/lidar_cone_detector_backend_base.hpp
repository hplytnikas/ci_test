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

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomous_msgs/msg/cone.hpp"
#include "autonomous_msgs/msg/cone_array.hpp"
#include "lidar_cone_detector_node.hpp"
#include "lidar_cone_detector_structs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "lidar_cone_detector_dbscan.hpp"

#include "easy/profiler.h"

namespace lidar_cone_detector {

class LidarConeDetectorBackendBase {
public:
  LidarConeDetectorBackendBase(const LidarParameters &lidar_params, const RuntimeParameters &runtime_params,
                               std::shared_ptr<NodeHandle> node_handler)
      : lidar_params_(lidar_params), runtime_params_(runtime_params), node_handler_(node_handler) {}

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr
  ProcessPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message);

  template <typename PointT> void PointCloudFilter(typename pcl::PointCloud<PointT>::Ptr point_cloud);

  template <typename PointT> void DownSampling(typename pcl::PointCloud<PointT>::Ptr cloud);

  template <typename PointT> void FilterGrids(typename pcl::PointCloud<PointT>::Ptr cloud);

  template <typename PointT>
  void SplitPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                       std::vector<typename pcl::PointCloud<PointT>::Ptr> &segments,
                       std::vector<double> &segments_threshold);

  template <typename PointT> void RemoveGroundPlane(typename pcl::PointCloud<PointT>::Ptr cloud, double threshold);

  template <typename PointT>
  void MergePointClouds(const std::vector<typename pcl::PointCloud<PointT>::Ptr> &segments,
                        typename pcl::PointCloud<PointT>::Ptr merged_cloud);

  virtual void MotionCompensation(const tf2::TimePoint &destination_time, const std::string &frame_id,
                                  void *point_cloud) = 0;

protected:
  Eigen::Affine3f TransformTfToEigen(const geometry_msgs::msg::Transform &transform_msg);

  LidarParameters lidar_params_;
  RuntimeParameters runtime_params_;
  std::shared_ptr<NodeHandle> node_handler_;
};

} // namespace lidar_cone_detector
