/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - Chenhao Sun    <chensun@student.ethz.ch>
 *   - Paul Gregoire    <gregoirep@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#define PCL_NO_PRECOMPILE

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <unordered_map>
#include <vector>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>

#include "autonomous_msgs/msg/cone.hpp"
#include "autonomous_msgs/msg/cone_array.hpp"

#include "lidar_cone_detector_structs.hpp"
#include <easy/profiler.h>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define DBSCAN_SUCCESS 0
#define FAILURE -3

namespace lidar_cone_detector {

template <typename PointT> class DBSCAN {
public:
  explicit DBSCAN(DbscanParameters params) : params_(params) {}

  std::vector<autonomous_msgs::msg::Cone> ComputeClusters(typename pcl::PointCloud<PointT>::Ptr point_cloud);

  std::vector<int> CalculateCluster(int point_index, float epsilon);

  void ComputeClusters(typename pcl::PointCloud<PointT>::Ptr point_cloudtypename,
                       typename pcl::PointCloud<PointT>::Ptr cloud);

  pcl::PointXYZ CalculateCentroid(const std::vector<int> &cluster_indices);

  float CalculateStdDev(const std::vector<int> &cluster_indices, const pcl::PointXYZ &centroid);

  float CalculateZSize(const std::vector<int> &cluster_indices);

  int ExpandCluster(int point_index, int current_cluster_id);

  int ExpandCluster(int point_index, int current_cluster_id, typename pcl::PointCloud<PointT>::Ptr cloud);

  inline double CalculateDistance(const PointT &point_core, const pcl::PointXYZ &centroid);

  std::vector<autonomous_msgs::msg::Cone> GetClusterCentroid();

  std::vector<float> GetClusterStdDev() { return cluster_std_dev_; }

  std::vector<int> GetClusterSizes() { return cluster_sizes_; }

  std::vector<float> GetClusterZSize() { return cluster_z_sizes_; }

private:
  DbscanParameters params_;

  typename pcl::PointCloud<PointT>::Ptr point_cloud_;
  unsigned int point_cloud_size_;

  std::vector<int> cluster_sizes_;
  std::vector<float> cluster_std_dev_;
  std::vector<float> cluster_z_sizes_;
  std::vector<int> cluster_ids_vector_;

  std::vector<pcl::PointXYZ> valid_centroids_;

  pcl::search::KdTree<PointT> kd_tree_;
};

} // namespace lidar_cone_detector

// Explicit template instantiation declarations
extern template class lidar_cone_detector::DBSCAN<lidar_cone_detector::LidarPointHesai>;
extern template class lidar_cone_detector::DBSCAN<lidar_cone_detector::LidarPointOuster>;
