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

#include <string>

#include <functional>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tuple>
#include <vector>

namespace constants {
constexpr char kOusterLidar[] = "ouster";
constexpr char kHesaiLidar[] = "hesai";
}; // namespace constants

namespace lidar_cone_detector {

enum PointCloudType { COMPENSATED, FILTERED };

struct SimpleHash {
  size_t operator()(const std::tuple<int, int, int> &key) const noexcept {
    auto &[x, y, z] = key;
    size_t h1 = std::hash<int>{}(x);
    size_t h2 = std::hash<int>{}(y);
    size_t h3 = std::hash<int>{}(z);

    // Use a combination approach: Shift and XOR
    return h1 ^ (h2 << 1) ^ (h3 << 2); // Simple yet effective for many use cases
  }
};

struct Zone {
  float x;
  float y;
  float eps_dbscan_m;
  int min_pts_dbscan;
  double std_threshold_orange;
  double max_cluster_std_dev;
};

struct DbscanParameters {
  bool enabled;
  double max_cluster_z;
  int max_pts_dbscan;
  double min_cluster_std;
  double min_z_size;
  std::vector<Zone> zones;
};

struct LidarParameters {
  double x_min_m;
  double x_max_m;
  double y_min_m;
  double y_max_m;
  double z_min_m;
  double z_max_m;
  double x_min_car_m;
  double x_max_car_m;
  double y_min_car_m;
  double y_max_car_m;

  double downsampling_ratio;

  int grid_num_x;
  int grid_num_y;

  double distance_threshold_near_ransac_m;
  double distance_threshold_far_ransac_m;
  double distance_threshold_far_near_m;
  int min_point_in_segment;
  double min_fraction_inlier;
  bool rain;

  bool grid_filter_enabled;
  double rough_clustering_grid_size_m;
  int rough_clustering_max_points_per_grid;

  int max_iteration;

  DbscanParameters cone_detection;
  DbscanParameters pre_cluster;
};

struct RuntimeParameters {
  std::string fixed_frame_id;
  bool motion_compensation_enabled;
  bool transform_cone_enabled;
  bool filter_enabled;
  bool downsampling_enabled;
  bool profiling_enabled;
  bool logging_enabled;
  bool visualization_enabled;
  bool debug_enabled;
};

struct LidarPointHesai {
  PCL_ADD_POINT4D;  // [m] x, y, z
  double timestamp; // [ns] timestamp since the start of the scan
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // EIGEN_ALIGN16;

struct LidarPointOuster {
  PCL_ADD_POINT4D; // [m] x, y, z
  uint32_t t;      // [ns] timestamp since the start of the scan
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // EIGEN_ALIGN16;

} // namespace lidar_cone_detector

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_cone_detector::LidarPointHesai,
                                  (float, x, x)(float, y, y)(float, z, z)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_cone_detector::LidarPointOuster,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint32_t, t, t))
