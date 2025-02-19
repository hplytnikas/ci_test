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

#include "lidar_cone_detector_backend/lidar_cone_detector_backend_base.hpp"

namespace lidar_cone_detector {

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
LidarConeDetectorBackendBase::ProcessPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message) {
  EASY_FUNCTION(profiler::colors::Blue);
  typename pcl::PointCloud<PointT>::Ptr point_cloud(new typename pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*point_cloud_message, *point_cloud);
  PointCloudFilter<PointT>(point_cloud);
  if (runtime_params_.downsampling_enabled) {
    DownSampling<PointT>(point_cloud);
  }
  std::vector<typename pcl::PointCloud<PointT>::Ptr> segments;
  std::vector<double> segments_threshold;
  SplitPointCloud<PointT>(point_cloud, segments, segments_threshold);

  for (int i = 0; i < segments.size(); i++) {
    auto &segment = segments[i];
    if (segment->points.size() < lidar_params_.min_point_in_segment) {
      segment->points.clear();
      continue;
    }
    double threshold = segments_threshold[i];
    RemoveGroundPlane<PointT>(segment, threshold);
  }
  typename pcl::PointCloud<PointT>::Ptr merged_point_cloud(new pcl::PointCloud<PointT>);
  MergePointClouds<PointT>(segments, merged_point_cloud);

  // if (lidar_params_.grid_filter_enabled) {
  //   FilterGrids<PointT>(merged_point_cloud);
  // }
  //
  typename std::shared_ptr<DBSCAN<PointT>> dbscan_precluster_ =
      std::make_shared<DBSCAN<PointT>>(lidar_params_.pre_cluster);
  typename pcl::PointCloud<PointT>::Ptr precluster_cloud(new pcl::PointCloud<PointT>);

  if (runtime_params_.debug_enabled)
    RCLCPP_INFO(node_handler_->get_logger(), "before precluster: %ld", merged_point_cloud->points.size());

  if (lidar_params_.pre_cluster.enabled) {
    dbscan_precluster_->ComputeClusters(merged_point_cloud, precluster_cloud);
  }

  if (runtime_params_.debug_enabled)
    RCLCPP_INFO(node_handler_->get_logger(), "after precluster: %ld", precluster_cloud->points.size());

  tf2::TimePoint destination_time = tf2::TimePoint(std::chrono::microseconds(point_cloud->header.stamp));
  // cast the point cloud to void
  if (lidar_params_.pre_cluster.enabled) {
    if (runtime_params_.motion_compensation_enabled) {
      MotionCompensation(destination_time, point_cloud->header.frame_id, static_cast<void *>(precluster_cloud.get()));
    }
    return precluster_cloud;

  } else {
    if (runtime_params_.motion_compensation_enabled) {
      MotionCompensation(destination_time, point_cloud->header.frame_id, static_cast<void *>(merged_point_cloud.get()));
    }
    return merged_point_cloud;
  }
}

template <typename PointT>
void LidarConeDetectorBackendBase::PointCloudFilter(typename pcl::PointCloud<PointT>::Ptr point_cloud) {
  EASY_FUNCTION(profiler::colors::Green);

  if (runtime_params_.filter_enabled) {
    pcl::PassThrough<PointT> pass_x;
    pcl::PassThrough<PointT> pass_y;
    pcl::PassThrough<PointT> pass_z;

    // Apply PassThrough filter on x-axis
    pass_x.setInputCloud(point_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(lidar_params_.x_min_m, lidar_params_.x_max_m);
    pass_x.filter(*point_cloud);

    // Apply PassThrough filter on y-axis
    pass_y.setInputCloud(point_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(lidar_params_.y_min_m, lidar_params_.y_max_m);
    pass_y.filter(*point_cloud);

    // Apply PassThrough filter on z-axis
    pass_z.setInputCloud(point_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(lidar_params_.z_min_m, lidar_params_.z_max_m);
    pass_z.filter(*point_cloud);

    // Set up conditional removal filter
    typename pcl::ConditionOr<PointT>::Ptr range_cond(new pcl::ConditionOr<PointT>());
    range_cond->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(
        new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, lidar_params_.x_min_car_m)));
    range_cond->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(
        new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, lidar_params_.x_max_car_m)));
    range_cond->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(
        new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LT, lidar_params_.y_min_car_m)));
    range_cond->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(
        new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, lidar_params_.y_max_car_m)));

    pcl::ConditionalRemoval<PointT> condrem;
    condrem.setInputCloud(point_cloud);
    condrem.setCondition(range_cond);
    condrem.setKeepOrganized(false);
    condrem.filter(*point_cloud);
  }
}

template <typename PointT>
void LidarConeDetectorBackendBase::DownSampling(typename pcl::PointCloud<PointT>::Ptr cloud) {
  EASY_FUNCTION(profiler::colors::Yellow);
  auto input_size = cloud->points.size();
  auto sample_size = static_cast<uint32_t>(input_size * (1 - lidar_params_.downsampling_ratio));

  pcl::ExtractIndices<PointT> extract;
  pcl::PointIndices::Ptr sample_indices(new pcl::PointIndices());
  std::vector<uint32_t> cloud_ind_vec(input_size);
  std::iota(std::begin(cloud_ind_vec), std::end(cloud_ind_vec), 0);
  std::random_shuffle(cloud_ind_vec.begin(), cloud_ind_vec.end());
  std::vector<uint32_t> cloud_ds_vec(cloud_ind_vec.begin(), cloud_ind_vec.begin() + sample_size);
  for (uint32_t i = 0; i < sample_size; i++) {
    sample_indices->indices.push_back(cloud_ds_vec[i]);
  }

  extract.setInputCloud(cloud);
  extract.setIndices(sample_indices);
  extract.setNegative(true);
  extract.filter(*cloud);
}

template <typename PointT> void LidarConeDetectorBackendBase::FilterGrids(typename pcl::PointCloud<PointT>::Ptr cloud) {
  EASY_FUNCTION(profiler::colors::Purple);

  // Using a map to store grid cells and their point indices
  std::unordered_map<std::tuple<int, int, int>, std::vector<int>, SimpleHash> grid_map;
  std::vector<int> indices_to_remove;

  // Populate the grid map with points
  for (int i = 0; i < cloud->points.size(); i++) {
    auto point = cloud->points[i];
    int grid_x = static_cast<int>(point.x / lidar_params_.rough_clustering_grid_size_m);
    int grid_y = static_cast<int>(point.y / lidar_params_.rough_clustering_grid_size_m);
    int grid_z = static_cast<int>(point.z / lidar_params_.rough_clustering_grid_size_m);

    std::tuple<int, int, int> grid_key = std::make_tuple(grid_x, grid_y, grid_z);
    grid_map[grid_key].push_back(i);
  }

  // Identify points to remove
  for (const auto &grid_cell : grid_map) {
    if (grid_cell.second.size() > lidar_params_.rough_clustering_max_points_per_grid) {
      indices_to_remove.insert(indices_to_remove.end(), grid_cell.second.begin(), grid_cell.second.end());
    }
  }

  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  indices->indices = indices_to_remove;

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.setNegative(true);
  extract.filter(*cloud);
}

template <typename PointT>
void LidarConeDetectorBackendBase::SplitPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                   std::vector<typename pcl::PointCloud<PointT>::Ptr> &segments,
                                                   std::vector<double> &segments_threshold) {
  EASY_FUNCTION(profiler::colors::Yellow);

  double x_min = lidar_params_.x_min_m;
  double x_max = lidar_params_.x_max_m;
  double y_min = lidar_params_.y_min_m;
  double y_max = lidar_params_.y_max_m;

  double x_length = (x_max - x_min) / lidar_params_.grid_num_x;
  double y_length = (y_max - y_min) / lidar_params_.grid_num_y;

  int x_index_max = lidar_params_.grid_num_x;

  for (const auto &point : cloud->points) {
    int index_x = (point.x - x_min) / x_length;
    int index_y = (point.y - y_min) / y_length;

    int segment_index = index_y * x_index_max + index_x;
    while (segments.size() <= segment_index) {
      segments.push_back(typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>));
    }

    segments[segment_index]->points.push_back(point);
  }

  for (int i = 0; i < lidar_params_.grid_num_y; i++) {
    for (int j = 0; j < lidar_params_.grid_num_x; j++) {
      double grid_center_x = x_min + x_length * j + x_length / 2;
      double grid_center_y = y_min + y_length * i + y_length / 2;
      double grid_center_distance = std::sqrt(grid_center_x * grid_center_x + grid_center_y * grid_center_y);
      if (grid_center_distance < lidar_params_.distance_threshold_far_near_m) {
        segments_threshold.push_back(lidar_params_.distance_threshold_near_ransac_m);
      } else {
        segments_threshold.push_back(lidar_params_.distance_threshold_far_ransac_m);
      }
    }
  }
}

template <typename PointT>
void LidarConeDetectorBackendBase::RemoveGroundPlane(typename pcl::PointCloud<PointT>::Ptr cloud, double threshold) {
  EASY_FUNCTION(profiler::colors::Grey);
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(lidar_params_.max_iteration);
  seg.setDistanceThreshold(threshold);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    cloud->points.clear();
    return;
  }

  int num_points_total = cloud->points.size();

  // RCLCPP_INFO(node_handler_->get_logger(), "[lidar cone detector 2] number of points in total: %d",
  // num_points_total);

  pcl::ExtractIndices<PointT> extractor;
  extractor.setInputCloud(cloud);
  extractor.setIndices(inliers);
  extractor.setNegative(true);
  extractor.filter(*cloud);

  // RCLCPP_INFO(node_handler_->get_logger(), "[lidar cone detector 2] fraction_inliers: %f", fraction_inliers);

  if (lidar_params_.rain) {
    if (coefficients->values[2] < 0) {
      for (size_t i = 0; i < coefficients->values.size(); ++i) {
        coefficients->values[i] = -coefficients->values[i];
      }
    }

    pcl::PointIndices::Ptr below_ground_indices(new pcl::PointIndices);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
      PointT point = cloud->points[i];
      // Plane equation: ax + by + cz + d = 0
      float distance_from_plane = coefficients->values[0] * point.x + coefficients->values[1] * point.y +
                                  coefficients->values[2] * point.z + coefficients->values[3];
      // If the point is below the plane, add its index to below_plane_indices
      if (distance_from_plane < 0) {
        below_ground_indices->indices.push_back(i);
      }
    }

    pcl::ExtractIndices<PointT> reflection_extractor;
    reflection_extractor.setInputCloud(cloud);
    reflection_extractor.setIndices(below_ground_indices);
    reflection_extractor.setNegative(true);
    reflection_extractor.filter(*cloud);
  }

  int num_points_remain = cloud->points.size();

  // RCLCPP_INFO(node_handler_->get_logger(), "[lidar cone detector 2] number of points remained: %d",
  // num_points_remain);

  float fraction_inliers = 1 - static_cast<float>(num_points_remain) / static_cast<float>(num_points_total);

  if (fraction_inliers < lidar_params_.min_fraction_inlier) {
    cloud->points.clear();
  }
}

template <typename PointT>
void LidarConeDetectorBackendBase::MergePointClouds(const std::vector<typename pcl::PointCloud<PointT>::Ptr> &segments,
                                                    typename pcl::PointCloud<PointT>::Ptr merged_cloud) {
  EASY_FUNCTION(profiler::colors::CreamWhite);
  for (const auto &segment : segments) {
    *merged_cloud += *segment;
  }
}

Eigen::Affine3f LidarConeDetectorBackendBase::TransformTfToEigen(const geometry_msgs::msg::Transform &transform_msg) {
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform =
      Eigen::Translation3f(transform_msg.translation.x, transform_msg.translation.y, transform_msg.translation.z) *
      Eigen::Quaternionf(transform_msg.rotation.w, transform_msg.rotation.x, transform_msg.rotation.y,
                         transform_msg.rotation.z);
  return transform;
}
// Explicit template instantiation for specific types
template typename pcl::PointCloud<LidarPointHesai>::Ptr
LidarConeDetectorBackendBase::ProcessPointCloud<LidarPointHesai>(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message);
template typename pcl::PointCloud<LidarPointOuster>::Ptr
LidarConeDetectorBackendBase::ProcessPointCloud<LidarPointOuster>(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message);

} // namespace lidar_cone_detector
