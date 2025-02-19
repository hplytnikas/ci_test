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

#include "lidar_cone_detector_dbscan.hpp"

namespace lidar_cone_detector {

template <typename PointT>
std::vector<autonomous_msgs::msg::Cone>
DBSCAN<PointT>::ComputeClusters(const typename pcl::PointCloud<PointT>::Ptr point_cloud) {
  EASY_FUNCTION(profiler::colors::Red);
  // Reset the class variables
  valid_centroids_.clear();
  cluster_std_dev_.clear();
  cluster_sizes_.clear();
  cluster_ids_vector_.clear();

  point_cloud_ = point_cloud;
  point_cloud_size_ = point_cloud_->size();
  cluster_ids_vector_.resize(point_cloud_size_, UNCLASSIFIED);
  kd_tree_.setInputCloud(point_cloud_);

  int current_cluster_id = 1;

  // Iterate through each point by its index
  for (size_t i = 0; i < point_cloud_size_; ++i) {
    if (cluster_ids_vector_[i] == UNCLASSIFIED) {
      // Try to expand cluster starting from the unclassified point
      if (ExpandCluster(i, current_cluster_id) != FAILURE) {
        current_cluster_id += 1; // Move to the next cluster ID only if expansion was successful
      }
    }
  }

  return GetClusterCentroid(); // Indicate successful completion
}

template <typename PointT>
void DBSCAN<PointT>::ComputeClusters(const typename pcl::PointCloud<PointT>::Ptr point_cloud,
                                     typename pcl::PointCloud<PointT>::Ptr precluster_cloud) {
  EASY_FUNCTION(profiler::colors::Red);
  // Reset the class variables
  valid_centroids_.clear();
  cluster_std_dev_.clear();
  cluster_z_sizes_.clear();
  cluster_sizes_.clear();
  cluster_ids_vector_.clear();

  point_cloud_ = point_cloud;
  point_cloud_size_ = point_cloud_->size();
  cluster_ids_vector_.resize(point_cloud_size_, UNCLASSIFIED);
  kd_tree_.setInputCloud(point_cloud_);

  int current_cluster_id = 1;

  // Iterate through each point by its index
  for (size_t i = 0; i < point_cloud_size_; ++i) {
    if (cluster_ids_vector_[i] == UNCLASSIFIED) {
      // Try to expand cluster starting from the unclassified point
      if (ExpandCluster(i, current_cluster_id, precluster_cloud) != FAILURE) {
        current_cluster_id += 1; // Move to the next cluster ID only if expansion was successful
      }
    }
  }

  // auto start_ite = std::chrono::high_resolution_clock::now();

  // for (size_t i = 0; i < cluster_ids_vector_.size(); ++i)
  // {
  //     if (cluster_ids_vector_[i] != NOISE)
  //     {
  //         precluster_cloud->points.push_back(point_cloud->points[i]);
  //     }
  // }
  // auto end_ite = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_gpu - start_gpu);
  // RCLCPP_INFO(this->get_logger(), "ite time: %f", duration.count() / 1000.0);

  precluster_cloud->width = precluster_cloud->points.size();
  precluster_cloud->height = 1;
  precluster_cloud->is_dense = true;
}

template <typename PointT>
int DBSCAN<PointT>::ExpandCluster(int point_index, int current_cluster_id,
                                  typename pcl::PointCloud<PointT>::Ptr precluster_cloud) {
  // float distance_from_sensor = std::sqrt(point_cloud_->points[point_index].x * point_cloud_->points[point_index].x +
  //                                        point_cloud_->points[point_index].y * point_cloud_->points[point_index].y +
  //                                        point_cloud_->points[point_index].z * point_cloud_->points[point_index].z);
  float x = point_cloud_->points[point_index].x;
  float y = point_cloud_->points[point_index].y;
  float z = point_cloud_->points[point_index].z;
  int min_pts = params_.zones.back().min_pts_dbscan;
  float max_cluster_std = params_.zones.back().max_cluster_std_dev;
  float epsilon = params_.zones.back().eps_dbscan_m;
  for (const auto &zone : params_.zones) {
    if (x <= zone.x && x >= -zone.x && y <= zone.y && y >= -zone.y) {
      min_pts = zone.min_pts_dbscan;
      max_cluster_std = zone.max_cluster_std_dev;
      epsilon = zone.eps_dbscan_m;
      // RCLCPP_INFO(rclcpp::get_logger("lidar_cone_detector"),
      //   "distance: %f, min_pts: %d, max_cluster_std: %f, epsilon: %f", distance_from_sensor, min_pts,
      //   max_cluster_std, epsilon);
      break;
    }
  }
  // RCLCPP_INFO(rclcpp::get_logger("lidar_cone_detector"),
  //   "after distance: %f, min_pts: %d, max_cluster_std: %f, epsilon: %f", distance_from_sensor, min_pts,
  //   max_cluster_std, epsilon);

  std::vector<int> cluster_seeds = CalculateCluster(point_index, epsilon);
  // std::vector<int> centroid = CalculateCentroid(cluster_seeds);
  // float standard_deviation = CalculateStdDev(cluster_seeds, centroid);

  if (cluster_seeds.size() < min_pts) {
    cluster_ids_vector_[point_index] = NOISE; // Mark the point as noise if it doesn't
                                              // meet the minPoints criterion
    return FAILURE;
  }
  for (int seed_index : cluster_seeds) {
    cluster_ids_vector_[seed_index] = current_cluster_id;
  }

  // Remove the core point from the seed list
  cluster_seeds.erase(std::remove(cluster_seeds.begin(), cluster_seeds.end(), point_index), cluster_seeds.end());

  // Iterate through each seed point, attempting to expand the cluster
  for (size_t i = 0; i < cluster_seeds.size(); ++i) {
    int current_index = cluster_seeds[i];
    std::vector<int> current_neighbors = CalculateCluster(current_index, epsilon);

    if (current_neighbors.size() >= min_pts) {
      // Try to expand the cluster with each neighbor
      for (int neighborIndex : current_neighbors) {
        if (cluster_ids_vector_[neighborIndex] == UNCLASSIFIED || cluster_ids_vector_[neighborIndex] == NOISE) {
          if (cluster_ids_vector_[neighborIndex] == UNCLASSIFIED) {
            // If the neighbor was unclassified, it's now part of the cluster
            cluster_seeds.push_back(neighborIndex); // Add new seed
          }
          cluster_ids_vector_[neighborIndex] = current_cluster_id; // Assign cluster ID
        }
      }
    }
  }
  // adding the cluster size and std dev in order to debug in lidar_pipeline.cpp
  // we removed the core point from the cluster seeds
  cluster_seeds.push_back(point_index);
  float z_size = CalculateZSize(cluster_seeds);
  pcl::PointXYZ centroid = CalculateCentroid(cluster_seeds);
  float standard_deviation = CalculateStdDev(cluster_seeds, centroid);
  if (standard_deviation > max_cluster_std || standard_deviation < params_.min_cluster_std ||
      centroid.z > params_.max_cluster_z || cluster_seeds.size() > params_.max_pts_dbscan ||
      z_size < params_.min_z_size) {
    cluster_ids_vector_[point_index] = NOISE;
    // Mark points as NOISE or handle them differently
    for (int seed_index : cluster_seeds) {
      cluster_ids_vector_[seed_index] = NOISE;
    }
    return FAILURE;
  }

  cluster_z_sizes_.push_back(z_size);
  valid_centroids_.push_back(centroid);
  cluster_std_dev_.push_back(standard_deviation);
  cluster_sizes_.push_back(cluster_seeds.size());

  for (size_t seed_index : cluster_seeds) {
    precluster_cloud->points.push_back(point_cloud_->points[seed_index]);
  }
  std::cout << precluster_cloud->points.size();

  // cluster_seeds is not used after this (reinitialization in the next iteration) thus nothing to worry about
  return DBSCAN_SUCCESS;
}

/* template <typename PointT>
int DBSCAN<PointT>::ExpandCluster(int point_index, int current_cluster_id,
                                  typename pcl::PointCloud<PointT>::Ptr precluster_cloud) {
  std::vector<int> cluster_seeds = CalculateCluster(point_index);
  pcl::PointXYZ centroid = CalculateCentroid(cluster_seeds);
  float standard_deviation = CalculateStdDev(cluster_seeds, centroid);

  if (cluster_seeds.size() < params_.min_pts_dbscan) {
    cluster_ids_vector_[point_index] = NOISE; // Mark the point as noise if it doesn't
                                              // meet the minPoints criterion
    return FAILURE;
  } else if (standard_deviation > params_.max_cluster_std_dev || centroid.z > params_.max_cluster_z) {
    for (int seed_index : cluster_seeds) {
      cluster_ids_vector_[seed_index] = NOISE;
    }
    return FAILURE;
  }

  valid_centroids_.push_back(centroid);
  cluster_std_dev_.push_back(standard_deviation);
  // Set the cluster ID for the core point and its neighbors
  for (int seed_index : cluster_seeds) {
    cluster_ids_vector_[seed_index] = current_cluster_id;
  }

  // Remove the core point from the seed list
  cluster_seeds.erase(std::remove(cluster_seeds.begin(), cluster_seeds.end(), point_index), cluster_seeds.end());

  // Iterate through each seed point, attempting to expand the cluster
  for (size_t i = 0; i < cluster_seeds.size(); ++i) {
    int current_index = cluster_seeds[i];
    std::vector<int> current_neighbors = CalculateCluster(current_index);

    if (current_neighbors.size() >= params_.min_pts_dbscan) {
      // Try to expand the cluster with each neighbor
      for (int neighborIndex : current_neighbors) {
        if (cluster_ids_vector_[neighborIndex] == UNCLASSIFIED || cluster_ids_vector_[neighborIndex] == NOISE) {
          if (cluster_ids_vector_[neighborIndex] == UNCLASSIFIED) {
            // If the neighbor was unclassified, it's now part of the cluster
            cluster_seeds.push_back(neighborIndex); // Add new seed
          }
          cluster_ids_vector_[neighborIndex] = current_cluster_id; // Assign cluster ID
        }
      }
    }
  }
  // adding the cluster size and std dev in order to debug in lidar_pipeline.cpp
  // we removed the core point from the cluster seeds
  cluster_seeds.push_back(point_index);
  standard_deviation = CalculateStdDev(cluster_seeds, centroid);
  if (standard_deviation > params_.max_cluster_std_dev) {
    cluster_ids_vector_[point_index] = NOISE;
    // Mark points as NOISE or handle them differently
    for (int seed_index : cluster_seeds) {
      cluster_ids_vector_[seed_index] = NOISE;
    }
    valid_centroids_.pop_back();
    cluster_std_dev_.pop_back();
    return FAILURE;
  }
  cluster_std_dev_.back() = standard_deviation;
  cluster_sizes_.push_back(cluster_seeds.size() + 1);
  for (size_t seed_index : cluster_seeds) {
    precluster_cloud->points.push_back(point_cloud_->points[seed_index]);
  }
  std::cout << precluster_cloud->points.size();
  // cluster_seeds is not used after this (reinitialization in the next iteration) thus nothing to worry about
  return DBSCAN_SUCCESS;
} */

template <typename PointT> int DBSCAN<PointT>::ExpandCluster(int point_index, int current_cluster_id) {
  // float distance_from_sensor = std::sqrt(point_cloud_->points[point_index].x * point_cloud_->points[point_index].x +
  //                                        point_cloud_->points[point_index].y * point_cloud_->points[point_index].y +
  //                                        point_cloud_->points[point_index].z * point_cloud_->points[point_index].z);
  float x = point_cloud_->points[point_index].x;
  float y = point_cloud_->points[point_index].y;
  float z = point_cloud_->points[point_index].z;
  int min_pts = params_.zones.back().min_pts_dbscan;
  float max_cluster_std = params_.zones.back().max_cluster_std_dev;
  float epsilon = params_.zones.back().eps_dbscan_m;
  for (const auto &zone : params_.zones) {
    if (x <= zone.x && x >= -zone.x && y <= zone.y && y >= -zone.y) {
      min_pts = zone.min_pts_dbscan;
      max_cluster_std = zone.max_cluster_std_dev;
      epsilon = zone.eps_dbscan_m;
      break;
    }
  }
  std::vector<int> cluster_seeds = CalculateCluster(point_index, epsilon);
  // std::vector<int> centroid = CalculateCentroid(cluster_seeds);
  // float standard_deviation = CalculateStdDev(cluster_seeds, centroid);

  if (cluster_seeds.size() < min_pts) {
    cluster_ids_vector_[point_index] = NOISE; // Mark the point as noise if it doesn't
                                              // meet the minPoints criterion
    return FAILURE;
  }
  for (int seed_index : cluster_seeds) {
    cluster_ids_vector_[seed_index] = current_cluster_id;
  }

  // Remove the core point from the seed list
  cluster_seeds.erase(std::remove(cluster_seeds.begin(), cluster_seeds.end(), point_index), cluster_seeds.end());

  // Iterate through each seed point, attempting to expand the cluster
  for (size_t i = 0; i < cluster_seeds.size(); ++i) {
    int current_index = cluster_seeds[i];
    std::vector<int> current_neighbors = CalculateCluster(current_index, epsilon);

    if (current_neighbors.size() >= min_pts) {
      // Try to expand the cluster with each neighbor
      for (int neighborIndex : current_neighbors) {
        if (cluster_ids_vector_[neighborIndex] == UNCLASSIFIED || cluster_ids_vector_[neighborIndex] == NOISE) {
          if (cluster_ids_vector_[neighborIndex] == UNCLASSIFIED) {
            // If the neighbor was unclassified, it's now part of the cluster
            cluster_seeds.push_back(neighborIndex); // Add new seed
          }
          cluster_ids_vector_[neighborIndex] = current_cluster_id; // Assign cluster ID
        }
      }
    }
  }
  // adding the cluster size and std dev in order to debug in lidar_pipeline.cpp
  // we removed the core point from the cluster seeds
  cluster_seeds.push_back(point_index);
  pcl::PointXYZ centroid = CalculateCentroid(cluster_seeds);
  float standard_deviation = CalculateStdDev(cluster_seeds, centroid);
  float z_size = CalculateZSize(cluster_seeds);
  if (standard_deviation > max_cluster_std || centroid.z > params_.max_cluster_z ||
      standard_deviation < params_.min_cluster_std || cluster_seeds.size() > params_.max_pts_dbscan ||
      z_size < params_.min_z_size) {
    cluster_ids_vector_[point_index] = NOISE;
    // Mark points as NOISE or handle them differently
    for (int seed_index : cluster_seeds) {
      cluster_ids_vector_[seed_index] = NOISE;
    }
    return FAILURE;
  }
  valid_centroids_.push_back(centroid);
  cluster_z_sizes_.push_back(z_size);
  cluster_std_dev_.push_back(standard_deviation);
  cluster_sizes_.push_back(cluster_seeds.size());

  // cluster_seeds is not used after this (reinitialization in the next iteration) thus nothing to worry about
  return DBSCAN_SUCCESS;
}

template <typename PointT> std::vector<int> DBSCAN<PointT>::CalculateCluster(int point_index, float epsilon) {
  std::vector<int> cluster_indices;
  std::vector<float> point_radius_squared_distance;
  const auto core_point = point_cloud_->points[point_index]; // The reference point

  int num_points = kd_tree_.radiusSearch(core_point, epsilon, cluster_indices, point_radius_squared_distance);
  cluster_indices.push_back(point_index);
  return cluster_indices;
}
template <typename PointT>
inline double DBSCAN<PointT>::CalculateDistance(const PointT &point_core, const pcl::PointXYZ &centroid) {
  return std::pow(point_core.x - centroid.x, 2) + std::pow(point_core.y - centroid.y, 2) +
         std::pow(point_core.z - centroid.z, 2);
}

template <typename PointT> pcl::PointXYZ DBSCAN<PointT>::CalculateCentroid(const std::vector<int> &cluster_indices) {
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  for (const auto &index : cluster_indices) {
    sumX += point_cloud_->points[index].x;
    sumY += point_cloud_->points[index].y;
    sumZ += point_cloud_->points[index].z;
  }
  int points_number = cluster_indices.size();
  return pcl::PointXYZ(sumX / points_number, sumY / points_number, sumZ / points_number);
}
template <typename PointT>
float DBSCAN<PointT>::CalculateStdDev(const std::vector<int> &cluster_indices, const pcl::PointXYZ &centroid) {
  float sum_squared_distance = 0.0;
  for (const auto &index : cluster_indices) {
    float dist = std::sqrt(CalculateDistance(point_cloud_->points[index], centroid));
    sum_squared_distance += dist * dist;
  }
  float mean_squared_distance = sum_squared_distance / cluster_indices.size();
  return std::sqrt(mean_squared_distance);
}

template <typename PointT> float DBSCAN<PointT>::CalculateZSize(const std::vector<int> &cluster_indices) {
  float z_size = 0.0;
  float z_min = 3.0;
  float z_max = -2.0;
  for (const auto &index : cluster_indices) {
    float z = point_cloud_->points[index].z;
    if (z > z_max) {
      z_max = z;
    }
    if (z < z_min) {
      z_min = z;
    }
  }

  return z_max - z_min;
}

template <typename PointT> std::vector<autonomous_msgs::msg::Cone> DBSCAN<PointT>::GetClusterCentroid() {
  std::vector<autonomous_msgs::msg::Cone> cone_array;
  // Iterate over valid centroids to populate cones
  for (size_t centroid_index = 0; centroid_index < valid_centroids_.size(); ++centroid_index) {
    const auto &centroid = valid_centroids_[centroid_index];
    float standard_deviation = cluster_std_dev_[centroid_index]; // Get the standard deviation of the cluster
    float std_threshold_orange = params_.zones.back().std_threshold_orange;
    // float centroid_distance_from_sensor =
    //     std::sqrt(centroid.x * centroid.x + centroid.y * centroid.y + centroid.z * centroid.z);
    for (const auto &zone : params_.zones) {
      if (centroid.x <= zone.x && centroid.x >= -zone.x && centroid.y <= zone.y && centroid.y >= -zone.y) {
        std_threshold_orange = zone.std_threshold_orange;
        break;
      }
    }
    autonomous_msgs::msg::Cone cone;
    cone.prob_cone = 1.0; // Assuming maximum probability for demonstration

    // Set the centroid position as the cone position
    cone.position.x = centroid.x;
    cone.position.y = centroid.y;
    cone.position.z = centroid.z;
    if (standard_deviation > std_threshold_orange) {
      cone.prob_type.orange = 0.0; // Higher probability for orange cones
      cone.prob_type.blue = 0.0;
      cone.prob_type.yellow = 0.0;
      cone.prob_type.orange_big = 1.0;
    } else {
      cone.prob_type.orange = 0.25;
      cone.prob_type.blue = 0.25;
      cone.prob_type.yellow = 0.25;
      cone.prob_type.orange_big = 0.25;
    }

    cone.is_observed = true; // Assuming the cone is observed for demonstration
    cone.pipeline = 2;

    // Add the populated cone to the cone array
    cone_array.push_back(cone);
  }

  return cone_array;
}
} // namespace lidar_cone_detector

template class lidar_cone_detector::DBSCAN<lidar_cone_detector::LidarPointHesai>;
template class lidar_cone_detector::DBSCAN<lidar_cone_detector::LidarPointOuster>;
