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
#include "lidar_cone_detector_frontend.hpp"

namespace lidar_cone_detector {

template <typename PointT>
LidarConeDetectorFrontend<PointT>::LidarConeDetectorFrontend(std::shared_ptr<NodeHandle> node_handler)
    : node_handler_(node_handler), debug_num_last_cone_array_(0) {
  // Load the parameter from the node
  LoadParameters();

  // setup callback
  auto point_cloud_callback_ = [this](const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    this->PointCloudCallback(msg);
  };

  // Initialize the subscriber and publisher
  node_handler_->CreatePointCloudSubscriber(topic_lidar_point_, 0, point_cloud_callback_);
  node_handler_->CreatePointCloudCompensatedPublisher(topic_compensated_pc_, 1);
  node_handler_->CreateConeArrayPublisher(topic_cone_array_, 10);
  node_handler_->CreateFilteredPointCloudPublisher(topic_debug_filtered_pc_, 1);
  node_handler_->CreateCentroidPublisher(topic_debug_centroid_dbscan_, 10);

  // inizialize the dbscan class
  dbscan_ = std::make_shared<DBSCAN<PointT>>(lidar_parameters_.cone_detection);

  // Inizialize the lidar cone detector backend
  if (lidar_mode_ == constants::kHesaiLidar)
    lidar_cone_detector_backend_ =
        std::make_shared<LidarConeDetectorBackendHesai>(lidar_parameters_, runtime_parameters_, node_handler_);
  else if (lidar_mode_ == constants::kOusterLidar)
    lidar_cone_detector_backend_ =
        std::make_shared<LidarConeDetectorBackendOuster>(lidar_parameters_, runtime_parameters_, node_handler_);
  else
    throw std::runtime_error("Invalid lidar mode: " + lidar_mode_);
}

template <typename PointT>
void LidarConeDetectorFrontend<PointT>::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  EASY_FUNCTION(profiler::colors::Red);
  if (runtime_parameters_.logging_enabled) {
    RCLCPP_INFO(node_handler_->get_logger(), "[lidar cone detector] Received point cloud message");
  }

  const typename pcl::PointCloud<PointT>::Ptr point_cloud =
      lidar_cone_detector_backend_->ProcessPointCloud<PointT>(msg);

  // Publish the compensated point cloud
  PublishPointCloud(point_cloud, msg->header.stamp, msg->header.frame_id, COMPENSATED);

  if (lidar_parameters_.grid_filter_enabled) {
    FilterGrids(point_cloud);
  }

  // Compute the cluster
  std::vector<autonomous_msgs::msg::Cone> cone_array = dbscan_->ComputeClusters(point_cloud);

  std::vector<autonomous_msgs::msg::Cone> transformed_cones;

  const auto duration = std::chrono::nanoseconds(msg->header.stamp.nanosec);
  if (runtime_parameters_.transform_cone_enabled) {
    transformed_cones = TransformCone(msg->header.frame_id, cone_array_frame_id_, tf2::TimePoint(duration), cone_array);
  } else {
    transformed_cones = cone_array;
  }

  // Publish the cone array
  PublishConeArray(transformed_cones, msg->header.stamp);

  if (runtime_parameters_.visualization_enabled) {
    // Publish the centroid markers
    PublishCentroidMarkers(transformed_cones, msg->header.stamp);
  }
  if (runtime_parameters_.debug_enabled) {
    std::vector<float> std_dev_ = dbscan_->GetClusterStdDev();
    std::vector<int> num_points_ = dbscan_->GetClusterSizes();
    std::vector<float> z_sizes = dbscan_->GetClusterZSize();
    for (size_t i = 0; i < transformed_cones.size(); i++) {
      RCLCPP_INFO(node_handler_->get_logger(), "%f", z_sizes[i]);
      autonomous_msgs::msg::Cone cone = transformed_cones[i];
      float distance = sqrt(pow(cone.position.x, 2) + pow(cone.position.y, 2) + pow(cone.position.z, 2));
      RCLCPP_INFO(node_handler_->get_logger(), "cone frame id: %s", cone_array_frame_id_.c_str());
      // RCLCPP_INFO(node_handler_->get_logger(), "prob_type: orange_big: %f", cone.prob_type.orange_big);
      RCLCPP_INFO(node_handler_->get_logger(),
                  "[lidar cone detector 2] Cone %ld: x: %f, y: %f,z: %f, std_dev: %f, z size: %f, number of points: "
                  "%d, distance: %f",
                  i, cone.position.x, cone.position.y, cone.position.z, std_dev_[i], z_sizes[i], num_points_[i],
                  distance);
    }
  }
}

template <typename PointT>
std::vector<autonomous_msgs::msg::Cone>
LidarConeDetectorFrontend<PointT>::TransformCone(const std::string &source_frame, const std::string &destination_frame,
                                                 const tf2::TimePoint &timestamp,
                                                 const std::vector<autonomous_msgs::msg::Cone> &cone_array) {
  std::vector<autonomous_msgs::msg::Cone> transformed_cones;
  std::optional<geometry_msgs::msg::TransformStamped> transform =
      node_handler_->GetTransform(destination_frame, source_frame, timestamp);

  if (!transform.has_value()) {
    PCL_ERROR("Exception during transform lookup from [%s] to [%s]: %s", source_frame.c_str(),
              destination_frame.c_str());
    return transformed_cones;
  }

  for (auto &cone : cone_array) {
    geometry_msgs::msg::Point cone_pos, transformed_cone_pos;
    autonomous_msgs::msg::Cone new_cone = cone;
    cone_pos.x = cone.position.x;
    cone_pos.y = cone.position.y;
    cone_pos.z = cone.position.z;
    tf2::doTransform(cone_pos, transformed_cone_pos, transform.value());
    new_cone.position.x = transformed_cone_pos.x;
    new_cone.position.y = transformed_cone_pos.y;
    new_cone.position.z = transformed_cone_pos.z;
    transformed_cones.push_back(new_cone);
  }
  return transformed_cones;
}

template <typename PointT>
void LidarConeDetectorFrontend<PointT>::FilterGrids(typename pcl::PointCloud<PointT>::Ptr cloud) {
  EASY_FUNCTION(profiler::colors::Purple);

  // Using a map to store grid cells and their point indices
  std::unordered_map<std::tuple<int, int, int>, std::vector<int>, SimpleHash> grid_map;
  std::vector<int> indices_to_remove;

  // Populate the grid map with points
  for (int i = 0; i < cloud->points.size(); i++) {
    auto point = cloud->points[i];
    int grid_x = static_cast<int>(point.x / lidar_parameters_.rough_clustering_grid_size_m);
    int grid_y = static_cast<int>(point.y / lidar_parameters_.rough_clustering_grid_size_m);
    int grid_z = static_cast<int>(point.z / lidar_parameters_.rough_clustering_grid_size_m);

    std::tuple<int, int, int> grid_key = std::make_tuple(grid_x, grid_y, grid_z);
    grid_map[grid_key].push_back(i);
  }

  // Identify points to remove
  for (const auto &grid_cell : grid_map) {
    if (grid_cell.second.size() > lidar_parameters_.rough_clustering_max_points_per_grid) {
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
void LidarConeDetectorFrontend<PointT>::PublishPointCloud(const typename pcl::PointCloud<PointT>::Ptr point_cloud,
                                                          const rclcpp::Time &timestamp, const std::string &frame_id,
                                                          const PointCloudType &type) {
  sensor_msgs::msg::PointCloud2::SharedPtr msg(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*point_cloud, *msg);
  msg->header.frame_id = frame_id;
  msg->header.stamp = timestamp;
  node_handler_->PublishPointCloud(*msg, type);
}

template <typename PointT>
void LidarConeDetectorFrontend<PointT>::PublishConeArray(const std::vector<autonomous_msgs::msg::Cone> &cone_array,
                                                         const rclcpp::Time &timestamp) {
  // Create the message
  autonomous_msgs::msg::ConeArray cone_array_msg;
  cone_array_msg.header.stamp = timestamp;
  cone_array_msg.header.frame_id = cone_array_frame_id_;

  // Fill the message
  for (const autonomous_msgs::msg::Cone &cone : cone_array) {
    cone_array_msg.cones.push_back(cone);
  }

  // Publish the message
  node_handler_->PublishConeArray(cone_array_msg);
}

template <typename PointT>
void LidarConeDetectorFrontend<PointT>::PublishCentroidMarkers(
    const std::vector<autonomous_msgs::msg::Cone> &cone_array, const rclcpp::Time &timestamp) {
  visualization_msgs::msg::MarkerArray marker_array;
  int markerId = 0;
  for (const autonomous_msgs::msg::Cone &cone : cone_array) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = cone_array_frame_id_;
    marker.header.stamp = timestamp;
    marker.ns = "centroids";
    marker.id = markerId++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cone.position.x;
    marker.pose.position.y = cone.position.y;
    marker.pose.position.z = cone.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    if (cone.prob_type.orange_big > 0.9) {
      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    }
    marker.color.a = 1.0;
    marker_array.markers.push_back(marker);
  }

  if (markerId < debug_num_last_cone_array_) {
    for (int i = markerId; i < debug_num_last_cone_array_; ++i) {
      visualization_msgs::msg::Marker delete_marker;
      delete_marker.header.frame_id = cone_array_frame_id_;
      delete_marker.id = i;
      delete_marker.ns = "centroids";
      delete_marker.action = visualization_msgs::msg::Marker::DELETE;
      marker_array.markers.push_back(delete_marker);
    }
  }

  debug_num_last_cone_array_ = markerId;

  node_handler_->PublishCentroid(marker_array);
}

template <typename PointT> void LidarConeDetectorFrontend<PointT>::LoadParameters() {
  RCLCPP_INFO(node_handler_->get_logger(), "[lidar cone detector] Loading Topics");

  // Load the parameters from the node
  lidar_mode_ = GetParam<std::string>("lidar_mode");

  // Load the parameters from the node
  cone_array_frame_id_ = GetParam<std::string>(lidar_mode_ + ".frame_id.cone_array");
  topic_debug_filtered_pc_ = GetParam<std::string>(lidar_mode_ + ".topics.publisher.topic_debug_filtered_pc");
  topic_cone_array_ = GetParam<std::string>(lidar_mode_ + ".topics.publisher.topic_cone_array");
  topic_compensated_pc_ = GetParam<std::string>(lidar_mode_ + ".topics.publisher.topic_compensated_pc");
  topic_lidar_point_ = GetParam<std::string>(lidar_mode_ + ".topics.subscriber.topic_lidar_point");
  topic_debug_centroid_dbscan_ = GetParam<std::string>(lidar_mode_ + ".topics.publisher.topic_debug_centroid_dbscan");

  LoadRuntimeParameters();
  LoadLidarParameter();
}

template <typename PointT> void LidarConeDetectorFrontend<PointT>::LoadRuntimeParameters() {
  RCLCPP_INFO(node_handler_->get_logger(), "[lidar cone detector] Loading runtime parameters");

  runtime_parameters_.fixed_frame_id = GetParam<std::string>(lidar_mode_ + ".frame_id.fixed");
  runtime_parameters_.motion_compensation_enabled = GetParam<bool>(lidar_mode_ + ".motion_compenation.enabled");
  runtime_parameters_.transform_cone_enabled = GetParam<bool>(lidar_mode_ + ".transform_cone.enabled");
  runtime_parameters_.downsampling_enabled = GetParam<bool>(lidar_mode_ + ".downsampling.enabled");
  runtime_parameters_.filter_enabled = GetParam<bool>(lidar_mode_ + ".filter.enabled");
  runtime_parameters_.profiling_enabled = GetParam<bool>(lidar_mode_ + ".profiling.enabled");
  runtime_parameters_.logging_enabled = GetParam<bool>(lidar_mode_ + ".logging.enabled");
  runtime_parameters_.visualization_enabled = GetParam<bool>(lidar_mode_ + ".visualization.enabled");
  runtime_parameters_.debug_enabled = GetParam<bool>(lidar_mode_ + ".debug.enabled");
}

template <typename PointT> void LidarConeDetectorFrontend<PointT>::LoadLidarParameter() {
  RCLCPP_INFO(node_handler_->get_logger(), "[lidar cone detector] Loading lidar parameters");

  lidar_parameters_.x_min_m = GetParam<double>(lidar_mode_ + ".parameters.x_min");
  lidar_parameters_.x_max_m = GetParam<double>(lidar_mode_ + ".parameters.x_max");
  lidar_parameters_.y_min_m = GetParam<double>(lidar_mode_ + ".parameters.y_min");
  lidar_parameters_.y_max_m = GetParam<double>(lidar_mode_ + ".parameters.y_max");
  lidar_parameters_.z_min_m = GetParam<double>(lidar_mode_ + ".parameters.z_min");
  lidar_parameters_.z_max_m = GetParam<double>(lidar_mode_ + ".parameters.z_max");
  lidar_parameters_.x_min_car_m = GetParam<double>(lidar_mode_ + ".parameters.x_min_car");
  lidar_parameters_.x_max_car_m = GetParam<double>(lidar_mode_ + ".parameters.x_max_car");
  lidar_parameters_.y_min_car_m = GetParam<double>(lidar_mode_ + ".parameters.y_min_car");
  lidar_parameters_.y_max_car_m = GetParam<double>(lidar_mode_ + ".parameters.y_max_car");

  lidar_parameters_.downsampling_ratio = GetParam<double>(lidar_mode_ + ".downsampling.ratio");
  lidar_parameters_.grid_num_x = GetParam<int>(lidar_mode_ + ".parameters.grid_num_x");
  lidar_parameters_.grid_num_y = GetParam<int>(lidar_mode_ + ".parameters.grid_num_y");
  lidar_parameters_.distance_threshold_near_ransac_m =
      GetParam<double>(lidar_mode_ + ".parameters.distance_threshold_ransac_near");
  lidar_parameters_.distance_threshold_far_ransac_m =
      GetParam<double>(lidar_mode_ + ".parameters.distance_threshold_ransac_far");
  lidar_parameters_.distance_threshold_far_near_m =
      GetParam<double>(lidar_mode_ + ".parameters.distance_threshold_far_near");
  lidar_parameters_.min_fraction_inlier = GetParam<double>(lidar_mode_ + ".parameters.min_fraction_inlier");

  lidar_parameters_.max_iteration = GetParam<int>(lidar_mode_ + ".ground_removal.max_iteration");

  lidar_parameters_.min_point_in_segment = GetParam<int>(lidar_mode_ + ".parameters.min_point_in_segment");
  lidar_parameters_.rain = GetParam<bool>(lidar_mode_ + ".parameters.rain");

  lidar_parameters_.grid_filter_enabled = GetParam<bool>(lidar_mode_ + ".parameters.grid_filter_enabled");
  lidar_parameters_.rough_clustering_grid_size_m =
      GetParam<double>(lidar_mode_ + ".parameters.rough_clustering_grid_size_m");
  lidar_parameters_.rough_clustering_max_points_per_grid =
      GetParam<int>(lidar_mode_ + ".parameters.rough_clustering_max_points_per_grid");

  lidar_parameters_.cone_detection.enabled = GetParam<bool>(lidar_mode_ + ".parameters.cone_detection.enabled");
  lidar_parameters_.cone_detection.max_pts_dbscan =
      GetParam<int>(lidar_mode_ + ".parameters.cone_detection.max_points_dbscan");
  lidar_parameters_.cone_detection.max_cluster_z =
      GetParam<double>(lidar_mode_ + ".parameters.cone_detection.max_cluster_z");
  lidar_parameters_.cone_detection.min_z_size = GetParam<double>(lidar_mode_ + ".parameters.cone_detection.min_z_size");
  lidar_parameters_.cone_detection.min_cluster_std =
      GetParam<double>(lidar_mode_ + ".parameters.cone_detection.min_cluster_std");

  std::vector<double> x_array = GetParam<std::vector<double>>(lidar_mode_ + ".parameters.cone_detection.zones.x");
  std::vector<double> y_array = GetParam<std::vector<double>>(lidar_mode_ + ".parameters.cone_detection.zones.y");
  std::vector<double> eps_dbscan_array =
      GetParam<std::vector<double>>(lidar_mode_ + ".parameters.cone_detection.zones.eps_dbscan_m");
  std::vector<int64_t> min_pts_dbscan_array =
      GetParam<std::vector<int64_t>>(lidar_mode_ + ".parameters.cone_detection.zones.min_points_dbscan");
  std::vector<double> std_threshold_orange_array =
      GetParam<std::vector<double>>(lidar_mode_ + ".parameters.cone_detection.zones.std_threshold_orange");
  std::vector<double> max_cluster_std_dev_array =
      GetParam<std::vector<double>>(lidar_mode_ + ".parameters.cone_detection.zones.max_cluster_std_dev");

  std::vector<Zone> zones;
  for (size_t i = 0; i < x_array.size(); ++i) {
    Zone zone;
    zone.x = x_array[i];
    zone.y = y_array[i];
    zone.eps_dbscan_m = eps_dbscan_array[i];
    zone.min_pts_dbscan = min_pts_dbscan_array[i];
    zone.std_threshold_orange = std_threshold_orange_array[i];
    zone.max_cluster_std_dev = max_cluster_std_dev_array[i];
    zones.push_back(zone);
  }
  lidar_parameters_.cone_detection.zones = zones;

  lidar_parameters_.pre_cluster.enabled = GetParam<bool>(lidar_mode_ + ".parameters.pre_cluster.enabled");
  lidar_parameters_.pre_cluster.max_pts_dbscan =
      GetParam<int>(lidar_mode_ + ".parameters.pre_cluster.max_points_dbscan");
  lidar_parameters_.pre_cluster.max_cluster_z = GetParam<double>(lidar_mode_ + ".parameters.pre_cluster.max_cluster_z");
  lidar_parameters_.pre_cluster.min_cluster_std =
      GetParam<double>(lidar_mode_ + ".parameters.pre_cluster.min_cluster_std");
  lidar_parameters_.pre_cluster.min_z_size = GetParam<double>(lidar_mode_ + ".parameters.pre_cluster.min_z_size");

  std::vector<double> pre_cluster_x_array =
      GetParam<std::vector<double>>(lidar_mode_ + ".parameters.cone_detection.zones.x");
  std::vector<double> pre_cluster_y_array =
      GetParam<std::vector<double>>(lidar_mode_ + ".parameters.cone_detection.zones.y");
  std::vector<double> pre_cluster_eps_dbscan_array =
      GetParam<std::vector<double>>(lidar_mode_ + ".parameters.pre_cluster.zones.eps_dbscan_m");
  std::vector<int64_t> pre_cluster_min_pts_dbscan_array =
      GetParam<std::vector<int64_t>>(lidar_mode_ + ".parameters.pre_cluster.zones.min_points_dbscan");
  std::vector<double> pre_cluster_std_threshold_orange_array =
      GetParam<std::vector<double>>(lidar_mode_ + ".parameters.pre_cluster.zones.std_threshold_orange");
  std::vector<double> pre_cluster_max_cluster_std_dev_array =
      GetParam<std::vector<double>>(lidar_mode_ + ".parameters.pre_cluster.zones.max_cluster_std_dev");

  std::vector<Zone> pre_cluster_zones;
  for (size_t i = 0; i < pre_cluster_x_array.size(); ++i) {
    Zone zone;
    zone.x = pre_cluster_x_array[i];
    zone.y = pre_cluster_y_array[i];
    zone.eps_dbscan_m = pre_cluster_eps_dbscan_array[i];
    zone.min_pts_dbscan = pre_cluster_min_pts_dbscan_array[i];
    zone.std_threshold_orange = pre_cluster_std_threshold_orange_array[i];
    zone.max_cluster_std_dev = pre_cluster_max_cluster_std_dev_array[i];
    pre_cluster_zones.push_back(zone);
  }
  lidar_parameters_.pre_cluster.zones = pre_cluster_zones;
}

/*
Safe ROS node parameter loading
*/
template <typename PointT>
template <typename Type>
Type LidarConeDetectorFrontend<PointT>::GetParam(const std::string &name) {
  Type val;
  std::string name_replaced = name;
  std::replace(name_replaced.begin(), name_replaced.end(), '/', '.');
  if (!node_handler_->get_parameter(name_replaced, val)) {
    RCLCPP_ERROR(node_handler_->get_logger(), "PARAMETER NOT FOUND: %s", name_replaced.c_str());
    rclcpp::shutdown();
  }

  return val;
}

} // namespace lidar_cone_detector

template class lidar_cone_detector::LidarConeDetectorFrontend<lidar_cone_detector::LidarPointHesai>;
template class lidar_cone_detector::LidarConeDetectorFrontend<lidar_cone_detector::LidarPointOuster>;
