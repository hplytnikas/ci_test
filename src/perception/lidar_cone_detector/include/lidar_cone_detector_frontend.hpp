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
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "lidar_cone_detector_backend/lidar_cone_detector_backend_base.hpp"
#include "lidar_cone_detector_backend/lidar_cone_detector_backend_hesai.hpp"
#include "lidar_cone_detector_backend/lidar_cone_detector_backend_ouster.hpp"
#include "lidar_cone_detector_dbscan.hpp"
#include "lidar_cone_detector_node.hpp"
#include "lidar_cone_detector_structs.hpp"

#include "autonomous_msgs/msg/cone.hpp"
#include "autonomous_msgs/msg/cone_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "easy/profiler.h"

namespace lidar_cone_detector {

class LidarConeDetectorFrontendBase {
public:
  virtual ~LidarConeDetectorFrontendBase() = default;
};

template <typename PointT> class LidarConeDetectorFrontend : public virtual LidarConeDetectorFrontendBase {
public:
  explicit LidarConeDetectorFrontend(std::shared_ptr<NodeHandle> nodeHandle);

private:
  // Methods
  template <class Type> Type GetParam(const std::string &name);

  void LoadParameters();

  void LoadRuntimeParameters();

  void LoadLidarParameter();

  std::vector<autonomous_msgs::msg::Cone> TransformCone(const std::string &source_frame,
                                                        const std::string &destination_frame,
                                                        const tf2::TimePoint &timestamp,
                                                        const std::vector<autonomous_msgs::msg::Cone> &cone_array);

  void FilterGrids(typename pcl::PointCloud<PointT>::Ptr cloud);

  void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void PublishPointCloud(const typename pcl::PointCloud<PointT>::Ptr point_cloud, const rclcpp::Time &timestamp,
                         const std::string &frame_id, const PointCloudType &type);

  void PublishConeArray(const std::vector<autonomous_msgs::msg::Cone> &cone_array, const rclcpp::Time &timestamp);

  void PublishCentroidMarkers(const std::vector<autonomous_msgs::msg::Cone> &cone_array, const rclcpp::Time &timestamp);

  // Attributes
  std::shared_ptr<NodeHandle> node_handler_;
  std::shared_ptr<LidarConeDetectorBackendBase> lidar_cone_detector_backend_;
  std::shared_ptr<DBSCAN<PointT>> dbscan_;

  std::string lidar_mode_;
  std::string topic_lidar_point_;
  std::string topic_cone_array_;
  std::string topic_compensated_pc_;
  std::string topic_debug_filtered_pc_;
  std::string cone_array_frame_id_;
  std::string topic_debug_centroid_dbscan_;

  RuntimeParameters runtime_parameters_;
  LidarParameters lidar_parameters_;

  bool debug_;

  int debug_num_last_cone_array_;

  std::string lidar_type_;
  std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> point_cloud_callback_;
};

} // namespace lidar_cone_detector

extern template class lidar_cone_detector::LidarConeDetectorFrontend<lidar_cone_detector::LidarPointHesai>;
extern template class lidar_cone_detector::LidarConeDetectorFrontend<lidar_cone_detector::LidarPointOuster>;
