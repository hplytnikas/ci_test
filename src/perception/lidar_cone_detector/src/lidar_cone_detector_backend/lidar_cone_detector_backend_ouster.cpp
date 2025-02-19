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

#include "lidar_cone_detector_backend/lidar_cone_detector_backend_ouster.hpp"

namespace lidar_cone_detector {

LidarConeDetectorBackendOuster::LidarConeDetectorBackendOuster(const LidarParameters &lidar_params,
                                                               const RuntimeParameters &runtime_params,
                                                               std::shared_ptr<NodeHandle> node_handler)
    : LidarConeDetectorBackendBase(lidar_params, runtime_params, node_handler) {}

void LidarConeDetectorBackendOuster::MotionCompensation(const tf2::TimePoint &destination_time,
                                                        const std::string &frame_id, void *point_cloud) {
  EASY_FUNCTION(profiler::colors::Orange);
  pcl::PointCloud<LidarPointOuster> *cloud = static_cast<pcl::PointCloud<LidarPointOuster> *>(point_cloud);
  // Specific implementation for Ouster
  uint32_t fails = 0;
  uint32_t success = 0;
  bool can_be_compensated = true;

  geometry_msgs::msg::PointStamped uncomp_point, comp_point;

  for (auto &point : cloud->points) {
    uncomp_point.point.x = point.x;
    uncomp_point.point.y = point.y;
    uncomp_point.point.z = point.z;

    auto duration_to_add = std::chrono::nanoseconds(point.t);
    tf2::TimePoint point_time = destination_time + duration_to_add;

    std::optional<geometry_msgs::msg::TransformStamped> tf_to_stamp = node_handler_->GetTransformBetweenTimestamps(
        frame_id, destination_time, frame_id, point_time, runtime_params_.fixed_frame_id);

    if (!tf_to_stamp.has_value()) {
      // fetch the most recent transformation
      tf_to_stamp = node_handler_->GetTransformBetweenTimestamps(frame_id, destination_time, frame_id,
                                                                 tf2::TimePointZero, runtime_params_.fixed_frame_id);
      // if still no transformation, do not compensate
      if (!tf_to_stamp.has_value()) can_be_compensated = false;
    }

    if (can_be_compensated) {
      tf2::doTransform(uncomp_point, comp_point, tf_to_stamp.value());
      point.x = comp_point.point.x;
      point.y = comp_point.point.y;
      point.z = comp_point.point.z;
    }
  }
}

} // namespace lidar_cone_detector
