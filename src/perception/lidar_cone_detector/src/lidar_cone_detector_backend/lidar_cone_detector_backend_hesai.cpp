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

#include "lidar_cone_detector_backend/lidar_cone_detector_backend_hesai.hpp"

namespace lidar_cone_detector {

LidarConeDetectorBackendHesai::LidarConeDetectorBackendHesai(const LidarParameters &lidar_params,
                                                             const RuntimeParameters &runtime_params,
                                                             std::shared_ptr<NodeHandle> node_handler)
    : LidarConeDetectorBackendBase(lidar_params, runtime_params, node_handler) {}

void LidarConeDetectorBackendHesai::MotionCompensation(const tf2::TimePoint &destination_time,
                                                       const std::string &frame_id, void *point_cloud) {
  EASY_FUNCTION(profiler::colors::Orange);
  pcl::PointCloud<LidarPointHesai> *cloud = static_cast<pcl::PointCloud<LidarPointHesai> *>(point_cloud);
  uint32_t fails = 0;
  uint32_t success = 0;
  bool transform_available = true;

  tf2::Transform T_F_S_original = tf2::Transform::getIdentity();

  std::optional<geometry_msgs::msg::TransformStamped> msg_T_F_S_original =
      node_handler_->GetTransform(frame_id, runtime_params_.fixed_frame_id, destination_time);
  if (!msg_T_F_S_original.has_value()) {
    // fetch the most recent transformation
    msg_T_F_S_original = node_handler_->GetTransform(runtime_params_.fixed_frame_id, frame_id, tf2::TimePointZero);
    // if still no transformation, do not compensate
    if (!msg_T_F_S_original.has_value()) transform_available = false;
  }

  if (transform_available) {
    tf2::fromMsg(msg_T_F_S_original.value().transform, T_F_S_original);
  }

  tf2::Transform T_S_F_original = T_F_S_original;

  pcl::PointXYZ uncomp_point, comp_point;
  tf2::Transform T_S_original__S_corrected = tf2::Transform::getIdentity();

  for (auto &point : cloud->points) {
    LidarPointHesai compensated_point = point;
    bool can_be_compensated = true;
    uncomp_point.x = point.x;
    uncomp_point.y = point.y;
    uncomp_point.z = point.z;

    // Convert the double to a duration
    std::chrono::duration<double> seconds_duration(point.timestamp);

    // Convert seconds to nanoseconds
    auto nanoseconds_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(seconds_duration);

    tf2::TimePoint point_time = tf2::TimePoint(nanoseconds_duration);

    tf2::Transform T_F_S_correct;
    std::optional<geometry_msgs::msg::TransformStamped> tf_to_stamp =
        node_handler_->GetTransform(runtime_params_.fixed_frame_id, frame_id, point_time);

    if (!tf_to_stamp.has_value()) {
      // if still no transformation, do not compensate
      can_be_compensated = false;
    }

    if (can_be_compensated) {
      tf2::fromMsg(tf_to_stamp.value().transform, T_F_S_correct);
      T_S_original__S_corrected = T_S_F_original * T_F_S_correct;

      tf2::Vector3 uncomp_point_tf2(uncomp_point.x, uncomp_point.y, uncomp_point.z);
      tf2::Vector3 comp_point_tf2 = T_S_original__S_corrected * uncomp_point_tf2;

      comp_point.x = comp_point_tf2.x();
      comp_point.y = comp_point_tf2.y();
      comp_point.z = comp_point_tf2.z();
    }

    point.x = comp_point.x;
    point.y = comp_point.y;
    point.z = comp_point.z;
  }
}

} // namespace lidar_cone_detector
