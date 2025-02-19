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

#include <memory>
#include <string>

#include "lidar_cone_detector_backend/lidar_cone_detector_backend_base.hpp"

namespace lidar_cone_detector {

class LidarConeDetectorBackendHesai : public LidarConeDetectorBackendBase {
public:
  LidarConeDetectorBackendHesai(const LidarParameters &lidar_params, const RuntimeParameters &runtime_params,
                                std::shared_ptr<NodeHandle> node_handler);

  void MotionCompensation(const tf2::TimePoint &destination_time, const std::string &frame_id,
                          void *point_cloud) override;

  void transformMsgToEigen(const geometry_msgs::msg::Transform &transform_msg,
                           Eigen::Affine3f &transform) { // NOLINT
    transform =
        Eigen::Translation3f(transform_msg.translation.x, transform_msg.translation.y, transform_msg.translation.z) *
        Eigen::Quaternionf(transform_msg.rotation.w, transform_msg.rotation.x, transform_msg.rotation.y,
                           transform_msg.rotation.z);
  }
};

} // namespace lidar_cone_detector
