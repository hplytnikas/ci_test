/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Christoforos Nicolaou <cnicolaou@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#pragma once

#include <ctime>
#include <string>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include "slam_common/aliases.hpp"

namespace slam {

namespace utils {

/*
 * Convert 2D pose to tf2 Transform object
 */
tf2::Transform Pose2DToTfTransform(const geometry_msgs::msg::Pose2D &pose);

/*
 * Convert tf geometry msg Transform object to 2D pose
 */
geometry_msgs::msg::Pose2D TransformToPose2D(const geometry_msgs::msg::Transform &transform);

/*
 * Convert tf2 Transform object to geometry msg Transform
 */
geometry_msgs::msg::Transform TfTransformToGeometryMsg(const tf2::Transform &tf_transform);

/*
 * Convert 2D pose to geometry msg Transform
 */
geometry_msgs::msg::Transform Pose2DToGeometryMsg(const geometry_msgs::msg::Pose2D &pose);

/*
 * Transform map using transformation from a 2D pose (transform)
 */
ConeMap TransformMap(const ConeMap &map, const geometry_msgs::msg::Pose2D &transform);

/*
 * Get a string for the current datetime
 */
std::string GetCurrentTimeString();

} // end namespace utils

} // end namespace slam
