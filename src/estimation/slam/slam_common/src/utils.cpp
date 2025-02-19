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
#include "slam_common/utils.hpp"

namespace slam {

namespace utils {

tf2::Transform Pose2DToTfTransform(const geometry_msgs::msg::Pose2D &pose) {
  tf2::Transform transform;
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(0.0, 0.0, pose.theta); // Convert theta to quaternion

  transform.setOrigin(tf2::Vector3(pose.x, pose.y, 0.0));
  transform.setRotation(tf_quat);

  return transform;
}

geometry_msgs::msg::Pose2D TransformToPose2D(const geometry_msgs::msg::Transform &transform) {
  geometry_msgs::msg::Pose2D pose;

  // Convert translation
  pose.x = transform.translation.x;
  pose.y = transform.translation.y;

  // Convert rotation (quaternion)
  tf2::Quaternion quat(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  pose.theta = yaw;

  return pose;
}

geometry_msgs::msg::Transform TfTransformToGeometryMsg(const tf2::Transform &tf_transform) {
  geometry_msgs::msg::Transform geom_transform;

  // Convert translation
  geom_transform.translation.x = tf_transform.getOrigin().x();
  geom_transform.translation.y = tf_transform.getOrigin().y();
  geom_transform.translation.z = tf_transform.getOrigin().z();

  // Convert rotation (quaternion)
  geom_transform.rotation.x = tf_transform.getRotation().x();
  geom_transform.rotation.y = tf_transform.getRotation().y();
  geom_transform.rotation.z = tf_transform.getRotation().z();
  geom_transform.rotation.w = tf_transform.getRotation().w();

  return geom_transform;
}

geometry_msgs::msg::Transform Pose2DToGeometryMsg(const geometry_msgs::msg::Pose2D &pose) {
  return TfTransformToGeometryMsg(Pose2DToTfTransform(pose));
}

ConeMap TransformMap(const ConeMap &map, const geometry_msgs::msg::Pose2D &transform) {
  ConeMap result = {};

  double x = transform.x;
  double y = transform.y;
  double theta = transform.theta;
  double s_theta = std::sin(theta);
  double c_theta = std::cos(theta);

  // For each cone in map
  for (const auto &cone : map) {
    autonomous_msgs::msg::Cone transformed_cone = cone;
    double cone_x = cone.position.x;
    double cone_y = cone.position.y;

    // Transform cone
    transformed_cone.position.x = (c_theta * cone_x) - (s_theta * cone_y) + x;
    transformed_cone.position.y = (s_theta * cone_x) + (c_theta * cone_y) + y;

    result.push_back(transformed_cone);
  }

  return result;
}

std::string GetCurrentTimeString() {
  // Get current time
  std::time_t now = std::time(nullptr);
  // Format time to a string "YYYY_MM_DD-HH_MM"
  char buf[19];
  std::strftime(buf, sizeof(buf), "%Y_%m_%d-%H_%M", std::localtime(&now));
  return std::string(buf);
}

} // end namespace utils

} // end namespace slam
