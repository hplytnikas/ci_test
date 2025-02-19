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

#include <autonomous_msgs/msg/point_with_confidence.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <gtsam/geometry/Pose2.h>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace slam {

/*
 * Lightweight data structure for storing a SLAM pose
 */
class StampedPose {
public:
  /*
   * Constructor
   */
  StampedPose(const double x, const double y, const double theta, const rclcpp::Time &timestamp)
      : x_(x), y_(y), theta_(theta), timestamp_(timestamp) {}

  /*
   * Default constructor
   */
  StampedPose() : StampedPose(0, 0, 0, rclcpp::Time(0)) {}

  /*
   * Constructor from gtsam::Pose2
   */
  StampedPose(const gtsam::Pose2 &pose, const rclcpp::Time &timestamp)
      : StampedPose(pose.x(), pose.y(), pose.theta(), timestamp) {}

  // Returns the pose in the format of geometry_msgs::msg::Pose2D
  geometry_msgs::msg::Pose2D AsPose2D() const {
    geometry_msgs::msg::Pose2D result;
    result.x = x_;
    result.y = y_;
    result.theta = theta_;
    return result;
  }

  // Returns the pose x, y in the format of geometry_msgs::msg::Point
  geometry_msgs::msg::Point AsPoint() const {
    geometry_msgs::msg::Point result;
    result.x = x_;
    result.y = y_;
    return result;
  }

  // Returns the pose in the GTSAM format of gtsam::Pose2
  gtsam::Pose2 AsPose2() const { return gtsam::Pose2(x_, y_, theta_); }

  // Returns a hash for the stamped pose
  uint64_t Hash() const { return timestamp_.nanoseconds(); }

  // String representation of the pose
  std::string ToString() const {
    std::ostringstream string_stream;
    string_stream << "StampedPose(";
    string_stream << std::floor(x_ * 100) / 100 << ", ";
    string_stream << std::floor(y_ * 100) / 100 << ", ";
    string_stream << std::floor(theta_ * 100) / 100 << ", ";
    string_stream << Hash() << ")";
    return string_stream.str();
  }

  /*
   * Returns true if the (x, y, theta) of the caller are each within
   * a given threshold (in absolute value) of the other StampedPose
   */
  bool IsApprox(const StampedPose &other, double threshold) const {
    return std::abs(x_ - other.X()) <= threshold && std::abs(y_ - other.Y()) <= threshold &&
           std::abs(theta_ - other.Theta()) <= threshold;
  }

  // Getter for timestamp_
  rclcpp::Time Timestamp() const { return timestamp_; } // Use rclcpp::Time

  // Getter for x_
  double X() const { return x_; }

  // Getter for y_
  double Y() const { return y_; }

  // Getter for theta_
  double Theta() const { return theta_; }

  // Returns a pose with all properties initialized to zero
  static StampedPose ZeroPose() { return StampedPose(0, 0, 0, rclcpp::Time(0)); }

  // Returns true if the pose is zero
  bool IsZeroPose() { return (x_ == 0 && y_ == 0 && theta_ == 0); }

  /*
   * Returns the difference vector between this and another pose. The returned
   * StampedPose inherits the timestamp from this pose.
   */
  StampedPose Delta(const StampedPose &other) const {
    return StampedPose(other.AsPose2().between(AsPose2()), timestamp_);
  }

  /*
   * Returns the composition between this and another pose. The returned
   * StampedPose inherits the timestamp from the other pose.
   */
  StampedPose Compose(const StampedPose &other) const {
    return StampedPose(AsPose2().compose(other.AsPose2()), other.timestamp_);
  }

private:
  // The (x, y, theta) parameterizing the 2D pose
  double x_, y_, theta_;
  // Time at which the pose was recorded
  rclcpp::Time timestamp_;
}; // end class StampedPose

} // end namespace slam
