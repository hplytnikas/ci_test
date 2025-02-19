/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Diego Garcia Soto <digarcia@ethz.ch>
 *   - Hironobu Akiyama <hakiyama@ethz.ch>
 *   - Jonas Ohnemus <johnemus@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "mpc_visualisation_node/mpc_visualisation_node.hpp"

MpcVisualisationNode::MpcVisualisationNode(std::string node_name) : Node(node_name) {
  // Prediciton horizon
  this->declare_parameter("prediction_horizon_topic", "/control/prediction_horizon");
  prediction_horizon_topic_ = this->get_parameter("prediction_horizon_topic").as_string();
  prediction_horizon_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(prediction_horizon_topic_, 1);

  // Fitted path
  this->declare_parameter("fitted_path_topic", "/control/fitted_path");
  fitted_path_topic_ = this->get_parameter("fitted_path_topic").as_string();
  fitted_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(fitted_path_topic_, 1);

  // MPC logging
  this->declare_parameter("mpc_logging_topic", "/control/mpc_logging");
  mpc_logging_topic_ = this->get_parameter("mpc_logging_topic").as_string();
  mpc_logging_publisher_ = this->create_publisher<control_msgs::msg::MpcLog>(mpc_logging_topic_, 1);

  // Reference projection
  this->declare_parameter("ref_project_topic", "/control/ref_projection");
  ref_project_topic_ = this->get_parameter("ref_project_topic").as_string();
  ref_project_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(ref_project_topic_, 1);
}

void MpcVisualisationNode::ShareReferenceProjection(double x, double y, double theta, double n) {
  visualization_msgs::msg::Marker projection_marker;
  projection_marker.header.frame_id = "base_link";
  projection_marker.header.stamp = this->get_clock()->now();
  projection_marker.ns = "reference_projection";
  projection_marker.id = 0;
  projection_marker.type = visualization_msgs::msg::Marker::ARROW;
  projection_marker.action = visualization_msgs::msg::Marker::ADD;
  projection_marker.pose.position.x = x;
  projection_marker.pose.position.y = y;
  projection_marker.pose.position.z = 0.0;

  // based on the unit vector from (x, y) along the angle theta
  // the arrow will be drawn orthogonal to it (positive rotation by 90 degrees in 2D)
  // to have the normal vector to the reference line
  tf2::Quaternion q;
  // n may be zero or negative, and depending on the sign, the normal vector will be in one direction or the other
  if (n >= 0) {
    q.setRPY(0, 0, theta + M_PI / 2);
  } else {
    q.setRPY(0, 0, theta - M_PI / 2);
  }
  projection_marker.pose.orientation.x = q.x();
  projection_marker.pose.orientation.y = q.y();
  projection_marker.pose.orientation.z = q.z();
  projection_marker.pose.orientation.w = q.w();

  // the scale of the arrow should be n if it is non-zero
  projection_marker.scale.x = std::max(std::abs(n), 0.05);
  projection_marker.scale.y = 0.5;
  projection_marker.scale.z = 0.5;
  projection_marker.color.r = 0.1;
  projection_marker.color.g = 0.8;
  projection_marker.color.b = 0.8;
  projection_marker.color.a = 0.7;

  ref_project_publisher_->publish(projection_marker);
}

void MpcVisualisationNode::SharePredictionHorizon(const Eigen::MatrixXd &poses) {
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  // iterate over the poses and add them to the markerarray as markers
  // set the id of the marker to the index of the pose
  for (size_t i = 0; i < poses.cols(); i++) {
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "prediction_horizon";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = poses.col(i)[0];
    marker.pose.position.y = poses.col(i)[1];
    marker.pose.position.z = 0.0;
    // convert the angle to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, poses.col(i)[2]);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.8;
    marker.color.g = 0.0;
    marker.color.b = 0.8;
    marker.color.a = 0.7;

    marker_array.markers.push_back(marker);
  }

  prediction_horizon_publisher_->publish(marker_array);
}

void MpcVisualisationNode::ShareFittedPath(const Eigen::MatrixXd &points) {
  nav_msgs::msg::Path path_msg;

  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = "base_link";

  path_msg.poses.resize(points.cols());
  for (size_t i = 0; i < points.cols(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = points.col(i)[0];
    pose.pose.position.y = points.col(i)[1];
    pose.pose.position.z = 0;
    // get orientation from the angle from point i to point i+1
    double yaw = 0;
    if (i < points.cols() - 1) {
      yaw = std::atan2(points.col(i + 1)[1] - points.col(i)[1], points.col(i + 1)[0] - points.col(i)[0]);
    } else {
      yaw = std::atan2(points.col(i)[1] - points.col(i - 1)[1], points.col(i)[0] - points.col(i - 1)[0]);
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path_msg.poses[i] = pose;
  }

  fitted_path_publisher_->publish(path_msg);
}

void MpcVisualisationNode::ShareMpcLogging(control_msgs::msg::MpcLog &message) {
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "base_link";
  mpc_logging_publisher_->publish(message);
}
