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

#pragma once

#include "control_msgs/msg/mpc_log.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class MpcVisualisationNode : public rclcpp::Node {
public:
  /**
   * @brief destructor of the MPC visualisation node
   */
  ~MpcVisualisationNode() {}

  /**
   * @brief constructor of the MPC visualisation node
   *
   * @param node_name name of the node
   */
  explicit MpcVisualisationNode(std::string node_name);

  /**
   * @brief publishes the prediction horizon
   *
   * @param points vector of points that represent the prediction horizon in vehicle frame
   */
  void SharePredictionHorizon(const Eigen::MatrixXd &poses);

  /**
   * @brief pusblishes the fitted path of the MPC (which it needs to do to get the curvature of the path)
   *
   * @param points vector of points that represent the fitted path in vehicle frame
   */
  void ShareFittedPath(const Eigen::MatrixXd &points);

  /**
   * @brief publishes the logging message of the MPC
   *
   * @param message logging message of the MPC
   */
  void ShareMpcLogging(control_msgs::msg::MpcLog &message);

  /**
   * @brief publishes a marker arrow in the vehicle frame
   * that goes from the reference line point (x, y) to the point defined
   * by the normal vector on the reference path
   * and the lateral deviation n
   *
   * (the tangent vector to the path is defined by (x,y) as starting point and theta as angle)
   *
   * @param x marker arrow start coordinate in vehicle frame
   * @param y marker arrow start coordinate in vehicle frame
   * @param theta angle of the tangent vector to the path
   * @param n lateral deviation from the path
   */
  void ShareReferenceProjection(double x, double y, double theta, double n);

private:
  // --- PUBLISHERS
  // Prediction horizon is published as a marker array as we also want to rotate the markers
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> prediction_horizon_publisher_;
  std::string prediction_horizon_topic_;

  // Fitted path is published as a point cloud
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> fitted_path_publisher_;
  std::string fitted_path_topic_;

  // Reference projection is published as a marker arrow poiniting from the reference line to the car
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> ref_project_publisher_;
  std::string ref_project_topic_;

  // MPC Logs are published as a custom message
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::MpcLog>> mpc_logging_publisher_;
  std::string mpc_logging_topic_;
};
