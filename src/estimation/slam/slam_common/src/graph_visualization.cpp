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

#include "slam_common/graph_visualization.hpp"

namespace slam {

GraphVisualization::GraphVisualization() {}

GraphVisualization::~GraphVisualization() {}

void GraphVisualization::AddPose(double x, double y) {
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;

  size_t pose_index = odometry_points_.size();
  size_t previous_index = 0;
  if (pose_index > 0) {
    previous_index = odometry_points_.size() - 1;
  } else {
    // First edge will be skipped when prior factor will be removed
    // Nothing
  }
  // Add edge
  odometry_edges_.push_back(std::make_pair(previous_index, pose_index));

  odometry_points_.push_back(point);
}

void GraphVisualization::AddLandmark(size_t landmark_id, double x, double y) {
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;

  size_t landmark_index = landmark_points_.size();
  if (landmark_id < landmark_points_.size()) {
    // Landmark aready exists
    landmark_index = landmark_id;
    landmark_points_[landmark_index] = point;
  } else { // New landmark
    landmark_points_.push_back(point);
  }

  // Latest pose index
  int pose_index = odometry_points_.size() - 1;
  // Add edge
  if (pose_index >= 0) {
    landmark_edges_.push_back(std::make_pair(pose_index, landmark_index));
  }
}

void GraphVisualization::PopOdometryEdge() { odometry_edges_.pop_front(); }

void GraphVisualization::PopLandmarkEdge() { landmark_edges_.pop_front(); }

std::vector<geometry_msgs::msg::Point> GraphVisualization::OdometryPoints() const { return odometry_points_; }

std::vector<geometry_msgs::msg::Point> GraphVisualization::LandmarkPoints() const { return landmark_points_; }

std::list<std::pair<int, int>> GraphVisualization::OdometryEdges() const { return odometry_edges_; }

std::list<std::pair<int, int>> GraphVisualization::LandmarkEdges() const { return landmark_edges_; }

} // end namespace slam
