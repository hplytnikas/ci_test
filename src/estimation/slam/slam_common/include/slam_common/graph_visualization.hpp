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

#include <geometry_msgs/msg/point.hpp>

#include <list>
#include <utility>
#include <vector>

#include "slam_common/aliases.hpp"
#include "slam_common/landmark.hpp"
#include "slam_common/stamped_pose.hpp"

namespace slam {

/*
 * Class for handling the structure of the graph for visualization.
 * It runs in parallel with the normal graph. All operations must
 * be done in both classes.
 */
class GraphVisualization {
public:
  /*
   * Constructor.
   */
  GraphVisualization();

  /*
   * Destructor.
   */
  ~GraphVisualization();

  // Add a pose point
  void AddPose(double x, double y);

  // Add a landmark point
  void AddLandmark(size_t landmark_id, double x, double y);

  // Pop oldest odometry edge
  void PopOdometryEdge();

  // Pop oldest landmark edge
  void PopLandmarkEdge();

  // Returns a vector of all pose points
  std::vector<geometry_msgs::msg::Point> OdometryPoints() const;
  // Returns a vector of all landmark points
  std::vector<geometry_msgs::msg::Point> LandmarkPoints() const;
  // Returns a list of pose-pose edges
  std::list<std::pair<int, int>> OdometryEdges() const;
  // Returns a list of pose-landmark edges
  std::list<std::pair<int, int>> LandmarkEdges() const;
  // Returns the number of pose-pose edges
  inline size_t NumOdometryEdges() const { return odometry_edges_.size(); }
  // Returns the number of pose-landmark edges
  inline size_t NumLandmarkEdges() const { return landmark_edges_.size(); }

private:
  // Book-keeping structures
  // Keeps track of pose points
  std::vector<geometry_msgs::msg::Point> odometry_points_;
  // Keeps track of landmark points
  std::vector<geometry_msgs::msg::Point> landmark_points_;
  // Keeps track of pose edges (previous to current)
  std::list<std::pair<int, int>> odometry_edges_;
  // Keeps track of pose to landmark edges (pose to landmark)
  std::list<std::pair<int, int>> landmark_edges_;
}; // end class GraphVisualization

} // end namespace slam
