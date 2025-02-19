/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2019-2023 Authors:
 *   - Lorenzo Codeluppi <lcodeluppi@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include <map>
#include <utility>
#include <vector>

#include "autonomous_msgs/msg/boundary.hpp"
#include "autonomous_msgs/msg/cone_array.hpp"
#include "boundary_estimation_graph.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "helpers.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace boundary_estimation {

class BoundaryEstimationVisualization {
public:
  BoundaryEstimationVisualization();

  void SetTrack(const std::vector<Point_2> &middle_line, const std::vector<Point_2> &left_boundary,
                const std::vector<Point_2> &right_boundary);

  void ResetVisualization();

  void ResetFinalPath();

  void SetMap(const autonomous_msgs::msg::ConeArray &global_map_);

  void SetPaths(const std::vector<std::vector<Point_2>> &candidate_paths);

  void SetFilteredPaths(const std::vector<std::vector<Point_2>> &filtered_candidate_paths);

  void SetTriangulationEdges(const std::vector<std::pair<Point_2, Point_2>> &triangulation_edges);

  void SetCrossedMidpoints(const std::vector<Point_2> &crossed_midpoints);

  void SetCrossedMidpointsLine(const std::vector<Point_2> &past_crossed_midpoints_line);

  void SetFinalPath(const std::vector<Point_2> &middle_line, const std::vector<Point_2> &left_boundary,
                    const std::vector<Point_2> &right_boundary);

  visualization_msgs::msg::Marker position_visual();

  visualization_msgs::msg::Marker cones_visual();

  visualization_msgs::msg::Marker left_bound_visual();

  visualization_msgs::msg::Marker right_bound_visual();

  visualization_msgs::msg::Marker midpoints_visual();

  visualization_msgs::msg::Marker crossed_midpoint_visual();

  visualization_msgs::msg::Marker crossed_car_position_visual();

  visualization_msgs::msg::Marker candidate_paths_visual();

  visualization_msgs::msg::Marker filtered_candidate_paths_visual();

  visualization_msgs::msg::Marker triangulation_edges_visual();

  visualization_msgs::msg::Marker final_left_bound_visual();

  visualization_msgs::msg::Marker final_right_bound_visual();

  visualization_msgs::msg::Marker final_midpoints_visual();

private:
  visualization_msgs::msg::Marker midpoints_visual_;
  visualization_msgs::msg::Marker crossed_midpoints_visual_;
  visualization_msgs::msg::Marker crossed_car_pos_visual_;
  visualization_msgs::msg::Marker left_bound_visual_;
  visualization_msgs::msg::Marker right_bound_visual_;
  visualization_msgs::msg::Marker position_visual_;
  visualization_msgs::msg::Marker cones_visual_;
  visualization_msgs::msg::Marker triangulation_edges_visual_;
  visualization_msgs::msg::Marker candidate_paths_visual_;
  visualization_msgs::msg::Marker filtered_candidate_paths_visual_;

  visualization_msgs::msg::Marker final_midpoints_visual_;
  visualization_msgs::msg::Marker final_left_bound_visual_;
  visualization_msgs::msg::Marker final_right_bound_visual_;
};
} // namespace boundary_estimation
