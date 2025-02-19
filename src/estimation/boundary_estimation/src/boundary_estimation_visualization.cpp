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

#include "boundary_estimation_visualization.hpp"

namespace boundary_estimation {
BoundaryEstimationVisualization::BoundaryEstimationVisualization() {
  int id_counter = 0;

  cones_visual_.ns = position_visual_.ns = left_bound_visual_.ns = right_bound_visual_.ns =
      final_right_bound_visual_.ns = midpoints_visual_.ns = crossed_midpoints_visual_.ns =
          triangulation_edges_visual_.ns = candidate_paths_visual_.ns = filtered_candidate_paths_visual_.ns =
              final_midpoints_visual_.ns = final_left_bound_visual_.ns = crossed_car_pos_visual_.ns = "ns_delaunay";

  cones_visual_.id = id_counter;
  id_counter++;
  position_visual_.id = id_counter;
  id_counter++;
  left_bound_visual_.id = id_counter;
  id_counter++;
  right_bound_visual_.id = id_counter;
  id_counter++;
  midpoints_visual_.id = id_counter;
  id_counter++;
  crossed_midpoints_visual_.id = id_counter;
  id_counter++;
  crossed_car_pos_visual_.id = id_counter;
  id_counter++;
  triangulation_edges_visual_.id = id_counter;
  id_counter++;
  candidate_paths_visual_.id = id_counter;
  id_counter++;
  filtered_candidate_paths_visual_.id = id_counter;
  id_counter++;
  final_midpoints_visual_.id = id_counter;
  id_counter++;
  final_right_bound_visual_.id = id_counter;
  id_counter++;
  final_left_bound_visual_.id = id_counter;

  cones_visual_.type = position_visual_.type = visualization_msgs::msg::Marker::POINTS;
  midpoints_visual_.type = left_bound_visual_.type = right_bound_visual_.type = final_right_bound_visual_.type =
      final_midpoints_visual_.type = final_left_bound_visual_.type = crossed_car_pos_visual_.type =
          crossed_midpoints_visual_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  triangulation_edges_visual_.type = candidate_paths_visual_.type = filtered_candidate_paths_visual_.type =
      visualization_msgs::msg::Marker::LINE_LIST;

  cones_visual_.action = position_visual_.action = left_bound_visual_.action = right_bound_visual_.action =
      midpoints_visual_.action = final_midpoints_visual_.action = crossed_midpoints_visual_.action =
          crossed_car_pos_visual_.action = triangulation_edges_visual_.action = candidate_paths_visual_.action =
              final_right_bound_visual_.action = filtered_candidate_paths_visual_.action =
                  final_left_bound_visual_.action = visualization_msgs::msg::Marker::ADD;

  cones_visual_.pose.orientation.w = position_visual_.pose.orientation.w = left_bound_visual_.pose.orientation.w =
      right_bound_visual_.pose.orientation.w = midpoints_visual_.pose.orientation.w =
          final_midpoints_visual_.pose.orientation.w = crossed_midpoints_visual_.pose.orientation.w =
              crossed_car_pos_visual_.pose.orientation.w = triangulation_edges_visual_.pose.orientation.w =
                  candidate_paths_visual_.pose.orientation.w = filtered_candidate_paths_visual_.pose.orientation.w =
                      final_right_bound_visual_.pose.orientation.w = final_left_bound_visual_.pose.orientation.w = 1.0;

  // Nothing is transparent
  cones_visual_.color.a = position_visual_.color.a = left_bound_visual_.color.a = right_bound_visual_.color.a =
      midpoints_visual_.color.a = crossed_midpoints_visual_.color.a = triangulation_edges_visual_.color.a =
          crossed_car_pos_visual_.color.a = candidate_paths_visual_.color.a = filtered_candidate_paths_visual_.color.a =
              final_midpoints_visual_.color.a = final_right_bound_visual_.color.a = final_left_bound_visual_.color.a =
                  1.0;

  // Cones are orange
  cones_visual_.color.r = 0.8;
  cones_visual_.color.g = 0.2;

  // Position is green
  position_visual_.color.g = 1.0;
  // paths are yellow
  candidate_paths_visual_.color.r = 0.5;
  candidate_paths_visual_.color.g = 0.5;
  // filtered paths are green
  filtered_candidate_paths_visual_.color.g = 1.0;

  // Left bound is blue
  left_bound_visual_.color.b = 1.0;
  final_left_bound_visual_.color.b = 1.0;

  // Right bound is yellow
  right_bound_visual_.color.r = 0.5;
  right_bound_visual_.color.g = 0.5;
  final_right_bound_visual_.color.r = 0.5;
  final_right_bound_visual_.color.g = 0.5;

  // Midpoints are red
  midpoints_visual_.color.r = 1.0;
  final_midpoints_visual_.color.r = 1.0;

  // Crossed midpoints are violet
  crossed_midpoints_visual_.color.b = 1.0;
  crossed_midpoints_visual_.color.r = 1.0;

  // Crossed car postion line is orange
  crossed_car_pos_visual_.color.r = 1.0;
  crossed_car_pos_visual_.color.g = 0.5;

  // [meters]
  cones_visual_.scale.x = cones_visual_.scale.y = position_visual_.scale.x = position_visual_.scale.y = 0.4;

  // [meters]
  left_bound_visual_.scale.x = right_bound_visual_.scale.x = crossed_midpoints_visual_.scale.x =
      crossed_midpoints_visual_.scale.y = midpoints_visual_.scale.x = midpoints_visual_.scale.y =
          crossed_car_pos_visual_.scale.x = crossed_car_pos_visual_.scale.y = final_midpoints_visual_.scale.x =
              final_left_bound_visual_.scale.x = final_left_bound_visual_.scale.y = final_right_bound_visual_.scale.x =
                  final_right_bound_visual_.scale.y = 0.2;

  // [meters]
  triangulation_edges_visual_.scale.x = candidate_paths_visual_.scale.x = filtered_candidate_paths_visual_.scale.x =
      0.05;
}

void BoundaryEstimationVisualization::SetMap(const autonomous_msgs::msg::ConeArray &global_map_) {
  std::vector<geometry_msgs::msg::Point> cones_viz;
  for (unsigned int i = 0; i < global_map_.cones.size(); ++i) {
    geometry_msgs::msg::Point cone_viz;
    cone_viz.x = global_map_.cones.at(i).position.x;
    cone_viz.y = global_map_.cones.at(i).position.y;
    cone_viz.z = global_map_.cones.at(i).position.z;

    cones_viz.push_back(cone_viz);
  }
  geometry_msgs::msg::Point position_viz;
  position_viz.x = 0.0;
  position_viz.y = 0.0;
  position_viz.z = 0.0;

  this->position_visual_.points.clear();
  this->position_visual_.points.push_back(position_viz);

  this->cones_visual_.points.clear();
  this->cones_visual_.points = cones_viz;
}

void BoundaryEstimationVisualization::SetTrack(const std::vector<Point_2> &middle_line,
                                               const std::vector<Point_2> &left_boundary,
                                               const std::vector<Point_2> &right_boundary) {
  std::vector<geometry_msgs::msg::Point> middle_line_viz = ParsePoint2VectorToPointVector(middle_line);

  std::vector<geometry_msgs::msg::Point> left_boundary_viz = ParsePoint2VectorToPointVector(left_boundary);

  std::vector<geometry_msgs::msg::Point> right_boundary_viz = ParsePoint2VectorToPointVector(right_boundary);

  this->midpoints_visual_.points = middle_line_viz;
  this->left_bound_visual_.points = left_boundary_viz;
  this->right_bound_visual_.points = right_boundary_viz;
}

void BoundaryEstimationVisualization::ResetVisualization() {
  // reset all visualizations to empty
  this->filtered_candidate_paths_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->candidate_paths_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->triangulation_edges_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->crossed_midpoints_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->final_midpoints_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->final_left_bound_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->final_right_bound_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->cones_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->position_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->midpoints_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->left_bound_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->right_bound_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->crossed_car_pos_visual_.points = std::vector<geometry_msgs::msg::Point>();
}

void BoundaryEstimationVisualization::ResetFinalPath() {
  // reset all visualizations to empty
  this->final_midpoints_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->final_left_bound_visual_.points = std::vector<geometry_msgs::msg::Point>();
  this->final_right_bound_visual_.points = std::vector<geometry_msgs::msg::Point>();
}

void BoundaryEstimationVisualization::SetPaths(const std::vector<std::vector<Point_2>> &candidate_paths) {
  this->candidate_paths_visual_.points.clear();
  for (unsigned int i = 0; i < candidate_paths.size(); ++i) {
    std::vector<geometry_msgs::msg::Point> path = ParsePoint2VectorToPointVector(candidate_paths.at(i));
    if (path.size() >= 2) {
      for (unsigned int j = 1; j < path.size(); ++j) {
        this->candidate_paths_visual_.points.push_back(path.at(j - 1));
        this->candidate_paths_visual_.points.push_back(path.at(j));
      }
    }
  }
}

void BoundaryEstimationVisualization::SetFilteredPaths(
    const std::vector<std::vector<Point_2>> &filtered_candidate_paths) {
  this->filtered_candidate_paths_visual_.points.clear();
  for (unsigned int i = 0; i < filtered_candidate_paths.size(); ++i) {
    std::vector<geometry_msgs::msg::Point> path = ParsePoint2VectorToPointVector(filtered_candidate_paths.at(i));
    if (path.size() >= 2) {
      for (unsigned int j = 1; j < path.size(); ++j) {
        this->filtered_candidate_paths_visual_.points.push_back(path.at(j - 1));
        this->filtered_candidate_paths_visual_.points.push_back(path.at(j));
      }
    }
  }
}

void BoundaryEstimationVisualization::SetTriangulationEdges(
    const std::vector<std::pair<Point_2, Point_2>> &triangulation_edges) {
  this->triangulation_edges_visual_.points.clear();

  for (unsigned int i = 0; i < triangulation_edges.size(); ++i) {
    geometry_msgs::msg::Point pt1 = CgalToGeometryMsgsPoint(triangulation_edges.at(i).first);
    geometry_msgs::msg::Point pt2 = CgalToGeometryMsgsPoint(triangulation_edges.at(i).second);

    std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> tri_edge_viz(pt1, pt2);
    this->triangulation_edges_visual_.points.push_back(tri_edge_viz.first);
    this->triangulation_edges_visual_.points.push_back(tri_edge_viz.second);
  }
}

// Visualize crossed midpoints
void BoundaryEstimationVisualization::SetCrossedMidpoints(const std::vector<Point_2> &crossed_midpoints) {
  std::vector<geometry_msgs::msg::Point> crossed_midpoints_viz = ParsePoint2VectorToPointVector(crossed_midpoints);

  this->crossed_midpoints_visual_.points.clear();
  this->crossed_midpoints_visual_.points = crossed_midpoints_viz;
}

void BoundaryEstimationVisualization::SetCrossedMidpointsLine(const std::vector<Point_2> &past_crossed_midpoints_line) {
  std::vector<geometry_msgs::msg::Point> crossed_midpoints_line_viz =
      ParsePoint2VectorToPointVector(past_crossed_midpoints_line);

  this->crossed_car_pos_visual_.points.clear();
  this->crossed_car_pos_visual_.points = crossed_midpoints_line_viz;
}

// Visualize full path
void BoundaryEstimationVisualization::SetFinalPath(const std::vector<Point_2> &middle_line,
                                                   const std::vector<Point_2> &left_boundary,
                                                   const std::vector<Point_2> &right_boundary) {
  std::vector<geometry_msgs::msg::Point> middle_line_viz = ParsePoint2VectorToPointVector(middle_line);

  std::vector<geometry_msgs::msg::Point> left_boundary_viz = ParsePoint2VectorToPointVector(left_boundary);

  std::vector<geometry_msgs::msg::Point> right_boundary_viz = ParsePoint2VectorToPointVector(right_boundary);

  this->final_midpoints_visual_.points = middle_line_viz;
  this->final_left_bound_visual_.points = left_boundary_viz;
  this->final_right_bound_visual_.points = right_boundary_viz;
}

visualization_msgs::msg::Marker BoundaryEstimationVisualization::cones_visual() { return cones_visual_; }

visualization_msgs::msg::Marker BoundaryEstimationVisualization::position_visual() { return position_visual_; }

visualization_msgs::msg::Marker BoundaryEstimationVisualization::crossed_midpoint_visual() {
  return crossed_midpoints_visual_;
}

visualization_msgs::msg::Marker BoundaryEstimationVisualization::midpoints_visual() { return midpoints_visual_; }

visualization_msgs::msg::Marker BoundaryEstimationVisualization::left_bound_visual() { return left_bound_visual_; }

visualization_msgs::msg::Marker BoundaryEstimationVisualization::right_bound_visual() { return right_bound_visual_; }

visualization_msgs::msg::Marker BoundaryEstimationVisualization::final_left_bound_visual() {
  return final_left_bound_visual_;
}

visualization_msgs::msg::Marker BoundaryEstimationVisualization::final_right_bound_visual() {
  return final_right_bound_visual_;
}

visualization_msgs::msg::Marker BoundaryEstimationVisualization::final_midpoints_visual() {
  return final_midpoints_visual_;
}

visualization_msgs::msg::Marker BoundaryEstimationVisualization::candidate_paths_visual() {
  return candidate_paths_visual_;
}

visualization_msgs::msg::Marker BoundaryEstimationVisualization::filtered_candidate_paths_visual() {
  return filtered_candidate_paths_visual_;
}

visualization_msgs::msg::Marker BoundaryEstimationVisualization::triangulation_edges_visual() {
  return triangulation_edges_visual_;
}

visualization_msgs::msg::Marker BoundaryEstimationVisualization::crossed_car_position_visual() {
  return crossed_car_pos_visual_;
}

} // namespace boundary_estimation
