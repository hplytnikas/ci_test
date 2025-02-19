/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023 - 2024  Authors:
 *   - Lorenzo Codeluppi <lcodeluppi@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/interpolation_functions.h>

#include "autonomous_msgs/msg/cone.hpp"
#include "boundary_estimation_structs.hpp"

namespace boundary_estimation {

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Triangulation_vertex_base_with_info_2<VertexInfo, Kernel> VertexBase;
typedef CGAL::Triangulation_data_structure_2<VertexBase> Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds> Triangulation;
typedef Triangulation::Face_handle Face_handle;
typedef Triangulation::Vertex_handle Vertex_handle;
typedef Kernel::Point_2 Point_2;

/**
 * Struct representing a single path in the graph.
 *
 * This struct encapsulates information about a single path in the graph
 */
struct SinglePath {
  std::vector<Face_handle> faces;
  std::vector<std::pair<std::string, Point_2>> midpoints;
  std::vector<Vertex_handle> left_bound;
  std::vector<Vertex_handle> right_bound;

  std::vector<double> gate_widths;
  std::vector<Kernel::Vector_2> gate_vector;
  std::vector<double> track_widths;
  std::vector<double> path_angles;
  std::vector<double> path_angle_changes;
  std::vector<double> left_cones_distance_apart;
  std::vector<double> right_cones_distance_apart;

  // Actually is the second since the first point is th car position
  bool filter_first_point;

  int wrong_color_left = 0;
  int wrong_color_right = 0;
  int lidar_cones_used = 0;
  int end_bound_type;

  double path_length;

  double total_cost;
  // Costs logging
  double length_cost;
  double gate_cost;
  double width_var_cost;
  double angle_max_cost;
  double presence_max_cost;
  double left_distance_var_cost;
  double right_distance_var_cost;
  double angle_smoothness_cost;

  double prior;
  double likelihood;
  double posterior;
};

class Graph {
public:
  explicit Graph(const std::vector<autonomous_msgs::msg::Cone> &cones_observed);

  /**
   * Compute triangulation based on observed cones.
   *
   * This function computes triangulation based on observed cones in the cone
   * map. It filters cones based on minimum presence probability and color
   * parameters. Triangulation is performed using Delaunay triangulation
   * algorithm.
   */
  void ComputeTriangulation(heuristics::ThresholdValues kThresholdValues, heuristics::ColorParams kColorParams);

  /**
   * Insert vertices into the graph.
   *
   * This function inserts a list of vertices, represented as points paired with
   * vertex information, into the graph's Delaunay triangulation.
   */
  void InsertVertexList(const std::vector<std::pair<Point_2, VertexInfo>> &point_list);

  /**
   * Check if a face is inside the graph.
   *
   * This function checks if a given face is inside the convex hull of the
   * graph's Delaunay triangulation.
   */
  bool IsOutsideTheGraph(const Face_handle &face);

  /**
   * Locate a point inside the graph.
   *
   * This function locates a given point inside the graph's Delaunay
   * triangulation and returns the face containing the point.
   */
  Face_handle LocateInsideTheGraph(const Point_2 &car_position);

  std::vector<Vertex_handle> GetVertices();

  std::vector<std::pair<Vertex_handle, Vertex_handle>> GetEdges();

  std::vector<std::pair<Point_2, Point_2>> GetEdgesForViz();

  std::optional<Vertex_handle> GetVertexById(const std::string &vertex_id);

private:
  void UpdateVertexMap();

  std::vector<autonomous_msgs::msg::Cone> cone_map;

  Triangulation del_tri_;

  std::unordered_map<std::string, Vertex_handle> graph_vertex_map_;
};

} // namespace boundary_estimation
