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

#include "boundary_estimation_graph.hpp"

namespace boundary_estimation {

Graph::Graph(const std::vector<autonomous_msgs::msg::Cone> &cones_observed) { cone_map = cones_observed; }

void Graph::UpdateVertexMap() {
  std::vector<Vertex_handle> vertices = GetVertices();
  for (const Vertex_handle &vertex : vertices) {
    std::string vertex_id = std::to_string(vertex->info().id);
    graph_vertex_map_[vertex_id] = vertex;
  }
}

std::optional<Vertex_handle> Graph::GetVertexById(const std::string &vertex_id) {
  if (graph_vertex_map_.find(vertex_id) != graph_vertex_map_.end()) {
    return graph_vertex_map_[vertex_id];
  }
  return std::nullopt;
}

void Graph::ComputeTriangulation(heuristics::ThresholdValues kThresholdValues, heuristics::ColorParams kColorParams) {
  std::vector<std::pair<Point_2, VertexInfo>> points_vector;
  for (const autonomous_msgs::msg::Cone &cone_observed : cone_map) {
    // Ignore cones that have a presence probability that is below the min
    // presence threshold and the orange cones
    if (cone_observed.prob_cone < kThresholdValues.minimum_presence_probability ||
        (!kColorParams.use_orange && cone_observed.pipeline & (constants::kSensorFusion | constants::kCameraOnly) &&
         std::max(cone_observed.prob_type.orange, cone_observed.prob_type.orange_big) >
             std::max(cone_observed.prob_type.blue, cone_observed.prob_type.yellow))) {
      continue;
    }

    Point_2 point = Point_2(cone_observed.position.x, cone_observed.position.y);

    VertexInfo vertex_info;
    vertex_info.id = cone_observed.id_cone;
    vertex_info.pipeline = cone_observed.pipeline;
    vertex_info.blue_prob = cone_observed.prob_type.blue;
    vertex_info.yellow_prob = cone_observed.prob_type.yellow;
    vertex_info.orange_prob = cone_observed.prob_type.orange;
    vertex_info.orange_big_prob = cone_observed.prob_type.orange_big;
    vertex_info.presence_prob = cone_observed.prob_cone;
    vertex_info.virtual_flag = false;

    points_vector.push_back(std::make_pair(point, vertex_info));
  }
  del_tri_.insert(points_vector.begin(), points_vector.end());
  // update the vertex map
  UpdateVertexMap();
}

void Graph::InsertVertexList(const std::vector<std::pair<Point_2, VertexInfo>> &point_list) {
  // Insert the point list into the Delaunay triangulation
  del_tri_.insert(point_list.begin(), point_list.end());
}

bool Graph::IsOutsideTheGraph(const Face_handle &face) {
  // check if the position is inside the convex hull
  return del_tri_.is_infinite(face);
}

Face_handle Graph::LocateInsideTheGraph(const Point_2 &car_position) {
  // locate the point inside the graph
  return del_tri_.locate(car_position);
}

/*
  Getter for triangulation vertices
*/
std::vector<Vertex_handle> Graph::GetVertices() {
  std::vector<Vertex_handle> all_vertices;

  for (Triangulation::Finite_vertices_iterator it = del_tri_.finite_vertices_begin();
       it != del_tri_.finite_vertices_end(); ++it) {
    all_vertices.push_back(it); // Store pointer to vertex
  }

  return all_vertices;
}

/*
 Getter for triangulation edges
*/
std::vector<std::pair<Vertex_handle, Vertex_handle>> Graph::GetEdges() {
  std::vector<std::pair<Vertex_handle, Vertex_handle>> all_edges;

  for (Triangulation::Finite_edges_iterator it = del_tri_.finite_edges_begin(); it != del_tri_.finite_edges_end();
       ++it) {
    Face_handle face = it->first;
    int index = it->second;
    Vertex_handle pt1 = face->vertex(face->cw(index));
    Vertex_handle pt2 = face->vertex(face->ccw(index));

    std::pair<Vertex_handle, Vertex_handle> edge(pt1, pt2);

    all_edges.push_back(edge);
  }

  return all_edges;
}

/*
  Getter for triangulation edges for visualization
*/
std::vector<std::pair<Point_2, Point_2>> Graph::GetEdgesForViz() {
  std::vector<std::pair<Point_2, Point_2>> edges;
  for (Triangulation::Finite_edges_iterator it = del_tri_.finite_edges_begin(); it != del_tri_.finite_edges_end();
       ++it) {
    Face_handle face = it->first;
    int index = it->second;
    Point_2 pt1 = face->vertex(face->cw(index))->point();
    Point_2 pt2 = face->vertex(face->ccw(index))->point();

    std::pair<Point_2, Point_2> edge(pt1, pt2);

    edges.push_back(edge);
  }

  return edges;
}

} // namespace boundary_estimation
