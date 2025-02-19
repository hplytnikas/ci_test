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

#include "boundary_estimation_backend.hpp"
#include <easy/profiler.h>

namespace boundary_estimation {

/*
Getter for candidate paths
*/
std::vector<std::vector<Point_2>> BoundaryEstimationBackend::GetCandidatePaths() {
  std::vector<std::vector<Point_2>> paths;

  for (unsigned int i = 0; i < candidates_paths_.size(); ++i) {
    std::vector<Point_2> path(ExtractSecondElementFromPairVector(candidates_paths_.at(i).midpoints));
    if (path.size() >= 2) {
      paths.push_back(path);
    }
  }

  return paths;
}

/*
Getter for filtered paths
*/
std::vector<std::vector<Point_2>> BoundaryEstimationBackend::GetFilteredCandidatePaths() {
  std::vector<std::vector<Point_2>> paths;

  for (std::multimap<double, SinglePath>::reverse_iterator itr = filtered_candidates_paths_viz_.rbegin();
       itr != filtered_candidates_paths_viz_.rend(); ++itr) {
    std::vector<Point_2> path(ExtractSecondElementFromPairVector(itr->second.midpoints));
    paths.push_back(path);
  }

  return paths;
}

CrossedPath BoundaryEstimationBackend::ComputeCrossedPath(const Point_2 &current_point, const Point_2 &previous_point,
                                                          const bool &midpoint_only) {
  EASY_FUNCTION(profiler::colors::BlueGrey);
  std::unordered_map<std::string, bool> midpoint_id_map;
  CrossedPath crossed_path;
  // segment from origin to previous point
  Kernel::Segment_2 segment = Kernel::Segment_2(current_point, previous_point);

  // Sample points from the vector so that we can locate not just the terminal face but also all the faces
  // in between
  std::vector<Point_2> sampled_points = SampleSegment(segment, 20);

  // Iterate over the sampled points to create a set of crossed faces
  for (const Point_2 &sampled_point : sampled_points) {
    Face_handle face = graph_->LocateInsideTheGraph(sampled_point);

    // If the point is outside the convex hull, return an empty crossed path
    if (graph_->IsOutsideTheGraph(face)) {
      continue;
    }
    // Iterate over the edges of the face
    for (int i = 0; i < 3; ++i) {
      Vertex_handle left_vertex = face->vertex(face->cw(i));
      Vertex_handle right_vertex = face->vertex(face->ccw(i));

      // Get the edge from the two vertices
      Kernel::Segment_2 edge = Kernel::Segment_2(left_vertex->point(), right_vertex->point());

      // Check if the segment intersects the edge
      if (CGAL::do_intersect(segment, edge)) {
        std::string midpoint_id = ComputeMidpointIndex(left_vertex, right_vertex);
        if (midpoint_id_map.find(midpoint_id) != midpoint_id_map.end()) {
          continue;
        }
        if (midpoint_id != "-") {
          midpoint_id_map[midpoint_id] = true;
          Point_2 midpoint = CGAL::midpoint(left_vertex->point(), right_vertex->point());
          // Calculate the distance of the midpoint from the segment
          double distance = PointToLineSegmentDistance(current_point.x(), current_point.y(), previous_point.x(),
                                                       previous_point.y(), midpoint.x(), midpoint.y());
          // If the distance is greater than the threshold, skip the midpoint
          if (distance > constants::kMidpointDistanceThreshold) {
            continue;
          }
          crossed_path.midpoints.push_back(std::make_pair(midpoint_id, midpoint));
          if (midpoint_only) {
            break;
          }
          std::optional<Point_2> left_vertex_point = ComputeVertexpointFromId(std::to_string(left_vertex->info().id));
          std::optional<Point_2> right_vertex_point = ComputeVertexpointFromId(std::to_string(right_vertex->info().id));

          if (left_vertex_point.has_value())
            crossed_path.left_bound.push_back(
                std::make_pair(std::to_string(left_vertex->info().id), left_vertex_point.value()));
          if (right_vertex_point.has_value())
            crossed_path.right_bound.push_back(
                std::make_pair(std::to_string(right_vertex->info().id), right_vertex_point.value()));
          break;
        }
      }
    }
  }
  return crossed_path;
}

// Given a list of vertex id, return a vector of Point_2 corresponding to the vertex point
std::optional<Point_2> BoundaryEstimationBackend::ComputeVertexpointFromId(std::string vertex_id) {
  EASY_FUNCTION(profiler::colors::Dark);
  if (vertex_id == "-") {
    return std::nullopt;
  }

  std::optional<Vertex_handle> vertex = graph_->GetVertexById(vertex_id);

  if (!vertex.has_value()) {
    if (logging_flag_)
      RCLCPP_ERROR(rclcpp::get_logger("delaunay_search"),
                   "[boundary_estimation - ComputeVertexpointFromId] "
                   "Vertex with ID %s not found in the graph",
                   vertex_id.c_str());
    return std::nullopt;
  }

  return vertex.value()->point();
}

// Given a list of midpoint id, return a vector of Point_2 corresponding to the midpoint point
std::optional<Point_2> BoundaryEstimationBackend::ComputeMidpointFromId(const std::string midpoint_id) {
  EASY_FUNCTION(profiler::colors::Brown);
  Point_2 midpoints;
  if (midpoint_id == "-") {
    return std::nullopt;
  }
  // Extract the two IDs from the key of the iterator
  std::string first_id_str = midpoint_id.substr(0, midpoint_id.find("-"));
  std::string second_id_str = midpoint_id.substr(midpoint_id.find("-") + 1);

  std::optional<Vertex_handle> first_vertex = graph_->GetVertexById(first_id_str);
  std::optional<Vertex_handle> second_vertex = graph_->GetVertexById(second_id_str);

  // Check if they have value
  if (!first_vertex.has_value() || !second_vertex.has_value()) {
    return std::nullopt;
  }

  Point_2 midpoint = CGAL::midpoint(first_vertex.value()->point(), second_vertex.value()->point());

  return midpoint;
}

CrossedPath BoundaryEstimationBackend::ComputePastMiddlePoints(const std::vector<std::string> &past_midpoints_ids,
                                                               const CrossedPath &last_crossed_path) {
  EASY_FUNCTION(profiler::colors::Brick);

  CrossedPath crossed_path = last_crossed_path;
  std::unordered_map<std::string, bool> id_map;

  if (past_midpoints_ids.empty()) {
    return crossed_path;
  }

  // Reverse the vector of past car positions
  std::vector<std::string> reversed_past_midpoints_ids = past_midpoints_ids;
  std::reverse(reversed_past_midpoints_ids.begin(), reversed_past_midpoints_ids.end());

  double vector_distance = 0.0;
  for (int i = 0; i < reversed_past_midpoints_ids.size() - 1; i++) {
    Point_2 current_point = ComputeMidpointFromId(reversed_past_midpoints_ids.at(i)).value();
    Point_2 next_point = ComputeMidpointFromId(reversed_past_midpoints_ids.at(i + 1)).value();
    if (IsPointInsideStartingFace(next_point)) {
      continue;
    }
    std::string first_id = reversed_past_midpoints_ids.at(i).substr(0, reversed_past_midpoints_ids.at(i).find("-"));
    std::string second_id = reversed_past_midpoints_ids.at(i).substr(reversed_past_midpoints_ids.at(i).find("-") + 1);

    std::optional<std::pair<std::string, std::string>> boundary_points_ids =
        CheckFinalPathBoundaryIds(first_id, second_id);

    if (boundary_points_ids.has_value()) {
      if (id_map.find(boundary_points_ids.value().first) == id_map.end()) {
        id_map[boundary_points_ids.value().first] = true;
        crossed_path.left_bound.push_back(std::make_pair(
            boundary_points_ids.value().first, ComputeVertexpointFromId(boundary_points_ids.value().first).value()));
      }
      if (id_map.find(boundary_points_ids.value().second) == id_map.end()) {
        id_map[boundary_points_ids.value().second] = true;
        crossed_path.right_bound.push_back(std::make_pair(
            boundary_points_ids.value().second, ComputeVertexpointFromId(boundary_points_ids.value().second).value()));
      }
    }
    crossed_path.midpoints.push_back(std::make_pair(reversed_past_midpoints_ids.at(i), current_point));

    vector_distance += PointToPointDistance(current_point.x(), current_point.y(), next_point.x(), next_point.y());
    // Stop when the distance is greater than the maximum distance
    if (vector_distance >= constants::kMaxLastMidpointDistance) break;
  }

  return crossed_path;
}

std::vector<std::pair<std::string, Point_2>>
BoundaryEstimationBackend::ConcatenateFinalPoints(const std::vector<std::pair<std::string, Point_2>> &past_points,
                                                  const std::vector<std::pair<std::string, Point_2>> &current_points,
                                                  const bool &is_midpoint) {
  // Concatenate the past middle points with the next path midpoints
  std::vector<std::pair<std::string, Point_2>> final_points;
  if (past_points.empty() && is_midpoint) {
    // Create a fake cone behind the car
    Point_2 fake_point_behind = Point_2(-1, 0);
    final_points.push_back(std::make_pair("-", fake_point_behind));
  } else {
    final_points.insert(final_points.end(), past_points.begin(), past_points.end());
    std::reverse(final_points.begin(), final_points.end());
  }
  // skip for sure the first one, which is the car position
  int skip_index = is_midpoint ? 1 : 0;
  final_points.insert(final_points.end(), current_points.begin() + skip_index, current_points.end());

  return FilterPath(final_points);
}

BoundaryLogicResult
BoundaryEstimationBackend::ComputeBoundedPath(const std::vector<autonomous_msgs::msg::Cone> &cones_observed,
                                              const Point_2 &past_car_position,
                                              const std::vector<std::string> &past_midpoints_ids) {
  EASY_FUNCTION(profiler::colors::DeepOrange);
  filtered_candidates_paths_viz_.clear();
  BoundaryLogicResult result;

  graph_ = std::make_unique<Graph>(cones_observed);
  graph_->ComputeTriangulation(kThresholdValues, kColorParams);

  // Compute crossed midpoint in the last iteration
  CrossedPath crossed_path_last_iteration = ComputeCrossedPath(Point_2(0, 0), past_car_position);
  // Iterate over the crossed path left and right boundary and update the map with the ID
  for (const std::pair<std::string, Point_2> &left_boundary_point : crossed_path_last_iteration.left_bound) {
    left_bound_map_[left_boundary_point.first] = true;
  }

  for (const std::pair<std::string, Point_2> &right_boundary_point : crossed_path_last_iteration.right_bound) {
    right_bound_map_[right_boundary_point.first] = true;
  }
  // Compute the list of past middle points for reference using the list of ids and the newly computed crossed path
  CrossedPath past_path = ComputePastMiddlePoints(past_midpoints_ids, crossed_path_last_iteration);

  std::optional<SinglePath> best_path = ComputeBestPath(ExtractSecondElementFromPairVector(past_path.midpoints));

  if (best_path.has_value()) {
    if (debug_flag_) {
      // Print blue color probability of the cone in the best path left boundary
      for (const Vertex_handle &left_boundary_point : best_path.value().left_bound) {
        // Print id and probability of the cone
        RCLCPP_INFO(rclcpp::get_logger("delaunay_search"),
                    "[boundary_estimation] Cone left %i: %f, %f, %f, %f position: %f, %f",
                    left_boundary_point->info().id, left_boundary_point->info().blue_prob,
                    left_boundary_point->info().yellow_prob, left_boundary_point->info().orange_prob,
                    left_boundary_point->info().orange_big_prob, left_boundary_point->point().x(),
                    left_boundary_point->point().y());
      }

      for (const Vertex_handle &right_boundary_point : best_path.value().right_bound) {
        // Print id and probability of the cone as well as his position
        RCLCPP_INFO(rclcpp::get_logger("delaunay_search"),
                    "[boundary_estimation] Cone right %i: %f, %f, %f, %f position: %f, %f",
                    right_boundary_point->info().id, right_boundary_point->info().blue_prob,
                    right_boundary_point->info().yellow_prob, right_boundary_point->info().orange_prob,
                    right_boundary_point->info().orange_big_prob, right_boundary_point->point().x(),
                    right_boundary_point->point().y());
      }
    }
    if (debug_flag_) {
      RCLCPP_INFO(rclcpp::get_logger("delaunay_search"),
                  "[boundary_estimation] Prior: %f, Likelihood: %f, Posterior: %f", best_path.value().prior,
                  best_path.value().likelihood, best_path.value().posterior);
    }

    // Use the FilterLastPathSegment to make the end boundary more reliable,
    // reduce unexpected shortcut and flickers
    SinglePath filtered_best_path = best_path.value();
    FilterLastPathSegment(filtered_best_path);

    result.midpoints = ConcatenateFinalPoints(past_path.midpoints, filtered_best_path.midpoints, true);
    result.left_bound = ConcatenateFinalPoints(past_path.left_bound,
                                               ExtractPointsFromVertexHandles(filtered_best_path.left_bound), false);
    result.right_bound = ConcatenateFinalPoints(past_path.right_bound,
                                                ExtractPointsFromVertexHandles(filtered_best_path.right_bound), false);
    result.posterior = filtered_best_path.posterior;
    result.success = true;
  }

  result.candidates = GetCandidatePaths();
  result.past_middle_points = past_path.midpoints;
  result.last_past_middle_points = crossed_path_last_iteration.midpoints;
  result.filtered_path_list = GetFilteredCandidatePaths();
  result.triangulation_edges = graph_->GetEdgesForViz();
  return result;
}

std::optional<SinglePath> BoundaryEstimationBackend::ComputeBestPath(const std::vector<Point_2> &past_middle_points) {
  EASY_FUNCTION(profiler::colors::Skin);
  std::multimap<double, unsigned int> current_ordered_leaves_ids;

  // Initialize starting triangle face triangle where the car is in
  double car_x = 0.0;
  double car_y = 0.0;
  Point_2 car_position = Point_2(car_x, car_y);

  std::optional<Face_handle> starting_face = InitializeStartingFaceWithVirtualPoints(car_position);

  // if the starting position is not inside the convex hull
  if (!starting_face.has_value()) {
    if (logging_flag_)
      RCLCPP_ERROR(rclcpp::get_logger("delaunay_search"), "Car not heading towards cones, "
                                                          "unable to place car in a "
                                                          "triangulation face");
    return std::nullopt;
  }
  // Store the first non virtual face to filter it out when computing the crossed midpoints
  if (initial_face_ids_.empty() && !IsFaceVirtual(starting_face.value())) {
    for (int i = 0; i < 3; i++) {
      initial_face_ids_.push_back(starting_face.value()->vertex(i)->info().id);
    }
    std::sort(initial_face_ids_.begin(), initial_face_ids_.end());
  }

  // Iterating over the Relaxation type: starts with no relaxation
  RelaxationType max_relaxation_type = kRuntimeConstraints.full_relaxation_enabled ? Total : Severe;
  for (int i = None; i <= max_relaxation_type; i++) {
    // Clear before every iteration
    current_ordered_leaves_ids.clear();
    RelaxationType relaxation_mode = static_cast<RelaxationType>(i);

    // inizialize the vector of candidate paths
    InitializeCandidatePaths(starting_face.value(), car_position, past_middle_points, relaxation_mode);

    int size = candidates_paths_.size();
    // if no candidate path we just return no path
    if (size < 1) {
      if (logging_flag_)
        RCLCPP_ERROR(rclcpp::get_logger("delaunay_search"),
                     "Unable to generate any candidate path at relaxation type: %i ", i);
      continue;
    }
    // Init ordered leaves
    for (int index = 0; index < size; ++index) {
      current_ordered_leaves_ids.insert(std::pair<double, unsigned int>(candidates_paths_[index].posterior, index));
    }

    SinglePath best_path = BeamSearch(past_middle_points, current_ordered_leaves_ids, relaxation_mode);
    // if the best path is to short we don't return it
    if (best_path.path_length < kThresholdValues.desirable_minimum_path_length_m &&
        relaxation_mode < max_relaxation_type) {
      if (logging_flag_)
        RCLCPP_ERROR(rclcpp::get_logger("delaunay_search"),
                     "[boundary_estimation] The path is not long enough (%f) with relaxation type: %i",
                     best_path.path_length, i);
      continue;
    }
    if (best_path.midpoints.size() > 3) return best_path;
    continue;
  }

  return std::nullopt;
}

std::optional<std::pair<std::string, std::string>>
BoundaryEstimationBackend::CheckFinalPathBoundaryIds(const std::string &first_id, const std::string &second_id) {
  EASY_FUNCTION(profiler::colors::Orange);
  // Check that there's only one occurency in the left boundary
  bool first_id_in_left = left_bound_map_.find(first_id) != left_bound_map_.end();
  bool second_id_in_left = left_bound_map_.find(second_id) != left_bound_map_.end();

  // Check that there's only one occurency in the right boundary
  bool first_id_in_right = right_bound_map_.find(first_id) != right_bound_map_.end();
  bool second_id_in_right = right_bound_map_.find(second_id) != right_bound_map_.end();

  bool valid_left = first_id_in_left ^ second_id_in_left;    // XOR: exactly one should be true
  bool valid_right = first_id_in_right ^ second_id_in_right; // XOR: exactly one should be true

  if (valid_left && valid_right) {
    std::string left_vertex, right_vertex;
    if (first_id_in_left) {
      left_vertex = first_id;
      right_vertex = second_id;
    } else {
      left_vertex = second_id;
      right_vertex = first_id;
    }
    return std::make_pair(left_vertex, right_vertex);
  }
  return std::nullopt;
}

std::vector<std::pair<std::string, Point_2>>
BoundaryEstimationBackend::FilterPath(const std::vector<std::pair<std::string, Point_2>> &path) {
  EASY_FUNCTION(profiler::colors::Green);
  std::vector<std::pair<std::string, Point_2>> filtered_path;
  std::unordered_map<std::string, bool> point_id_map;
  for (const std::pair<std::string, Point_2> &point : path) {
    if (point_id_map.find(point.first) == point_id_map.end() || point.first == "0-0") {
      point_id_map[point.first] = true;
      filtered_path.push_back(point);
    }
  }
  return filtered_path;
}

std::vector<std::pair<std::string, Point_2>>
BoundaryEstimationBackend::FilterPathOverlay(const std::vector<std::pair<std::string, Point_2>> &path) {
  EASY_FUNCTION(profiler::colors::Pink);
  std::vector<std::pair<std::string, Point_2>> filtered_path;
  Point_2 first_path_point = path.front().second;
  // Iterate over the path in reverse order and check if the point are behind the first point
  int point_to_remove = 0;
  for (int i = path.size() - 1; i >= path.size() - 1 - constants::kMaxPointToCheckForOverlay; i--) {
    Point_2 current_point = path.at(i).second;
    if (current_point.x() > first_path_point.x()) {
      // remove this point from the path
      point_to_remove++;
    } else {
      break;
    }
  }
  // return the filtered path
  return std::vector<std::pair<std::string, Point_2>>(path.begin(), path.end() - point_to_remove);
}

std::vector<std::pair<std::string, Point_2>>
BoundaryEstimationBackend::CheckFirstAndLastPoint(const std::vector<std::pair<std::string, Point_2>> &path) {
  EASY_FUNCTION(profiler::colors::BlueGrey);
  std::vector<std::pair<std::string, Point_2>> filtered_path;
  Point_2 first_path_point = path.front().second;
  Point_2 last_path_point = path.back().second;
  // Check if they are close enough
  double distance =
      PointToPointDistance(first_path_point.x(), first_path_point.y(), last_path_point.x(), last_path_point.y());
  if (distance < constants::kMaxPointDistanceFirstAndLast) {
    // remove the last point
    filtered_path = std::vector<std::pair<std::string, Point_2>>(path.begin(), path.end() - 1);
  } else {
    filtered_path = path;
  }

  // return the filtered path
  return filtered_path;
}

FinalPath BoundaryEstimationBackend::FinalPathSanityChecks(const FinalPath &final_path) {
  EASY_FUNCTION(profiler::colors::Red);
  FinalPath filtered_final_path;
  filtered_final_path.midpoints = FilterPath(final_path.midpoints);
  filtered_final_path.left_bound = FilterPath(final_path.left_bound);
  filtered_final_path.right_bound = FilterPath(final_path.right_bound);

  filtered_final_path.midpoints = FilterPathOverlay(filtered_final_path.midpoints);
  filtered_final_path.left_bound = FilterPathOverlay(filtered_final_path.left_bound);
  filtered_final_path.right_bound = FilterPathOverlay(filtered_final_path.right_bound);

  filtered_final_path.midpoints = CheckFirstAndLastPoint(filtered_final_path.midpoints);
  filtered_final_path.left_bound = CheckFirstAndLastPoint(filtered_final_path.left_bound);
  filtered_final_path.right_bound = CheckFirstAndLastPoint(filtered_final_path.right_bound);

  return filtered_final_path;
}

FinalPath BoundaryEstimationBackend::ComputeFullPath(const std::vector<std::string> &past_midpoints_ids) {
  EASY_FUNCTION(profiler::colors::Blue);
  std::unordered_map<std::string, bool> id_map;

  std::vector<std::pair<std::string, Point_2>> crossed_midpoints;
  std::vector<std::pair<std::string, Point_2>> crossed_left_boundary;
  std::vector<std::pair<std::string, Point_2>> crossed_right_boundary;

  // Iterate over the past midpoints id, extract the 2 left and right vertex id
  // and compute the midpoints
  for (const std::string &midpoint_id : past_midpoints_ids) {
    std::optional<Point_2> midpoint = ComputeMidpointFromId(midpoint_id);
    if (midpoint.has_value()) crossed_midpoints.push_back(std::make_pair(midpoint_id, midpoint.value()));

    std::string first_id = midpoint_id.substr(0, midpoint_id.find("-"));
    std::string second_id = midpoint_id.substr(midpoint_id.find("-") + 1);

    std::optional<std::pair<std::string, std::string>> boundary_points_ids =
        CheckFinalPathBoundaryIds(first_id, second_id);

    if (boundary_points_ids.has_value()) {
      if (id_map.find(boundary_points_ids.value().first) == id_map.end()) {
        id_map[boundary_points_ids.value().first] = true;
        crossed_left_boundary.push_back(std::make_pair(
            boundary_points_ids.value().first, ComputeVertexpointFromId(boundary_points_ids.value().first).value()));
      }
      if (id_map.find(boundary_points_ids.value().second) == id_map.end()) {
        id_map[boundary_points_ids.value().second] = true;
        crossed_right_boundary.push_back(std::make_pair(
            boundary_points_ids.value().second, ComputeVertexpointFromId(boundary_points_ids.value().second).value()));
      }
    }
  }

  FinalPath final_path;
  final_path.midpoints = crossed_midpoints;
  final_path.left_bound = crossed_left_boundary;
  final_path.right_bound = crossed_right_boundary;

  FinalPath sanitized_final_path = FinalPathSanityChecks(final_path);

  // append the first point to each array to close the loop
  sanitized_final_path.midpoints.push_back(final_path.midpoints.front());
  sanitized_final_path.left_bound.push_back(final_path.left_bound.front());
  sanitized_final_path.right_bound.push_back(final_path.right_bound.front());

  return sanitized_final_path;
}

std::string BoundaryEstimationBackend::ComputeMidpointIndex(const Vertex_handle &left_vertex,
                                                            const Vertex_handle &right_vertex) {
  // Extract the id from the two vertex
  int left_id = left_vertex->info().id;
  int right_id = right_vertex->info().id;
  // Do not consider virtual edges
  if (left_id < 0 || right_id < 0) {
    return "-";
  }
  return std::to_string(std::max(left_id, right_id)) + "-" + std::to_string(std::min(left_id, right_id));
}

void BoundaryEstimationBackend::InitializeCandidatePaths(const Face_handle &starting_face, const Point_2 &car_position,
                                                         const std::vector<Point_2> &past_middle_points,
                                                         const RelaxationType &relaxation_mode) {
  EASY_FUNCTION(profiler::colors::RichYellow);
  // initialize empty candidate paths vector
  candidates_paths_.clear();
  // initialize starting point of all the paths (current position)
  SinglePath init_path;
  init_path.faces.push_back(starting_face);
  init_path.midpoints.push_back(std::make_pair("-", car_position));
  init_path.path_length = 0.0;
  init_path.gate_cost = 0.0;
  init_path.path_angles.push_back(0.0);
  init_path.angle_max_cost = 0.0;
  init_path.presence_max_cost = 0.0;
  init_path.likelihood = 1.0;

  // Calculate the centroid (center) of the starting face
  Point_2 v0 = starting_face->vertex(0)->point();
  Point_2 v1 = starting_face->vertex(1)->point();
  Point_2 v2 = starting_face->vertex(2)->point();
  Point_2 face_center_point = CGAL::centroid(v0, v1, v2);

  // Initialize candidate paths from starting face
  for (int i = 0; i < 3; ++i) {
    Face_handle next_face = starting_face->neighbor(i);

    bool acyclic = CheckAcyclic(init_path, next_face);
    bool is_allowed = true;

    if (acyclic) {
      Vertex_handle first_left = starting_face->vertex(starting_face->cw(i));
      Vertex_handle first_right = starting_face->vertex(starting_face->ccw(i));

      Point_2 first_midpoint = CGAL::midpoint(first_left->point(), first_right->point());

      SinglePath grown_path = init_path;
      grown_path.faces.push_back(next_face);

      // Get path segment length
      // Use face center point to have better computation of track width, difference in length is negligible
      Kernel::Vector_2 next_vector(face_center_point, first_midpoint);
      double next_vector_length = std::sqrt(next_vector.squared_length());

      // Get distance between cones
      Kernel::Vector_2 right_to_left_vector(first_right->point(), first_left->point());
      double right_to_left_vector_length = std::sqrt(right_to_left_vector.squared_length());

      // Calculate the vectors to be used for finding the perpendicular
      // component of the triangle edge to the path (track width)
      next_vector = next_vector / next_vector_length;
      right_to_left_vector = right_to_left_vector / right_to_left_vector_length;

      // Car pose is at origin thus minus 0.0 can omitted
      double dx = first_midpoint.x();
      double dy = first_midpoint.y();
      double heading_of_path = std::atan2(dy, dx);
      // Car heading is also at 0.0 thus change in heading is equal to heading
      // of path
      double change_in_heading = heading_of_path;

      // Add gate width and intersection angle regardless of distance from it
      grown_path.gate_widths.push_back(right_to_left_vector_length);
      grown_path.gate_vector.push_back(right_to_left_vector);

      double nearest_distance_to_edge =
          PointToLineSegmentDistance(first_right->point().x(), first_right->point().y(), first_left->point().x(),
                                     first_left->point().y(), car_position.x(), car_position.y());

      // Calculate the track widths
      double intersection_angle = std::acos(next_vector * right_to_left_vector);
      double track_width = right_to_left_vector_length * std::sin(intersection_angle);

      // if the first midpoint is too close to the car, then control will have
      // problem with the steering angle to high, so we don't append to the path
      // the midpoint and don't update his likelihood but still we push it as a
      // candidate
      bool is_far_enough_from_midpoint = nearest_distance_to_edge > constants::kMaxMidpointDistanceFromCarPosition;
      if (is_far_enough_from_midpoint) {
        // if the path is not within the treshold then we can consider it, be we
        // don't update its info
        if (!CheckWithinThresholdConstraint(right_to_left_vector_length, track_width, change_in_heading,
                                            next_vector_length, relaxation_mode)) {
          is_allowed = false;
        }
        // Only update path info if the car is enough far from the midpoint AND
        // is within threshold
        if (is_allowed) {
          // update path info
          grown_path.path_length = init_path.path_length + next_vector_length;
          grown_path.path_angles.push_back(heading_of_path);
          grown_path.path_angle_changes.push_back(std::abs(change_in_heading));
          grown_path.track_widths.push_back(track_width);
        }
      }
      // if the car is not enough far from the first midpoint, it can still be a
      // good path, we just need to skip the first midpoint, so we still add it
      // to the candidates set, and update his posterior. On the other hand, if
      // the path is OK it's trivial that we want to add to the candidate set
      if (is_allowed) {
        // update bounds and midpoint
        std::string index = ComputeMidpointIndex(first_left, first_right);
        grown_path.midpoints.push_back(std::make_pair(index, first_midpoint));
        grown_path.left_bound.push_back(first_left);
        grown_path.right_bound.push_back(first_right);
        grown_path.filter_first_point = !is_far_enough_from_midpoint;
        // update cost used to compute the prior
        UpdateCostOfPath(grown_path);
        // Update likelihood
        UpdatePathLikelihood(grown_path);

        grown_path.prior = exp(-grown_path.total_cost * kFeatureWeights.prior_w);
        grown_path.posterior = grown_path.likelihood * grown_path.prior;

        // Push to candidate paths
        candidates_paths_.push_back(grown_path);
      }
    }
  }
}

SinglePath BoundaryEstimationBackend::BeamSearch(const std::vector<Point_2> &past_middle_points,
                                                 std::multimap<double, unsigned int> &current_ordered_leaves_ids,
                                                 const RelaxationType &relaxation_mode) {
  EASY_FUNCTION(profiler::colors::PaleGold);
  // Initialize the best path
  SinglePath best_path;
  best_path.posterior = -DBL_MAX;

  // Initialize the ordered map for the next leaves that will be created
  std::multimap<double, unsigned int> next_ordered_leaves_ids;

  for (unsigned int i = 0; i < kRuntimeConstraints.number_of_iterations; ++i) {
    while (!current_ordered_leaves_ids.empty()) {
      // Grow the best id (best has the largest posterior value and is placed at
      // the back)
      unsigned int parent_id = current_ordered_leaves_ids.rbegin()->second;
      // Erase the id once it has been stored in parent id
      current_ordered_leaves_ids.erase(prev(current_ordered_leaves_ids.end()));
      SinglePath grown_path_left;
      SinglePath grown_path_right;
      bool left_path_grown_successfully;
      bool right_path_grown_successfully;

      SinglePath path_to_grow = candidates_paths_.at(parent_id);

      LeafResult possible_paths = GrowLeaf(path_to_grow, past_middle_points, relaxation_mode);

      if (possible_paths.first.has_value()) {
        candidates_paths_.push_back(possible_paths.first.value());
        UpdateCandidatePathsAfterBeamSearch(possible_paths.first.value(), best_path, next_ordered_leaves_ids);
      }

      if (possible_paths.second.has_value()) {
        candidates_paths_.push_back(possible_paths.second.value());
        UpdateCandidatePathsAfterBeamSearch(possible_paths.second.value(), best_path, next_ordered_leaves_ids);
      }
    }

    // If no more leaves, search stops
    if (next_ordered_leaves_ids.empty()) {
      return best_path;
    } else {
      current_ordered_leaves_ids = next_ordered_leaves_ids;
      next_ordered_leaves_ids.clear();
    }
  }
  return best_path;
}

void BoundaryEstimationBackend::UpdateCandidatePathsAfterBeamSearch(
    const SinglePath &new_path, SinglePath &best_path, std::multimap<double, unsigned int> &next_ordered_leaves_ids) {
  EASY_FUNCTION(profiler::colors::Olive);
  unsigned int path_id = candidates_paths_.size() - 1;

  filtered_candidates_paths_viz_.insert(std::pair<double, SinglePath>(new_path.posterior, new_path));

  if (new_path.posterior > best_path.posterior && new_path.midpoints.size() >= 2 &&
      new_path.path_length >= kThresholdValues.minimum_path_length_m) {
    best_path = new_path;
  }
  next_ordered_leaves_ids.insert(std::pair<double, unsigned int>(new_path.posterior, path_id));

  // Remove worst leaf
  if (next_ordered_leaves_ids.size() > kRuntimeConstraints.max_number_of_leaves) {
    next_ordered_leaves_ids.erase(next_ordered_leaves_ids.begin());
  }
}

LeafResult BoundaryEstimationBackend::GrowLeaf(const SinglePath &path_to_grow,
                                               const std::vector<Point_2> &past_middle_points,
                                               const RelaxationType &relaxation_mode) {
  EASY_FUNCTION(profiler::colors::LightGreen);
  Vertex_handle current_left = path_to_grow.left_bound.back();
  Vertex_handle current_right = path_to_grow.right_bound.back();
  Face_handle current_face = path_to_grow.faces.back();

  // If path doesn't go somewhere we return empty path both for left and right
  if (graph_->IsOutsideTheGraph(current_face)) return std::make_pair(std::nullopt, std::nullopt);

  Vertex_handle next_vertex = current_face->vertex(current_face->cw(current_face->index(current_left)));
  Face_handle next_face_left = current_face->neighbor(current_face->index(current_right));
  Face_handle next_face_right = current_face->neighbor(current_face->index(current_left));

  std::optional<SinglePath> left = ExtendPathToNextVertex(path_to_grow, next_face_left, Left, next_vertex, current_left,
                                                          past_middle_points, relaxation_mode);

  std::optional<SinglePath> right = ExtendPathToNextVertex(path_to_grow, next_face_right, Right, next_vertex,
                                                           current_right, past_middle_points, relaxation_mode);

  return std::make_pair(left, right);
}

Point_2 BoundaryEstimationBackend::FindPointForHeading(const std::vector<std::pair<std::string, Point_2>> &midpoints,
                                                       const bool &filter_first_point) {
  if (midpoints.size() == 2 && filter_first_point) {
    return Point_2(0, 0);
  }
  // return last midpoint
  return midpoints.rbegin()[0].second;
}

std::optional<SinglePath> BoundaryEstimationBackend::ExtendPathToNextVertex(
    const SinglePath &path_to_grow, const Face_handle &next_face, const Directions directions,
    const Vertex_handle &next_vertex, const Vertex_handle &complement_of_next_vertex,
    const std::vector<Point_2> &past_middle_points, const RelaxationType &relaxation_mode) {
  EASY_FUNCTION(profiler::colors::DarkGreen);
  SinglePath grown_path = path_to_grow;

  bool acyclic = CheckAcyclic(path_to_grow, next_face);
  bool non_dual_use_of_cone;
  bool is_allowed = true;

  if (directions == Left) {
    non_dual_use_of_cone = std::find(path_to_grow.right_bound.begin(), path_to_grow.right_bound.end(), next_vertex) ==
                           path_to_grow.right_bound.end();
  } else {
    non_dual_use_of_cone = std::find(path_to_grow.left_bound.begin(), path_to_grow.left_bound.end(), next_vertex) ==
                           path_to_grow.left_bound.end();
  }

  // Stop if path is cyclic or cones from one side is now being reused on the
  // other side
  if (!acyclic || !non_dual_use_of_cone) {
    return std::nullopt;
  }

  Point_2 next_midpoint = CGAL::midpoint(complement_of_next_vertex->point(), next_vertex->point());

  grown_path.faces.push_back(next_face);
  // Last midpoint, used for heading and width calculation.
  Point_2 last_midpoint = path_to_grow.midpoints.rbegin()[0].second;
  // Get path segment length
  Kernel::Vector_2 next_vector_wo_vp(last_midpoint, next_midpoint);
  double next_vector_wo_vp_length = std::sqrt(next_vector_wo_vp.squared_length());

  // Get distance between cones
  Kernel::Vector_2 next_to_complement_vector(next_vertex->point(), complement_of_next_vertex->point());
  double next_to_complement_vector_length = std::sqrt(next_to_complement_vector.squared_length());

  // Calculate the vector to be used for finding the perpendicular component
  // of the triangle edge to the path (track width)
  next_to_complement_vector = next_to_complement_vector / next_to_complement_vector_length;
  next_vector_wo_vp = next_vector_wo_vp / next_vector_wo_vp_length;

  // Get heading
  double dx_wo_vp = next_midpoint.x() - last_midpoint.x();
  double dy_wo_vp = next_midpoint.y() - last_midpoint.y();
  double next_heading_wo_vp = std::atan2(dy_wo_vp, dx_wo_vp);
  double change_in_heading = AngleDifferenceRad(next_heading_wo_vp, path_to_grow.path_angles.back());

  // Add gate width and intersection angle regardless of distance from it
  grown_path.gate_widths.push_back(next_to_complement_vector_length);
  grown_path.gate_vector.push_back(next_to_complement_vector);

  double distance_of_cones_at_same_bound_apart = 0;
  // Update bound distance
  int size_need_to_compute_distance = path_to_grow.filter_first_point ? 1 : 0;
  if (directions == Left && path_to_grow.right_bound.size() > size_need_to_compute_distance) {
    distance_of_cones_at_same_bound_apart =
        std::sqrt(CGAL::squared_distance(path_to_grow.right_bound.back()->point(), next_vertex->point()));
  } else if (directions == Right && path_to_grow.left_bound.size() > size_need_to_compute_distance) {
    distance_of_cones_at_same_bound_apart =
        std::sqrt(CGAL::squared_distance(path_to_grow.left_bound.back()->point(), next_vertex->point()));
  }

  // Calculate the track widths
  double intersection_angle = std::acos(next_vector_wo_vp * next_to_complement_vector);
  double track_width = next_to_complement_vector_length * std::sin(intersection_angle);

  double nearest_distance_to_edge = PointToLineSegmentDistance(
      complement_of_next_vertex->point().x(), complement_of_next_vertex->point().y(), next_vertex->point().x(),
      next_vertex->point().y(), last_midpoint.x(), last_midpoint.y());

  // Same logic as per InitializeCandidatePaths
  if (nearest_distance_to_edge > kThresholdValues.minimum_path_segment_length_m) {
    if (!CheckWithinThresholdConstraint(next_to_complement_vector_length, track_width, change_in_heading,
                                        next_vector_wo_vp_length, relaxation_mode)) {
      is_allowed = false;
    }

    if (directions == Left) {
      is_allowed = is_allowed && CheckBoundarySegments(grown_path, distance_of_cones_at_same_bound_apart, next_vertex,
                                                       NULL, relaxation_mode);
    } else {
      is_allowed = is_allowed && CheckBoundarySegments(grown_path, distance_of_cones_at_same_bound_apart, NULL,
                                                       next_vertex, relaxation_mode);
    }

    // only if the segment is enough long and the path is valid
    if (is_allowed) {
      // update path info
      grown_path.path_length = path_to_grow.path_length + next_vector_wo_vp_length;
      grown_path.track_widths.push_back(track_width);
      grown_path.path_angles.push_back(next_heading_wo_vp);
      grown_path.path_angle_changes.push_back(
          std::abs(AngleDifferenceRad(grown_path.path_angles.end()[-1], grown_path.path_angles.end()[-2])));
    }
  }

  if (!is_allowed) return std::nullopt;

  std::string index = ComputeMidpointIndex(complement_of_next_vertex, next_vertex);
  grown_path.midpoints.push_back(std::make_pair(index, next_midpoint));

  // Update the bounds
  if (directions == Left) {
    grown_path.right_bound.push_back(next_vertex);
    grown_path.end_bound_type = Right;
    if (grown_path.left_bound.empty()) {
      grown_path.left_bound.push_back(complement_of_next_vertex);
    }
  } else {
    grown_path.left_bound.push_back(next_vertex);
    grown_path.end_bound_type = Left;
    if (grown_path.right_bound.empty()) {
      grown_path.right_bound.push_back(complement_of_next_vertex);
    }
  }

  // Update distance between cones
  if (distance_of_cones_at_same_bound_apart > constants::kMinimumDistanceSameBoundCones) {
    if (directions == Left) {
      grown_path.right_cones_distance_apart.push_back(distance_of_cones_at_same_bound_apart);
    } else {
      grown_path.left_cones_distance_apart.push_back(distance_of_cones_at_same_bound_apart);
    }
  }
  // update cost
  UpdateCostOfPath(grown_path);
  // update likelihood (going Left means the next vertex is Right and
  // vice-versa)
  UpdatePathLikelihood(grown_path);

  grown_path.prior = exp(-grown_path.total_cost * kFeatureWeights.prior_w);
  grown_path.posterior = grown_path.likelihood * grown_path.prior;
  return grown_path;
}

void BoundaryEstimationBackend::UpdatePathLikelihood(SinglePath &grown_path) {
  EASY_FUNCTION(profiler::colors::DarkTeal);
  // Initialize color likelihood
  double color_likelihood = 1.0;

  // Calculate the length of the path
  int total_length = grown_path.path_length;

  // Calculate color likelihood for left bound
  for (const Vertex_handle &left_vertex : grown_path.left_bound) {
    // Continue if it doesn't have color information (lidar pipeline)
    double max_orange_color_prob = std::max(left_vertex->info().orange_prob, left_vertex->info().orange_big_prob);
    double max_color_prob = std::max(left_vertex->info().yellow_prob, left_vertex->info().blue_prob);
    if (max_color_prob == max_orange_color_prob && max_color_prob == 0.25) continue;
    // Compute the entropy of the cone
    std::vector<double> probs = {left_vertex->info().orange_prob, left_vertex->info().orange_big_prob,
                                 left_vertex->info().yellow_prob, left_vertex->info().blue_prob};
    double entropy = ComputeEntropy(probs);
    if (entropy > constants::kMaxEntropyToConsiderColor) {
      continue;
    }
    bool is_orange = max_orange_color_prob > max_color_prob;
    if (is_orange && max_orange_color_prob >= 0.25) continue;
    color_likelihood *= left_vertex->info().blue_prob;
  }

  // Calculate color likelihood for right bound
  for (const Vertex_handle &right_vertex : grown_path.right_bound) {
    double max_orange_color_prob = std::max(right_vertex->info().orange_prob, right_vertex->info().orange_big_prob);
    double max_color_prob = std::max(right_vertex->info().yellow_prob, right_vertex->info().blue_prob);
    // Continue if it doesn't have color information
    if (max_color_prob == max_orange_color_prob && max_color_prob == 0.25) continue;
    std::vector<double> probs = {right_vertex->info().orange_prob, right_vertex->info().orange_big_prob,
                                 right_vertex->info().yellow_prob, right_vertex->info().blue_prob};
    double entropy = ComputeEntropy(probs);
    if (entropy > constants::kMaxEntropyToConsiderColor) {
      continue;
    }
    bool is_orange = max_orange_color_prob > max_color_prob;
    if (is_orange && max_orange_color_prob >= 0.25) continue;
    color_likelihood *= right_vertex->info().yellow_prob;
  }

  // Normalize the color likelihood by the total length of the path
  if (total_length > 0) {
    color_likelihood = std::pow(color_likelihood, 1.0 / total_length);
  }
  // Update the likelihood of the grown path
  grown_path.likelihood *= color_likelihood;
}

void BoundaryEstimationBackend::UpdateCostOfPath(SinglePath &path) const {
  EASY_FUNCTION(profiler::colors::DarkBrown);

  double path_deviation = std::abs(path.path_length - kFeatureSetpoints.desired_length_m);
  path.length_cost = path_deviation / kFeatureSetpoints.desired_length_m;

  double max_angle_deviation = Max(path.path_angle_changes) - kFeatureSetpoints.desired_max_angle_change_rad;
  path.angle_max_cost = std::max(0.0, max_angle_deviation / kFeatureSetpoints.desired_max_angle_change_rad);

  double track_width_deviation = StandardDeviation(path.track_widths) - kFeatureSetpoints.desired_witdh_std_m;
  path.width_var_cost = std::max(0.0, track_width_deviation / kFeatureSetpoints.desired_witdh_std_m);

  double left_cone_distance_deviation =
      StandardDeviation(path.left_cones_distance_apart) - kFeatureSetpoints.desired_boundary_std_m;
  path.left_distance_var_cost = std::max(0.0, left_cone_distance_deviation / kFeatureSetpoints.desired_boundary_std_m);

  double right_cone_distance_deviation =
      StandardDeviation(path.right_cones_distance_apart) - kFeatureSetpoints.desired_boundary_std_m;
  path.right_distance_var_cost =
      std::max(0.0, right_cone_distance_deviation / kFeatureSetpoints.desired_boundary_std_m);

  path.total_cost =
      (kFeatureWeights.length_error_w * path.length_cost) + (kFeatureWeights.track_width_w * path.width_var_cost) +
      (kFeatureWeights.step_angle_w * path.angle_max_cost) +
      (kFeatureWeights.cones_distance_var_w * (path.left_distance_var_cost + path.right_distance_var_cost));
}

std::optional<Face_handle> BoundaryEstimationBackend::InitializeStartingFace(const Point_2 &car_position) {
  Face_handle starting_face = graph_->LocateInsideTheGraph(car_position);
  // If heading_point_short_ outside convex hull
  if (starting_face == nullptr || graph_->IsOutsideTheGraph(starting_face)) {
    return std::nullopt;
  }
  return starting_face;
}

std::optional<Face_handle>
BoundaryEstimationBackend::InitializeStartingFaceWithVirtualPoints(const Point_2 &car_position) {
  // try first to localice inside the convex hull, if impossible add virtual
  // points
  std::optional<Face_handle> initial_face = InitializeStartingFace(car_position);
  if (initial_face.has_value()) return initial_face.value();

  // Create a vertex slightly (1.0m) behind car
  Point_2 new_vertex_position_right = Point_2(car_position.x() - 1.0, car_position.y() - 1.0);
  Point_2 new_vertex_position_left = Point_2(car_position.x() - 1.0, car_position.y() + 1.0);

  // Prevents new vertex from being too close (1m) to existing cone vertex
  bool add_new_vertex_right = true;
  bool add_new_vertex_left = true;

  std::vector<Vertex_handle> vertex_list = graph_->GetVertices();
  for (const auto &vertex : vertex_list) {
    Kernel::Vector_2 vector1(vertex->point(), new_vertex_position_right);
    Kernel::Vector_2 vector2(vertex->point(), new_vertex_position_left);

    if (vector1.squared_length() < 1) {
      add_new_vertex_right = false;
    }

    if (vector2.squared_length() < 1) {
      add_new_vertex_left = false;
    }
  }

  std::vector<std::pair<Point_2, VertexInfo>> points_vector;

  // Virtual right vertex
  if (add_new_vertex_right) {
    VertexInfo new_vertex_info_right;
    new_vertex_info_right.id = -1;
    new_vertex_info_right.pipeline = constants::kSensorFusion;
    new_vertex_info_right.blue_prob = 0.0;
    new_vertex_info_right.yellow_prob = 1.0;
    new_vertex_info_right.orange_prob = 0.0;
    new_vertex_info_right.orange_big_prob = 0.0;
    new_vertex_info_right.presence_prob = 1.0;
    new_vertex_info_right.virtual_flag = true;

    points_vector.push_back(std::make_pair(new_vertex_position_right, new_vertex_info_right));
  }

  // Virtual left vertex
  if (add_new_vertex_left) {
    VertexInfo new_vertex_info_left;
    new_vertex_info_left.id = -1;
    new_vertex_info_left.pipeline = constants::kSensorFusion;
    new_vertex_info_left.blue_prob = 1.0;
    new_vertex_info_left.yellow_prob = 0.0;
    new_vertex_info_left.orange_prob = 0.0;
    new_vertex_info_left.orange_big_prob = 0.0;
    new_vertex_info_left.presence_prob = 1.0;
    new_vertex_info_left.virtual_flag = true;

    points_vector.push_back(std::make_pair(new_vertex_position_left, new_vertex_info_left));
  }

  graph_->InsertVertexList(points_vector);

  Face_handle starting_face = graph_->LocateInsideTheGraph(car_position);

  // If still outside convex hull
  if (graph_->IsOutsideTheGraph(starting_face)) {
    return std::nullopt;
  }

  return starting_face;
}

bool BoundaryEstimationBackend::IsFaceVirtual(const Face_handle &face) {
  for (int i = 0; i < 3; ++i) {
    if (face->vertex(i)->info().virtual_flag) {
      return true;
    }
  }
  return false;
}

bool BoundaryEstimationBackend::IsPointInsideStartingFace(const Point_2 &point) {
  Face_handle located_face = graph_->LocateInsideTheGraph(point);
  std::vector<int> located_vertices_ids;

  for (int i = 0; i < 3; ++i) {
    located_vertices_ids.push_back(located_face->vertex(i)->info().id);
  }

  std::sort(located_vertices_ids.begin(), located_vertices_ids.end());

  return initial_face_ids_ == located_vertices_ids;
}

bool BoundaryEstimationBackend::CheckWithinThresholdConstraint(const double &next_distance_betweeen_cones,
                                                               const double &next_track_width,
                                                               const double &next_angle_change,
                                                               const double &next_path_segment_length,
                                                               const RelaxationType &relaxation_mode) {
  for (int relaxation_index = None; relaxation_index <= relaxation_mode; relaxation_index++) {
    bool is_allowed = true;
    RelaxationType temp_relaxation_mode = static_cast<RelaxationType>(relaxation_index);

    if (temp_relaxation_mode == Total) return true;

    if (next_track_width <=
            kThresholdValues.minimum_track_width_m - relaxation_index * kOffsetValues.minimum_track_width_offset_m ||
        next_track_width >=
            kThresholdValues.maximum_track_width_m + relaxation_index * kOffsetValues.maximum_track_width_offset_m) {
      // if(logging_flag_)
      //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("delaunay_search"),
      //                   "[boundary_estimation] The track width is not compliant: "<<next_track_width);
      is_allowed = false;
    }
    if (is_allowed &&
        (next_distance_betweeen_cones <= kThresholdValues.minimum_segment_width_m -
                                             relaxation_index * kOffsetValues.minimum_segment_width_offset_m ||
         next_distance_betweeen_cones >= kThresholdValues.maximum_segment_width_m +
                                             relaxation_index * kOffsetValues.maximum_segment_width_offset_m)) {
      // if(logging_flag_)
      //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("delaunay_search"),
      //                    "[boundary_estimation] The edge is not compliant");
      is_allowed = false;
    }
    if (is_allowed &&
        std::abs(next_angle_change) >= kThresholdValues.maximum_angle_change_rad +
                                           relaxation_index * kOffsetValues.maximum_angle_change_offset_rad) {
      // if(logging_flag_)
      //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("delaunay_search"),
      //                    "[boundary_estimation] The angle change is not compliant");
      is_allowed = false;
    }
    if (is_allowed &&
        next_path_segment_length >= kThresholdValues.maximum_path_segment_length_m +
                                        relaxation_index * kOffsetValues.maximum_path_segment_length_offset_m) {
      // if(logging_flag_)
      //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("delaunay_search"),
      //                    "[boundary_estimation] The segment is not compliant");
      is_allowed = false;
    }
    if (!is_allowed) continue;
    return true;
  }
  return false;
}

bool BoundaryEstimationBackend::CheckBoundarySegments(SinglePath path, double distance_of_cones,
                                                      Vertex_handle next_right, Vertex_handle next_left,
                                                      const RelaxationType &relaxation_mode) {
  // Check if both boundaries have too distant subsequent cones
  // This is done to allow sporadic missing cones in one of the two boundaries,
  // which could lead to longer segment
  for (int relaxation_index = None; relaxation_index <= relaxation_mode; relaxation_index++) {
    bool is_allowed = true;
    int number_of_too_distant_cones = 0;
    RelaxationType temp_relaxation_mode = static_cast<RelaxationType>(relaxation_index);

    if (temp_relaxation_mode == Total) return true;

    if (distance_of_cones > kThresholdValues.maximum_boundary_segment_length_m) {
      number_of_too_distant_cones++;
    }
    for (unsigned int j = 0; j < path.left_cones_distance_apart.size(); ++j) {
      if (path.left_cones_distance_apart.at(j) > kThresholdValues.maximum_boundary_segment_length_m) {
        number_of_too_distant_cones++;
      }
    }
    for (unsigned int j = 0; j < path.right_cones_distance_apart.size(); ++j) {
      if (path.right_cones_distance_apart.at(j) > kThresholdValues.maximum_boundary_segment_length_m) {
        number_of_too_distant_cones++;
      }
    }
    is_allowed =
        number_of_too_distant_cones < kThresholdValues.maximum_number_of_far_distant_cones +
                                          relaxation_index * kOffsetValues.maximum_number_of_far_distant_cones_offset;
    if (!is_allowed) {
      // RCLCPP_ERROR(rclcpp::get_logger("delaunay_search"),
      //                     "[boundary_estimation] The segment is not compliant, too many far away cones with
      //                     relaxation type  %i ", relaxation_index);
      continue;
    }
    return is_allowed;
  }
  return false;
}

bool BoundaryEstimationBackend::CheckAcyclic(const SinglePath &path_to_be_expanded, const Face_handle &next_face) {
  if (std::find(path_to_be_expanded.faces.begin(), path_to_be_expanded.faces.end(), next_face) ==
      path_to_be_expanded.faces.end()) {
    return true;
  }

  return false;
}

void BoundaryEstimationBackend::FilterLastPathSegment(SinglePath &filtered_path) {
  if (filtered_path.path_angle_changes.back() > kThresholdValues.maximum_end_angle_change_rad) {
    if (filtered_path.end_bound_type == Left && filtered_path.left_bound.size() >= 3) {
      filtered_path.left_bound.pop_back();
      filtered_path.midpoints.pop_back();
    } else if (filtered_path.end_bound_type == Right && filtered_path.right_bound.size() >= 3) {
      filtered_path.right_bound.pop_back();
      filtered_path.midpoints.pop_back();
    }
  }
}

} // namespace boundary_estimation
