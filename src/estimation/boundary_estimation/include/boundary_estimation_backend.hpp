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

#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "autonomous_msgs/msg/cone.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "boundary_estimation_graph.hpp"
#include "boundary_estimation_structs.hpp"
#include "helpers.hpp"

namespace boundary_estimation {

// Result of the function GrowLeaf, first element is Left path, second is Right
typedef std::pair<std::optional<SinglePath>, std::optional<SinglePath>> LeafResult;

// Struct that stores all the result of the backend and pass it to the frontend
struct BoundaryLogicResult {
  std::vector<std::pair<Point_2, Point_2>> triangulation_edges;
  std::vector<std::vector<Point_2>> filtered_path_list;
  std::vector<std::pair<std::string, Point_2>> past_middle_points;
  std::vector<std::pair<std::string, Point_2>> last_past_middle_points;
  std::vector<std::vector<Point_2>> candidates;
  std::vector<std::pair<std::string, Point_2>> midpoints;
  std::vector<std::pair<std::string, Point_2>> left_bound;
  std::vector<std::pair<std::string, Point_2>> right_bound;
  double posterior;
  bool success = false;
};

struct FinalPath {
  std::vector<std::pair<std::string, Point_2>> midpoints;
  std::vector<std::pair<std::string, Point_2>> left_bound;
  std::vector<std::pair<std::string, Point_2>> right_bound;
};

struct CrossedPath {
  std::vector<std::pair<std::string, Point_2>> midpoints;
  std::vector<std::pair<std::string, Point_2>> left_bound;
  std::vector<std::pair<std::string, Point_2>> right_bound;
};

class BoundaryEstimationBackend {
public:
  BoundaryEstimationBackend(const heuristics::FeatureSetpoints &feature_setpoints,
                            const heuristics::FeatureWeights &feature_weights,
                            const heuristics::ThresholdValues &threshold_values,
                            const heuristics::OffsetValues &offset_values, const heuristics::ColorParams &color_params,
                            const heuristics::RuntimeConstraints &runtime_constraints, const bool &logging_flag = false,
                            const bool &debug_flag = false)
      : kFeatureSetpoints(feature_setpoints), kFeatureWeights(feature_weights), kThresholdValues(threshold_values),
        kOffsetValues(offset_values), kColorParams(color_params), kRuntimeConstraints(runtime_constraints),
        logging_flag_(logging_flag), debug_flag_(debug_flag) {}
  /**
   - Generate candidate path and add corresponding vertices.
  -
  - This function generates a candidate path based on observed cones and
  previous car positions.
  - It computes the middle line, left and right boundaries, and triangulation.
  - If a valid path is found, it filters the path and extracts its points for
  further processing.
  -
  - Parameters:
  -   - cones_observed: Cone array received.
  -   - past_car_positions: Vector of previous car positions.
  -   - past_midpaths_with_posterior: History of previously computed midpath
  used for temporal similarity check.
  -
  - Returns:
  -   BoundaryLogicResult containing generated paths and vertices.
  -   If a path is successfully generated, successfull flag will be set to true,
  otherwise false.
  */
  BoundaryLogicResult ComputeBoundedPath(const std::vector<autonomous_msgs::msg::Cone> &cones_observed,
                                         const Point_2 &past_car_position,
                                         const std::vector<std::string> &past_midpoints_ids);

  FinalPath ComputeFullPath(const std::vector<std::string> &past_midpoints_ids);

  // Getter for visualization's data
  std::vector<std::vector<Point_2>> GetCandidatePaths();
  std::vector<std::vector<Point_2>> GetFilteredCandidatePaths();

private:
  /**
   * Uses Delaunay triangulation over the full mesh to find the starting face
   * for boundary computation (where the car is located).
   *
   * Parameters:
   *   - car_position: The (x, y) position of the car.
   *
   * Returns:
   *   - std::optional<Face_handle>: The triangle face where the car is located,
   * wrapped in an optional. Returns std::nullopt if unsuccessful.
   */
  std::optional<Face_handle> InitializeStartingFace(const Point_2 &car_position);

  /**
   * Uses Delaunay triangulation with added virtual points in order to compute
   * the starting face even when impossible with only the real vertices.
   *
   * Parameters:
   *   - car_position: The (x, y) position of the car.
   *
   * Returns:
   *   - std::optional<Face_handle>: The triangle face where the car is located,
   * wrapped in an optional. Returns std::nullopt if unsuccessful.
   */
  std::optional<Face_handle> InitializeStartingFaceWithVirtualPoints(const Point_2 &car_position);

  /**
   - Candidate paths initialization, beginning at starting face.
  -
  - This function initializes candidate paths starting from a given face in the
  Delaunay triangulation.
  - It computes various attributes of the candidate paths, such as midpoints,
  gate widths, track widths,
  - path angles, and updates their likelihood and posterior probabilities.
  -
  - Parameters:
  -   - starting_face: Triangle face where the car is located.
  -   - car_position: (x,y) position of the car.
  -   - past_middle_points: Vector of previous middle line points.
  */
  void InitializeCandidatePaths(const Face_handle &starting_face, const Point_2 &car_position,
                                const std::vector<Point_2> &past_middle_points, const RelaxationType &relaxation_mode);
  /**
   - Generate candidate paths and use beam search to find the best one.
  -
  - This function generates candidate and uses beam search
  - to find the best path among the candidates. It initializes starting triangle
  face
  - with virtual points and sets up candidate paths for further processing.
  -
  - Parameters:
  -   - past_middle_points: Vector of previous middle line points.
  -   - past_midpaths_with_posterior: History of previously computed midpath
  used for temporal similarity check.
  -
  - Returns:
  -   An optional SinglePath representing the best path found by beam search, if
  successfully computed.
  -   Otherwise, returns std::nullopt.
  */
  std::optional<SinglePath> ComputeBestPath(const std::vector<Point_2> &past_middle_points);

  /*
    Perform checks against threshold parameters to determine if the given values
    fall within the acceptable range.

  Input:
    - next_distance_betweeen_cones: Distance between cones
    - next_track_width: Track width
    - next_angle_change: Angle change
    - next_path_segment_length: Length of the path segment

  Returns:
    - bool: True if all values fall within the acceptable range defined by the
  threshold parameters, false otherwise.
  */
  bool CheckWithinThresholdConstraint(const double &next_distance_betweeen_cones, const double &next_track_width,
                                      const double &next_angle_change, const double &next_path_segment_length,
                                      const RelaxationType &relaxation_mode);

  /*
  Check that the path expansion does not create cycles.

  Input:
      - path_to_be_expanded: The path to be expanded.
      - next_face: The next face to expand into.
      - past_middle_points: Vector of previous middle line points.

  Returns:
      - bool: True if the path expansion does not create cycles, false
  otherwise.
  */
  bool CheckAcyclic(const SinglePath &path_to_be_expanded, const Face_handle &next_face);

  /**
   * UpdateCostOfPath
   *
   * Updates the cost of a path based on the weights and parameters.
   *
   * Parameters:
   *   - path: The path to evaluate and update its cost.
   */
  void UpdateCostOfPath(SinglePath &path) const;

  /**
   * Updates the likelihood of a path based on the cone it encounters.
   *
   * Parameters:
   *   - cone: The cone vertex whose probability is used to update the
   * likelihood.
   *   - grown_path: The path whose likelihood is being updated.
   *   - direction: The direction of the cone relative to the path (Left or
   * Right).
   *
   * Behavior:
   *   - If the cone is detected by sensors fusion or camera only and it's not
   * orange:
   *     - If it's a left cone:
   *       - If the cone is most likely to be the opposite color, a counter is
   * increased.
   *       - If the counter exceeds a threshold, the likelihood is decreased
   * significantly.
   *       - Otherwise, the likelihood is increased slightly.
   *     - If it's a right cone:
   *       - Similar to the left cone case.
   *   - If the cone is detected by lidar only:
   *     - A counter for lidar cones used is incremented.
   *     - If the number of lidar cones used is within a certain range, the
   * likelihood is increased slightly.
   *
   * Behavior Notes:
   *   - The likelihood adjustment is based on color constraints and tolerance
   * parameters.
   *   - If the likelihood falls below a certain threshold, it is set to
   * negative infinity.
   */
  void UpdatePathLikelihood(SinglePath &grown_path);

  /*
    Get previous middle line points
    Input:
      - past_car_positions = vector of previous car positions
  */
  CrossedPath ComputePastMiddlePoints(const std::vector<std::string> &past_midpoints_ids,
                                      const CrossedPath &last_crossed_midpoints);

  /**
   - Beam search implementation.
  -
  - This function implements beam search to find the best path among candidate
  paths.
  - It iteratively grows paths and selects the best path based on posterior
  probabilities.
  -
  - Parameters:
  -   - past_middle_points: Vector of previous middle line points.
  -   - past_midpaths_with_posterior: History of previously computed midpath
  used for temporal similarity check.
  -   - current_ordered_leaves_ids: Current map of leaves for beam search.
  -
  - Returns:
  -   The best path found by beam search.
  */
  SinglePath BeamSearch(const std::vector<Point_2> &past_middle_points,
                        std::multimap<double, unsigned int> &current_ordered_leaves_ids,
                        const RelaxationType &relaxation_mode);

  /**
   * This function extends a path in the beam search tree towards left and right
   * directions. It computes the next vertices and faces for both left and right
   * extensions, then grows the path accordingly.
   *
   * Parameters:
   *   - path_to_grow: Path to extend.
   *   - past_middle_points: Vector of previous middle line points.
   *   - past_midpaths_with_posterior: History of previously computed midpath
   * used for temporal similarity check.
   *
   * Returns:
   *   A pair of optional SinglePath objects representing the left and right
   * extended paths. If successful, the pair contains the extended paths;
   * otherwise, it contains std::nullopt.
   */
  LeafResult GrowLeaf(const SinglePath &path_to_grow, const std::vector<Point_2> &past_middle_points,
                      const RelaxationType &relaxation_mode);

  /**
   * This function extends a path in the beam search tree towards the next
   * vertex, considering constraints such as path acyclicity and non-dual use of
   * cones.
   *
   * Parameters:
   *   - path_to_extend: Path to extend.
   *   - past_midpaths_with_posterior: History of previously computed midpath
   *                                    used for temporal similarity check.
   *   - next_face: Triangular face of the next vertex.
   *   - directions: Direction of extension (Left or Right).
   *   - next_vertex: Next vertex for triangulation.
   *   - complement_of_next_vertex: Complement vertex of the next vertex.
   *   - past_middle_points: Vector of previous middle line points.
   *
   * Returns:
   *   An optional SinglePath object representing the extended path if
   * successful, otherwise std::nullopt.
   */
  std::optional<SinglePath> ExtendPathToNextVertex(const SinglePath &path_to_grow, const Face_handle &next_face,
                                                   const Directions directions, const Vertex_handle &next_vertex,
                                                   const Vertex_handle &complement_of_next_vertex,
                                                   const std::vector<Point_2> &past_middle_points,
                                                   const RelaxationType &relaxation_mode);

  /**
   * UpdateCandidatePathsAfterBeamSearch
   *
   * This function updates the vector of candidate paths and associated data
   * structures after performing beam search. It adds a newly computed path to
   * the candidate paths, updates the best path, and adjusts the map of leaves
   * for the next iteration of beam search.
   *
   * Parameters:
   *   - new_path: The newly computed path.
   *   - best_path: The best path among the candidates.
   *   - next_ordered_leaves_ids: The map of leaves for the next iteration of
   * beam search.
   */
  void UpdateCandidatePathsAfterBeamSearch(const SinglePath &new_path, SinglePath &best_path,
                                           std::multimap<double, unsigned int> &next_ordered_leaves_ids);

  /*
  Check if the boundary segments of the path meet certain criteria, including
  distance between cones and angle of boundary segments.

  Input:
      - path: The path whose boundary segments are to be checked.
      - distance_of_cones: The distance between cones.
      - next_right: Pointer to the next vertex on the right boundary.
      - next_left: Pointer to the next vertex on the left boundary.

  Returns:
      - bool: True if the boundary segments meet the criteria, false otherwise.
  */
  bool CheckBoundarySegments(SinglePath path, double distance_of_cones, Vertex_handle next_right,
                             Vertex_handle next_left, const RelaxationType &relaxation_mode);

  /*
  Check if the angles between boundary segments meet certain criteria.

  Input:
      - boundary: Vector of vertices representing the boundary.
      - next_vertex: Pointer to the next vertex.
      - direction: Enum indicating if the next vertex is Left or Right
  is correct.

  Returns:
      - bool: True if the angles between boundary segments meet the criteria,
  false otherwise.
*/
  bool CheckBoundaryAngles(std::vector<Vertex_handle> boundary, Vertex_handle next_vertex, const Directions &direction);

  /*
  Function to remove the last segment of the path if the last angle change
  exceeds a threshold.

  Input:
      - filtered_path: The path to filter.

  Output:
      - void
  */
  void FilterLastPathSegment(SinglePath &filtered_path);

  std::vector<std::pair<std::string, Point_2>>
  ConcatenateFinalPoints(const std::vector<std::pair<std::string, Point_2>> &past_midpoints,
                         const std::vector<std::pair<std::string, Point_2>> &current_midpoints,
                         const bool &is_midpoint);

  std::string ComputeMidpointIndex(const Vertex_handle &left_vertex, const Vertex_handle &right_vertex);

  Point_2 FindPointForHeading(const std::vector<std::pair<std::string, Point_2>> &midpoints,
                              const bool &filter_first_point);

  std::optional<Point_2> ComputeMidpointFromId(const std::string midpoint_id);

  std::optional<Point_2> ComputeVertexpointFromId(std::string vertex_id);

  CrossedPath ComputeCrossedPath(const Point_2 &current_point, const Point_2 &previous_point,
                                 const bool &midpoint_only = false);

  std::optional<std::pair<std::string, std::string>> CheckFinalPathBoundaryIds(const std::string &first_id,
                                                                               const std::string &second_id);

  std::vector<std::pair<std::string, Point_2>> FilterPath(const std::vector<std::pair<std::string, Point_2>> &path);

  std::vector<std::pair<std::string, Point_2>>
  CheckFirstAndLastPoint(const std::vector<std::pair<std::string, Point_2>> &path);

  std::vector<std::pair<std::string, Point_2>>
  FilterPathOverlay(const std::vector<std::pair<std::string, Point_2>> &path);

  FinalPath FinalPathSanityChecks(const FinalPath &final_path);

  bool IsPointInsideStartingFace(const Point_2 &point);

  bool IsFaceVirtual(const Face_handle &face);

  std::multimap<double, SinglePath> filtered_candidates_paths_viz_;

  std::unique_ptr<Graph> graph_;

  std::vector<SinglePath> candidates_paths_;

  std::vector<int> initial_face_ids_;

  std::unordered_map<std::string, bool> left_bound_map_;
  std::unordered_map<std::string, bool> right_bound_map_;

  const heuristics::FeatureSetpoints kFeatureSetpoints;
  const heuristics::FeatureWeights kFeatureWeights;
  const heuristics::ThresholdValues kThresholdValues;
  const heuristics::OffsetValues kOffsetValues;
  const heuristics::ColorParams kColorParams;
  const heuristics::RuntimeConstraints kRuntimeConstraints;

  bool logging_flag_;
  bool debug_flag_;
};
} // namespace boundary_estimation
