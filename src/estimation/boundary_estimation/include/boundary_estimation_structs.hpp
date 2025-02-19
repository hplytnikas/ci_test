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
#include <vector>

namespace boundary_estimation {

namespace constants {
// 1 = yolo_only, 2 = lidar_only, 4 = sensor_fusion
extern const int kCameraOnly;
extern const int kLidarOnly;
extern const int kSensorFusion;
extern const double kMaxMidpointDistanceFromCarPosition;
extern const double kMaxLastMidpointDistance;
extern const double kMinimumDistanceSameBoundCones;
extern const int kMinimumConsecutivesCrossedMidpoints;
extern const double kMinimumAngleDeviationBetweenCrossedPoints;
extern const double kPathLengthBehind;
extern const double kPathLengthAhead;
extern const double kMidpointDistanceThreshold;
extern const int kMaxTempPastMipointIds;
extern const int kMaxPointToCheckForOverlay;
extern const double kMaxEntropyToConsiderColor;
extern const double kMaxPointDistanceFirstAndLast;
} // namespace constants

enum Directions { Left, Right };

enum Headings { Ahead, Behind };

enum RelaxationType { None, Light, Medium, Severe, Total };

struct PrevPath {
  std::vector<std::string> midpoints;
  std::vector<std::string> left_boundaries;
  std::vector<std::string> right_boundaries;
  double posterior;

  // Constructor that initializes the members
  PrevPath(const std::vector<std::string> &mid, const std::vector<std::string> &left,
           const std::vector<std::string> &right, double post)
      : midpoints(mid), left_boundaries(left), right_boundaries(right), posterior(post) {}
};

// Struct representing information associated with a vertex
struct VertexInfo {
  int id;                 // Identifier
  int pipeline;           // Pipeline
  double orange_prob;     // Probability of being orange
  double orange_big_prob; // Probability of being big orange
  double blue_prob;       // Probability of being blue
  double yellow_prob;     // Probability of being yellow
  double presence_prob;   // Probability of presence
  bool virtual_flag;      // Flag indicating virtual status
};

namespace heuristics {

// Struct defining parameters related to feature setpoints to calculare the
// prior
struct FeatureSetpoints {
  float desired_length_m;             // [meters] desired length of the path
  float desired_witdh_std_m;          // [meters] width variance considered reasonable
  float desired_max_angle_change_rad; // [degree] angle between 2 segments of the
                                      // path considered reasonable
  float desired_boundary_std_m;       // [meters] reasonable variance of the
                                      // distance of the boundary cones
};

// Struct defining weights for the features to calculare the prior (this weight
// have to some up to 1, exept for the prior_w, and cones_distance_var_w is
// counted twice)
struct FeatureWeights {
  float length_error_w;       // Weight for length error
  float track_width_w;        // Weight for track width
  float step_angle_w;         // Weight for step angle
  float cones_distance_var_w; // Weight for cones distance variation
  float prior_w;              // Weight for prior probability
};

// Struct defining threshold values
struct ThresholdValues {
  int maximum_number_of_far_distant_cones; // [scalar] Maximum number of cones that can be
                                           // far the previous one
  float minimum_segment_width_m;           // [meters] minimum width allowed between a
                                           // left and a right cone
  float maximum_segment_width_m;           // [meters] maximum width allowed between a
                                           // left and a right cone
  float minimum_track_width_m;             // [meters] minimum width allowed for a track
  float maximum_track_width_m;             // [meters] maximum width allowed for a track
  float maximum_angle_change_rad;          // [degree] maximum angle changed allowed
                                           // between edges in a path
  float maximum_end_angle_change_rad;      // [degree] maximum angle changed allowed
                                           // between the last two segments
  float minimum_presence_probability;      // [probability] minimum probability of
                                           // the cone existance to be considered in
                                           // the graph
  float minimum_path_segment_length_m;     // [meters] minimum lenght of the segment
                                           // from 2 midpoints
  float maximum_path_segment_length_m;     // [meters] maximum lenght of the segment
                                           // from 2 midpoints
  float maximum_boundary_segment_length_m; // [meters] maximum lenght of the
                                           // segment from 2 consecutive
                                           // boundary cones
  float minimum_path_length_m;             // [meters] minimum lenght of the full path
  float desirable_minimum_path_length_m;   // [meters] desirable minimum lenght of the
                                           // full path
};

struct OffsetValues {
  int maximum_number_of_far_distant_cones_offset; // [scalar] Maximum number of cones that can be
                                                  // far the previous one
  float minimum_segment_width_offset_m;           // [meters] offset for minimum width allowed
                                                  // between a left and a right cone
  float maximum_segment_width_offset_m;           // [meters] offset for maximum width allowed

  float minimum_track_width_offset_m;         // [meters] offset for minimum width allowed for a
                                              // track
  float maximum_track_width_offset_m;         // [meters] offset for maximum width allowed for a
                                              // track
  float maximum_angle_change_offset_rad;      // [rad] offset for maximum angle changed
                                              // allowed between edges in a path
  float minimum_path_segment_length_offset_m; // [meters] offset for minimum path segment length
  float maximum_path_segment_length_offset_m; // [meters] offset for maximum path segment length
};

// Struct defining parameters related to colors in order to compute the
// likelihood
struct ColorParams {
  bool use_orange; // Flag indicating whether orange color should be used on the
                   // graph or not
};

// Struct defining runtime constraints used to generate the paths in the Beam
// search
struct RuntimeConstraints {
  float max_number_of_leaves;   // max number of leaves saved; if there are more,
                                // the worst are discarded
  float number_of_iterations;   // after this number of iterations, search stops
  bool full_relaxation_enabled; // flag to enable full relaxation
};
} // namespace heuristics
} // namespace boundary_estimation
