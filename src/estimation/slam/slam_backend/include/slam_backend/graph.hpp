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

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <list>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "slam_common/aliases.hpp"
#include "slam_common/graph_visualization.hpp"
#include "slam_common/landmark.hpp"
#include "slam_common/stamped_pose.hpp"

namespace slam {

using gtsam::BearingRangeFactor;
using gtsam::BetweenFactor;
using gtsam::NonlinearFactorGraph;
using gtsam::Point2;
using gtsam::Pose2;
using gtsam::PriorFactor;
using gtsam::Rot2;
using gtsam::Symbol;
using gtsam::Values;
using gtsam::Vector2;
using gtsam::Vector3;
using gtsam::noiseModel::Diagonal;

/*
 * Class for encapsulating GTSAM's NonlinearFactorGraph and the mechanisms
 * for tracking and managing poses, landmarks, and pruning operations.
 */
class Graph {
public:
  /*
   * Constructor.
   */
  explicit Graph(size_t max_edges, unsigned char pose_symbol, unsigned char landmark_symbol,
                 Diagonal::shared_ptr pose_noise, Diagonal::shared_ptr pose_prior_noise,
                 Diagonal::shared_ptr landmark_prior_noise, Diagonal::shared_ptr landmark_noise);

  /*
   * Destructor.
   */
  ~Graph();

  // Creates the initial landmarks for a fixed map
  void AddLandmarkPriors(const ConeMap &fixed_map);

  // Adds a new pose to the graph
  void AddPose(const StampedPose &pose_difference);

  // Adds a vector of landmarks to the graph
  void AddLandmarks(const std::vector<Landmark> &landmarks);

  // Adds a landmark to the graph
  void AddLandmark(const Landmark &landmark);

  // Prunes the graph by removing the oldest odometry/landmark edges up to
  // max_edges_
  void PruneGraph();

  // Updates the graph estimates with optimization result
  void UpdateEstimates(gtsam::Values optimization_result);

  // Returns the updated pose estimates based on the optimization result
  std::vector<StampedPose> UpdatedPoseEstimates() const;

  // Returns the updated map estimate based on the optimization result
  ConeMap UpdatedMapEstimate() const;

  // Get a gtsam::NonlinearFactorGraph of the current graph
  gtsam::NonlinearFactorGraph NonlinearFactorGraph() const;
  // Get the graph visualization instance
  std::shared_ptr<GraphVisualization> GetGraphVisualization() const;
  // Get the initial estimates based on the keys actually used in the graph
  Values FilteredInitialEstimate() const;

  // Get the number of poses
  size_t NumOdometryNodes() const;
  // Get the number of landmarks
  size_t NumLandmarkNodes() const;

  // Get the number of edges
  size_t NumEdges() const;

private:
  // Maxmum number of edges in graph
  size_t max_edges_;

  // GTSAM graph symbols
  unsigned char pose_symbol_;
  unsigned char landmark_symbol_;

  // A noise model for odometry measurements
  Diagonal::shared_ptr pose_noise_;
  // A noise model for the odometry prior
  Diagonal::shared_ptr pose_prior_noise_;
  // A noise model for landmark prior (skidpad)
  Diagonal::shared_ptr landmark_prior_noise_;
  // A noise model for landmarks
  Diagonal::shared_ptr landmark_noise_;

  // The core graph which is optimized to produce estimates
  gtsam::NonlinearFactorGraph graph_gtsam_;

  // A parallel class for visualization
  std::shared_ptr<GraphVisualization> graph_visualization_;

  // Estimated "guess" of odometry and landmarks
  // Is updated after every optimization step.
  Values estimates_;

  // Book-keeping structures
  // Keeps track of pose keys and the pose in order
  std::list<std::pair<gtsam::Key, StampedPose>> odometry_key_pose_queue_;
  // Keeps track of all factor indexes of factors connected to each pose
  std::unordered_map<gtsam::Key, std::set<size_t>> odometry_key_to_factor_indexes_;
  // Mapping between factor indexes and their keys (nodes of each edge)
  std::unordered_map<size_t, std::set<gtsam::Key>> factor_index_to_keys_;
  // Keeps track of how many times each landmark is connected somewhere
  std::map<gtsam::Key, size_t> landmark_key_counts_;
  // Keeps track of the number of poses for symbol creation
  size_t pose_node_count = 0;
  // Number of edges between poses
  size_t num_pose_edges = 0;
  // Number of pose to landmark edges
  size_t num_landmark_edges = 0;

  // Keeps track of the latest odometry symbol added
  Symbol latest_pose_symbol_;

  // Adds the first pose to the graph
  void AddPosePrior(const StampedPose &prior_pose);

  // Creates a GTSAM symbol for the poses
  Symbol MakePoseSymbol(uint64_t pose_id);

  // Creates a GTSAM symbol for the landmarks
  Symbol MakeLandmarkSymbol(uint64_t landmark_id);
}; // end class Graph

} // end namespace slam
