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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "slam_backend/graph.hpp"
#include "slam_backend/slam_backend.hpp"
#include "slam_common/aliases.hpp"
#include "slam_common/graph_visualization.hpp"
#include "slam_common/slam_node.hpp"
#include "slam_common/visualizer.hpp"

namespace slam {

using gtsam::GaussNewtonOptimizer;
using gtsam::GaussNewtonParams;
using gtsam::LevenbergMarquardtOptimizer;
using gtsam::LevenbergMarquardtParams;
using gtsam::NonlinearFactorGraph;
using gtsam::Pose2;
using gtsam::Values;
using gtsam::Vector2;
using gtsam::Vector3;
using gtsam::noiseModel::Diagonal;

/*
 * Class for for the slam backend algorithm GraphSLAM.
 */
class GraphSLAM : public SlamBackend {
public:
  /*
   * Constructor.
   */
  explicit GraphSLAM(std::shared_ptr<SlamNode> node_handle, std::shared_ptr<Visualizer> visualizer);

  /*
   * Destructor.
   */
  ~GraphSLAM();

  // Set the global map to a fixed map (Skidpad)
  void SetFixedMap(const ConeMap &fixed_map) override;

  // Enables online switch from mapping to localization mode
  void SwitchToLocalizationMode() override;

  // Update the backend with the current landmarks
  void Update(const StampedPose pose_difference, const std::vector<Landmark> &landmarks) override;

  // Optimize graph using GTSAM library and update estimates
  void Optimize() override;

private:
  // Graph
  std::unique_ptr<Graph> graph_;
  // Timestamp for previous iteration
  rclcpp::Time previous_timestamp_;
  // Optimization parameters
  LevenbergMarquardtParams optimization_parameters;
  // Flag to enable pruning
  bool enable_pruning_;

  // Initializes everything related to this class
  void Init();

  // Loads parameters/noise models for graph and initializes it
  void InitGraph();

  // Loads parameters for the optimizer
  void LoadOptimizationParameters();

  // Performs checks to see if there might be a bug
  void SanityChecks() const;

  // Updates to the latest pose and map estimates
  void UpdateEstimates(const gtsam::Values &optimization_result);
}; // end class GraphSLAM

} // end namespace slam
