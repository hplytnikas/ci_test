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

#include "slam_backend/graph_slam.hpp"

#include <easy/profiler.h>

namespace slam {

/*
 ******************************************
 ******** INITIALIZATION AND SETUP ********
 ******************************************
 */

GraphSLAM::GraphSLAM(std::shared_ptr<SlamNode> node_handle, std::shared_ptr<Visualizer> visualizer)
    : SlamBackend(node_handle, visualizer) {
  Init();
}

GraphSLAM::~GraphSLAM() {}

void GraphSLAM::Init() {
  InitGraph();
  LoadOptimizationParameters();
}

void GraphSLAM::InitGraph() {
  const size_t max_edges = node_handle_->GetParameter<size_t>("slam_backend.graph_slam.max_edges", 1000);

  auto pose_symbol_str = node_handle_->GetParameter<std::string>("slam_backend.graph_slam.pose_symbol", "X");
  auto pose_symbol = pose_symbol_str.empty() ? 'X' : pose_symbol_str[0];

  auto landmark_symbol_str = node_handle_->GetParameter<std::string>("slam_backend.graph_slam.landmark_symbol", "L");
  auto landmark_symbol = landmark_symbol_str.empty() ? 'L' : landmark_symbol_str[0];

  // Load all noise models
  std::vector<double> noise;
  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.graph_slam.pose_noise",
                                                          std::vector<double>{1, 1, 1});
  if (noise.size() != 3) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "GraphSLAM: Odometry noise model should have 3 params, but has " << noise.size());
    noise = {1, 1, 1};
  }
  const Diagonal::shared_ptr pose_noise = Diagonal::Sigmas(Vector3(noise[0], noise[1], noise[2]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.graph_slam.pose_prior_noise",
                                                          std::vector<double>{1, 1, 1});
  if (noise.size() != 3) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "GraphSLAM: Pose prior noise model should have 3 params, but has " << noise.size());
    noise = {1, 1, 1};
  }
  const Diagonal::shared_ptr pose_prior_noise = Diagonal::Sigmas(Vector3(noise[0], noise[1], noise[2]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.graph_slam.landmark_prior_noise",
                                                          std::vector<double>{0.02, 0.02});
  if (noise.size() != 2) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "GraphSLAM: Landmark prior noise model should have 2 params, but has " << noise.size());
    noise = {0.02, 0.02};
  }
  const Diagonal::shared_ptr landmark_prior_noise = Diagonal::Sigmas(Vector2(noise[0], noise[1]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.graph_slam.landmark_noise",
                                                          std::vector<double>{0.1, 0.7});
  if (noise.size() != 2) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "GraphSLAM: Landmark prior noise model should have 2 params, but has " << noise.size());
    noise = {0.1, 0.7};
  }
  const Diagonal::shared_ptr landmark_noise = Diagonal::Sigmas(Vector2(noise[0], noise[1]));

  // Create graph
  graph_ = std::make_unique<Graph>(max_edges, pose_symbol, landmark_symbol, pose_noise, pose_prior_noise,
                                   landmark_prior_noise, landmark_noise);
  // Get the first pose estimate
  pose_estimates_ = graph_->UpdatedPoseEstimates();
  enable_pruning_ = false;

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "GraphSLAM: Graph initialized and noise models set.");
}

void GraphSLAM::LoadOptimizationParameters() {
  const auto optimizer_iterations = node_handle_->GetParameter<int>("slam_backend.graph_slam.optimizer_iterations", 20);
  const auto optimizer_error_tolerance =
      node_handle_->GetParameter<double>("slam_backend.graph_slam.optimizer_error_tolerance", 0.0001);

  optimization_parameters.maxIterations = optimizer_iterations;
  optimization_parameters.relativeErrorTol = optimizer_error_tolerance;
  optimization_parameters.absoluteErrorTol = optimizer_error_tolerance;

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "GraphSLAM: Optimization parameters loaded.");
}

void GraphSLAM::SetFixedMap(const ConeMap &fixed_map) {
  graph_->AddLandmarkPriors(fixed_map);
  global_map_estimate_ = fixed_map;
  is_map_fixed_ = true;
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "GraphSLAM: Fixed map set.");
}

void GraphSLAM::SwitchToLocalizationMode() {
  // Fix landmark positions
  graph_->AddLandmarkPriors(global_map_estimate_);
  // Enable pruning after first lap
  enable_pruning_ = true;
}

/*
 ******************************************
 ************** GRAPH UPDATES *************
 ******************************************
 */

void GraphSLAM::Update(const StampedPose pose_difference, const std::vector<Landmark> &landmarks) {
  EASY_FUNCTION(profiler::colors::Yellow); // Time this function

  // Add new pose to the graph
  graph_->AddPose(pose_difference);

  // Add landmarks to graph
  for (const auto &landmark : landmarks) {
    // Do not allow addition of unassociated landmarks on fixed map
    if (landmark.AssociationId() == -1 && is_map_fixed_) {
      RCLCPP_WARN_STREAM(node_handle_->get_logger(), "GraphSLAM: Attempting to add landmark to fixed map.");
    } else {
      graph_->AddLandmark(landmark);
    }
  }

  SanityChecks();

  // Visualize raw graph
  if (visualizer_->IsVisualizationEnabled()) {
    RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), "GraphSLAM: Num poses: " << graph_->NumOdometryNodes());
    visualizer_->VisualizeRawGraph(graph_->GetGraphVisualization(), pose_difference.Timestamp());
  }
}

void GraphSLAM::SanityChecks() const {
  EASY_FUNCTION(profiler::colors::Green); // Time this function
  const size_t num_cones = graph_->NumLandmarkNodes();
  if (num_cones > 300 /*300 hardcoded param*/) {
    RCLCPP_WARN_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                                "GraphSLAM: Is there a bug? SLAM thinks there are " << num_cones << " cones.");
  }
}

/*
 ******************************************
 *********** GRAPH OPTIMIZATION ***********
 ******************************************
 */

void GraphSLAM::Optimize() {
  EASY_FUNCTION(profiler::colors::Orange); // Time this function

  // Prune graph before optimizing
  if (enable_pruning_) {
    graph_->PruneGraph();
  }

  RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                              "GraphSLAM: Number of edges: " << graph_->NumEdges());

  try {
    LevenbergMarquardtOptimizer optimizer(graph_->NonlinearFactorGraph(), graph_->FilteredInitialEstimate(),
                                          optimization_parameters);
    // Optimize graph
    const Values result = optimizer.optimize();

    RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                                "GraphSLAM: Optimization finished. (iterations=" << optimizer.iterations() << ", error="
                                                                                 << optimizer.error() << ")");

    if (result.empty()) {
      RCLCPP_WARN_STREAM(node_handle_->get_logger(), "GraphSLAM: Empty result after graph optimization");
      return;
    }

    // Update pose and map estimates
    UpdateEstimates(result);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "GraphSLAM: Optimization failed with exception: " << e.what());
    return;
  }
}

void GraphSLAM::UpdateEstimates(const gtsam::Values &optimization_result) {
  EASY_FUNCTION(profiler::colors::Green); // Time this function

  // Update the graph estimates
  graph_->UpdateEstimates(optimization_result);

  // Update pose estimates
  pose_estimates_ = graph_->UpdatedPoseEstimates();

  // Update map estimate if map is not fixed
  if (!is_map_fixed_) {
    global_map_estimate_ = graph_->UpdatedMapEstimate();
  }
}

} // end namespace slam
