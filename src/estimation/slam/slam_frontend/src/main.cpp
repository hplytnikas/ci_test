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

#include <easy/profiler.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <sstream>
#include <string>

#include "slam_backend/fixed_lag_smoothing_backend.hpp"
#include "slam_backend/graph_slam.hpp"
#include "slam_backend/slam_backend.hpp"
#include "slam_common/aliases.hpp"
#include "slam_common/mission_type.hpp"
#include "slam_common/slam_node.hpp"
#include "slam_common/utils.hpp"
#include "slam_common/visualizer.hpp"
#include "slam_frontend/data_association.hpp"
#include "slam_frontend/global_map_tracker.hpp"
#include "slam_frontend/map_loader.hpp"
#include "slam_frontend/nn_data_association.hpp"
#include "slam_frontend/slam_frontend.hpp"

namespace {
// Default algorithms
constexpr const char *kDefaultDataAssociationAlgorithm = "nearest_neighbor";
constexpr const char *kDefaultBackendAlgorithm = "graph_slam";
} // namespace

// Factory for Data Association class
std::unique_ptr<slam::DataAssociation> CreateDataAssociation(std::shared_ptr<slam::SlamNode> node) {
  // Fetch data association algorithm, default NN
  const std::string algorithm =
      node->GetParameter<std::string>("data_association.algorithm", kDefaultDataAssociationAlgorithm);
  if (algorithm == "nearest_neighbor") {
    return std::make_unique<slam::NearestNeighborDataAssociation>(node);
  } else {
    RCLCPP_WARN_STREAM(node->get_logger(), "Invalid data association algorithm: " << algorithm
                                                                                  << "! Initializing default: "
                                                                                  << kDefaultDataAssociationAlgorithm);
    return std::make_unique<slam::NearestNeighborDataAssociation>(node);
  }
  // else if (algorithm == "another_algorithm") {
  //   return std::make_unique<AnotherDataAssociation>(node);
  // }
}

// Factory for Slam Backend class
std::unique_ptr<slam::SlamBackend> CreateSlamBackend(std::shared_ptr<slam::SlamNode> node,
                                                     std::shared_ptr<slam::Visualizer> visualizer) {
  // Fetch slam backend algorithm, default GraphSLAM
  const std::string algorithm = node->GetParameter<std::string>("slam_backend.algorithm", kDefaultBackendAlgorithm);
  if (algorithm == "graph_slam") {
    return std::make_unique<slam::GraphSLAM>(node, visualizer);
  } else if (algorithm == "fixed_lag_smoothing") {
    return std::make_unique<slam::FixedLagSmoothingBackend>(node, visualizer);
  } else {
    RCLCPP_WARN_STREAM(node->get_logger(), "Invalid backend algorithm: " << algorithm << "! Initializing default: "
                                                                         << kDefaultBackendAlgorithm);
    return std::make_unique<slam::GraphSLAM>(node, visualizer);
  }
  // else if (algorithm == "another_algorithm") {
  //   return std::make_unique<AnotherSlamBackend>(node);
  // }
}

// Factory for Global Map Tracker class
std::unique_ptr<slam::GlobalMapTracker> CreateGlobalMapTracker(std::shared_ptr<slam::SlamNode> node) {
  return std::make_unique<slam::GlobalMapTracker>(node);
}

// Create and run SLAM node
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Load all parameters from yaml without having to explicitly declare each
  // parameter
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  // Construct profiling filename with formatted date and time (same as rosbag)
  std::string prof_filename = "slam_" + slam::utils::GetCurrentTimeString() + ".prof";

  // Create a shared pointer to SlamNode
  auto slam_node = std::make_shared<slam::SlamNode>("slam", "/", options);

  // Create a shared pointer to Visualizer
  auto visualizer = std::make_shared<slam::Visualizer>(slam_node);

  // Create the GlobalMapTracker instance
  auto global_map_tracker = CreateGlobalMapTracker(slam_node);

  // Create the DataAssociation instance using the factory
  auto data_association = CreateDataAssociation(slam_node);

  // Create the SlamBackend instance using the factory
  auto slam_backend = CreateSlamBackend(slam_node, visualizer);

  bool profiling_enabled = slam_node->GetParameter<bool>("profiling.enabled", false);
  if (profiling_enabled) {
    EASY_PROFILER_ENABLE; // Enable profiling
  }

  // Create an instance of SlamFrontend and pass the SlamNode shared pointer
  auto slam_frontend = std::make_unique<slam::SlamFrontend>(slam_node, visualizer, std::move(data_association),
                                                            std::move(slam_backend), std::move(global_map_tracker));

  // Spin slam node
  rclcpp::spin(slam_node);
  rclcpp::shutdown();

  if (profiling_enabled) {
    std::string save_file_path = slam_node->GetParameter<std::string>("profiling.save_file_path", "");
    prof_filename = save_file_path + prof_filename;
    profiler::dumpBlocksToFile(prof_filename.c_str());
    RCLCPP_INFO_STREAM(slam_node->get_logger(), "SLAM: Profiling data saved to " << prof_filename);
  }

  return 0;
}
