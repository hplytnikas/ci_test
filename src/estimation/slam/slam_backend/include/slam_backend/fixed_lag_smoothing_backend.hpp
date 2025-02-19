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
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "slam_backend/slam_backend.hpp"
#include "slam_common/aliases.hpp"
#include "slam_common/slam_node.hpp"
#include "slam_common/visualizer.hpp"

namespace slam {

using gtsam::BatchFixedLagSmoother;
using gtsam::BearingRangeFactor;
using gtsam::BetweenFactor;
using gtsam::LevenbergMarquardtOptimizer;
using gtsam::LevenbergMarquardtParams;
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
 * Class for for the slam backend algorithm FixedLagSmoothingBackend.
 */
class FixedLagSmoothingBackend : public SlamBackend {
public:
  /*
   * Constructor.
   */
  explicit FixedLagSmoothingBackend(std::shared_ptr<SlamNode> node_handle, std::shared_ptr<Visualizer> visualizer);

  /*
   * Destructor.
   */
  ~FixedLagSmoothingBackend();

  // Set the global map to a fixed map (Skidpad)
  void SetFixedMap(const ConeMap &fixed_map) override;

  // Enables online switch from mapping to localization mode
  void SwitchToLocalizationMode() override;

  // Update the backend with the current landmarks
  void Update(const StampedPose pose_difference, const std::vector<Landmark> &landmarks) override;

  // Optimize graph using GTSAM library and update estimates
  void Optimize() override;

private:
  // Value of current timestep for the lag
  double time_ = 0.0;
  // Number of poses added
  int num_poses_ = 1;

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

  // Maps the pose keys to their timestamp
  std::map<gtsam::Key, rclcpp::Time> pose_key_to_timestamp_;

  // Optimization parameters
  LevenbergMarquardtParams optimization_parameters_;
  BatchFixedLagSmoother smoother_;

  // Initializes everything related to this class
  void Init();

  // Loads parameters for smoother and initializes it
  void InitSmoother();

  // Loads the noise models for the smoother
  void LoadNoiseModels();

  // Loads parameters for the optimizer
  void LoadOptimizationParameters();

  // Add a pose in the smoother
  void AddPose(const StampedPose pose_difference, NonlinearFactorGraph &new_factors, Values &new_values,
               BatchFixedLagSmoother::KeyTimestampMap &new_timestamps);

  // Add landmarks in the smoother
  void AddLandmarks(const std::vector<Landmark> &landmarks, NonlinearFactorGraph &new_factors, Values &new_values,
                    BatchFixedLagSmoother::KeyTimestampMap &new_timestamps);

  // Add landmark priors in the smoother
  void AddLandmarkPriors(const ConeMap &fixed_map);

  // Updates to the latest pose and map estimates
  void UpdateEstimates(const gtsam::Values &result);

  // Create a GTSAM key for desired pose
  Symbol MakePoseSymbol(uint64_t pose_id) const;
  // Create a GTSAM key for desired landmark
  Symbol MakeLandmarkSymbol(uint64_t landmark_id) const;
}; // end class FixedLagSmoothingBackend

} // end namespace slam
