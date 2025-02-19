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

#include "slam_backend/fixed_lag_smoothing_backend.hpp"

#include <easy/profiler.h>

namespace slam {

FixedLagSmoothingBackend::FixedLagSmoothingBackend(std::shared_ptr<SlamNode> node_handle,
                                                   std::shared_ptr<Visualizer> visualizer)
    : SlamBackend(node_handle, visualizer) {
  Init();
}

FixedLagSmoothingBackend::~FixedLagSmoothingBackend() {}

void FixedLagSmoothingBackend::Init() {
  LoadNoiseModels();
  LoadOptimizationParameters();
  InitSmoother();
}

void FixedLagSmoothingBackend::LoadNoiseModels() {
  std::vector<double> noise;
  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.fixed_lag_smoothing.pose_noise",
                                                          std::vector<double>{1, 1, 1});
  if (noise.size() != 3) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "FixedLagSmoothingBackend: Odometry noise model should have 3 params, but has "
                            << noise.size());
    noise = {1, 1, 1};
  }
  pose_noise_ = Diagonal::Sigmas(Vector3(noise[0], noise[1], noise[2]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.fixed_lag_smoothing.pose_prior_noise",
                                                          std::vector<double>{1, 1, 1});
  if (noise.size() != 3) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "FixedLagSmoothingBackend: Pose prior noise model should have 3 params, but has "
                            << noise.size());
    noise = {1, 1, 1};
  }
  pose_prior_noise_ = Diagonal::Sigmas(Vector3(noise[0], noise[1], noise[2]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.fixed_lag_smoothing.landmark_prior_noise",
                                                          std::vector<double>{0.02, 0.02});
  if (noise.size() != 2) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "FixedLagSmoothingBackend: Landmark prior noise model should have 2 params, but has "
                            << noise.size());
    noise = {0.02, 0.02};
  }
  landmark_prior_noise_ = Diagonal::Sigmas(Vector2(noise[0], noise[1]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.fixed_lag_smoothing.landmark_noise",
                                                          std::vector<double>{0.1, 0.7});
  if (noise.size() != 2) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "FixedLagSmoothingBackend: Landmark prior noise model should have 2 params, but has "
                            << noise.size());
    noise = {0.1, 0.7};
  }
  landmark_noise_ = Diagonal::Sigmas(Vector2(noise[0], noise[1]));
}

void FixedLagSmoothingBackend::LoadOptimizationParameters() {
  const auto optimizer_iterations =
      node_handle_->GetParameter<int>("slam_backend.fixed_lag_smoothing.optimizer_iterations", 20);
  const auto optimizer_error_tolerance =
      node_handle_->GetParameter<double>("slam_backend.fixed_lag_smoothing.optimizer_error_tolerance", 0.0001);

  optimization_parameters_.maxIterations = optimizer_iterations;
  optimization_parameters_.relativeErrorTol = optimizer_error_tolerance;
  optimization_parameters_.absoluteErrorTol = optimizer_error_tolerance;
  // optimization_parameters_.setVerbosity("ERROR");

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "FixedLagSmoothingBackend: Optimization parameters loaded.");
}

void FixedLagSmoothingBackend::InitSmoother() {
  const double lag = node_handle_->GetParameter<double>("slam_backend.fixed_lag_smoothing.lag", 750.0);

  auto pose_symbol_str = node_handle_->GetParameter<std::string>("slam_backend.fixed_lag_smoothing.pose_symbol", "X");
  pose_symbol_ = pose_symbol_str.empty() ? 'X' : pose_symbol_str[0];

  auto landmark_symbol_str =
      node_handle_->GetParameter<std::string>("slam_backend.fixed_lag_smoothing.landmark_symbol", "L");
  landmark_symbol_ = landmark_symbol_str.empty() ? 'L' : landmark_symbol_str[0];

  // Initialize smoother
  smoother_ = BatchFixedLagSmoother(lag, optimization_parameters_);

  NonlinearFactorGraph new_factors;
  Values new_values;
  BatchFixedLagSmoother::KeyTimestampMap new_timestamps;

  // Create prior factor for first pose
  const auto prior_key = MakePoseSymbol(0);
  new_factors.push_back(PriorFactor<Pose2>(prior_key, StampedPose::ZeroPose().AsPose2(), pose_prior_noise_));
  new_values.insert(prior_key, StampedPose::ZeroPose().AsPose2());
  new_timestamps[prior_key] = time_;
  time_ += 1.0;

  // Add prior to optimizer
  smoother_.update(new_factors, new_values, new_timestamps);

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "FixedLagSmoothingBackend: Graph initialized and noise models set.");
}

void FixedLagSmoothingBackend::SetFixedMap(const ConeMap &fixed_map) {
  AddLandmarkPriors(fixed_map);
  time_ += 1.0;
  global_map_estimate_ = fixed_map;
  is_map_fixed_ = true;
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "FixedLagSmoothingBackend: Fixed map set.");
}

void FixedLagSmoothingBackend::SwitchToLocalizationMode() { AddLandmarkPriors(global_map_estimate_); }

void FixedLagSmoothingBackend::Update(const StampedPose pose_difference, const std::vector<Landmark> &landmarks) {
  EASY_FUNCTION(profiler::colors::Yellow); // Time this function
  NonlinearFactorGraph new_factors;
  Values new_values;
  BatchFixedLagSmoother::KeyTimestampMap new_timestamps;

  AddPose(pose_difference, new_factors, new_values, new_timestamps);
  AddLandmarks(landmarks, new_factors, new_values, new_timestamps);

  time_ += 1.0;
  num_poses_++;

  try {
    // Update and optimize
    BatchFixedLagSmoother::Result optimizer_result = smoother_.update(new_factors, new_values, new_timestamps);
    RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                                "FixedLagSmoothingBackend: Update and optimization finished. (iterations="
                                    << optimizer_result.iterations << ", error=" << optimizer_result.error << ")");
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "FixedLagSmoothingBackend: Update or optimization failed with exception: " << e.what());
    return;
  }
}

void FixedLagSmoothingBackend::AddPose(const StampedPose pose_difference, NonlinearFactorGraph &new_factors,
                                       Values &new_values, BatchFixedLagSmoother::KeyTimestampMap &new_timestamps) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function
  const auto previous_pose_key = MakePoseSymbol(num_poses_ - 1);
  const auto current_pose_key = MakePoseSymbol(num_poses_);
  // Calculate the current pose using latest estimate and difference
  Pose2 latest_pose2 = smoother_.calculateEstimate<Pose2>(previous_pose_key);
  Pose2 current_pose2 = latest_pose2.compose(pose_difference.AsPose2());
  // Add factor
  new_factors.push_back(
      BetweenFactor<Pose2>(previous_pose_key, current_pose_key, pose_difference.AsPose2(), pose_noise_));
  new_values.insert(current_pose_key, current_pose2);
  new_timestamps[current_pose_key] = time_;
  pose_key_to_timestamp_[current_pose_key] = pose_difference.Timestamp();
}

void FixedLagSmoothingBackend::AddLandmarks(const std::vector<Landmark> &landmarks, NonlinearFactorGraph &new_factors,
                                            Values &new_values,
                                            BatchFixedLagSmoother::KeyTimestampMap &new_timestamps) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function
  const Values current_estimates = smoother_.calculateEstimate();
  const auto current_pose_key = MakePoseSymbol(num_poses_);
  // Add landmarks
  for (const auto &landmark : landmarks) {
    const auto landmark_key = MakeLandmarkSymbol(landmark.Cone().id_cone);
    const Point2 point = Point2(landmark.X(), landmark.Y());

    new_factors.push_back(BearingRangeFactor<Pose2, Point2>(
        current_pose_key, landmark_key, Rot2::fromAngle(landmark.Bearing()), landmark.Range(), landmark_noise_));

    if (!current_estimates.exists(landmark_key) && !new_values.exists(landmark_key)) {
      new_values.insert(landmark_key, point);
    }

    new_timestamps[landmark_key] = time_;
  }
}

void FixedLagSmoothingBackend::AddLandmarkPriors(const ConeMap &fixed_map) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function
  NonlinearFactorGraph new_factors;
  Values new_values;
  BatchFixedLagSmoother::KeyTimestampMap new_timestamps;

  // Get current estimates
  const Values current_estimates = smoother_.calculateEstimate();
  // For each cone in the fixed map
  for (const auto &cone : fixed_map) {
    const int landmark_id = cone.id_cone;
    const Point2 point = Point2(cone.position.x, cone.position.y);
    const auto landmark_key = MakeLandmarkSymbol(landmark_id);

    // Create prior factor
    new_factors.push_back(PriorFactor<Point2>(landmark_key, point, landmark_prior_noise_));

    if (!current_estimates.exists(landmark_key) && !new_values.exists(landmark_key)) {
      new_values.insert(landmark_key, point);
    }
  }

  // Add priors to optimizer
  smoother_.update(new_factors, new_values);
}

void FixedLagSmoothingBackend::Optimize() {
  EASY_FUNCTION(profiler::colors::Orange); // Time this function

  // Get optimized result
  const Values result = smoother_.calculateEstimate();

  UpdateEstimates(result);
}

void FixedLagSmoothingBackend::UpdateEstimates(const Values &result) {
  pose_estimates_.clear();
  global_map_estimate_.clear();

  for (const auto &key : result.keys()) {
    if (Symbol(key).chr() == pose_symbol_) {
      // Get optimized pose
      const Pose2 &p2 = result.at<Pose2>(key);
      // Update it with timestamp
      const StampedPose stamped(p2, pose_key_to_timestamp_[key]);
      pose_estimates_.push_back(stamped);
    } else if (!is_map_fixed_) {
      // Get optimized landmark position
      const Point2 &cone_result = result.at<Point2>(key);
      autonomous_msgs::msg::Cone cone;
      // Important to retrieve cone id
      cone.id_cone = Symbol(key).index(); // Retrieve cone id from key
      cone.position.x = cone_result.x();
      cone.position.y = cone_result.y();
      // We don't care about anything else, only id, x and y of cones
      cone.position.z = 0;
      cone.is_observed = 0;
      cone.position_covariance.x_x = 0;
      cone.position_covariance.y_y = 0;
      cone.position_covariance.x_y = 0;
      global_map_estimate_.push_back(cone);
    }
  }
}

Symbol FixedLagSmoothingBackend::MakePoseSymbol(uint64_t pose_id) const { return Symbol(pose_symbol_, pose_id); }
Symbol FixedLagSmoothingBackend::MakeLandmarkSymbol(uint64_t landmark_id) const {
  return Symbol(landmark_symbol_, landmark_id);
}

} // end namespace slam
