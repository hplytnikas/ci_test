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

#include "slam_backend/graph.hpp"

#include <easy/profiler.h>

namespace slam {

Graph::Graph(size_t max_edges, unsigned char pose_symbol, unsigned char landmark_symbol,
             Diagonal::shared_ptr pose_noise, Diagonal::shared_ptr pose_prior_noise,
             Diagonal::shared_ptr landmark_prior_noise, Diagonal::shared_ptr landmark_noise)
    : max_edges_(max_edges), pose_symbol_(pose_symbol), landmark_symbol_(landmark_symbol), pose_noise_(pose_noise),
      pose_prior_noise_(pose_prior_noise), landmark_prior_noise_(landmark_prior_noise),
      landmark_noise_(landmark_noise) {
  // First pose is (0, 0, 0)
  graph_visualization_ = std::make_shared<GraphVisualization>();
  AddPosePrior(StampedPose::ZeroPose());
}

Graph::~Graph() {}

void Graph::AddPosePrior(const StampedPose &prior_pose) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function
  // Will be the first pose symbol
  latest_pose_symbol_ = MakePoseSymbol(pose_node_count++);
  Pose2 prior_pose2 = prior_pose.AsPose2();

  // Create prior factor for pose
  const auto prior = PriorFactor<Pose2>(latest_pose_symbol_, prior_pose2, pose_prior_noise_);

  // Update book-keeping structures
  const size_t factor_index = graph_gtsam_.size();
  const auto key = latest_pose_symbol_.key();
  odometry_key_pose_queue_.push_back(std::make_pair(key, prior_pose));
  odometry_key_to_factor_indexes_[key].insert(factor_index);
  factor_index_to_keys_[factor_index].insert(key);

  // Update visualization
  graph_visualization_->AddPose(prior_pose.X(), prior_pose.Y());

  // Add prior to graph and node to initial estimates
  graph_gtsam_.add(prior);
  estimates_.insert(latest_pose_symbol_, prior_pose2);
}

void Graph::AddLandmarkPriors(const ConeMap &fixed_map) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function
  // Add Landmark priors
  for (const auto &cone : fixed_map) {
    const int landmark_id = cone.id_cone;
    const Point2 point = Point2(cone.position.x, cone.position.y);
    const Symbol landmark_symbol = MakeLandmarkSymbol(landmark_id);

    // Update book-keeping structures
    landmark_key_counts_[landmark_symbol.key()]++;

    // Update visualization
    graph_visualization_->AddLandmark(landmark_id, cone.position.x, cone.position.y);

    // Add prior factor for landmark
    const auto factor = PriorFactor<Point2>(landmark_symbol, point, landmark_prior_noise_);

    graph_gtsam_.add(factor);
    if (!estimates_.exists(landmark_symbol)) {
      estimates_.insert(landmark_symbol, point);
    }
  }
}

void Graph::AddPose(const StampedPose &pose_difference) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function
  const auto current_timestamp = pose_difference.Timestamp();

  // Get latest estimate
  Pose2 latest_pose2 = estimates_.at<Pose2>(latest_pose_symbol_.key());

  // Get current estimate by composing previous estimate and difference
  Pose2 current_pose2 = latest_pose2.compose(pose_difference.AsPose2());

  Symbol current_pose_symbol = MakePoseSymbol(pose_node_count++);

  // Create between factor for poses
  const auto factor =
      BetweenFactor<Pose2>(latest_pose_symbol_, current_pose_symbol, pose_difference.AsPose2(), pose_noise_);

  // Update book-keeping structures
  const size_t factor_index = graph_gtsam_.size();
  const auto current_key = current_pose_symbol.key();
  const auto latest_key = latest_pose_symbol_.key();
  odometry_key_pose_queue_.push_back(std::make_pair(current_key, StampedPose(current_pose2, current_timestamp)));
  odometry_key_to_factor_indexes_[current_key].insert(factor_index);
  odometry_key_to_factor_indexes_[latest_key].insert(factor_index);
  factor_index_to_keys_[factor_index].insert(current_key);
  factor_index_to_keys_[factor_index].insert(latest_key);

  // Update visualization
  graph_visualization_->AddPose(current_pose2.x(), current_pose2.y());

  // Add factor to graph and node to initial estimates
  graph_gtsam_.add(factor);
  estimates_.insert(current_pose_symbol, current_pose2);

  // Store latest pose symbol
  latest_pose_symbol_ = current_pose_symbol;
  num_pose_edges++;
}

void Graph::AddLandmark(const Landmark &landmark) {
  EASY_FUNCTION(profiler::colors::Red);         // Time this function
  size_t landmark_id = landmark.Cone().id_cone; // Cone id surely exists

  const Point2 point = Point2(landmark.X(), landmark.Y());
  const Symbol landmark_symbol = MakeLandmarkSymbol(landmark_id);

  // Create factor between latest pose and landmark
  const auto factor = BearingRangeFactor<Pose2, Point2>(
      latest_pose_symbol_, landmark_symbol, Rot2::fromAngle(landmark.Bearing()), landmark.Range(), landmark_noise_);

  // Update book-keeping structures
  const size_t factor_index = graph_gtsam_.size();
  const auto odom_key = latest_pose_symbol_.key();
  const auto landmark_key = landmark_symbol.key();
  odometry_key_to_factor_indexes_[odom_key].insert(factor_index);
  landmark_key_counts_[landmark_key]++;
  factor_index_to_keys_[factor_index].insert(odom_key);
  factor_index_to_keys_[factor_index].insert(landmark_key);

  // Update visualization
  graph_visualization_->AddLandmark(landmark_id, landmark.X(), landmark.Y());

  // Add factor to graph and landmark to initial estimates
  graph_gtsam_.add(factor);
  if (!estimates_.exists(landmark_symbol)) {
    estimates_.insert(landmark_symbol, point);
  }

  num_landmark_edges++;
}

// Unused for now
void Graph::AddLandmarks(const std::vector<Landmark> &landmarks) {
  for (const auto &landmark : landmarks) {
    AddLandmark(landmark);
  }
}

void Graph::PruneGraph() {
  EASY_FUNCTION(profiler::colors::Brown); // Time this function
  // TODO(Christoforos)
  /*
  More structured way to do this is keep a certain time history.
  Otherwise you might get useless edges connecting two poses
  that are kind of free floating and wasting some computation.
  */
  while (num_pose_edges + num_landmark_edges > max_edges_) {
    // Get and pop oldest pose
    const auto odometry_key_timestamp_pair = odometry_key_pose_queue_.front();
    odometry_key_pose_queue_.pop_front();
    num_pose_edges--;

    // Update visualization
    graph_visualization_->PopOdometryEdge();

    const auto odometry_key = odometry_key_timestamp_pair.first;

    // For every factor (edge) associated with this pose
    for (const auto &factor_index : odometry_key_to_factor_indexes_.at(odometry_key)) {
      // Decrement counter for each landmark connected with this pose
      for (const auto &key : factor_index_to_keys_.at(factor_index)) {
        if (landmark_key_counts_.count(key) > 0) {
          if (landmark_key_counts_[key] <= 1) {
            landmark_key_counts_.erase(key);
          } else {
            landmark_key_counts_[key]--;
          }

          // Update visualization
          graph_visualization_->PopLandmarkEdge();

          num_landmark_edges--;
        }
      }

      // Remove factor from graph
      graph_gtsam_.remove(factor_index);
    }

    // Remove entry for this pose
    odometry_key_to_factor_indexes_.erase(odometry_key);

    /*
    TODO(Christoforos)
    For the "correct" way it's marginalization.
    Basically collapse the current estimate and covariance of the
    pose/landmark to a single prior factor, but you can think of
    that as a possible improvement for later
    */
    // // Need to create a prior factor for the first remaining pose
    // auto &p = odometry_key_pose_queue_.front();
    // const auto key = p.first;
    // const auto prior = PriorFactor<Pose2>(Symbol(key), p.second.AsPose2(),
    //                                       pose_prior_noise_);
    // const size_t factor_index = graph_gtsam_.size();
    // odometry_key_to_factor_indexes_[key].insert(factor_index);
    // factor_index_to_keys_[factor_index].insert(key);
    // graph_gtsam_.add(prior);
  }
}

void Graph::UpdateEstimates(gtsam::Values optimization_result) {
  estimates_.clear();
  estimates_ = optimization_result;
}

std::vector<StampedPose> Graph::UpdatedPoseEstimates() const {
  std::vector<StampedPose> updated_pose_estimates;
  updated_pose_estimates.reserve(NumOdometryNodes());

  for (auto &itr : odometry_key_pose_queue_) {
    // Get optimized pose
    const gtsam::Pose2 &p2 = estimates_.at<gtsam::Pose2>(itr.first);
    // Update it with timestamp
    const StampedPose stamped(p2, itr.second.Timestamp());
    updated_pose_estimates.push_back(stamped);
  }

  return updated_pose_estimates;
}

ConeMap Graph::UpdatedMapEstimate() const {
  ConeMap updated_map_estimate;
  updated_map_estimate.reserve(NumLandmarkNodes());

  // Add all current cone nodes to the map
  for (auto &itr : landmark_key_counts_) {
    // Get optimized landmark position
    const Point2 &cone_result = estimates_.at<Point2>(itr.first);
    autonomous_msgs::msg::Cone cone;

    // Important to retrieve cone id
    cone.id_cone = Symbol(itr.first).index(); // Retrieve cone id from key

    cone.position.x = cone_result.x();
    cone.position.y = cone_result.y();

    // We don't care about anything else, only id, x and y of cones
    cone.position.z = 0;
    cone.position_covariance.x_x = 0;
    cone.position_covariance.y_y = 0;
    cone.position_covariance.x_y = 0;
    updated_map_estimate.push_back(cone);
  }

  return updated_map_estimate;
}

gtsam::NonlinearFactorGraph Graph::NonlinearFactorGraph() const { return graph_gtsam_; }

std::shared_ptr<GraphVisualization> Graph::GetGraphVisualization() const {
  EASY_FUNCTION(profiler::colors::Green);
  return graph_visualization_;
}

Values Graph::FilteredInitialEstimate() const {
  // Exclude pruned nodes
  gtsam::Values filtered;

  // Iterate through all keys in estimates_
  for (const auto &key_value : estimates_) {
    const auto &key = key_value.key;
    // Check if the key exists in odometry_key_to_factor_indexes_ or
    // landmark_key_counts_
    if (odometry_key_to_factor_indexes_.count(key) > 0 || landmark_key_counts_.count(key) > 0) {
      // If the key meets the criteria, insert it into the filtered Values
      filtered.insert(key, estimates_.at(key));
    }
  }

  return filtered;
}

Symbol Graph::MakePoseSymbol(uint64_t pose_id) { return Symbol(pose_symbol_, pose_id); }

Symbol Graph::MakeLandmarkSymbol(uint64_t landmark_id) { return Symbol(landmark_symbol_, landmark_id); }

size_t Graph::NumOdometryNodes() const { return odometry_key_pose_queue_.size(); }

size_t Graph::NumLandmarkNodes() const { return landmark_key_counts_.size(); }

size_t Graph::NumEdges() const { return num_pose_edges + num_landmark_edges; }

} // end namespace slam
