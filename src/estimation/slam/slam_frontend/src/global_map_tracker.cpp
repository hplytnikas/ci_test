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

#include "slam_frontend/global_map_tracker.hpp"

#include <easy/profiler.h>

namespace slam {

GlobalMapTracker::GlobalMapTracker(std::shared_ptr<SlamNode> node_handle)
    : node_handle_(node_handle), mapping_mode_(true) {
  LoadParameters();
}

GlobalMapTracker::~GlobalMapTracker() {}

void GlobalMapTracker::LoadParameters() {
  probability_updates_enabled_ =
      node_handle_->GetParameter<bool>("global_map_tracker.probability_updates_enabled", false);
  cone_ignore_enabled_ = node_handle_->GetParameter<bool>("global_map_tracker.cone_ignore_enabled", false);
  log_throttle_ms_ = 1000 * node_handle_->GetParameter<double>("logging.throttle", 2.0);
  color_trust_close_threshold_ =
      node_handle_->GetParameter<double>("global_map_tracker.color_trust.close_threshold", 7.0);
  color_trust_far_threshold_ = node_handle_->GetParameter<double>("global_map_tracker.color_trust.far_threshold", 15.0);
  color_trust_min_value_ = node_handle_->GetParameter<double>("global_map_tracker.color_trust.min_value", 0.4);
  min_observation_count_ = node_handle_->GetParameter<int>("global_map_tracker.min_observation_count", 3);
  accepted_cone_observation_timeframe_s_ =
      node_handle_->GetParameter<int>("global_map_tracker.accepted_cone_observation_timeframe_s", 5);
  min_cone_probability_ = node_handle_->GetParameter<double>("global_map_tracker.min_cone_probability", 0.4);
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "GlobalMapTracker: Parameters loaded.");
}

void GlobalMapTracker::SetFixedMap(const ConeMap &fixed_map) {
  size_t global_cone_id = 0;
  for (const auto &cone : fixed_map) {
    global_map_.push_back(GlobalCone(cone));
    global_map_.back().cone.id_cone = global_cone_id++;
  }

  mapping_mode_ = false;
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "GlobalMapTracker: Fixed map set.");
}

void GlobalMapTracker::SwitchToLocalizationMode() {
  mapping_mode_ = false;
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "GlobalMapTracker: Switched to localization mode.");
}

ConeMap GlobalMapTracker::FullGlobalMap() {
  // Return a copy of the global map
  ConeMap result_map;
  for (const auto &cone : global_map_) {
    result_map.push_back(cone.cone);
  }
  return result_map;
}

ConeMap GlobalMapTracker::OnlineMap() {
  ConeMap result_map;
  // Return a copy of the global map without cones that are ignored
  for (const auto &cone : global_map_) {
    if (!cone.ignored) {
      result_map.push_back(cone.cone);
    }
  }
  return result_map;
}

void GlobalMapTracker::ProcessAssociations(const std::vector<Association> &associations,
                                           ConeMap &observations_global_frame, const StampedPose &pose) {
  EASY_FUNCTION(profiler::colors::Green); // Time this function
  for (const auto &association : associations) {
    autonomous_msgs::msg::Cone &observed_cone_global_frame = observations_global_frame[association.observed_cone_index];

    if (association.global_map_cone_id == -1) {
      if (mapping_mode_) {
        AddNewCone(observed_cone_global_frame, pose);
      }
      // Association id remains -1
    } else {
      GlobalCone &global_map_cone = global_map_[association.global_map_cone_id];
      global_map_cone.last_timestamp = pose.Timestamp();
      global_map_cone.observation_count++;

      if (mapping_mode_ && probability_updates_enabled_) {
        UpdateIsConeProbability(global_map_cone.cone, observed_cone_global_frame);
        UpdateConeColorProbabilities(global_map_cone, observed_cone_global_frame, pose);
      }

      // Update id for global frame observation (important for landmark creation in frontend)
      observed_cone_global_frame.id_cone = association.global_map_cone_id;
    }
  }

  if (mapping_mode_ && cone_ignore_enabled_) {
    // For all cones (new and existing) in the global map
    for (auto &global_map_cone : global_map_) {
      IgnoreConeIfIrrelevant(global_map_cone, pose);
    }
  }
}

void GlobalMapTracker::AddNewCone(autonomous_msgs::msg::Cone &cone, const StampedPose &pose) {
  // Create new id for the cone
  cone.id_cone = global_map_.size();

  global_map_.push_back(GlobalCone(cone, pose.Timestamp()));
  global_map_.back().observation_count = 1;

  // If cone igonre is enabled, ignore the cone by default
  global_map_.back().ignored = cone_ignore_enabled_;

  // Don't set trust if color-probabilities are the same (lidar only)
  if (TrustColor(cone)) {
    double range = hypot(cone.position.x - pose.X(), cone.position.y - pose.Y());
    global_map_.back().cumulated_color_trust = CalculateColorTrust(range);
  } else {
    global_map_.back().cumulated_color_trust = 0.0;
  }
}

void GlobalMapTracker::UpdateMap(const ConeMap &new_cones) {
  if (!mapping_mode_) {
    return;
  }

  for (const auto &new_cone : new_cones) {
    // Update existing cone with corrected position
    autonomous_msgs::msg::Cone &cone = global_map_[new_cone.id_cone].cone;
    cone.position = new_cone.position;
    cone.position_covariance = new_cone.position_covariance;
  }
}

// Probability updates
void GlobalMapTracker::UpdateIsConeProbability(autonomous_msgs::msg::Cone &global_map_cone,
                                               autonomous_msgs::msg::Cone &observed_cone) {
  global_map_cone.prob_cone = 1 - ((1 - global_map_cone.prob_cone) * (1 - observed_cone.prob_cone));
}

void GlobalMapTracker::UpdateConeColorProbabilities(GlobalCone &global_map_cone,
                                                    autonomous_msgs::msg::Cone &observed_cone,
                                                    const StampedPose &pose) {
  // Don't update if color-probabilities are the same (lidar only)
  if (!TrustColor(observed_cone)) {
    return;
  }

  double range = hypot(observed_cone.position.x - pose.X(), observed_cone.position.y - pose.Y());
  double new_color_trust = CalculateColorTrust(range);
  // Calculate new color probabilities
  global_map_cone.cone.prob_type.blue = global_map_cone.cumulated_color_trust * global_map_cone.cone.prob_type.blue +
                                        new_color_trust * observed_cone.prob_type.blue;
  global_map_cone.cone.prob_type.yellow =
      global_map_cone.cumulated_color_trust * global_map_cone.cone.prob_type.yellow +
      new_color_trust * observed_cone.prob_type.yellow;
  global_map_cone.cone.prob_type.orange =
      global_map_cone.cumulated_color_trust * global_map_cone.cone.prob_type.orange +
      new_color_trust * observed_cone.prob_type.orange;
  global_map_cone.cone.prob_type.orange_big =
      global_map_cone.cumulated_color_trust * global_map_cone.cone.prob_type.orange_big +
      new_color_trust * observed_cone.prob_type.orange_big;

  global_map_cone.cumulated_color_trust += new_color_trust;

  // Normalization constant is sum of new color probabilities
  double normalization_constant = global_map_cone.cone.prob_type.blue + global_map_cone.cone.prob_type.yellow +
                                  global_map_cone.cone.prob_type.orange + global_map_cone.cone.prob_type.orange_big;

  // Divide by normalization constant to make color probabilities sum up to 1
  if (normalization_constant > 0.0) {
    global_map_cone.cone.prob_type.blue /= normalization_constant;
    global_map_cone.cone.prob_type.yellow /= normalization_constant;
    global_map_cone.cone.prob_type.orange /= normalization_constant;
    global_map_cone.cone.prob_type.orange_big /= normalization_constant;
    return;
  }

  RCLCPP_WARN_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                              "GlobalMapTracker: Cone " << global_map_cone.cone.id_cone
                                                        << " has all-zero cone type probabilities.");
}

double GlobalMapTracker::CalculateColorTrust(double range) const {
  if (range <= color_trust_close_threshold_) {
    return 1.0;
  }

  if (range >= color_trust_far_threshold_) {
    return color_trust_min_value_;
  }

  // Correct, follows equation of a line. Trust is a number between color_trust_far_ and 1
  double ratio = (color_trust_far_threshold_ - range) / (color_trust_far_threshold_ - color_trust_close_threshold_);
  return color_trust_min_value_ + ratio * (1 - color_trust_min_value_);
}

bool GlobalMapTracker::TrustColor(const autonomous_msgs::msg::Cone &cone) const {
  // Trust color if probabilities are different (0.25 for all colors in lidar only)
  return !(cone.prob_type.yellow == cone.prob_type.blue && cone.prob_type.yellow >= cone.prob_type.orange &&
           cone.prob_type.yellow >= cone.prob_type.orange_big);
}

void GlobalMapTracker::IgnoreConeIfIrrelevant(GlobalCone &cone, const StampedPose &pose) {
  // If cone was ignored and has be seen enough times, make it visible again
  if (cone.ignored && cone.observation_count >= min_observation_count_) {
    cone.ignored = false;
  }

  // Skip newly updated cones
  if (cone.ignored || cone.last_timestamp == pose.Timestamp()) {
    return;
  }

  // Ignore cone if it has not been observed for a long time and not enough observations
  int timeframe = pose.Timestamp().seconds() - cone.last_timestamp.seconds();
  if (cone.observation_count < min_observation_count_ && timeframe > accepted_cone_observation_timeframe_s_) {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "GlobalMapTracker: Cone " << cone.cone.id_cone
                                                                             << " has been observed " << timeframe
                                                                             << " seconds ago and will be ignored");
    cone.ignored = true;
    return;
  }

  // Ignore cone if its probability is too low
  if (cone.cone.prob_cone < min_cone_probability_) {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                       "GlobalMapTracker: Cone " << cone.cone.id_cone << " probability is too low and will be ignored: "
                                                 << cone.cone.prob_cone);
    cone.ignored = true;
    return;
  }
}

} // end namespace slam
