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

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <vector>

#include "autonomous_msgs/msg/cone.hpp"
#include "slam_common/aliases.hpp"
#include "slam_common/association.hpp"
#include "slam_common/landmark.hpp"
#include "slam_common/slam_node.hpp"
#include "slam_common/stamped_pose.hpp"
#include "slam_common/utils.hpp"
#include "slam_frontend/data_association.hpp"

namespace slam {

class GlobalMapTracker {
public:
  /*
   * TODO(Christoforos) If we want probabilistic distance function,
   * recommended to store covariances too.
   * Struct to store cone probability and observation information
   */
  struct GlobalCone {
    autonomous_msgs::msg::Cone cone;
    int observation_count = 0;
    double cumulated_color_trust = 0.0;
    bool ignored = false;
    rclcpp::Time last_timestamp;

    explicit GlobalCone(autonomous_msgs::msg::Cone cone) : cone(cone) {}
    GlobalCone(autonomous_msgs::msg::Cone cone, rclcpp::Time timestamp) : cone(cone), last_timestamp(timestamp) {}
  };

  /*
   * Constructor.
   */
  explicit GlobalMapTracker(std::shared_ptr<SlamNode> node_handle);

  /*
   * Destructor.
   */
  ~GlobalMapTracker();

  // Sets the global map to a fixed preloaded map
  void SetFixedMap(const ConeMap &fixed_map);

  // Switches to localization mode
  void SwitchToLocalizationMode();

  // Getter for the global map
  ConeMap FullGlobalMap();

  // Ignores ignored cones and returns the online map
  ConeMap OnlineMap();

  /*
   * Process the data associations to match existing cones. In mapping mode, if
   * a cone does not exist on the map it is added and is directly updated with a
   * new id. If the map is fixed, the cone is not added but its global stats are
   * updated.
   */
  void ProcessAssociations(const std::vector<Association> &associations, ConeMap &observations_global_frame,
                           const StampedPose &pose);

  // Updates existing cones with the corrected positions
  void UpdateMap(const ConeMap &new_cones);

private:
  // Reference to slam node
  std::shared_ptr<SlamNode> node_handle_;

  // Parameters
  bool mapping_mode_;
  bool probability_updates_enabled_;
  bool cone_ignore_enabled_;
  double log_throttle_ms_;
  double color_trust_close_threshold_;
  double color_trust_far_threshold_;
  double color_trust_min_value_;
  int min_observation_count_;
  int accepted_cone_observation_timeframe_s_;
  double min_cone_probability_;
  void LoadParameters();

  // Adds a new cone to the global map
  void AddNewCone(autonomous_msgs::msg::Cone &cone, const StampedPose &pose);

  // Updates the cone is cone probability
  void UpdateIsConeProbability(autonomous_msgs::msg::Cone &global_map_cone, autonomous_msgs::msg::Cone &observed_cone);

  // Updates the cone color probabilities
  void UpdateConeColorProbabilities(GlobalCone &global_map_cone, autonomous_msgs::msg::Cone &observed_cone,
                                    const StampedPose &pose);

  // Returns the color trust based on the distance to the cone
  double CalculateColorTrust(double range) const;

  bool TrustColor(const autonomous_msgs::msg::Cone &cone) const;

  // Performs certain checks to see if cone should be ignored
  void IgnoreConeIfIrrelevant(GlobalCone &cone, const StampedPose &pose);

  // The global map of cones
  std::vector<GlobalCone> global_map_;
};

} // end namespace slam
