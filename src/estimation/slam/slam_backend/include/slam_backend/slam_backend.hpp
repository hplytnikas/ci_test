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

#include <memory>
#include <vector>

#include "slam_common/aliases.hpp"
#include "slam_common/landmark.hpp"
#include "slam_common/slam_node.hpp"
#include "slam_common/stamped_pose.hpp"
#include "slam_common/visualizer.hpp"

namespace slam {

/*
 * Base class for for the slam backend. Inherit from this class and
 * override the methods for a different backend algorithm.
 */
class SlamBackend {
public:
  /*
   * Constructor.
   */
  explicit SlamBackend(std::shared_ptr<SlamNode> node_handle, std::shared_ptr<Visualizer> visualizer);

  /*
   * Destructor.
   */
  virtual ~SlamBackend();

  // Getters
  // Returns the estimate of the most recent pose added to the graph
  StampedPose LatestPoseEstimate() const;

  // Returns the estimate of all poses, ordered from first to latest
  std::vector<StampedPose> PoseEstimates() const;

  // Returns the number of unique cones in the global map
  size_t ConeCount() const;

  // Returns the estimate of the global map
  ConeMap GlobalMapEstimate() const;

  // Sets the map to be fixed (Skidpad)
  virtual void SetFixedMap(const ConeMap &fixed_map) = 0;

  // Enables online switch from mapping to localization mode
  virtual void SwitchToLocalizationMode() = 0;

  /*!
   * Updates backend with new landmark measurements
   *
   * @param current_timestamp - Current timestamp
   * @param landmarks - Cone landmarks to be added
   * @return void
   */
  virtual void Update(const StampedPose pose_difference, const std::vector<Landmark> &landmarks) = 0;

  /*
   * This function should be called to update the estimate of observed
   * cones and all vehicle poses up until the current moment in time.
   */
  virtual void Optimize() = 0;

protected:
  // Allows derived classes to access node_handle_
  std::shared_ptr<SlamNode> node_handle_;
  // Allows derived classes to access visualizer_
  std::shared_ptr<Visualizer> visualizer_;
  // The estimate of all poses, ordered from first to latest
  std::vector<StampedPose> pose_estimates_;
  // The SLAM estimate of the global map
  ConeMap global_map_estimate_;
  // Indicates if map is fixed
  bool is_map_fixed_;
  // Log throttle milliseconds
  int log_throttle_ms_;
}; // end class SlamBackend

} // end namespace slam
