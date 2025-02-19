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

// #include <easy/profiler.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

#include "autonomous_msgs/msg/cone_array.hpp"
#include "slam_backend/slam_backend.hpp"
#include "slam_common/aliases.hpp"
#include "slam_common/association.hpp"
#include "slam_common/landmark.hpp"
#include "slam_common/mission_type.hpp"
#include "slam_common/slam_node.hpp"
#include "slam_common/stamped_pose.hpp"
#include "slam_common/utils.hpp"
#include "slam_common/visualizer.hpp"
#include "slam_frontend/data_association.hpp"
#include "slam_frontend/global_map_tracker.hpp"
#include "slam_frontend/map_loader.hpp"

namespace slam {

/*
 * Main class that handles the whole SLAM algorithm. Everything
 * is done in the ConeCallback() function when receiving observations
 * from PERCEPTION.
 */
class SlamFrontend {
public:
  /*
   * Constructor.
   */
  explicit SlamFrontend(std::shared_ptr<SlamNode> node_handle, std::shared_ptr<Visualizer> visualizer,
                        std::unique_ptr<DataAssociation> data_association, std::unique_ptr<SlamBackend> slam_backend,
                        std::unique_ptr<GlobalMapTracker> global_map_tracker);

  /*
   * Destructor.
   */
  ~SlamFrontend();

private:
  // Initializes node subscribers
  void InitSubscribers();

  // Sets all parameters
  void LoadParameters();

  // Loads the map if needed
  void LoadMapIfNeeded();

  // Process observed cones. Called when PERCEPTION publishes new observations
  void ConeCallback(const autonomous_msgs::msg::ConeArray &msg);

  // Helper to convert associations to landmarks
  std::vector<Landmark> AssociationsToLandmarks(const ConeMap &observations_local_frame,
                                                const ConeMap &observations_global_frame,
                                                const std::vector<Association> &associations);

  // Checks the progress of the mission
  void CheckMissionProgress();

  // Helper to see if vehicle has moved
  inline bool HasVehicleMoved() const { return node_handle_->IsVehicleMoving(); }

  // Helper to see if mission has finished
  inline bool HasMissionFinished() const { return node_handle_->HasMissionFinished(); }

  // Reference to slam node
  std::shared_ptr<SlamNode> node_handle_;
  // Reference to visualizer
  std::shared_ptr<Visualizer> visualizer_;
  // Data association reference
  std::unique_ptr<DataAssociation> data_association_;
  // Slam backend reference
  std::unique_ptr<SlamBackend> slam_backend_;
  // Global map tracker reference
  std::unique_ptr<GlobalMapTracker> global_map_tracker_;
  // Map loader reference
  std::unique_ptr<MapLoader> map_loader_;

  // Keeps track of the previous pose in map->base_link
  StampedPose previous_pose_;

  // Parameters
  // If set to true slam creates the map
  bool mapping_mode_;
  // Determines if map will be saved when mission finished
  bool save_map_on_mission_finished_;
  // Determines if map will be saved when mission finished
  bool save_trajectory_on_mission_finished_;
  // Determines if acceleration map will be generated
  bool generate_acceleration_map_;
  // Determines if acceleration map has been generated
  bool acceleration_map_generated_;
  // Determines the mission type
  MissionType mission_type_;
  // Log throttle milliseconds
  int log_throttle_ms_;
}; // end class SlamFrontend

} // end namespace slam
