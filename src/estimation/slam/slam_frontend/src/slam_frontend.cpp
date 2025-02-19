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

#include "slam_frontend/slam_frontend.hpp"

#include <easy/profiler.h>

namespace slam {

SlamFrontend::SlamFrontend(std::shared_ptr<SlamNode> node_handle, std::shared_ptr<Visualizer> visualizer,
                           std::unique_ptr<DataAssociation> data_association, std::unique_ptr<SlamBackend> slam_backend,
                           std::unique_ptr<GlobalMapTracker> global_map_tracker)
    : node_handle_(node_handle), visualizer_(visualizer), data_association_(std::move(data_association)),
      slam_backend_(std::move(slam_backend)), global_map_tracker_(std::move(global_map_tracker)) {
  LoadParameters();

  map_loader_ = std::make_unique<MapLoader>(node_handle_);
  LoadMapIfNeeded();

  InitSubscribers();

  // Initially publish zero pose
  node_handle_->BroadcastInitialPose(node_handle_->get_clock()->now());
  previous_pose_ = StampedPose::ZeroPose();
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "SlamFrontend: Initialization done. First map->odom published.");
}

SlamFrontend::~SlamFrontend() {}

void SlamFrontend::InitSubscribers() {
  node_handle_->CreateConeCallbackSubscriber(std::bind(&SlamFrontend::ConeCallback, this,
                                                       std::placeholders::_1)); // Create subscribers
}

void SlamFrontend::LoadParameters() {
  mission_type_ = static_cast<MissionType>(
      node_handle_->GetParameter<int>("mission_type", static_cast<int>(MissionType::kAutocross)));
  mapping_mode_ = node_handle_->GetParameter<bool>("mapping_mode", true);
  save_map_on_mission_finished_ = node_handle_->GetParameter<bool>("save_map_on_mission_finished", true);
  save_trajectory_on_mission_finished_ = node_handle_->GetParameter<bool>("save_trajectory_on_mission_finished", true);
  log_throttle_ms_ = 1000 * node_handle_->GetParameter<double>("logging.throttle", 2.0);
  generate_acceleration_map_ = false;
  acceleration_map_generated_ = false;

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "SlamFrontend: Parameters loaded - Mission type: "
                                                     << mission_type_ << ". Mapping mode: " << mapping_mode_);
}

void SlamFrontend::LoadMapIfNeeded() {
  // Load fixed map in Skidpad
  if (mission_type_ == MissionType::kSkidpad) {
    const ConeMap &fixed_map = map_loader_->LoadMap();
    // First create the global map (set cone ids)
    global_map_tracker_->SetFixedMap(fixed_map);
    // Then set that map in backend
    slam_backend_->SetFixedMap(global_map_tracker_->FullGlobalMap());
  } else if (mission_type_ == MissionType::kTrackdrive && !mapping_mode_) {
    // Load map in trackdrive if mapping mode is false
    const ConeMap &fixed_map = map_loader_->LoadMap();
    // First create the global map (set cone ids)
    global_map_tracker_->SetFixedMap(fixed_map);
    // Then set that map in backend
    slam_backend_->SetFixedMap(global_map_tracker_->FullGlobalMap());
  } else if (mission_type_ == MissionType::kAcceleration && !mapping_mode_) {
    // Load map in acceleration if mapping mode is false
    // Map is generated after perception sends the first non-empty cone array
    generate_acceleration_map_ = true;
  }
}

void SlamFrontend::ConeCallback(const autonomous_msgs::msg::ConeArray &msg) {
  EASY_FUNCTION(profiler::colors::Magenta); // Time this function

  // Check if mission has ended - no need to execute the whole callback
  if (HasMissionFinished()) {
    // TODO(Christoforos) Throttle
    RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                                "SlamFrontend: Mission has finished.");
    if (save_map_on_mission_finished_) {
      map_loader_->SaveMap(global_map_tracker_->OnlineMap());
      save_map_on_mission_finished_ = false;
    }

    if (save_trajectory_on_mission_finished_) {
      map_loader_->SaveTrajectory(slam_backend_->PoseEstimates());
      save_trajectory_on_mission_finished_ = false;
    }

    return;
  }

  const rclcpp::Time current_timestamp = msg.header.stamp;
  ConeMap observations_local_frame = msg.cones;

  // Retrieve odom->base_link current pose
  const auto current_fetch = node_handle_->FetchTF(current_timestamp, "odom", "base_link");
  if (!current_fetch.first) {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "SlamFrontend: TF lookup failed, exiting frontend.");
    return;
  }

  // Calculate difference between previous odom->base_link pose - timestamp is current
  StampedPose pose_difference = current_fetch.second.Delta(previous_pose_);

  // Store previous odom->base_link pose for calculating differences
  // Must be done before check if vehicle has moved!
  previous_pose_ = current_fetch.second;

  // Calculate estimated current pose in global map frame - timestamp is current
  // (Add this difference to the latest estimate)
  StampedPose current_pose = slam_backend_->LatestPoseEstimate().Compose(pose_difference);

  // Transform cone map from local to global frame using map->base_link
  // transformation
  ConeMap observations_global_frame = utils::TransformMap(observations_local_frame, current_pose.AsPose2D());

  // If vehicle did not move or mission ended, return
  if (!HasVehicleMoved()) {
    // First broadcast initial pose so it is not lost from the buffer
    node_handle_->BroadcastInitialPose(current_timestamp);

    // Publish observed cones as map
    autonomous_msgs::msg::ConeArray cone_array;
    cone_array.header.frame_id = "map";
    cone_array.header.stamp = current_timestamp;
    cone_array.cones = observations_global_frame;

    // Generate acceleration map if needed
    if (generate_acceleration_map_ && !acceleration_map_generated_) {
      const ConeMap fixed_map = map_loader_->GenerateAccelerationMap(observations_global_frame);
      if (fixed_map.size() > 0) {
        // First create the global map (set cone ids)
        global_map_tracker_->SetFixedMap(fixed_map);
        // Then set that map in backend
        slam_backend_->SetFixedMap(global_map_tracker_->FullGlobalMap());
        // generate_acceleration_map_ = false;
        acceleration_map_generated_ = true;
        RCLCPP_INFO_STREAM(node_handle_->get_logger(), "SlamFrontend: Acceleration map generated and set.");
      }
    }

    // If we are in accel mode and map needs to be generated, publish if it has successfully been generated
    if (generate_acceleration_map_) {
      node_handle_->PublishAccelerationMapStatus(acceleration_map_generated_);
    }

    // Visualize map
    if (visualizer_->IsVisualizationEnabled()) {
      // Ids needed for visualization
      for (size_t id = 0; id < cone_array.cones.size(); id++) {
        cone_array.cones[id].id_cone = id;
      }
      visualizer_->VisualizeGlobalMap(cone_array.cones, current_timestamp);
    }
    node_handle_->PublishOnlineMap(cone_array);
    RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                                "SlamFrontend: Vehicle has still not moved.");
    return;
  }

  // Get full global map
  const ConeMap global_map = global_map_tracker_->FullGlobalMap();

  RCLCPP_DEBUG_STREAM(node_handle_->get_logger(),
                      "SlamFrontend: Transformed cones (local_frame-global_frame-current_map): "
                          << observations_local_frame.size() << "-" << observations_global_frame.size() << "-"
                          << global_map.size());

  // Data association
  const std::vector<Association> &associations = data_association_->Associate(observations_global_frame, global_map);

  // Process associations and add new cones to the map
  global_map_tracker_->ProcessAssociations(associations, observations_global_frame, current_pose);

  // Visualize map and associations
  if (visualizer_->IsVisualizationEnabled()) {
    // Need to get updated map to display associations
    // TODO(Christoforos) might need to be OnlineMap()
    visualizer_->VisualizeGlobalMap(global_map_tracker_->OnlineMap(), current_timestamp);
    visualizer_->VisualizeAssociations(observations_global_frame, global_map_tracker_->FullGlobalMap(), associations,
                                       current_timestamp);
  }
  RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                              "SlamFrontend: Data association done");

  // Convert associations to landmarks
  std::vector<Landmark> landmarks =
      AssociationsToLandmarks(observations_local_frame, observations_global_frame, associations);

  RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                              "SlamFrontend: Associations processed");

  // Update backend with landmarks
  slam_backend_->Update(pose_difference, landmarks);

  RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                              "SlamFrontend: Backend updated");

  // Optimize backend
  slam_backend_->Optimize();

  RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                              "SlamFrontend: Backend optimized");

  // Publish backend last pose, which is map->baselink (from optimizer)
  // Uses the fetched odom->baselink to calculate inverse transformation
  node_handle_->BroadcastEstimatedSlamPose(slam_backend_->LatestPoseEstimate(), current_fetch.second);

  // Update the global map
  global_map_tracker_->UpdateMap(slam_backend_->GlobalMapEstimate());

  RCLCPP_DEBUG_STREAM(node_handle_->get_logger(),
                      "SlamFrontend: Global map cones: " << slam_backend_->GlobalMapEstimate().size());

  // Visualize poses
  if (visualizer_->IsVisualizationEnabled()) {
    visualizer_->VisualizePoses(slam_backend_->PoseEstimates(), current_timestamp);
  }

  // TODO(Christoforos)
  // // Convert the local map to global frame again with new SLAM pose
  // slam_pose = backend_isam_.pose_estimate();
  // if (!mapping_mode_) {
  //   const ConeMap observations_global_frame2 =
  //   slam_backend::utils::TransformMap(
  //       observations_local_frame, {slam_pose.x(), slam_pose.y(),
  //       slam_pose.theta()});
  //   UpdateLocalConsistency(associations, observations_global_frame2,
  //   global_map);
  //    //--> LocalConsistency is the avg error between associated cones from
  //    //global frame and global map
  // }

  // Check the mission's progress
  CheckMissionProgress();

  // Publish map
  autonomous_msgs::msg::ConeArray cone_array;
  cone_array.header.frame_id = "map";
  cone_array.header.stamp = current_timestamp;
  cone_array.cones = global_map_tracker_->OnlineMap();
  node_handle_->PublishOnlineMap(cone_array);

  RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                              "SlamFrontend: Cone callback finished.");
}

std::vector<Landmark> SlamFrontend::AssociationsToLandmarks(const ConeMap &observations_local_frame,
                                                            const ConeMap &observations_global_frame,
                                                            const std::vector<Association> &associations) {
  std::vector<Landmark> landmarks;
  landmarks.reserve(associations.size());

  for (const auto &association : associations) {
    const int observed_cone_index = association.observed_cone_index;

    autonomous_msgs::msg::Cone cone_in_local_frame = observations_local_frame[observed_cone_index];
    autonomous_msgs::msg::Cone cone_in_global_frame = observations_global_frame[observed_cone_index];

    const double local_x = cone_in_local_frame.position.x;
    const double local_y = cone_in_local_frame.position.y;
    // Needed for graph constraint
    const double range = std::hypot(local_x, local_y);
    const double bearing = std::atan2(local_y, local_x);

    // Mapping mode: accept new cones even if they're not associated
    // Localization mode: only accept cone if they're associated (cones with
    // assoc -1 will be ignored)
    if (mapping_mode_ || association.global_map_cone_id != -1) {
      landmarks.push_back(Landmark(cone_in_global_frame, association.global_map_cone_id, range, bearing));
    }
  }

  return landmarks;
}

void SlamFrontend::CheckMissionProgress() {
  if (mission_type_ == MissionType::kTrackdrive) {
    // Switch to localization mode after 1st trackdrive lap
    if (mapping_mode_ && node_handle_->LapCount() >= 1) {
      slam_backend_->SwitchToLocalizationMode();
      global_map_tracker_->SwitchToLocalizationMode();
      mapping_mode_ = false;
      RCLCPP_INFO_STREAM(node_handle_->get_logger(), "SlamFrontend: Switched to localization mode.");
    }
  }
}

} // end namespace slam
