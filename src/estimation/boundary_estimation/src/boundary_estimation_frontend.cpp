/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023 - 2024  Authors:
 *   - Lorenzo Codeluppi <lcodeluppi@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "boundary_estimation_frontend.hpp"
#include <easy/profiler.h>

namespace boundary_estimation {

namespace {
constexpr char kLidarOnlyMode[] = "lidar_only";
constexpr char kCameraOnlyMode[] = "camera_only";
constexpr char kSensorFusionMode[] = "sensor_fusion";
}; // namespace

// Load parameters and setup ROS node
BoundaryEstimationFrontend::BoundaryEstimationFrontend(std::shared_ptr<NodeHandle> nh)
    : node_handler_(nh), lap_counter_completed_(false), past_car_position_(Point_2(0, 0)), full_map_available_(false),
      feedback_successfull_(false) {
  LoadParameters();

  // Load correct backend logic
  boundary_estimation_backend_ = std::make_unique<BoundaryEstimationBackend>(
      this->LoadFeatureSetPoints(), this->LoadFeatureWeights(), this->LoadThresholdValues(), this->LoadOffsetValues(),
      this->LoadColorParams(), this->LoadRuntimeConstraints(), logging_flag_, debug_flag_);

  // create subscriber and publisher
  auto map_callback = [this](const autonomous_msgs::msg::ConeArray &msg) { this->OnlineMapCallback(msg); };

  auto lap_counter_callabck = [this](const std_msgs::msg::Int32 &msg) { this->LapCounterCallback(msg); };

  auto global_map_feedback_callback = [this](const std_msgs::msg::Bool &msg) { this->GlobalMapFeedbackCallback(msg); };

  node_handler_->CreateLapCounterSubscriber(lap_counter_topic_name_, 1, lap_counter_callabck);
  node_handler_->CreateOnlineMapSubscriber(cone_array_topic_name_, 1, map_callback);
  node_handler_->CreateGlobalMapFeedbackSubscriber(global_map_feedback_topic_name_, 1, global_map_feedback_callback);

  node_handler_->CreateBoundaryPublisher(track_topic_name_, 1);
  node_handler_->CreateVisualizerPublisher(visual_topic_name_, 1);
  node_handler_->CreateFinalPathPublisher(final_path_topic_name_, 1);
  node_handler_->CreateFinalPathVisualPublisher(global_map_topic_name_, 1);
}

void BoundaryEstimationFrontend::PublishFinalPath() {
  // Get the crossed midpoints in base_link frame
  if (!full_map_available_) {
    FinalPath final_path = boundary_estimation_backend_->ComputeFullPath(last_past_midpoints_list_);

    // Transform the point back into the global frame
    final_middle_line_ = TransformPairPoints(final_path.midpoints, current_transform_.inverse());
    final_left_boundary_ = TransformPairPoints(final_path.left_bound, current_transform_.inverse());
    final_right_boundary_ = TransformPairPoints(final_path.right_bound, current_transform_.inverse());
    // Log size of the final path
    RCLCPP_INFO(node_handler_->get_logger(), "[boundary estimation] Final Path Size: %ld", final_middle_line_.size());
    RCLCPP_INFO(node_handler_->get_logger(), "[boundary estimation] Final Left Boundary Size: %ld",
                final_left_boundary_.size());
    RCLCPP_INFO(node_handler_->get_logger(), "[boundary estimation] Final Right Boundary Size: %ld",
                final_right_boundary_.size());
    // Visualize the global map in local frame every time is computed
    delaunay_search_viz_.SetFinalPath(ExtractSecondElementFromPairVector(final_middle_line_),
                                      ExtractSecondElementFromPairVector(final_left_boundary_),
                                      ExtractSecondElementFromPairVector(final_right_boundary_));

    final_middle_line_with_confidence_ = ParsePoint2VectorToPointWithConfidenceVector(final_middle_line_);
    final_left_boundary_with_confidence_ = ParsePoint2VectorToPointWithConfidenceVector(final_left_boundary_);
    final_right_boundary_with_confidence_ = ParsePoint2VectorToPointWithConfidenceVector(final_right_boundary_);

    full_map_available_ = true;
    PublishFinalPathMessage(delaunay_search_viz_);
  }

  PublishFinalBoundaryMessage(final_middle_line_with_confidence_, final_left_boundary_with_confidence_,
                              final_right_boundary_with_confidence_);
  // Print debug statement
  RCLCPP_INFO(node_handler_->get_logger(), "[boundary estimation] Final Path Published");
}

void BoundaryEstimationFrontend::GlobalMapFeedbackCallback(const std_msgs::msg::Bool &global_map_feedback) {
  // Print global_map_feedback data
  if (global_map_feedback.data) {
    RCLCPP_INFO(node_handler_->get_logger(), "[boundary estimation] Global Map Feedback is successful.");
    // Either kill the node or do nothing
    feedback_successfull_ = true;
  } else {
    RCLCPP_WARN(node_handler_->get_logger(), "[boundary estimation] Global Map Feedback is not successful.");
    full_map_available_ = false;
    lap_counter_completed_ = false;
    delaunay_search_viz_.ResetFinalPath();
    PublishFinalPathMessage(delaunay_search_viz_);
  }
}

void BoundaryEstimationFrontend::LapCounterCallback(const std_msgs::msg::Int32 &lap_count) {
  // If lap is already completed, do nothing
  RCLCPP_INFO(node_handler_->get_logger(), "[boundary estimation] Lap Counter callback executed.");
  // Set the lap_counter_completed flag to true
  lap_counter_completed_ = true;
  if (!feedback_successfull_) {
    // Publish the final path
    PublishFinalPath();
    // reset the list of past midpoints
    last_past_midpoints_list_.clear();
  }
  // Reset the midpoints map to keep storing the list of previous midpoints for local paths
  past_midpoints_map_.clear();
}

void BoundaryEstimationFrontend::UpdatePastCrossedMidpoints(
    const tf2::Stamped<tf2::Transform> &map_to_base_link_transform,
    const std::vector<std::pair<std::string, Point_2>> &last_past_crossed_midpoints) {
  // Keep publishing the global map if the lap counter has been triggered untill the global planner's feedback is
  // received
  if (lap_counter_completed_ && !feedback_successfull_) {
    PublishFinalPath();
  }

  for (const std::pair<std::string, Point_2> &point : last_past_crossed_midpoints) {
    if (past_midpoints_map_.find(point.first) != past_midpoints_map_.end()) {
      continue;
    }
    past_midpoints_map_[point.first] = true;
    temp_last_past_midpoints_list_.push_back(point.first);
    if (!feedback_successfull_) last_past_midpoints_list_.push_back(point.first);

    // Keep the size of temp_last_past_midpoints_list_ constant for memory effienciency
    if (temp_last_past_midpoints_list_.size() > constants::kMaxTempPastMipointIds) {
      temp_last_past_midpoints_list_.erase(temp_last_past_midpoints_list_.begin());
    }
  }

  tf2::Vector3 current_position_vector = map_to_base_link_transform.inverse().getOrigin();

  Point_2 current_point_map_frame = Point_2(current_position_vector.getX(), current_position_vector.getY());

  past_car_position_ = current_point_map_frame;
}

/*
Computes the path
*/
void BoundaryEstimationFrontend::OnlineMapCallback(const autonomous_msgs::msg::ConeArray &global_map_) {
  EASY_FUNCTION(profiler::colors::Red);
  bool execution_allowed = true;

  //  Do nothing if cone array is empty
  if (global_map_.cones.size() < 3) {
    if (logging_flag_)
      RCLCPP_WARN(node_handler_->get_logger(), "[boundary_estimation] cone array received has less than 3 cones.");
    execution_allowed = false;
  } else {
    if (logging_flag_) RCLCPP_INFO(node_handler_->get_logger(), "cone array received");
  }

  current_time_ = global_map_.header.stamp;
  tf2::TimePoint time_point = tf2::TimePoint(std::chrono::nanoseconds(current_time_.nanoseconds()));

  std::optional<geometry_msgs::msg::TransformStamped> transform =
      node_handler_->GetTransform("map", "base_link", time_point);

  if (!transform.has_value()) {
    if (logging_flag_) RCLCPP_WARN(node_handler_->get_logger(), "[boundary_estimation] transform not available.");
    execution_allowed = false;
  }

  if (execution_allowed) {
    tf2::Stamped<tf2::Transform> stamped_transform;
    tf2::fromMsg(transform.value(), stamped_transform);

    current_transform_ = stamped_transform;

    // Local version of the cone array received by SLAM
    autonomous_msgs::msg::ConeArray local_cone_array = ConvertGlobalToLocalMap(global_map_, stamped_transform);

    // List of previous past car position from TFs cached (they do not
    // corrispond to previous midpoints, but previous position in map frame)
    Point_2 past_car_position_local_frame = TransformPoint(past_car_position_, stamped_transform);

    BoundaryLogicResult backend_result = boundary_estimation_backend_->ComputeBoundedPath(
        local_cone_array.cones, past_car_position_local_frame, temp_last_past_midpoints_list_);

    std::vector<Point_2> midpoints = ExtractSecondElementFromPairVector(backend_result.midpoints);
    std::vector<Point_2> left_bound = ExtractSecondElementFromPairVector(backend_result.left_bound);
    std::vector<Point_2> right_bound = ExtractSecondElementFromPairVector(backend_result.right_bound);

    if (backend_result.success) {
      std::vector<autonomous_msgs::msg::PointWithConfidence> middle_line =
          ParsePoint2VectorToPointWithConfidenceVector(backend_result.midpoints);

      std::vector<autonomous_msgs::msg::PointWithConfidence> left_boundary =
          ParsePoint2VectorToPointWithConfidenceVector(backend_result.left_bound);

      std::vector<autonomous_msgs::msg::PointWithConfidence> right_boundary =
          ParsePoint2VectorToPointWithConfidenceVector(backend_result.right_bound);

      PublishBoundaryMessage(middle_line, left_boundary, right_boundary);

    } else {
      if (logging_flag_)
        RCLCPP_WARN(node_handler_->get_logger(), "[boundary_estimation] Unable to generate bounded path");
    }

    // Compute crossed midpoints in map frame
    UpdatePastCrossedMidpoints(stamped_transform, backend_result.last_past_middle_points);

    if (visualizer_flag_) {
      // Set viz for bounded path and map
      std::vector<Point_2> past_crossed_midpoints =
          ExtractSecondElementFromPairVector(backend_result.past_middle_points);
      delaunay_search_viz_.SetTrack(midpoints, left_bound, right_bound);

      delaunay_search_viz_.SetMap(global_map_);

      delaunay_search_viz_.SetPaths(backend_result.candidates);

      delaunay_search_viz_.SetFilteredPaths(backend_result.filtered_path_list);

      delaunay_search_viz_.SetTriangulationEdges(backend_result.triangulation_edges);

      delaunay_search_viz_.SetCrossedMidpoints(past_crossed_midpoints);

      PublishVisualizerMessage(delaunay_search_viz_);
    }
  }
}

/*
Safe ROS node parameter loading
*/
template <typename Type> Type BoundaryEstimationFrontend::GetParam(const std::string &name) {
  Type val;
  std::string name_replaced = name;
  std::replace(name_replaced.begin(), name_replaced.end(), '/', '.');
  if (!node_handler_->get_parameter(name_replaced, val)) {
    RCLCPP_ERROR(node_handler_->get_logger(), "PARAMETER NOT FOUND: %s", name_replaced.c_str());
    rclcpp::shutdown();
  }

  return val;
}

/**
 * Convert a global map of cones to a local map.
 *
 * This function takes a global map of cones and a transformation to convert
 * coordinates from a global reference frame to a local reference frame. It
 * returns a modified local map of cones after applying the transformation.
 */
autonomous_msgs::msg::ConeArray
BoundaryEstimationFrontend::ConvertGlobalToLocalMap(const autonomous_msgs::msg::ConeArray &global_map_,
                                                    const tf2::Stamped<tf2::Transform> stamped_transform) {
  autonomous_msgs::msg::ConeArray local_cone_array;
  local_cone_array.header.frame_id = "base_link";
  local_cone_array.header.stamp = current_time_;

  header_lm_ = local_cone_array.header;

  for (auto &cone : global_map_.cones) {
    autonomous_msgs::msg::Cone local_cone;
    local_cone = cone;

    tf2::Transform point_transform;

    // Transform cone from global to local frame
    point_transform.setOrigin(tf2::Vector3(cone.position.x, cone.position.y, 0.0));
    point_transform = stamped_transform * point_transform;
    tf2::Vector3 point_position_vector = point_transform.getOrigin();
    local_cone.position.x = point_position_vector.getX();
    local_cone.position.y = point_position_vector.getY();

    local_cone_array.cones.push_back(local_cone);
  }
  return local_cone_array;
}

/*
Publish bounded path
*/
void BoundaryEstimationFrontend::PublishBoundaryMessage(
    const std::vector<autonomous_msgs::msg::PointWithConfidence> &middle_line,
    const std::vector<autonomous_msgs::msg::PointWithConfidence> &left_boundary,
    const std::vector<autonomous_msgs::msg::PointWithConfidence> &right_boundary) const {
  autonomous_msgs::msg::Boundary boundary;
  boundary.header = header_lm_;
  boundary.header.stamp = current_time_;
  boundary.left_boundary = left_boundary;
  boundary.right_boundary = right_boundary;
  boundary.middle_line = middle_line;

  node_handler_->PublishBoundary(boundary);
}

/*
Publish final bounded path
*/
void BoundaryEstimationFrontend::PublishFinalBoundaryMessage(
    const std::vector<autonomous_msgs::msg::PointWithConfidence> &middle_line,
    const std::vector<autonomous_msgs::msg::PointWithConfidence> &left_boundary,
    const std::vector<autonomous_msgs::msg::PointWithConfidence> &right_boundary) const {
  autonomous_msgs::msg::Boundary boundary;
  boundary.header.frame_id = "map";
  boundary.header.stamp = node_handler_->get_clock()->now();
  boundary.left_boundary = left_boundary;
  boundary.right_boundary = right_boundary;
  boundary.middle_line = middle_line;

  node_handler_->PublishFinalBoundary(boundary);
}

/*
Publish visualization
*/
void BoundaryEstimationFrontend::PublishVisualizerMessage(BoundaryEstimationVisualization &delaunay_search_viz) const {
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(delaunay_search_viz.position_visual());
  marker_array.markers.push_back(delaunay_search_viz.left_bound_visual());
  marker_array.markers.push_back(delaunay_search_viz.right_bound_visual());
  marker_array.markers.push_back(delaunay_search_viz.midpoints_visual());
  marker_array.markers.push_back(delaunay_search_viz.filtered_candidate_paths_visual());
  marker_array.markers.push_back(delaunay_search_viz.triangulation_edges_visual());
  for (auto &marker : marker_array.markers) {
    marker.header.frame_id = header_lm_.frame_id;
    marker.header.stamp = current_time_;
  }
  node_handler_->PublishVisualizer(marker_array);
}

void BoundaryEstimationFrontend::PublishFinalPathMessage(BoundaryEstimationVisualization &delaunay_search_viz) const {
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(delaunay_search_viz.final_left_bound_visual());
  marker_array.markers.push_back(delaunay_search_viz.final_right_bound_visual());
  marker_array.markers.push_back(delaunay_search_viz.final_midpoints_visual());
  for (auto &marker : marker_array.markers) {
    marker.header.frame_id = "map";
    marker.header.stamp = current_time_;
  }
  node_handler_->PublishFinalPathVisual(marker_array);
}

/*
Load topic names and flags
*/
void BoundaryEstimationFrontend::LoadParameters() {
  RCLCPP_INFO(node_handler_->get_logger(), "[boundary estimation] Loading Topic Names and Flags");

  track_topic_name_ = GetParam<std::string>("topics/track_topic_name");
  cone_array_topic_name_ = GetParam<std::string>("topics/cone_array_topic_name");
  lap_counter_topic_name_ = GetParam<std::string>("topics/lap_counter_topic_name");
  final_path_topic_name_ = GetParam<std::string>("topics/final_path_topic_name");
  visual_topic_name_ = GetParam<std::string>("topics/visual_topic_name");
  global_map_feedback_topic_name_ = GetParam<std::string>("topics/global_map_feedback_topic_name");
  visualizer_flag_ = GetParam<bool>("visualization/enabled");
  profiling_flag_ = GetParam<bool>("profiling/enabled");
  logging_flag_ = GetParam<bool>("logging/enabled");
  debug_flag_ = GetParam<bool>("debug/enabled");
  perception_mode_ = GetParam<std::string>("perception_mode");
  global_map_topic_name_ = GetParam<std::string>("topics/global_map_topic_name");
}

/*
Getter for topic name of modes
*/
std::string BoundaryEstimationFrontend::GetModeTopicName(Pipeline mode) {
  switch (mode) {
  case Pipeline::Lidar_Only:
    return kLidarOnlyMode;
  case Pipeline::Yolo_Only:
    return kCameraOnlyMode;
  case Pipeline::Sensor_Fusion:
    return kSensorFusionMode;
  default:
    return kSensorFusionMode;
  }
}

/*
Load parameters based on given mode
*/
heuristics::FeatureSetpoints BoundaryEstimationFrontend::LoadFeatureSetPoints() {
  // std::string perception_mode_ = GetModeTopicName(mode);

  RCLCPP_INFO_STREAM(node_handler_->get_logger(),
                     "[boundary estimation] Loading Feature Setpoints for mode: " << perception_mode_);

  heuristics::FeatureSetpoints feature_setpoints;

  feature_setpoints.desired_length_m = GetParam<float>(perception_mode_ + "/desired_length");
  feature_setpoints.desired_witdh_std_m = GetParam<float>(perception_mode_ + "/desired_witdh_std");
  feature_setpoints.desired_max_angle_change_rad =
      GetParam<float>(perception_mode_ + "/desired_max_angle_change") / 180.0 * M_PI;
  feature_setpoints.desired_boundary_std_m = GetParam<float>(perception_mode_ + "/desired_boundary_std");

  return feature_setpoints;
}

/*
Load weights based on given mode
*/
heuristics::FeatureWeights BoundaryEstimationFrontend::LoadFeatureWeights() {
  // std::string perception_mode_ = GetModeTopicName(mode);

  RCLCPP_INFO_STREAM(node_handler_->get_logger(),
                     "[boundary estimation] Loading Feature Weights for mode: " << perception_mode_);

  heuristics::FeatureWeights feature_weights;

  feature_weights.length_error_w = GetParam<float>(perception_mode_ + "/length_error_w");
  feature_weights.track_width_w = GetParam<float>(perception_mode_ + "/track_width_w");
  feature_weights.step_angle_w = GetParam<float>(perception_mode_ + "/step_angle_w");
  feature_weights.cones_distance_var_w = GetParam<float>(perception_mode_ + "/cones_distance_var_w");
  feature_weights.prior_w = GetParam<float>(perception_mode_ + "/prior_w");

  return feature_weights;
}

/*
Load thresolds parameters based on given mode
*/
heuristics::ThresholdValues BoundaryEstimationFrontend::LoadThresholdValues() {
  // std::string perception_mode_ = GetModeTopicName(mode);

  RCLCPP_INFO_STREAM(node_handler_->get_logger(),
                     "[boundary estimation] Loading Threshold Values for mode: " << perception_mode_);

  heuristics::ThresholdValues threshold_values;

  threshold_values.maximum_number_of_far_distant_cones =
      GetParam<int>(perception_mode_ + "/maximum_number_of_far_distant_cones");
  threshold_values.minimum_segment_width_m = GetParam<float>(perception_mode_ + "/minimum_segment_width");
  threshold_values.maximum_segment_width_m = GetParam<float>(perception_mode_ + "/maximum_segment_width");
  threshold_values.minimum_track_width_m = GetParam<float>(perception_mode_ + "/minimum_track_width");
  threshold_values.maximum_track_width_m = GetParam<float>(perception_mode_ + "/maximum_track_width");
  threshold_values.maximum_angle_change_rad =
      GetParam<float>(perception_mode_ + "/maximum_angle_change") / 180.0 * M_PI;
  threshold_values.maximum_end_angle_change_rad =
      GetParam<float>(perception_mode_ + "/maximum_end_angle_change") / 180.0 * M_PI;
  threshold_values.minimum_presence_probability = GetParam<float>(perception_mode_ + "/minimum_presence_probability");
  threshold_values.minimum_path_segment_length_m = GetParam<float>(perception_mode_ + "/minimum_path_segment_length");
  threshold_values.maximum_path_segment_length_m = GetParam<float>(perception_mode_ + "/maximum_path_segment_length");
  threshold_values.maximum_boundary_segment_length_m =
      GetParam<float>(perception_mode_ + "/maximum_boundary_segment_length");
  threshold_values.minimum_path_length_m = GetParam<float>(perception_mode_ + "/minimum_path_length");
  threshold_values.desirable_minimum_path_length_m =
      GetParam<float>(perception_mode_ + "/desirable_minimum_path_length");
  return threshold_values;
}

/*
Load offset parameters based on given mode
*/
heuristics::OffsetValues BoundaryEstimationFrontend::LoadOffsetValues() {
  // std::string perception_mode_ = GetModeTopicName(mode);

  RCLCPP_INFO_STREAM(node_handler_->get_logger(),
                     "[boundary estimation] Loading Offset Values for mode: " << perception_mode_);

  heuristics::OffsetValues offset_values;

  offset_values.maximum_number_of_far_distant_cones_offset =
      GetParam<int>(perception_mode_ + "/maximum_number_of_far_distant_cones_relaxation_offset");
  offset_values.minimum_segment_width_offset_m =
      GetParam<float>(perception_mode_ + "/minimum_segment_width_relaxation_offset");
  offset_values.maximum_segment_width_offset_m =
      GetParam<float>(perception_mode_ + "/maximum_segment_width_relaxation_offset");
  offset_values.minimum_track_width_offset_m =
      GetParam<float>(perception_mode_ + "/minimum_track_width_relaxation_offset");
  offset_values.maximum_track_width_offset_m =
      GetParam<float>(perception_mode_ + "/maximum_track_width_relaxation_offset");
  offset_values.maximum_angle_change_offset_rad =
      GetParam<float>(perception_mode_ + "/maximum_angle_change_relaxation_offset") / 180.0 * M_PI;
  offset_values.minimum_path_segment_length_offset_m =
      GetParam<float>(perception_mode_ + "/minimum_path_segment_length_relaxation_offset");
  offset_values.maximum_path_segment_length_offset_m =
      GetParam<float>(perception_mode_ + "/maximum_path_segment_length_relaxation_offset");

  return offset_values;
}

/*
Load color handling parameters based on given mode
*/
heuristics::ColorParams BoundaryEstimationFrontend::LoadColorParams() {
  // std::string perception_mode_ = GetModeTopicName(mode);

  RCLCPP_INFO_STREAM(node_handler_->get_logger(),
                     "[boundary estimation] Loading Color Parameters for mode: " << perception_mode_);

  heuristics::ColorParams color_params;
  color_params.use_orange = GetParam<bool>(perception_mode_ + "/use_orange");

  return color_params;
}

/*
Load Delaunay triangulation parameters based on given mode
*/
heuristics::RuntimeConstraints BoundaryEstimationFrontend::LoadRuntimeConstraints() {
  // std::string perception_mode_ = GetModeTopicName(mode);

  RCLCPP_INFO_STREAM(node_handler_->get_logger(),
                     "[boundary estimation] Loading Runtime Constraints for mode: " << perception_mode_);

  heuristics::RuntimeConstraints runtime_constraints;

  runtime_constraints.max_number_of_leaves = GetParam<float>(perception_mode_ + "/max_number_of_leaves");
  runtime_constraints.number_of_iterations = GetParam<float>(perception_mode_ + "/number_of_iterations");
  runtime_constraints.full_relaxation_enabled = GetParam<bool>("full_relaxation_enabled");

  return runtime_constraints;
}

} // namespace boundary_estimation
