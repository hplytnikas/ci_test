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

#pragma once

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32.hpp"

#include "autonomous_msgs/msg/boundary.hpp"
#include "autonomous_msgs/msg/cone.hpp"
#include "autonomous_msgs/msg/cone_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "boundary_estimation_backend.hpp"
#include "boundary_estimation_structs.hpp"
#include "boundary_estimation_visualization.hpp"
#include "helpers.hpp"
#include "node_handle.hpp"

namespace boundary_estimation {

enum Pipeline { Yolo_Only, Lidar_Only, Sensor_Fusion };

class BoundaryEstimationFrontend {
public:
  explicit BoundaryEstimationFrontend(std::shared_ptr<NodeHandle> nodeHandle);

  void PublishBoundaryMessage(const std::vector<autonomous_msgs::msg::PointWithConfidence> &middle_line,
                              const std::vector<autonomous_msgs::msg::PointWithConfidence> &left_boundary,
                              const std::vector<autonomous_msgs::msg::PointWithConfidence> &right_boundary) const;

  void PublishFinalBoundaryMessage(const std::vector<autonomous_msgs::msg::PointWithConfidence> &middle_line,
                                   const std::vector<autonomous_msgs::msg::PointWithConfidence> &left_boundary,
                                   const std::vector<autonomous_msgs::msg::PointWithConfidence> &right_boundary) const;

private:
  template <class Type> Type GetParam(const std::string &name);

  void ComputePathWithFullTrack(const tf2::Transform &stamped_transform);

  autonomous_msgs::msg::ConeArray ConvertGlobalToLocalMap(const autonomous_msgs::msg::ConeArray &global_map_,
                                                          const tf2::Stamped<tf2::Transform> stamped_transform);

  void PublishVisualizerMessage(BoundaryEstimationVisualization &delaunay_search_viz) const;

  void PublishFinalPathMessage(BoundaryEstimationVisualization &delaunay_search_viz) const;

  void OnlineMapCallback(const autonomous_msgs::msg::ConeArray &cone_array);

  void LapCounterCallback(const std_msgs::msg::Int32 &lap_count);

  void GlobalMapFeedbackCallback(const std_msgs::msg::Bool &global_map_feedback);

  void UpdatePastCrossedMidpoints(const tf2::Stamped<tf2::Transform> &map_to_base_link_transform,
                                  const std::vector<std::pair<std::string, Point_2>> &last_past_crossed_midpoints);

  void LoadParameters();

  void SelectCorrectPipelineConfig(BoundaryEstimationBackend *&delaunay_search_logic, const int pipeline);

  void PublishFinalPath();

  std::string GetModeTopicName(Pipeline mode);

  heuristics::FeatureSetpoints LoadFeatureSetPoints();

  heuristics::FeatureWeights LoadFeatureWeights();

  heuristics::ThresholdValues LoadThresholdValues();

  heuristics::OffsetValues LoadOffsetValues();

  heuristics::ColorParams LoadColorParams();

  heuristics::RuntimeConstraints LoadRuntimeConstraints();

  std::shared_ptr<NodeHandle> node_handler_;

  rclcpp::Time current_time_;
  tf2::Transform current_transform_;

  std_msgs::msg::Header header_lm_;

  std::unique_ptr<BoundaryEstimationBackend> boundary_estimation_backend_;
  BoundaryEstimationVisualization delaunay_search_viz_;

  // Parameters to be loaded
  std::string track_topic_name_;
  std::string cone_array_topic_name_;
  std::string visual_topic_name_;
  std::string lap_counter_topic_name_;
  std::string final_path_topic_name_;
  std::string global_map_feedback_topic_name_;
  std::string global_map_topic_name_;
  bool logging_flag_;
  bool visualizer_flag_;
  bool profiling_flag_;
  bool debug_flag_;
  std::string perception_mode_;

  Point_2 past_car_position_;
  std::unordered_map<std::string, bool> crossed_edge_map_;
  std::unordered_map<std::string, bool> past_midpoints_map_;
  std::vector<std::string> last_past_midpoints_list_;
  std::vector<std::string> temp_last_past_midpoints_list_;

  // Flag for lap counter
  bool lap_counter_completed_;
  bool full_map_available_;
  bool feedback_successfull_;

  // History boundaries with their posterior likelihood, to tackle flickering
  // problem
  int size_midpath_history_;
  std::deque<PrevPath> past_paths_;

  // Final path vector
  std::vector<std::pair<std::string, Point_2>> final_middle_line_;
  std::vector<std::pair<std::string, Point_2>> final_left_boundary_;
  std::vector<std::pair<std::string, Point_2>> final_right_boundary_;
  std::vector<autonomous_msgs::msg::PointWithConfidence> final_middle_line_with_confidence_;
  std::vector<autonomous_msgs::msg::PointWithConfidence> final_left_boundary_with_confidence_;
  std::vector<autonomous_msgs::msg::PointWithConfidence> final_right_boundary_with_confidence_;
};

} // namespace boundary_estimation
