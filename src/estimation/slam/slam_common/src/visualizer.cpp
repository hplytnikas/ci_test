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
#include "slam_common/visualizer.hpp"

#include <easy/profiler.h>

namespace slam {

Visualizer::Visualizer(std::shared_ptr<SlamNode> node_handle) : node_handle_(node_handle) { LoadParameters(); }

Visualizer::~Visualizer() {}

void Visualizer::LoadParameters() {
  visualization_enabled_ = node_handle_->GetParameter<bool>("visualization_enabled", false);

  // Set common colors
  blue_.r = 0;
  blue_.g = 0;
  blue_.b = 1;
  blue_.a = 1;
  yellow_.r = 1;
  yellow_.g = 1;
  yellow_.b = 0;
  yellow_.a = 1;
  orange_.r = 1;
  orange_.g = 0.65;
  orange_.b = 0;
  orange_.a = 1;
  gray_.r = 1;
  gray_.g = 1;
  gray_.b = 1;
  gray_.a = 1;
  RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                     "Visualizer: Initialized with visualization enabled:" << visualization_enabled_);
}

bool Visualizer::IsVisualizationEnabled() const { return visualization_enabled_; }

void Visualizer::VisualizeGlobalMap(const ConeMap &cone_map, const rclcpp::Time &timestamp) const {
  EASY_FUNCTION(profiler::colors::Blue); // Time this function
  if (cone_map.size() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray map_markers;
  for (const auto &cone : cone_map) {
    visualization_msgs::msg::Marker marker = CreateMarker(timestamp, "cones", visualization_msgs::msg::Marker::SPHERE,
                                                          visualization_msgs::msg::Marker::MODIFY);
    marker.pose.position.x = cone.position.x;
    marker.pose.position.y = cone.position.y;
    marker.pose.position.z = 0.01; // Slightly above ground for visibility

    if (cone.prob_type.blue > cone.prob_type.yellow && cone.prob_type.blue > cone.prob_type.orange &&
        cone.prob_type.blue > cone.prob_type.orange_big) {
      marker.color = blue_;
    } else if (cone.prob_type.yellow > cone.prob_type.blue && cone.prob_type.yellow > cone.prob_type.orange &&
               cone.prob_type.yellow > cone.prob_type.orange_big) {
      marker.color = yellow_;
    } else if (cone.prob_type.orange > cone.prob_type.blue && cone.prob_type.orange > cone.prob_type.yellow &&
               cone.prob_type.orange > cone.prob_type.orange_big) {
      marker.color = orange_;
    } else if (cone.prob_type.orange_big > cone.prob_type.blue && cone.prob_type.orange_big > cone.prob_type.yellow &&
               cone.prob_type.orange_big > cone.prob_type.orange) {
      marker.color = orange_;
    } else {
      marker.color = gray_; // Default gray
    }
    marker.id = cone.id_cone;
    map_markers.markers.push_back(marker);
  }

  node_handle_->PublishGlobalMapVisualization(map_markers);
}

void Visualizer::VisualizePoses(const std::vector<StampedPose> &poses, const rclcpp::Time &timestamp) const {
  EASY_FUNCTION(profiler::colors::Blue); // Time this function
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "map";
  pose_array.header.stamp = timestamp;

  for (const auto &pose : poses) {
    geometry_msgs::msg::Pose geom_pose;
    const auto tf = utils::Pose2DToGeometryMsg(pose.AsPose2D());

    geom_pose.position.x = pose.X();
    geom_pose.position.y = pose.Y();
    geom_pose.position.z = 0;
    geom_pose.orientation = tf.rotation;

    pose_array.poses.push_back(geom_pose);
  }

  node_handle_->PublishPosesVisualization(pose_array);
}

void Visualizer::VisualizeAssociations(const ConeMap &observations, const ConeMap &global_map,
                                       const std::vector<Association> &associations,
                                       const rclcpp::Time &timestamp) const {
  EASY_FUNCTION(profiler::colors::Blue); // Time this function
  if (global_map.size() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  for (auto &association : associations) {
    // Define the marker and its specs
    auto line_marker = CreateMarker(timestamp, "associations", visualization_msgs::msg::Marker::ARROW,
                                    visualization_msgs::msg::Marker::ADD);
    line_marker.id = markers.markers.size();
    line_marker.scale.z = 0.2;
    line_marker.color.a = 1.0;
    line_marker.color.r = 1.0;

    // Get the coordinates of the cones associated
    int global_map_cone_id = association.global_map_cone_id;
    int global_map_index = 0;

    // Skip if not associated
    if (global_map_cone_id == -1) {
      continue;
    }

    // Get the index of the global map where the cone_id is the id from
    // association
    size_t global_map_size = global_map.size();
    for (size_t idx = 0; idx < global_map_size; ++idx) {
      if (global_map[idx].id_cone == global_map_cone_id) {
        global_map_index = static_cast<int>(idx);
        break;
      }
    }

    auto observed_cone = observations[association.observed_cone_index];
    auto global_map_cone = global_map[global_map_index];

    // Specify starting point of arrow of observed cone
    geometry_msgs::msg::Point startPoint;
    startPoint.x = observed_cone.position.x;
    startPoint.y = observed_cone.position.y;
    startPoint.z = observed_cone.position.z;
    line_marker.points.push_back(startPoint);

    // Specify end point of arrow of global map cone
    geometry_msgs::msg::Point endPoint;
    endPoint.x = global_map_cone.position.x;
    endPoint.y = global_map_cone.position.y;
    endPoint.z = global_map_cone.position.z;
    line_marker.points.push_back(endPoint);

    markers.markers.push_back(line_marker);
  }

  node_handle_->PublishAssociationsVisualization(markers);
}

void Visualizer::VisualizeRawGraph(const std::shared_ptr<GraphVisualization> graph,
                                   const rclcpp::Time &timestamp) const {
  EASY_FUNCTION(profiler::colors::Blue); // Time this function
  auto node_marker = CreateMarker(timestamp, "nodes", visualization_msgs::msg::Marker::SPHERE_LIST,
                                  visualization_msgs::msg::Marker::MODIFY);
  node_marker.id = 0;
  auto edge_marker = CreateMarker(timestamp, "edges", visualization_msgs::msg::Marker::LINE_LIST,
                                  visualization_msgs::msg::Marker::ADD);
  edge_marker.id = 1;

  edge_marker.color.r = 0;
  edge_marker.color.g = 0;
  edge_marker.color.b = 0.1;
  edge_marker.color.a = 1;
  edge_marker.scale.x = 0.05; // line width meters

  const auto &landmark_points = graph->LandmarkPoints();
  const auto &odometry_points = graph->OdometryPoints();
  std::unordered_set<size_t> used_landmark_indices;
  std::unordered_set<size_t> used_odometry_indices;

  // Insert edge points
  for (const auto &edge : graph->LandmarkEdges()) {
    edge_marker.points.push_back(odometry_points[edge.first]);
    edge_marker.points.push_back(landmark_points[edge.second]);
    used_odometry_indices.insert(edge.first);
    used_landmark_indices.insert(edge.second);
  }

  for (const auto &edge : graph->OdometryEdges()) {
    edge_marker.points.push_back(odometry_points[edge.first]);
    edge_marker.points.push_back(odometry_points[edge.second]);
    used_odometry_indices.insert(edge.first);
    used_odometry_indices.insert(edge.second);
  }

  // Insert landmark node points
  for (const auto &idx : used_landmark_indices) {
    node_marker.points.push_back(landmark_points.at(idx));
    node_marker.colors.push_back(blue_);
  }

  // Insert pose node points
  for (const auto &idx : used_odometry_indices) {
    node_marker.points.push_back(odometry_points.at(idx));
    node_marker.colors.push_back(orange_);
  }

  visualization_msgs::msg::MarkerArray markers;
  markers.markers = {node_marker, edge_marker};

  node_handle_->PublishRawGraphVisualization(markers);
}

visualization_msgs::msg::Marker Visualizer::CreateMarker(const rclcpp::Time &timestamp, std::string ns, int32_t type,
                                                         int32_t action) const {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = timestamp;

  marker.frame_locked = true; // If false there is weird flickering
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2; // meters
  marker.scale.y = 0.2; // meters
  marker.scale.z = 0.2; // meters

  marker.type = type;
  marker.action = action;
  marker.ns = ns;

  return marker;
}

} // end namespace slam
