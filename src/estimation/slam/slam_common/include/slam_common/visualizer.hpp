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

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "slam_common/aliases.hpp"
#include "slam_common/association.hpp"
#include "slam_common/graph_visualization.hpp"
#include "slam_common/slam_node.hpp"
#include "slam_common/stamped_pose.hpp"
#include "slam_common/utils.hpp"

namespace slam {

/*
 * A class for visualizing different parts of the SLAM flow. Converts
 * main structures to marker/pose arrays for visualization. Uses the
 * SlamNode publishers to publish the marker/pose arrays.
 */
class Visualizer {
public:
  /*
   * Constructor
   */
  explicit Visualizer(std::shared_ptr<SlamNode> node_handle);

  /*
   * Destructor.
   */
  ~Visualizer();

  // Visualize the global cone map
  void VisualizeGlobalMap(const ConeMap &cone_map, const rclcpp::Time &timestamp) const;

  // Visualize all estimated poses until this point
  void VisualizePoses(const std::vector<StampedPose> &poses, const rclcpp::Time &timestamp) const;

  // Visualize the graph used in GraphSLAM
  void VisualizeRawGraph(const std::shared_ptr<GraphVisualization> graph, const rclcpp::Time &timestamp) const;

  // Visualize the data associations between local/global cones
  void VisualizeAssociations(const ConeMap &observations, const ConeMap &global_map,
                             const std::vector<Association> &associations, const rclcpp::Time &timestamp) const;

  bool IsVisualizationEnabled() const;

private:
  // Shared pointer to slam node
  std::shared_ptr<SlamNode> node_handle_;

  // Parameters
  bool visualization_enabled_ = false;
  void LoadParameters();

  // Helper to create a marker
  visualization_msgs::msg::Marker CreateMarker(const rclcpp::Time &timestamp, std::string ns, int32_t type,
                                               int32_t action) const;

  std_msgs::msg::ColorRGBA blue_;
  std_msgs::msg::ColorRGBA yellow_;
  std_msgs::msg::ColorRGBA orange_;
  std_msgs::msg::ColorRGBA gray_;
}; // end class Visualizer

} // end namespace slam
