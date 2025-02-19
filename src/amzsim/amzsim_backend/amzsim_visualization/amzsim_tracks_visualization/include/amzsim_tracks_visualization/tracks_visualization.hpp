/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023 Authors:
 *   - Jonas Grütter <jgruette@ethz.ch>
 *   - Bartosz Mila <bamila@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "amzsim_utils/utils.hpp"

namespace tracks_visualization {

struct Vec2d {
  double x = 0, y = 0;
};

class TracksVisualizer : public rclcpp::Node {
public:
  /*!
   * Constructor.
   * @param options the ROS2 node options.
   */
  explicit TracksVisualizer(const rclcpp::NodeOptions &options);

  /*!
   * Destructor.
   */
  virtual ~TracksVisualizer() = default;

  /**
   * Allows to display a marker in Rviz
   * This might be useful to understand this function:
   * http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
   * @param id Id of the marker, has to be different for each one
   * @param x x coord of the marker
   * @param y y coord of the marker
   * @param shape Shape of the marker (e.g.
   * visualization_msgs::msg::Marker::CUBE)
   * @param color RGB Color of the marker
   *
   * @return A marker
   */
  visualization_msgs::msg::Marker createMarker(int id, double x, double y, uint32_t shape,
                                               std::vector<float> color = {1.0, 1.0, 1.0});

  /**
   * Allows to display a cone in Rviz using createMarker()
   * @param id Id of the marker representing the cone, has to be different for
   * each one
   * @param x x coord of the marker
   * @param y y coord of the marker
   * @param type Type of cone, can be "cone_blue" or "cone_yellow"
   *
   * @return A marker
   */
  visualization_msgs::msg::Marker createCone(int id, double x, double y, std::string type);

  /**
   * Reads a CSV file and stores the (x,y) position of the cone "categoryName"
   * in data
   * @param filename Name of the file/track from which the cone position will be
   * loaded
   * @param categoryName Category of the cone of which position will be read,
   * "big_orange", "orange", "blue", "yellow"
   * @param data Vector containing the (x,y) position of the cones from
   * categoryName
   * @return nothing
   */
  void loadCategoryFromCSV(const std::string &filename, const std::string &categoryName, std::vector<Vec2d> &data);

  /**
   * Loads each position of each type of cones into the respective buffers
   * @param filetype type of the file that will be read "CSV"
   * @return nothing
   */
  void loadCones();

  /**
   * Returns the path to track csv file based on track name argument passed to
   * launch file.
   * @param none
   * @return Path to track csv file
   */
  std::string loadTrackPath();

  /**
   * Publishes the markers (i.e. cones) into RViz
   * @param none
   * @return nothing
   */
  void publishMarkers();

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::vector<Vec2d> cones_blue_buffer_;
  std::vector<Vec2d> cones_yellow_buffer_;
  std::vector<Vec2d> cones_orange_buffer_;
  std::vector<Vec2d> cones_orange_big_buffer_;
  std::vector<Vec2d> cones_none_buffer_;

  std::vector<visualization_msgs::msg::Marker> array_marker_;
};

} // namespace tracks_visualization
