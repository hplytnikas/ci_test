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

#include "amzsim_tracks_visualization/tracks_visualization.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <string>

namespace tracks_visualization {
/**
 * Constructor of tracks_visualizer
 * Initializes the nodehandle
 */
TracksVisualizer::TracksVisualizer(const rclcpp::NodeOptions &options) : Node("tracks_visualization", options) {
  // Initialize the publisher
  marker_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker",
                                                              rclcpp::QoS(100)); // Adjust the QoS settings as needed

  this->declare_parameter<std::string>("track_path", "default_value1");

  // Additional initialization as needed
  RCLCPP_INFO(this->get_logger(), "tracks_visualization node launched!");
}

/*
    Create a Rviz marker following the same
    procedure as in
   http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
*/
visualization_msgs::msg::Marker TracksVisualizer::createMarker(int id, double x, double y, uint32_t shape,
                                                               std::vector<float> color) {
  visualization_msgs::msg::Marker marker;
  // Set the frame ID and timestamp
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();

  // Set the namespace and id for this marker
  marker.ns = "track_viz";
  marker.id = id;

  // Set the marker type
  marker.type = shape;

  // Set the marker action
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Set the pose of the marker
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1.0;

  marker.lifetime = rclcpp::Duration(0, 0); // or set a specific duration

  return marker;
}

/*
    Create a cone Rviz marker using the mesh of the cones specified in the mesh
   files below
*/
visualization_msgs::msg::Marker TracksVisualizer::createCone(int id, double x, double y, std::string type) {
  // If the marker is a cone, its shape is a mesh ressource
  // Declare the marker to be returned
  visualization_msgs::msg::Marker marker;
  // Declare the color of the cone, it is different for each type of cone
  std::vector<float> color(3);

  // Check which type of cone it is
  // For each type: Select the right color, create the marer, add the mesh
  if (type == "cone_blue") {
    // Blue
    color = {0.0f, 0.0f, 1.0f};
    marker = createMarker(id, x, y, visualization_msgs::msg::Marker::MESH_RESOURCE, color);
    marker.mesh_resource = "package://amzsim_tracks_visualization/meshes/"
                           "cone_blue.dae"; // We have to specify a color
                                            // as it is a marker
  } else if (type == "cone_yellow") {
    // Yellow
    color = {1.0f, 1.0f, 0.0f};
    marker = createMarker(id, x, y, visualization_msgs::msg::Marker::MESH_RESOURCE, color);
    marker.mesh_resource = "package://amzsim_tracks_visualization/meshes/cone_yellow.dae";
  } else if (type == "cone_orange") {
    // orange
    color = {1.0f, 0.0f, 0.0f};
    marker = createMarker(id, x, y, visualization_msgs::msg::Marker::MESH_RESOURCE, color);
    marker.mesh_resource = "package://amzsim_tracks_visualization/meshes/cone_orange.dae";
  } else if (type == "cone_orange_big") {
    // orange
    color = {1.0f, 0.0f, 0.0f};
    marker = createMarker(id, x, y, visualization_msgs::msg::Marker::MESH_RESOURCE, color);
    marker.mesh_resource = "package://amzsim_tracks_visualization/meshes/cone_orange_big.dae";
  } else if (type == "cone_none") {
    // orange
    color = {0.0f, 0.0f, 0.0f};
    marker = createMarker(id, x, y, visualization_msgs::msg::Marker::MESH_RESOURCE, color);
    marker.mesh_resource = "package://amzsim_tracks_visualization/meshes/cone_light_gray.dae";
  } else {
    // It is not a type
    RCLCPP_ERROR(this->get_logger(), "This is not a type of cone!");
    rclcpp::shutdown();
  }
  return marker;
}

/**
 * Reads a CSV file and stores the (x,y) position of the cone "categoryName" in
 * data
 * @param filename Name of the file/track from which the cone position will be
 * loaded
 * @param categoryName Category of the cone of which position will be read
 * 'blue'
 * @param data Vector containing the (x,y) position of the cones from
 * categoryName
 * @return nothing
 */
void TracksVisualizer::loadCategoryFromCSV(const std::string &filename, const std::string &categoryName,
                                           std::vector<Vec2d> &data) {
  // Open the CSV file
  std::ifstream file(filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "FILE NOT OPEN");
    rclcpp::shutdown();
  }
  std::string line;

  // Read each line of the file
  while (getline(file, line)) {
    // Create a string stream from the line
    std::stringstream line_stream(line);

    // Read the tag, x, and y values from the line
    std::string tag;
    // Stores the tag
    getline(line_stream, tag, ',');

    // If the tag is "blue", store the x,y value in the vector data
    if (tag == categoryName) {
      std::string x_str, y_str;
      getline(line_stream, x_str, ',');
      getline(line_stream, y_str, ',');
      double x = std::stod(x_str);
      double y = std::stod(y_str);
      // ROS_INFO("%f", y);
      tracks_visualization::Vec2d temp;
      temp.x = x;
      temp.y = y;
      data.push_back(temp);
    }
  }
}

/*
    Uses loadCategoryFrom() to load each position of each type of cones into the
   respective buffers for YAML file filetype is the type of the file: CSV or
   YAML
*/
void TracksVisualizer::loadCones() {
  std::string trackPath = loadTrackPath();

  // call loadCategoryFrom with every buffer
  loadCategoryFromCSV(trackPath, "blue", cones_blue_buffer_);
  loadCategoryFromCSV(trackPath, "yellow", cones_yellow_buffer_);
  loadCategoryFromCSV(trackPath, "orange", cones_orange_buffer_);
  loadCategoryFromCSV(trackPath, "orange_big", cones_orange_big_buffer_);
  loadCategoryFromCSV(trackPath, "none", cones_none_buffer_);
}

std::string TracksVisualizer::loadTrackPath() {
  std::string trackPath;
  if (!this->get_parameter("track_path", trackPath)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get track_path parameter");
  } else {
    RCLCPP_INFO(this->get_logger(), "Loaded track from path: %s", trackPath.c_str());
  }

  return trackPath;
}

/*
    Publishes the markers (i.e. cones) into RViz
*/
void TracksVisualizer::publishMarkers() {
  // Temporary variables to store position of the cones
  double x, y;

  // Fill the array_marker_ with all the blue cones markers
  for (size_t i = 0; i < cones_blue_buffer_.size(); ++i) {
    x = cones_blue_buffer_[i].x;
    y = cones_blue_buffer_[i].y;
    array_marker_.push_back(createCone(i, x, y, "cone_blue"));
  }
  // Fill the array_marker_ with all the yellow cones markers
  for (size_t i = 0; i < cones_yellow_buffer_.size(); ++i) {
    x = cones_yellow_buffer_[i].x;
    y = cones_yellow_buffer_[i].y;
    array_marker_.push_back(createCone(i + cones_blue_buffer_.size(), x, y,
                                       "cone_yellow")); // id = i+cones_blue_buffer.size() since
                                                        // each id must be different
  }

  // Fill the array_marker_ with all the orange cones markers
  for (size_t i = 0; i < cones_orange_buffer_.size(); ++i) {
    x = cones_orange_buffer_[i].x;
    y = cones_orange_buffer_[i].y;
    array_marker_.push_back(createCone(i + cones_blue_buffer_.size() + cones_yellow_buffer_.size(), x, y,
                                       "cone_orange")); // id = i+... since each id must be different
  }

  // Fill the array_marker_ with all the big orange cones markers
  for (size_t i = 0; i < cones_orange_big_buffer_.size(); ++i) {
    x = cones_orange_big_buffer_[i].x;
    y = cones_orange_big_buffer_[i].y;
    array_marker_.push_back(
        createCone(i + cones_blue_buffer_.size() + cones_yellow_buffer_.size() + cones_orange_buffer_.size(), x, y,
                   "cone_orange_big")); // id = i+... since each id must be different
  }

  // Fill the array_marker_ with all the big orange cones markers
  for (size_t i = 0; i < cones_none_buffer_.size(); ++i) {
    x = cones_none_buffer_[i].x;
    y = cones_none_buffer_[i].y;
    array_marker_.push_back(createCone(i + cones_blue_buffer_.size() + cones_yellow_buffer_.size() +
                                           cones_orange_buffer_.size() + cones_orange_big_buffer_.size(),
                                       x, y,
                                       "cone_none")); // id = i+... since each id must be different
  }

  // Define a rate for controlling the loop frequency
  rclcpp::Rate rate(0.01); // 10 Hz, adjust as needed
  rclcpp::Rate rate_sleep(1);

  // Main loop for publishing markers
  while (rclcpp::ok()) {
    // Check if Rviz is subscribed to the topic
    if (marker_pub_->get_subscription_count() < 1) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Please create a subscriber to the marker");
      rate_sleep.sleep();
      // std::this_thread::sleep_for(1s);
      continue;
    }

    // Publish the markers
    for (auto &marker : array_marker_) {
      marker_pub_->publish(marker);
    }

    // Spin and sleep
    rclcpp::spin(shared_from_this());
    rate.sleep();
  }
}
} // namespace tracks_visualization
