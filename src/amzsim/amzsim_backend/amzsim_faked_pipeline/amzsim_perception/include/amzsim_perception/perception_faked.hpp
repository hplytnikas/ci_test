/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Romir Damle <rdamle@ethz.ch>
 *   - Emil Fahrig <efahrig@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "amzsim_msgs/msg/state.hpp"
#include "autonomous_msgs/msg/cone.hpp"
#include "autonomous_msgs/msg/cone_array.hpp"

using std::ifstream;
using std::normal_distribution;
using std::string;
using std::stringstream;
using std::vector;
using std::chrono::milliseconds;

namespace perception_faked {

struct cone_struct {
  string tag;
  double x = 0, y = 0;
};

class Perception_Faker {
public:
  Perception_Faker(rclcpp::Node::SharedPtr nh, string csvfile);
  ~Perception_Faker() = default;
  void csv_to_cone_array(string csvfile);
  void publish_cones_in_fov();
  autonomous_msgs::msg::ConeArray array_of_cones_to_msg(vector<cone_struct> array_of_cones, rclcpp::Time t_look);
  void load_perception_config();
  void assign_per_params();
  void add_noise_cones(std::vector<cone_struct> &array_of_cones_in_fov);

private:
  // Nodehandler
  rclcpp::Node::SharedPtr node_;
  //  Publisher
  rclcpp::Publisher<autonomous_msgs::msg::ConeArray>::SharedPtr cone_array_pub;

  // Useful for tf
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  // Rviz FOV publisher
  // ros::Publisher marker_pub;

  // Array containing all the cone
  vector<cone_struct> array_of_cones;

  rclcpp::Clock::SharedPtr clock;

  // Field of view parameters
  double fov_r_;
  double fov_angle_;
  double fov_r_fus_;
  double fov_angle_fus_;
  double fov_r_cam_;
  double fov_angle_cam_;
  double fov_r_lid_;
  double fov_angle_lid_;

  // Frequency at which the node runs
  int frequency_;

  // Indicator if GT per is used
  bool perception_gt;

  // Indicator if GT est is used
  bool estimation_gt;

  // Indicator of perception pipeline
  int pipeline_id;
  std::string pipeline_id_str;

  // Noise distribution of the cone
  double mean_x_;
  double mean_y_;
  double mean_x_fus_;
  double mean_y_fus_;
  double mean_x_cam_;
  double mean_y_cam_;
  double mean_x_lid_;
  double mean_y_lid_;
  double stddev_x_;
  double stddev_y_;
  double stddev_x_fus_;
  double stddev_y_fus_;
  double stddev_x_cam_;
  double stddev_y_cam_;
  double stddev_x_lid_;
  double stddev_y_lid_;
};
}; // namespace perception_faked
