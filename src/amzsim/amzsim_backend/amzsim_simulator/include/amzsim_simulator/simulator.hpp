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

#include "amzsim_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

const float DEGREE = M_PI / 180.0;

using std::placeholders::_1;
// using namespace std;

class Simulator {
public:
  explicit Simulator(rclcpp::Node::SharedPtr nh);
  virtual ~Simulator() = default;
  void publish_odom();
  void state_callback(const amzsim_msgs::msg::State::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;

  // to convert yaw angle to quaternion
  geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw) const;

  // subscriber to retreive current state information
  rclcpp::Subscription<amzsim_msgs::msg::State>::SharedPtr subscription_;

  // to broadcast tf2 frames to rviz2
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

  // store messages before publishing them
  geometry_msgs::msg::TransformStamped odom_trans;
  nav_msgs::msg::Odometry odom;

  // create a publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

  rclcpp::Clock clock;

  // State of the car
  // Positions in inertial frame
  double x;
  double y;
  double yaw;
  // Velocities in vehicle frame
  double vx;
  double vy;
  double dyaw;
};
