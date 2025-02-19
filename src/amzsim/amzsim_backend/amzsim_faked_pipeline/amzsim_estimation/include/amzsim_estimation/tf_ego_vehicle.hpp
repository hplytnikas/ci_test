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
#include "autonomous_msgs/msg/cone_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// using namespace std;

using std::placeholders::_1;

namespace tf_ego_vehicle {

// OOP Used because listener +publisher node

/*!
 * Class that listens to tf world->ego and ground truth position and publishes
 * ego->vehicle transforms
 */
class Listener_Publisher {
private:
  // Nodhandle
  rclcpp::Node::SharedPtr node_;

  // Subscriber for the state of the vehicle
  rclcpp::Subscription<amzsim_msgs::msg::State>::SharedPtr sub;
  rclcpp::Subscription<autonomous_msgs::msg::ConeArray>::SharedPtr cone_array_sub_;
  rclcpp::Publisher<autonomous_msgs::msg::ConeArray>::SharedPtr cone_array_pub_est;

  rclcpp::Clock::SharedPtr clock;

  // State of the car in vehicle frame
  double x = 0;
  double y = 0;
  double yaw = 0;
  rclcpp::Time t;

  tf2_ros::StaticTransformBroadcaster broadcaster_;
  // Buffer for the world-> ego transform
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

public:
  explicit Listener_Publisher(rclcpp::Node::SharedPtr nh);
  void SendInitialTransform();
  void car_stateCallback(const amzsim_msgs::msg::State::SharedPtr msg);
  void PerceptionConeCallback(const autonomous_msgs::msg::ConeArray cones_msg);
};

} // namespace tf_ego_vehicle
