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

#include "amzsim_estimation/tf_ego_vehicle.hpp"

namespace tf_ego_vehicle {

// Constructor: Initialize the subscriber
Listener_Publisher::Listener_Publisher(rclcpp::Node::SharedPtr nh)
    : node_(nh), broadcaster_(node_), tfBuffer_(node_->get_clock()), tfListener_(tfBuffer_) {
  sub = node_->create_subscription<amzsim_msgs::msg::State>("/amzsim/car/state", 5,
                                                            bind(&Listener_Publisher::car_stateCallback, this, _1));
  cone_array_sub_ = node_->create_subscription<autonomous_msgs::msg::ConeArray>(
      "/perception/cone_array", 1, bind(&Listener_Publisher::PerceptionConeCallback, this, _1));

  cone_array_pub_est = node_->create_publisher<autonomous_msgs::msg::ConeArray>("/estimation/online_map", 1);

  SendInitialTransform();
}

// Callback for state of the car. Updates the attributes of the class containing
// the state of the car when the message is received
void Listener_Publisher::car_stateCallback(const amzsim_msgs::msg::State::SharedPtr msg) {
  x = msg->x;
  y = msg->y;
  yaw = msg->yaw;
  t = msg->header.stamp;
}

// Publish map-odom transform
void Listener_Publisher::SendInitialTransform() {
  geometry_msgs::msg::TransformStamped map_odom_transform;
  // Set map->odom transform to the identity first such that controller gets
  // information (otherwise it won t work)
  map_odom_transform.header.stamp = node_->get_clock()->now();
  map_odom_transform.header.frame_id = "map";
  map_odom_transform.child_frame_id = "odom";
  map_odom_transform.transform.translation.x = 0;
  map_odom_transform.transform.translation.y = 0;
  map_odom_transform.transform.translation.z = 0;

  map_odom_transform.transform.rotation.x = 0;
  map_odom_transform.transform.rotation.y = 0;
  map_odom_transform.transform.rotation.z = 0;
  map_odom_transform.transform.rotation.w = 1;
  broadcaster_.sendTransform(map_odom_transform);
}

// Publish map-odom transform
void Listener_Publisher::PerceptionConeCallback(const autonomous_msgs::msg::ConeArray cones_msg) {
  geometry_msgs::msg::TransformStamped map_odom_transform;

  map_odom_transform.header.stamp = cones_msg.header.stamp;
  map_odom_transform.header.frame_id = "map";
  map_odom_transform.child_frame_id = "odom";
  map_odom_transform.transform.translation.x = 0;
  map_odom_transform.transform.translation.y = 0;
  map_odom_transform.transform.translation.z = 0;

  map_odom_transform.transform.rotation.x = 0;
  map_odom_transform.transform.rotation.y = 0;
  map_odom_transform.transform.rotation.z = 0;
  map_odom_transform.transform.rotation.w = 1;
  broadcaster_.sendTransform(map_odom_transform);

  cone_array_pub_est->publish(cones_msg);
}

} // namespace tf_ego_vehicle
