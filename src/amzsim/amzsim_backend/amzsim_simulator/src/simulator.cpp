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

#include "amzsim_simulator/simulator.hpp"

Simulator::Simulator(rclcpp::Node::SharedPtr nh) : node_(nh) {
  subscription_ = node_->create_subscription<amzsim_msgs::msg::State>("/amzsim/car/state", 10,
                                                                      std::bind(&Simulator::state_callback, this, _1));
  odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("map", 10);
  odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // Initializes the state to 0
  x = 0;
  y = 0;
  yaw = 0;
  // Velocities in vehicle frame
  vx = 0;
  vy = 0;
  dyaw = 0;

  RCLCPP_INFO(node_->get_logger(), "Simulator node launched!");
}

geometry_msgs::msg::Quaternion Simulator::euler_to_quaternion(double roll, double pitch, double yaw) const {
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  return tf2::toMsg(quat);
}

void Simulator::state_callback(const amzsim_msgs::msg::State::SharedPtr msg) {
  x = msg->x;
  y = msg->y;
  yaw = msg->yaw;

  vx = msg->vx;
  vy = msg->vy;
  dyaw = msg->dyaw;
}

void Simulator::publish_odom() {
  rclcpp::Time current_time = clock.now();
  geometry_msgs::msg::Quaternion odom_quat = euler_to_quaternion(0, 0, yaw);

  // first, we'll publish the transform over tf
  // geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "map";
  odom_trans.child_frame_id = "base_link_sim";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster->sendTransform(odom_trans);

  // next, we'll publish the odometry message over ROS
  // nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "map";

  // set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link_sim";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = dyaw;

  odom_pub->publish(odom);
}
