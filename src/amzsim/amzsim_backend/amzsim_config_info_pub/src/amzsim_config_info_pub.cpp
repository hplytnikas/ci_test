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

#include "amzsim_config_info_pub/amzsim_config_info_pub.h"

ConfigInfoPub::ConfigInfoPub(rclcpp::Node::SharedPtr nh) : rate_(frequency), node_(nh) {
  pub_info_ = node_->create_publisher<amzsim_msgs::msg::ConfigInfo>("/amzsim/config_info", 100);
}

void ConfigInfoPub::publishConfigInfo() {
  amzsim_msgs::msg::ConfigInfo msg;

  string discipline;
  string track;
  string control_node;
  string rosbag_path;
  bool lapopt_launched_before;
  bool rosbag_record;
  bool rosbag_play;
  bool ve_gt;
  bool perception_gt;
  bool estimation_gt;
  bool grip_estimation;

  // Get parameters from launch file
  node_->get_parameter("discipline_name", discipline);
  node_->get_parameter("track_path", track);
  node_->get_parameter("control_node", control_node);
  node_->get_parameter("lapopt_launched", lapopt_launched_before);
  node_->get_parameter("record_rosbag", rosbag_record);
  node_->get_parameter("play_rosbag", rosbag_play);
  node_->get_parameter("rosbag_path", rosbag_path);
  node_->get_parameter("ve_gt", ve_gt);
  node_->get_parameter("perception_gt", perception_gt);
  node_->get_parameter("estimation_gt", estimation_gt);
  node_->get_parameter("grip_estimation", grip_estimation);

  // Define message
  msg.discipline = discipline;
  msg.track = track;
  msg.lapopt_launched_before = lapopt_launched_before;
  msg.rosbag_record = rosbag_record;
  msg.rosbag_play = rosbag_play;
  msg.rosbag_path = rosbag_path;
  msg.ve_gt = ve_gt;
  msg.perception_gt = perception_gt;
  msg.estimation_gt = estimation_gt;
  msg.grip_estimation = grip_estimation;

  // Publishes message
  while (rclcpp::ok()) {
    pub_info_->publish(msg);
    rclcpp::spin_some(node_);
    rate_.sleep();
  }
}
