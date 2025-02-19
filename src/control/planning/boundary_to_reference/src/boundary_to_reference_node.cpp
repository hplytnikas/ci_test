/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Diego Garcia Soto <digarcia@ethz.ch>
 *   - Hironobu Akiyama <hakiyama@ethz.ch>
 *   - Jonas Ohnemus <johnemus@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "autonomous_msgs/msg/boundary.hpp"
#include "autonomous_msgs/msg/point_with_confidence.hpp"
#include "control_msgs/msg/controller_ref.hpp"
#include "control_msgs/msg/reference_state.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

class BoundaryToReferenceNode : public rclcpp::Node {
public:
  BoundaryToReferenceNode() : Node("boundary_to_reference_node") {
    this->declare_parameter<double>("vx_ref", 5.0);
    vx_ref_ = this->get_parameter("vx_ref").as_double();
    this->declare_parameter<std::string>("bounded_path_topic", "/estimation/bounded_path");
    bounded_path_topic_ = this->get_parameter("bounded_path_topic").as_string();
    this->declare_parameter<std::string>("reference_topic", "/planning/reference");
    reference_topic_ = this->get_parameter("reference_topic").as_string();

    subscription_ = this->create_subscription<autonomous_msgs::msg::Boundary>(
        bounded_path_topic_, 1, std::bind(&BoundaryToReferenceNode::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<control_msgs::msg::ControllerRef>(reference_topic_, 1);
  }

private:
  void topic_callback(const autonomous_msgs::msg::Boundary::SharedPtr msg) {
    control_msgs::msg::ControllerRef reference_msg;
    reference_msg.header = msg->header;

    for (const auto &point : msg->middle_line) {
      control_msgs::msg::ReferenceState ref_state;
      ref_state.position = point.position;
      ref_state.boundary_left = 1.5;
      ref_state.boundary_right = 1.5;
      ref_state.vx_ref = vx_ref_;
      reference_msg.reference_trajectory.push_back(ref_state);
    }

    publisher_->publish(reference_msg);
  }

  rclcpp::Subscription<autonomous_msgs::msg::Boundary>::SharedPtr subscription_;
  rclcpp::Publisher<control_msgs::msg::ControllerRef>::SharedPtr publisher_;
  double vx_ref_;
  std::string bounded_path_topic_;
  std::string reference_topic_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoundaryToReferenceNode>());
  rclcpp::shutdown();
  return 0;
}
