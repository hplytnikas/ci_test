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

#include "acceleration_path_planner/acceleration_path_planner_node.hpp"

AccelerationPathPlanner::AccelerationPathPlanner(std::string node_name) : Node(node_name) {
  // Get parameters
  param_listener_ =
      std::make_shared<acceleration_path_planner_params::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // Publishers
  reference_pub_ = this->create_publisher<control_msgs::msg::ControllerRef>(params_.reference_topic, 1);

  // Timers
  reference_timer_ = this->create_wall_timer(std::chrono::duration<double>(1 / params_.reference_publish_frequency),
                                             std::bind(&AccelerationPathPlanner::publishReference, this));
}

AccelerationPathPlanner::~AccelerationPathPlanner() {}

void AccelerationPathPlanner::publishReference() {
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }
  control_msgs::msg::ControllerRef ref_msg;
  ref_msg.header.stamp = this->now(); // It should not be now
  ref_msg.header.frame_id = "map";
  // Create a straight center line
  for (int i = 0; i < params_.num_points; i++) {
    control_msgs::msg::ReferenceState state;
    state.position.x = i * params_.reference_resolution;
    state.position.y = 0;
    state.boundary_left = params_.lane_width / 2;
    state.boundary_right = params_.lane_width / 2;
    state.vx_ref = params_.target_velocity;
    ref_msg.reference_trajectory.push_back(state);
  }
  reference_pub_->publish(ref_msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<AccelerationPathPlanner> acceleration_path_planner =
      std::make_shared<AccelerationPathPlanner>("acceleration_path_planner_node");
  rclcpp::spin(acceleration_path_planner);
  rclcpp::shutdown();
  return 0;
}
