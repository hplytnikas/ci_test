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

#include "amzsim_fakecontrol/control_publisher.h"

ControlPublisher::ControlPublisher() : Node("control_publisher") {
  publisher_res_state_ = this->create_publisher<amzsim_msgs::msg::ResState>("/amzsim/res_state", 10);
  publisher_car_command_ = this->create_publisher<vcu_msgs::msg::CarCommand>("/control/car_command", 10);
  publishControl();
}
void ControlPublisher::publishControl() {
  rclcpp::Rate rate(10);
  auto current_time = clock.now();

  while (rclcpp::ok()) {
    amzsim_msgs::msg::ResState res_state;
    vcu_msgs::msg::CarCommand car_command;

    // res state message
    res_state.emergency = 0;
    res_state.on_off_switch = 0;
    res_state.push_button = 0;
    res_state.communication_interrupted = 0;

    // car command message
    car_command.a_x[0] = 1;
    car_command.a_x[1] = 1;
    car_command.a_x[2] = 1;
    car_command.steering_angle[0] = 0.0;
    car_command.steering_angle[1] = 0.0;
    car_command.steering_angle[2] = 0.0;

    // publish all messages
    publisher_res_state_->publish(res_state);
    publisher_car_command_->publish(car_command);

    rate.sleep();
  }
}
