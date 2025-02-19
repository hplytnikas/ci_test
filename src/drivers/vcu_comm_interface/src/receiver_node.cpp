/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Stanislaw Piasecki <stanislaw.piasecki@inf.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include <rclcpp/rclcpp.hpp>

#include "vcu_comm_interface/receiver.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vcu_comm_interface::Receiver>());
  rclcpp::shutdown();
  return 0;
}
