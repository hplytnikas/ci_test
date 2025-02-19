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

#include "amzsim_codegen_model/amzsim_model.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("amzsim_model");
  AmzsimModel car(node);

  rclcpp::Rate rate(car.get_frequency());
  double dt = car.get_dt();

  while (rclcpp::ok()) {
    car.update(dt);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
