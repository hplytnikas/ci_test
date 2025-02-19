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

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ControlPublisher>());
  rclcpp::shutdown();
  return 0;
}
