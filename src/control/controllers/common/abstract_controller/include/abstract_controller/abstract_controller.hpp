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

#pragma once

#include "controller_node/controller_node.hpp"
#include <memory>
#include <vcu_msgs/msg/car_command.hpp>

class AbstractController {
public:
  virtual ~AbstractController() {}
  explicit AbstractController(std::shared_ptr<ControllerNode> controller_node);

protected:
  // implemented in the specific controller class.
  // It updates the control command based on the
  // current state and the desired trajectory
  // it calls functions on the controller node to get the current state
  // and the reference
  virtual vcu_msgs::msg::CarCommand updateControlCommand() = 0;

  // eg. a PID controller would need to reset the integral term
  virtual void resetController() = 0;

  // Owns the node that is used to communicate with the rest of the pipeline
  // with ROS2
  std::shared_ptr<ControllerNode> controller_node_;
};
