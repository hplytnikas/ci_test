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

#include "inspection_controller/inspection_controller.hpp"

InspectionController::InspectionController(std::shared_ptr<ControllerNode> controller_node,
                                           std::shared_ptr<InspectionVisualizationNode> inspection_visualization_node,
                                           std::shared_ptr<inspection_controller_params::ParamListener> param_listener)
    : AbstractController(controller_node), inspection_visualization_node_(inspection_visualization_node),
      param_listener_(param_listener) {
  resetController();
  params_ = param_listener_->get_params();
  if (controller_node_->GetControllerFrequency() <= 0) {
    throw std::runtime_error("Controller frequency must be positive");
  }
  dt_ = 1.0 / controller_node_->GetControllerFrequency();
  if (std::isnan(dt_) || std::isinf(dt_) || dt_ <= 0) {
    throw std::runtime_error("Cannot have a NaN, Inf or negative dt");
  }
}

InspectionController::~InspectionController() {}

vcu_msgs::msg::CarCommand InspectionController::updateControlCommand() {
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  vcu_msgs::msg::CarCommand command;
  if (time_ > params_.duration) {
    inspection_visualization_node_->publishMissionFinished(true);
    command.a_x[0] = command.a_x[1] = command.a_x[2] = 0;
    command.steering_angle[0] = command.steering_angle[1] = command.steering_angle[2] = 0;
    return command;
  }
  // inspection_visualization_node_->publishMissionFinished(false);

  const double ax = params_.ax;
  const double steering = params_.steering_amplitude * std::sin(2 * M_PI * params_.steering_frequency * time_);

  command.a_x[0] = command.a_x[1] = command.a_x[2] = ax;
  command.steering_angle[0] = command.steering_angle[1] = command.steering_angle[2] = steering;

  time_ += dt_;
  return command;
}

void InspectionController::resetController() { time_ = 0.0; }
