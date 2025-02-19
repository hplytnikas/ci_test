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

#include <abstract_controller/abstract_controller.hpp>

AbstractController::AbstractController(std::shared_ptr<ControllerNode> controller_node)
    : controller_node_(controller_node) {
  controller_node_->SetUpdateFunction(std::bind(&AbstractController::updateControlCommand, this));
  controller_node_->SetResetFunction(std::bind(&AbstractController::resetController, this));
}
