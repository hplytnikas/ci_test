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

#include "abstract_controller/abstract_controller.hpp"
#include "controller_node/controller_node.hpp"
#include "inspection_controller_params.hpp"
#include "inspection_visualization_node/inspection_visualization_node.hpp"
#include "vcu_msgs/msg/car_command.hpp"
#include <cmath>
#include <memory>

class InspectionController : public AbstractController {
public:
  InspectionController(std::shared_ptr<ControllerNode> controller_node,
                       std::shared_ptr<InspectionVisualizationNode> inspection_visualization_node,
                       std::shared_ptr<inspection_controller_params::ParamListener> param_listener);
  ~InspectionController() override;

  vcu_msgs::msg::CarCommand updateControlCommand() override;
  void resetController() override;

private:
  std::shared_ptr<InspectionVisualizationNode> inspection_visualization_node_;
  std::shared_ptr<inspection_controller_params::ParamListener> param_listener_;
  inspection_controller_params::Params params_;

  double time_;
  double dt_;
};
