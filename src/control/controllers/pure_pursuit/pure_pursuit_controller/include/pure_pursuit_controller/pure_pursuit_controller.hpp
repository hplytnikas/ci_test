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
#include "pure_pursuit_visualization_node/pure_pursuit_visualization_node.hpp"

#include <algorithm>
#include <cmath>
#include <easy/profiler.h>
#include <iostream>
#include <memory>
#include <vector>

#include <control_msgs/msg/pid_state.hpp>
// #include <geometry_msgs/msg/point32.hpp>
#include <vcu_msgs/msg/car_command.hpp>

#include <pure_pursuit_controller_params.hpp>

class PurePursuitController : public AbstractController {
public:
  ~PurePursuitController() {}
  PurePursuitController(std::shared_ptr<ControllerNode> controller_node,
                        std::shared_ptr<PurePursuitVisualizationNode> pure_pursuit_visualization_node,
                        std::shared_ptr<pure_pursuit_controller_params::ParamListener> param_listener);

  // Implementation of updateControlCommand method
  vcu_msgs::msg::CarCommand updateControlCommand() override;

  void resetController() noexcept override { pid_previous_error_ = 0; }

  // typedef std::vector<autonomous_msgs::msg::PointWithConfidence>::const_iterator PathIterator;
  typedef ControllerNode::Reference::const_iterator PathIterator;

private:
  // Owns the node that is used to communicate with the rest of the pipeline
  // with ROS2 for visualization
  std::shared_ptr<PurePursuitVisualizationNode> pure_pursuit_visualization_node_;

  // PARAMETERS
  std::shared_ptr<pure_pursuit_controller_params::ParamListener> param_listener_;
  pure_pursuit_controller_params::Params params_;

  // time step
  double dt_;

  // PID INTERNAL VARIABLES
  double pid_previous_error_;
  double pid_lateral_previous_error_;

  // LONGITUDINAL AND LATERAL CONTROL METHODS
  double updateLongitudinalControlCommand(const ControllerNode::Reference &reference, double dx);

  double longitudinalPidIteration(double dx, double reference_velocity);
  double lateralPidIteration(double lateral_deviation);

  double updateYawRate(double steering_angle, double dx, double wheelbase);

  double updateLateralControlCommand(const ControllerNode::Reference &reference, double dx);

  // Given the iterators for the path and closest point, it returns the
  // lookahead point
  ControllerNode::ReferenceState findLookaheadPoint(const PathIterator closest_point, const PathIterator begin,
                                                    const PathIterator end, const double distance) const;

  // Iterates through the path until it finds a point outside the lookahead
  // distance Starts at the closest point and loops back if it reaches the end
  // of the path
  PathIterator getLookaheadPointIterator(const PathIterator closest_point, const PathIterator begin,
                                         const PathIterator end, const double distance) const;

  // Pure function that returns with the point just before the lookahead
  // distance and the point just after the lookahead distance interpolating them
  // it returns the point exactly at the lookahead distance
  ControllerNode::ReferenceState interpolatePointsForDistance(const PathIterator &p1, const PathIterator &p2,
                                                              const double distance) const;
};
