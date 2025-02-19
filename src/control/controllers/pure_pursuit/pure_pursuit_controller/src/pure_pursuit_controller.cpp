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

#include "pure_pursuit_controller/pure_pursuit_controller.hpp"

PurePursuitController::PurePursuitController(
    std::shared_ptr<ControllerNode> controller_node,
    std::shared_ptr<PurePursuitVisualizationNode> pure_pursuit_visualization_node,
    std::shared_ptr<pure_pursuit_controller_params::ParamListener> param_listener)
    : AbstractController(controller_node), pure_pursuit_visualization_node_(pure_pursuit_visualization_node),
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

  controller_node_->LogInfo("PurePursuitController: Controller initialized");
}

vcu_msgs::msg::CarCommand PurePursuitController::updateControlCommand() {
  EASY_FUNCTION(profiler::colors::Magenta);
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  const ControllerNode::State state = controller_node_->GetState();
  const ControllerNode::Reference &reference = controller_node_->GetReference();
  if (reference.rows() < 2) {
    throw ControllerNode::ControllerStepException("Not enough points in the path, found " +
                                                  std::to_string(reference.rows()));
  }
  pure_pursuit_visualization_node_->shareReference(reference);

  const double vel = std::hypot(state.dx, state.dy);

  const double ax = updateLongitudinalControlCommand(reference, vel);
  const double steering_angle = updateLateralControlCommand(reference, vel);
  const double yaw_rate = updateYawRate(steering_angle / params_.steering_multiplier, vel, params_.l_f + params_.l_r);
  if (std::isnan(ax)) [[unlikely]] {
    throw ControllerNode::ControllerStepException("Longitudinal control output is nan");
  }
  if (std::isnan(steering_angle)) [[unlikely]] {
    throw ControllerNode::ControllerStepException("Steering angle output is nan");
  }
  // This is done in a very hacky way, but it is a quick fix. The controller should not be aware of the mission
  if (params_.mission == "acceleration" && !pure_pursuit_visualization_node_->getAccelMapCreated()) {
    throw ControllerNode::ControllerStepException("Accel map not created");
  }
  vcu_msgs::msg::CarCommand car_command;
  car_command.a_x[0] = car_command.a_x[1] = car_command.a_x[2] = ax;
  car_command.steering_angle[0] = car_command.steering_angle[1] = car_command.steering_angle[2] = steering_angle;
  car_command.yaw_rate[0] = car_command.yaw_rate[1] = car_command.yaw_rate[2] = yaw_rate;
  return car_command;
}

double PurePursuitController::updateYawRate(double steering_angle, double vel, double wheelbase) {
  EASY_FUNCTION(profiler::colors::Green);
  // radius = wheelbase / tan(steering_angle)
  // yaw_rate = dx / radius
  return (vel * std::tan(steering_angle)) / wheelbase;
}

double PurePursuitController::updateLongitudinalControlCommand(const ControllerNode::Reference &reference, double vel) {
  EASY_FUNCTION(profiler::colors::Cyan)
  // Get the closest point to the vehicle
  // It uses the lambda function to compare the points
  const auto closest_point = std::min_element(reference.cbegin(), reference.cend(), [](const auto &p1, const auto &p2) {
    return std::hypot(p1.x, p1.y) < std::hypot(p2.x, p2.y);
  });

  // Find the lookahead point
  const ControllerNode::ReferenceState lookahead_point =
      findLookaheadPoint(closest_point, reference.cbegin(), reference.cend(), params_.velocity_lookahead_distance);

  return longitudinalPidIteration(vel, lookahead_point.velocity);
}

double PurePursuitController::longitudinalPidIteration(double vel, double reference_velocity) {
  EASY_FUNCTION(profiler::colors::Cyan);
  const double error = reference_velocity - vel;
  const double derivative = (error - pid_previous_error_) / dt_;
  const double output = params_.kp_longitudinal * error + params_.kd_longitudinal * derivative;
  pid_previous_error_ = error;
  const double output_clamped =
      std::min(std::max(output, -params_.max_acceleration_output), params_.max_acceleration_output);

  // Visualize the PID state
  control_msgs::msg::PidState pid_state;
  pid_state.reference = reference_velocity;
  pid_state.measured = vel;
  pid_state.error = error;
  pid_state.derivate_error = derivative;
  pid_state.integral_error = 0;
  pid_state.output = output_clamped;
  pid_state.p_part = params_.kp_longitudinal * error;
  pid_state.i_part = 0;
  pid_state.d_part = params_.kd_longitudinal * derivative;
  pure_pursuit_visualization_node_->shareLongitudinalPidState(pid_state);
  return output_clamped;
}

double PurePursuitController::lateralPidIteration(double lateral_deviation) {
  EASY_FUNCTION(profiler::colors::Cyan);
  const double error = lateral_deviation; // reference is always 0
  const double derivative = (error - pid_lateral_previous_error_) / dt_;
  const double output = params_.kp_lateral * error;
  const double output_clamped = std::min(std::max(output, -params_.max_steering_pid), params_.max_steering_pid);

  // Visualize the PID state
  control_msgs::msg::PidState pid_state;
  pid_state.reference = 0;
  pid_state.measured = lateral_deviation;
  pid_state.error = error;
  pid_state.derivate_error = derivative;
  pid_state.integral_error = 0;
  pid_state.output = output_clamped;
  pid_state.p_part = params_.kp_longitudinal * error;
  pid_state.i_part = 0;
  pid_state.d_part = 0;
  pure_pursuit_visualization_node_->shareLateralPidState(pid_state);
  return output_clamped;
}

double PurePursuitController::updateLateralControlCommand(const ControllerNode::Reference &reference, double vel) {
  EASY_FUNCTION(profiler::colors::Cyan)
  // Get the closest point to the vehicle
  // It uses the lambda function to compare the points
  const auto closest_point = std::min_element(reference.cbegin(), reference.cend(), [](const auto &p1, const auto &p2) {
    return std::hypot(p1.x, p1.y) < std::hypot(p2.x, p2.y);
  });
  // NOTE: this assumes there are many points in the path, so it works well for skidpad and accel
  // If the points are too sparse, this mesurement will be wrong
  const double lateral_deviation = std::copysign(std::hypot(closest_point->x, closest_point->y), closest_point->y);
  pure_pursuit_visualization_node_->shareLateralDeviation(lateral_deviation);

  // Calculate lookahead distance
  const double lookahead_distance = std::clamp(params_.lookahead_distance_factor * vel, params_.lookahead_distance_min,
                                               params_.lookahead_distance_max);
  controller_node_->LogInfo("Lookahead distance is %f\n", lookahead_distance);

  // Find the lookahead point
  const ControllerNode::ReferenceState lookahead_point =
      findLookaheadPoint(closest_point, reference.cbegin(), reference.cend(), lookahead_distance);
  pure_pursuit_visualization_node_->shareLookaheadPoint(lookahead_point);

  // radius = (x^2 + y^2) / (2 * y) = lookahead_distance^2 / 2 * y
  // steering_angle = atan(wheelbase / radius)
  // Actuall x^2 + y^2 = lookahead_distance^2 if we calculated the point
  // correctly I do it this way because if we skip the interpolation we could
  // use the outside point as an approximation of the lookahead point Added wb/2
  // as x has to be from the rear axle
  // TODO(diego): we are assuming base_link is in the middle of front and rear
  // axles
  const double x = lookahead_point.x + params_.l_r;
  const double y = lookahead_point.y;
  const double steering_angle =
      params_.steering_multiplier * std::atan2((params_.l_f + params_.l_r) * 2 * y, (x * x + y * y));

  const double steering_angle_pid = lateralPidIteration(lateral_deviation);

  if (steering_angle + steering_angle_pid < -params_.max_steering ||
      steering_angle + steering_angle_pid > params_.max_steering) {
    controller_node_->LogWarn("Steering angle is out of bounds: %f\n", steering_angle);
  }
  return std::clamp(steering_angle + steering_angle_pid, -params_.max_steering, params_.max_steering);
}

ControllerNode::ReferenceState PurePursuitController::findLookaheadPoint(const PathIterator closest_point,
                                                                         const PathIterator begin,
                                                                         const PathIterator end,
                                                                         const double distance) const {
  EASY_FUNCTION(profiler::colors::Yellow);
  // Get the first point outside the lookahead distance
  const PathIterator outside_point = getLookaheadPointIterator(closest_point, begin, end, distance);

  // TODO(digarcia) this has been removed, as in no situation we will handle looped paths. Remove if we see this is
  // still not necesary in the future

  // const PathIterator inside_point = std::prev(outside_point);
  const PathIterator inside_point = outside_point == closest_point ? closest_point : std::prev(outside_point);

  // Interpolate between the outside and inside points to get a point exectly at
  // the lookahead distance
  const ControllerNode::ReferenceState lookahead_point =
      interpolatePointsForDistance(inside_point, outside_point, distance);

  return lookahead_point;
}

ControllerNode::ReferenceState PurePursuitController::interpolatePointsForDistance(const PathIterator &p1,
                                                                                   const PathIterator &p2,
                                                                                   const double distance) const {
  EASY_FUNCTION(profiler::colors::Yellow);
  // p1 is the inside point and p2 is the outside point
  // p1_x ^2 + p1_y^2 < distance^2 < p2_x^2 + p2_y^2

  // The point we are looking for is p1 + t * (p2 - p1) such that:
  // (p1_x + t * dx)^2 + (p2_y + t * dy)^2 = lookahead_distance_^2
  // -->
  // t^2 * (dx^2 + dy^2) + 2 * t * (p1_x * dx + p1_y * dy) +
  // (p1_x^2 + p1_y^2 - lookahead_distance_^2) = 0
  const double dx = p2->x - p1->x;
  const double dy = p2->y - p1->y;
  const double a = dx * dx + dy * dy;
  if (a == 0.0) {
    // If the points are the same, return the point
    return *p1;
  }
  const double b = 2 * (p1->x * dx + p1->y * dy);
  const double c = p1->x * p1->x + p1->y * p1->y - distance * distance;
  const double discriminant = b * b - 4 * a * c;
  const double t = (-b + std::sqrt(discriminant)) / (2 * a); // t is always positive
  if (std::isnan(t)) {
    controller_node_->LogWarn("t is nan, returning p1");
    return *p1;
  }
  ControllerNode::ReferenceState lookahead_point;
  lookahead_point.x = p1->x + t * dx;
  lookahead_point.y = p1->y + t * dy;
  // Velocity is also interpolated
  lookahead_point.velocity = p1->velocity + t * (p2->velocity - p1->velocity);
  return lookahead_point;
}

// Iterates through the path until it finds a point outside the lookahead
// distance Starts at the closest point and loops back if it reaches the end of
// the path
PurePursuitController::PathIterator PurePursuitController::getLookaheadPointIterator(const PathIterator closest_point,
                                                                                     const PathIterator begin,
                                                                                     const PathIterator end,
                                                                                     const double distance) const {
  EASY_FUNCTION(profiler::colors::Yellow);
  PathIterator it = closest_point;
  while (it != end) {
    if (std::hypot(it->x, it->y) >= distance) {
      return it;
    }
    it++;
    // TODO(digarcia): remove if we don't need looped paths in the future
    // if (it == end) {
    //   it = begin;
    // }
  }
  throw ControllerNode::ControllerStepException("No lookahead point found");
}
