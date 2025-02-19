/*******************************************************************************
 * AMZ Driverless Project                                                      *
 * Copyright (c) 2023-2024                                                     *
 * Authors:                                                                    *
 *   - Diego Garcia Soto <digarcia@ethz.ch>                                    *
 *   - Hironobu Akiyama <hakiyama@ethz.ch>                                     *
 *   - Jonas Ohnemus <johnemus@ethz.ch>                                        *
 *                                                                             *
 * All rights reserved.                                                        *
 *                                                                             *
 * Unauthorized copying of this file, via any medium, is strictly prohibited.  *
 * Proprietary and confidential.                                               *
 ******************************************************************************/

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <easy/profiler.h>

#include "autonomous_msgs/msg/boundary.hpp"
#include "autonomous_msgs/msg/point_with_confidence.hpp"
#include "control_msgs/msg/controller_ref.hpp"
#include "control_msgs/msg/planning_log.hpp"
#include "control_msgs/msg/reference_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "vcu_msgs/msg/velocity_estimation.hpp"
#include <nav_msgs/msg/path.hpp>

#include "min_curvature_opt/min_curvature_opt.hpp"

// Parameter handler
#include "local_planner_params.hpp"

#include <memory>
#include <string>

/*******************************************************************************
 * class LocalPlannerNode                                                      *
 ******************************************************************************/
/**
 * @brief Local planner node
 *
 * This class is the main node for the local planner. It subscribes to the boundary message:
 * - path_topic: /estimation/bounded_path
 *
 * It publishes the following messages:
 * - path_topic: /planning/local_reference
 * - path_viz_topic: /planning/viz/local_reference
 */
class LocalPlannerNode : public rclcpp::Node {
  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PUBLIC
   *______________________________________________*/
public:
  /**
   * @brief Construct a new Local Planner Node object
   */
  explicit LocalPlannerNode(std::string node_name);

  /**
   * @brief Destroy the Local Planner Node object
   */
  ~LocalPlannerNode() = default;

private:
  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PRIVATE : SUBSCRIBERS
   *______________________________________________*/
  /**
   * @brief Subscriber for the boundary message
   */
  rclcpp::Subscription<autonomous_msgs::msg::Boundary>::SharedPtr path_subscriber_;

  /**
   * @brief Subscriber for the velocity estimation message
   */
  rclcpp::Subscription<vcu_msgs::msg::VelocityEstimation>::SharedPtr velocity_estimation_subscriber_;

  /**
   * @brief Subscriber for the travelled distance message
   */
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr travelled_distance_subscriber_;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PRIVATE : PUBLISHERS
   *______________________________________________*/
  /**
   * @brief Publisher for the local path
   */
  rclcpp::Publisher<control_msgs::msg::ControllerRef>::SharedPtr local_reference_publisher_;

  /**
   * @brief Publisher for the visualisation of the local path
   */
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_reference_viz_publisher_;

  /**
   * @brief Publisher for the log of the local path
   */
  rclcpp::Publisher<control_msgs::msg::PlanningLog>::SharedPtr log_publisher_;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PRIVATE : CALLBACKS
   *______________________________________________*/
  /**
   * @brief Callback function for the boundary message
   *
   * @param msg The boundary message
   */
  void PathCallback(const autonomous_msgs::msg::Boundary::SharedPtr msg);

  /**
   * @brief Callback function for the velocity estimation message
   *
   * @param msg The velocity estimation message
   */
  void VelocityEstimationCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg);

  /**
   * @brief Callback function for the travelled distance message
   *
   * @param msg The velocity estimation message
   */
  void TravelledDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PRIVATE : MEMBERS
   *______________________________________________*/
  /**
   * @brief Object for the minimum curvature optimiser
   */
  std::shared_ptr<MinCurvatureOpt> min_curvature_opt_;

  /**
   * @brief Parameters for the local planner
   */
  std::shared_ptr<local_planner_params::ParamListener> param_listener_;

  /**
   * @brief Structure of the parameters defined by the parameter handler
   */
  local_planner_params::Params params_;

  /**
   * @brief Current velocity of the car, computed from longitudinal and lateral velocities.
   */
  double current_velocity_;

  /**
   * @brief Travelled distance.
   */
  double travelled_distance_;

  /**
   * @brief Straight line distance.
   */
  double straight_line_distance_;

  /**
   * @brief Last travelled distance.
   */
  double last_travelled_distance_;

  /**
   * @brief Log message to fill and publish
   */
  control_msgs::msg::PlanningLog log_msg_ = control_msgs::msg::PlanningLog();

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PRIVATE : METHODS
   *______________________________________________*/
  /**
   * @brief Compute the terminal speed target.
   */
  Eigen::VectorXd GetSpeedTarget(const Eigen::VectorXd &curvature_profile,
                                 const control_msgs::msg::ControllerRef &reference);
};
