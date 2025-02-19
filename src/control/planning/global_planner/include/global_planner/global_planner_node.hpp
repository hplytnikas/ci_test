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
#include "autonomous_msgs/msg/cone_array.hpp"
#include "autonomous_msgs/msg/point_with_confidence.hpp"
#include "control_msgs/msg/controller_ref.hpp"
#include "control_msgs/msg/planning_log.hpp"
#include "control_msgs/msg/reference_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "vcu_msgs/msg/velocity_estimation.hpp"
#include <nav_msgs/msg/path.hpp>

#include "tf2/exceptions.h"
#include "tf2/utils.h" //tf2::doTransform
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "min_curvature_opt/min_curvature_opt.hpp"

// Parameter handler
#include "global_planner_params.hpp"

#include <memory>
#include <optional>
#include <string>

/*******************************************************************************
 * class GlobalPlannerNode                                                     *
 ******************************************************************************/
/**
 * @brief Global planner node
 *
 * This class is the main node for the Global planner. It subscribes to the boundary message:
 * - path_topic: /estimation/final_path
 *
 * It publishes the following messages:
 * - path_topic: /planning/global_reference
 * - path_viz_topic: /planning/viz/global_reference
 * - flag_topic: /planning/global_flag
 */
class GlobalPlannerNode : public rclcpp::Node {
  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PUBLIC
   *______________________________________________*/
public:
  /**
   * @brief Construct a new Global Planner Node object
   */
  explicit GlobalPlannerNode(std::string node_name);

  /**
   * @brief Destroy the Global Planner Node object
   */
  ~GlobalPlannerNode() = default;

  /**
   * @brief Exception for the Global planner
   */
  class GlobalPlannerStepException : public std::runtime_error {
  public:
    explicit GlobalPlannerStepException(const std::string &what_arg) : std::runtime_error(what_arg) {}
  };

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
   * @brief Subscriber for the boundary message
   */
  rclcpp::Subscription<autonomous_msgs::msg::ConeArray>::SharedPtr estimation_subscriber_;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PRIVATE : PUBLISHERS
   *______________________________________________*/
  /**
   * @brief Publisher for the Global path
   */
  rclcpp::Publisher<control_msgs::msg::ControllerRef>::SharedPtr global_reference_publisher_;

  /**
   * @brief Publisher for the visualisation of the Global path
   */
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_reference_viz_publisher_;

  /**
   * @brief Publisher of a flag to indicate that the global reference has been received
   */
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr global_flag_publisher_;

  /**
   * @brief Publisher for the log of the global planner
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
   * @brief Timer callback function
   */
  void TimerCallback();

  /**
   * @brief Callback function for the SLAM message to make sure the latest transform is available
   *
   * @param msg Only used to get the timestamp
   */
  void EstimationCallback(const autonomous_msgs::msg::ConeArray &msg);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PRIVATE : MEMBERS
   *______________________________________________*/
  /**
   * @brief Sampling resolution for the global map
   */
  int sampling_;
  /**
   * @brief Timer for the Global planner
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Buffer for the tf2
   */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief Listener for the tf2
   */
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  /**
   * @brief Message to store the reference for the controllers
   */
  control_msgs::msg::ControllerRef reference_;

  /**
   * @brief Latest reference message which contains the full map
   */
  autonomous_msgs::msg::Boundary latest_reference_msg_;

  /**
   * @brief Latest velocity estimation message which contains the latest driven speed
   */
  vcu_msgs::msg::VelocityEstimation latest_velocity_msg_;

  /**
   * @brief Vector that contains the lateral deviation of the path from the reference middle line
   */
  Eigen::MatrixX2d lateral_deviation_;

  /**
   * @brief Vector that contains the curvature profile of the path
   */
  Eigen::VectorXd curvature_profile_;

  /**
   * @brief Vector that contains the driven speeds
   */
  Eigen::VectorXd driven_speeds_;

  /**
   * @brief Object for the minimum curvature optimiser
   */
  std::shared_ptr<MinCurvatureOpt> min_curvature_opt_;

  /**
   * @brief Parameters for the Global planner
   */
  std::shared_ptr<global_planner_params::ParamListener> param_listener_;

  /**
   * @brief Structure of the parameters defined by the parameter handler
   */
  global_planner_params::Params params_;

  /**
   * @brief Flag to check if global map has been received
   */
  bool global_map_received_ = false;

  /**
   * @brief Log message to fill and publish
   */
  control_msgs::msg::PlanningLog log_msg_ = control_msgs::msg::PlanningLog();

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PRIVATE : METHODS
   *______________________________________________*/
  /**
   * @brief Transform the reference from map to base_link, then cuts to certain radius and publish it
   *
   * @param msg The message used to get the timestamp if set
   */
  void PublishTransformedReference(const std::optional<autonomous_msgs::msg::ConeArray> &msg_opt);

  /**
   * @brief Compute the terminal speed target.
   */
  void GetSpeedTarget();

  /**
   * @brief Function to call when the optimisation has failed. It will publish a "false" boolean for BE and clear the
   * reference message.
   */
  void OptimisationFailed();
};
