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

#include <local_planner/local_planner_node.hpp>

/*******************************************************************************
 * Constructor                                                                 *
 ******************************************************************************/
LocalPlannerNode::LocalPlannerNode(std::string node_name) : Node(node_name) {
  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Parameters handler
   *______________________________________________*/
  param_listener_ = std::make_shared<local_planner_params::ParamListener>(get_node_parameters_interface());

  params_ = param_listener_->get_params();

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Subscribers
   *______________________________________________*/
  path_subscriber_ = this->create_subscription<autonomous_msgs::msg::Boundary>(
      params_.path_subscriber_topic, 1, std::bind(&LocalPlannerNode::PathCallback, this, std::placeholders::_1));

  velocity_estimation_subscriber_ = this->create_subscription<vcu_msgs::msg::VelocityEstimation>(
      params_.VE_subscriber_topic, 1,
      std::bind(&LocalPlannerNode::VelocityEstimationCallback, this, std::placeholders::_1));

  travelled_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      params_.travelled_distance_subscriber_topic, 1,
      std::bind(&LocalPlannerNode::TravelledDistanceCallback, this, std::placeholders::_1));

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Publishers
   *______________________________________________*/
  local_reference_publisher_ =
      this->create_publisher<control_msgs::msg::ControllerRef>(params_.local_reference_publisher_topic, 1);

  local_reference_viz_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>(params_.local_reference_viz_publisher_topic, 1);

  log_publisher_ = this->create_publisher<control_msgs::msg::PlanningLog>(params_.log_publisher_topic, 1);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Solver set up
   *______________________________________________*/
  hpipm_args args{params_.mode,       params_.iter_max,           params_.alpha_min,
                  params_.mu0,        params_.tol_stat,           params_.tol_eq,
                  params_.tol_ineq,   params_.tol_comp,           params_.reg_prim,
                  params_.reg_dual,   params_.warm_start,         params_.pred_corr,
                  params_.split_step, params_.sampling_resolution};

  int nb_iter = params_.nb_iter;
  bool first_point_fixed = params_.first_point_fixed;
  bool last_point_fixed = params_.last_point_fixed;
  bool closed = params_.closed_path;
  double max_curvature = params_.max_curvature;
  double max_distance = params_.max_distance;

  min_curvature_opt_ =
      std::make_shared<MinCurvatureOpt>(args, nb_iter, first_point_fixed, last_point_fixed, closed, max_curvature,
                                        max_distance, params_.car_width, params_.safety_margin);
}

/*******************************************************************************
 * PathCallback                                                                *
 ******************************************************************************/
void LocalPlannerNode::PathCallback(const autonomous_msgs::msg::Boundary::SharedPtr msg) {
  EASY_FUNCTION(profiler::colors::CreamWhite);

  EASY_BLOCK("Log message update parameters");

  // clear the log_msg_ member variable
  log_msg_ = control_msgs::msg::PlanningLog();
  log_msg_.header = msg->header;

  // Update parameters if they have changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  // fill the log_msg_ member variable with things that are known (const. parameters)

  log_msg_.frequency = -1.0; // frequency inherited from the BE message

  log_msg_.enable_opt = params_.enable_opt;
  log_msg_.enable_prof = params_.enable_profiler;
  log_msg_.enable_viz = params_.enable_visualisation;
  log_msg_.enable_current_vel_init = params_.enable_current_vel_init;
  log_msg_.enable_straight_line_stopper = params_.enable_straight_line_stopper;
  log_msg_.enable_safe_set_learning = false;

  log_msg_.safe_velocity_multiplier = 0.0;
  log_msg_.safe_velocity_blend = 0.0;

  log_msg_.total_safety_dist = params_.car_width / 2.0 + params_.safety_margin;
  log_msg_.nb_iter = params_.nb_iter;

  log_msg_.max_radius_for_path_from_car = 1000.0;

  log_msg_.first_point_fixed = params_.first_point_fixed;
  log_msg_.last_point_fixed = params_.last_point_fixed;

  log_msg_.sampling_resolution = params_.sampling_resolution;

  log_msg_.max_lat_acc = params_.max_lateral_acceleration;
  log_msg_.min_longit_speed = params_.min_longitudinal_speed;
  log_msg_.max_longit_speed = params_.max_longitudinal_speed;

  log_msg_.curv_smoothness = params_.curvature_smoothness;
  log_msg_.gaussian_kernel_std = params_.gaussian_kernel_std;

  log_msg_.straight_line_distance_thresh = params_.straight_line_distance_threshold;
  log_msg_.straight_line_curvature_thresh = params_.straight_line_curvature_threshold;

  log_msg_.max_curvature = params_.max_curvature;
  log_msg_.max_distance = params_.max_distance;
  log_msg_.kappa_shift = params_.kappa_shift;

  log_msg_.hpipm_mode = params_.mode;
  log_msg_.hpipm_iter_max = params_.iter_max;
  log_msg_.hpipm_alpha_min = params_.alpha_min;
  log_msg_.hpipm_mu0 = params_.mu0;
  log_msg_.hpipm_tol_stat = params_.tol_stat;
  log_msg_.hpipm_tol_eq = params_.tol_eq;
  log_msg_.hpipm_tol_ineq = params_.tol_ineq;
  log_msg_.hpipm_tol_comp = params_.tol_comp;
  log_msg_.hpipm_reg_prim = params_.reg_prim;
  log_msg_.hpipm_reg_dual = params_.reg_dual;
  log_msg_.hpipm_warm_start = params_.warm_start;
  log_msg_.hpipm_pred_corr = params_.pred_corr;
  log_msg_.hpipm_split_step = params_.split_step;

  EASY_END_BLOCK;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Save the message
   *______________________________________________*/
  autonomous_msgs::msg::Boundary latest_reference_msg = *msg;

  if (latest_reference_msg.header.frame_id != "base_link") {
    RCLCPP_WARN(this->get_logger(), "The path is not in the base_link frame! Local Planner expects it to be there.");
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Process the path if the optimisation is enabled
   *______________________________________________*/
  if (params_.enable_opt) {
    /*** Convert the path message to an Eigen::Matrix2d ***/
    int size = latest_reference_msg.middle_line.size();
    int size_left = latest_reference_msg.left_boundary.size();
    int size_right = latest_reference_msg.right_boundary.size();

    Eigen::MatrixX2d optimal_path(size, 2);
    Eigen::MatrixX2d boundaries_left(size_left, 2);
    Eigen::MatrixX2d boundaries_right(size_right, 2);

    int counter = 0;
    int counter_left = 0;
    int counter_right = 0;

    optimal_path.setZero();
    boundaries_left.setZero();
    boundaries_right.setZero();

    for (int i = 0; i < size; i++) {
      optimal_path(counter, 0) = latest_reference_msg.middle_line[i].position.x;
      log_msg_.x_points_be.push_back(latest_reference_msg.middle_line[i].position.x);

      optimal_path(counter, 1) = latest_reference_msg.middle_line[i].position.y;
      log_msg_.y_points_be.push_back(latest_reference_msg.middle_line[i].position.y);

      counter++;
    }
    for (int i = 0; i < size_left; i++) {
      boundaries_left(counter_left, 0) = latest_reference_msg.left_boundary[i].position.x;
      boundaries_left(counter_left, 1) = latest_reference_msg.left_boundary[i].position.y;

      counter_left++;
    }
    for (int i = 0; i < size_right; i++) {
      boundaries_right(counter_right, 0) = latest_reference_msg.right_boundary[i].position.x;
      boundaries_right(counter_right, 1) = latest_reference_msg.right_boundary[i].position.y;

      counter_right++;
    }
    optimal_path.conservativeResize(counter, 2);
    boundaries_left.conservativeResize(counter_left, 2);
    boundaries_right.conservativeResize(counter_right, 2);

    log_msg_.num_points_be = counter;
    log_msg_.optimization_status = 0; // Status 0: Success for now

    if (counter < 2) {
      RCLCPP_ERROR(this->get_logger(), "Not enough points in the path");

      log_msg_.optimization_status = 2; // Status 2: Not enough points in the path
      log_publisher_->publish(log_msg_);
      return;
    }

    /*** Declare zero vectors ***/
    Eigen::MatrixX2d lateral_deviation = Eigen::MatrixX2d::Zero(params_.sampling_resolution, 2);
    Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(params_.sampling_resolution);

    /*** Call the optimiser ***/
    min_curvature_opt_->GetOptimalReferencePath(optimal_path, lateral_deviation, curvature_profile, true,
                                                boundaries_left, boundaries_right);

    /*** Check the output of the solver ***/
    EASY_BLOCK("Check the output of the solver");

    if (optimal_path.hasNaN()) {
      RCLCPP_ERROR(this->get_logger(), "Optimiser failed : NaN values in the path");

      log_msg_.optimization_status = 3; //  Status 3: NaN values in the path
      log_publisher_->publish(log_msg_);
      return;
    } else if (optimal_path.isApprox(1 * Eigen::MatrixXd::Ones(params_.sampling_resolution, 2), 1e-9)) {
      RCLCPP_ERROR(this->get_logger(), "Optimiser failed : Points are not ordered correctly.");

      log_msg_.optimization_status = 4; //  Status 4: Points are not ordered correctly (or duplicated)
      log_publisher_->publish(log_msg_);
      return;
    } else if (optimal_path.isApprox(2 * Eigen::MatrixXd::Ones(params_.sampling_resolution, 2), 1e-9)) {
      RCLCPP_ERROR(this->get_logger(), "Optimiser failed : Issues while checking the curvature");

      log_msg_.optimization_status = 5; //  Status 5: Issues while checking the curvature
      log_publisher_->publish(log_msg_);
      return;
    } else if (optimal_path.isApprox(3 * Eigen::MatrixXd::Ones(params_.sampling_resolution, 2), 1e-9)) {
      RCLCPP_ERROR(this->get_logger(), "Optimiser failed : Missing a point");

      log_msg_.optimization_status = 6; //  Status 6: Missing a point
      log_publisher_->publish(log_msg_);
      return;
    } else if (optimal_path.norm() < 1e-9) {
      RCLCPP_ERROR(this->get_logger(), "Optimiser failed");

      log_msg_.optimization_status = 1; // Status 1: Optimiser failed
      log_publisher_->publish(log_msg_);
      return;
    }

    EASY_END_BLOCK;

    /*** Create a new message and insert the new coordinates ***/
    EASY_BLOCK("Create a new message and insert the new coordinates");

    control_msgs::msg::ControllerRef new_reference_msg;

    for (int i = 0; i < optimal_path.rows(); i++) {
      control_msgs::msg::ReferenceState ref_state;

      ref_state.position.x = optimal_path(i, 0);
      ref_state.position.y = optimal_path(i, 1);
      ref_state.position.z = 0;

      new_reference_msg.reference_trajectory.push_back(ref_state);
    }

    // RCLCPP_INFO(this->get_logger(), "Optimiser succeeded");

    new_reference_msg.header = latest_reference_msg.header;
    new_reference_msg.header.frame_id = "base_link";

    EASY_END_BLOCK;

    /*** Compute the speed profile ***/
    Eigen::VectorXd speed_target = GetSpeedTarget(curvature_profile, new_reference_msg);

    /*** Fill the boundaries considering a width of 3m and the velocities ***/
    EASY_BLOCK("Fill the boundaries");

    for (int i = 0; i < optimal_path.rows(); i++) {
      new_reference_msg.reference_trajectory[i].boundary_left = lateral_deviation(i, 0);
      new_reference_msg.reference_trajectory[i].boundary_right = lateral_deviation(i, 1);

      new_reference_msg.reference_trajectory[i].vx_ref = speed_target(i);
    }

    EASY_END_BLOCK;

    /*** Publish the new reference ***/
    EASY_BLOCK("Publish the new reference");

    local_reference_publisher_->publish(new_reference_msg);

    EASY_END_BLOCK;

    /*** Publish the visualisation ***/
    EASY_BLOCK("Visualisation");

    if (params_.enable_visualisation) {
      nav_msgs::msg::Path path_msg;
      path_msg.header = new_reference_msg.header;

      path_msg.poses.resize(optimal_path.rows());
      for (int i = 0; i < optimal_path.rows(); i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = new_reference_msg.header;
        pose.pose.position.x = optimal_path(i, 0);
        pose.pose.position.y = optimal_path(i, 1);
        pose.pose.position.z = 0;
        path_msg.poses[i] = pose;
      }
      local_reference_viz_publisher_->publish(path_msg);
    }

    EASY_END_BLOCK;

    /*** Publish the log message ***/
    log_publisher_->publish(log_msg_);

    return;
  } else {
    /*** Directly publish the reference ***/
    control_msgs::msg::ControllerRef new_reference_msg;

    log_msg_.optimization_status = -1; // Status -1: No optimisation
    log_msg_.num_points_be = static_cast<int>(latest_reference_msg.middle_line.size());

    for (size_t i = 0; i < latest_reference_msg.middle_line.size(); i++) {
      control_msgs::msg::ReferenceState ref_state;

      ref_state.position.x = latest_reference_msg.middle_line[i].position.x;
      log_msg_.x_points_be.push_back(latest_reference_msg.middle_line[i].position.x);

      ref_state.position.y = latest_reference_msg.middle_line[i].position.y;
      log_msg_.y_points_be.push_back(latest_reference_msg.middle_line[i].position.y);

      ref_state.position.z = 0;
      ref_state.vx_ref = 2.5;

      new_reference_msg.reference_trajectory.push_back(ref_state);
    }

    new_reference_msg.header = latest_reference_msg.header;
    new_reference_msg.header.frame_id = "base_link";

    local_reference_publisher_->publish(new_reference_msg);

    /*** Publish the visualisation ***/
    if (params_.enable_visualisation) {
      nav_msgs::msg::Path path_msg;
      path_msg.header = new_reference_msg.header;

      path_msg.poses.resize(latest_reference_msg.middle_line.size());
      for (size_t i = 0; i < latest_reference_msg.middle_line.size(); i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = new_reference_msg.header;
        pose.pose.position.x = latest_reference_msg.middle_line[i].position.x;
        pose.pose.position.y = latest_reference_msg.middle_line[i].position.y;
        pose.pose.position.z = 0;
        path_msg.poses[i] = pose;
      }
      local_reference_viz_publisher_->publish(path_msg);
    }

    /*** Publish the log message ***/
    log_publisher_->publish(log_msg_);

    return;
  }
}

/*******************************************************************************
 * VelocityEstimationCallback                                                  *
 ******************************************************************************/
void LocalPlannerNode::VelocityEstimationCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg) {
  vcu_msgs::msg::VelocityEstimation latest_velocity_msg = *msg;

  current_velocity_ = std::hypot(latest_velocity_msg.vel.x, latest_velocity_msg.vel.y);
}

/*******************************************************************************
 * TravelledDistanceCallback                                                  *
 ******************************************************************************/
void LocalPlannerNode::TravelledDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  travelled_distance_ = static_cast<double>(msg->data);
}

/*******************************************************************************
 * GetSpeedTarget                                                              *
 ******************************************************************************/
Eigen::VectorXd LocalPlannerNode::GetSpeedTarget(const Eigen::VectorXd &curvature_profile,
                                                 const control_msgs::msg::ControllerRef &reference) {
  EASY_FUNCTION(profiler::colors::Black);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Pre-process the curvature profile
   *______________________________________________*/
  Eigen::VectorXd curvature = curvature_profile;
  Eigen::VectorXd original_curvature = curvature_profile;

  curvature = min_curvature_opt_->ProcessCurvature(original_curvature, false, params_.curvature_smoothness,
                                                   params_.gaussian_kernel_std);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute the speed targets
   *______________________________________________*/
  Eigen::VectorXd speed_targets = min_curvature_opt_->GetSpeedTargets(
      curvature, false, params_.max_lateral_acceleration, params_.min_longitudinal_speed,
      params_.max_longitudinal_speed, params_.kappa_shift);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Current velocity initialisation
   *______________________________________________*/
  if (params_.enable_current_vel_init) {
    speed_targets = speed_targets.array().min(current_velocity_);
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Straight line stopper function
   *______________________________________________*/
  if (params_.enable_straight_line_stopper) {
    if (curvature(curvature.size() - 2) < params_.straight_line_curvature_threshold) {
      straight_line_distance_ += (travelled_distance_ - last_travelled_distance_);
      last_travelled_distance_ = travelled_distance_;

      if (straight_line_distance_ > params_.straight_line_distance_threshold) {
        speed_targets = speed_targets.array().min(params_.min_longitudinal_speed * 2);
      }
    } else {
      straight_line_distance_ = 0.0;
    }
  }

  return speed_targets;
}
