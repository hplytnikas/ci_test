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

#include <global_planner/global_planner_node.hpp>

/*******************************************************************************
 * Constructor                                                                 *
 ******************************************************************************/
GlobalPlannerNode::GlobalPlannerNode(std::string node_name) : Node(node_name) {
  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Parameters handler
   *______________________________________________*/
  param_listener_ = std::make_shared<global_planner_params::ParamListener>(get_node_parameters_interface());

  params_ = param_listener_->get_params();

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Subscribers
   *______________________________________________*/
  path_subscriber_ = this->create_subscription<autonomous_msgs::msg::Boundary>(
      params_.path_subscriber_topic, 1, std::bind(&GlobalPlannerNode::PathCallback, this, std::placeholders::_1));

  velocity_estimation_subscriber_ = this->create_subscription<vcu_msgs::msg::VelocityEstimation>(
      params_.VE_subscriber_topic, 1,
      std::bind(&GlobalPlannerNode::VelocityEstimationCallback, this, std::placeholders::_1));

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Publishers
   *______________________________________________*/
  global_reference_publisher_ =
      this->create_publisher<control_msgs::msg::ControllerRef>(params_.global_reference_publisher_topic, 1);

  global_reference_viz_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>(params_.global_reference_viz_publisher_topic, 1);

  global_flag_publisher_ = this->create_publisher<std_msgs::msg::Bool>(params_.global_flag_publisher_topic, 1);

  log_publisher_ = this->create_publisher<control_msgs::msg::PlanningLog>(params_.log_publisher_topic, 1);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Timer or subscriber
   *______________________________________________*/
  if (params_.global_planner_frequency > 0.0) {
    RCLCPP_WARN(this->get_logger(), "Global Planner timer frequency: %f", params_.global_planner_frequency);

    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / params_.global_planner_frequency),
                                     std::bind(&GlobalPlannerNode::TimerCallback, this));
  } else {
    // Subscribe to the boundary message
    estimation_subscriber_ = this->create_subscription<autonomous_msgs::msg::ConeArray>(
        params_.estimation_subscriber_topic, 1,
        std::bind(&GlobalPlannerNode::EstimationCallback, this, std::placeholders::_1));
    RCLCPP_WARN(this->get_logger(), "Global Planner runs according to callback to estimation");
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * TF2
   *______________________________________________*/
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Solver set up
   *______________________________________________*/
  sampling_ = static_cast<int>(std::ceil(params_.track_length / params_.points_spacing));

  hpipm_args args{params_.mode,       params_.iter_max,  params_.alpha_min,  params_.mu0,      params_.tol_stat,
                  params_.tol_eq,     params_.tol_ineq,  params_.tol_comp,   params_.reg_prim, params_.reg_dual,
                  params_.warm_start, params_.pred_corr, params_.split_step, sampling_};

  int nb_iter = params_.nb_iter;
  bool first_point_fixed = params_.first_point_fixed;
  bool last_point_fixed = params_.last_point_fixed;
  bool closed = params_.closed_path;
  double max_curvature = params_.max_curvature;
  double max_distance = params_.max_distance;

  min_curvature_opt_ =
      std::make_shared<MinCurvatureOpt>(args, nb_iter, first_point_fixed, last_point_fixed, closed, max_curvature,
                                        max_distance, params_.car_width, params_.safety_margin);

  // driven speeds
  driven_speeds_ = Eigen::VectorXd::Zero(sampling_);

  RCLCPP_INFO(this->get_logger(), "Global Planner Node has been started");
}

/*******************************************************************************
 * PathCallback                                                                *
 ******************************************************************************/
void GlobalPlannerNode::PathCallback(const autonomous_msgs::msg::Boundary::SharedPtr msg) {
  // listen to the final path and finally set the global_map_received_ to true if all the steps are successful

  EASY_BLOCK("Log message update parameters");
  // reset the log message
  log_msg_ = control_msgs::msg::PlanningLog();
  log_msg_.header = msg->header;

  // Update parameters if they have changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  // fill the log_msg_ member variable with things that are known (const. parameters)
  log_msg_.frequency = params_.global_planner_frequency;

  log_msg_.enable_opt = params_.enable_opt;
  log_msg_.enable_prof = params_.enable_profiler;
  log_msg_.enable_viz = params_.enable_visualisation;
  log_msg_.enable_current_vel_init = false;
  log_msg_.enable_straight_line_stopper = false;
  log_msg_.enable_safe_set_learning = params_.enable_safe_set_learning;

  log_msg_.safe_velocity_multiplier = params_.safe_velocity_multiplier;
  log_msg_.safe_velocity_blend = params_.safe_velocity_blend;

  log_msg_.total_safety_dist = params_.car_width / 2.0 + params_.safety_margin;
  log_msg_.nb_iter = params_.nb_iter;

  log_msg_.max_radius_for_path_from_car = params_.path_radius;

  log_msg_.first_point_fixed = params_.first_point_fixed;
  log_msg_.last_point_fixed = params_.last_point_fixed;

  log_msg_.sampling_resolution = sampling_;

  log_msg_.max_lat_acc = params_.max_lateral_acceleration;
  log_msg_.min_longit_speed = params_.min_longitudinal_speed;
  log_msg_.max_longit_speed = params_.max_longitudinal_speed;

  log_msg_.curv_smoothness = params_.curvature_smoothness;
  log_msg_.gaussian_kernel_std = params_.gaussian_kernel_std;

  log_msg_.straight_line_distance_thresh = 0.0;
  log_msg_.straight_line_curvature_thresh = 0.0;

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

  try {
    EASY_FUNCTION(profiler::colors::CreamWhite);

    /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
     * Save the message
     *______________________________________________*/
    EASY_BLOCK("Log Points and Save the BE message");

    if (global_map_received_) {
      RCLCPP_WARN(this->get_logger(), "Global Planner has received a new global map, WTF are you doing Lollo??");
      return;
    }

    latest_reference_msg_ = *msg; // map frame

    // print the points (log them)
    for (int i = 0; i < latest_reference_msg_.middle_line.size(); ++i) {
      // fill the log message
      log_msg_.x_points_be.push_back(latest_reference_msg_.middle_line[i].position.x);
      log_msg_.y_points_be.push_back(latest_reference_msg_.middle_line[i].position.y);
    }

    log_msg_.num_points_be = static_cast<int>(latest_reference_msg_.middle_line.size());

    EASY_END_BLOCK;

    /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
     * Process the path if the optimisation is enabled
     *______________________________________________*/
    params_ = param_listener_->get_params();

    int size = latest_reference_msg_.middle_line.size();
    int size_left = latest_reference_msg_.left_boundary.size();
    int size_right = latest_reference_msg_.right_boundary.size();

    Eigen::MatrixX2d optimal_path(size, 2);
    Eigen::MatrixX2d boundaries_left(size_left, 2);
    Eigen::MatrixX2d boundaries_right(size_right, 2);

    optimal_path.setZero();
    boundaries_left.setZero();
    boundaries_right.setZero();

    if (params_.enable_opt) {
      /*** Convert the path message to an Eigen::Matrix2d ***/
      EASY_BLOCK("Convert the path message to an Eigen::Matrix2d");

      // so far optimization not failed
      log_msg_.optimization_status = 0;

      int counter = 0;
      int counter_left = 0;
      int counter_right = 0;

      for (int i = 0; i < size; ++i) {
        optimal_path(counter, 0) = latest_reference_msg_.middle_line[i].position.x;
        optimal_path(counter, 1) = latest_reference_msg_.middle_line[i].position.y;

        ++counter;
      }
      for (int i = 0; i < size_left; ++i) {
        boundaries_left(counter_left, 0) = latest_reference_msg_.left_boundary[i].position.x;
        boundaries_left(counter_left, 1) = latest_reference_msg_.left_boundary[i].position.y;

        ++counter_left;
      }
      for (int i = 0; i < size_right; ++i) {
        boundaries_right(counter_right, 0) = latest_reference_msg_.right_boundary[i].position.x;
        boundaries_right(counter_right, 1) = latest_reference_msg_.right_boundary[i].position.y;

        ++counter_right;
      }
      optimal_path.conservativeResize(counter, 2);
      boundaries_left.conservativeResize(counter_left, 2);
      boundaries_right.conservativeResize(counter_right, 2);

      if (counter < 2) {
        OptimisationFailed();

        log_msg_.optimization_status = 2; // Not enough points in the path to optimise

        throw GlobalPlannerNode::GlobalPlannerStepException("Not enough points in the path to optimise");
      }

      /*** Declare zero vectors ***/
      lateral_deviation_ = Eigen::MatrixX2d::Zero(sampling_, 2);
      curvature_profile_ = Eigen::VectorXd::Zero(sampling_);

      EASY_END_BLOCK;

      /*** Call the optimiser ***/
      min_curvature_opt_->GetOptimalReferencePath(optimal_path, lateral_deviation_, curvature_profile_, true,
                                                  boundaries_left, boundaries_right);

      /*** Check the output of the solver ***/
      if (optimal_path.hasNaN()) {
        OptimisationFailed();

        log_msg_.optimization_status = 3; // NaN values in the path

        throw GlobalPlannerNode::GlobalPlannerStepException("Optimiser failed : NaN values in the path");
      } else if (optimal_path.isApprox(1 * Eigen::MatrixXd::Ones(sampling_, 2), 1e-9)) {
        OptimisationFailed();

        log_msg_.optimization_status = 4; // Points are not ordered correctly or duplicate

        throw GlobalPlannerNode::GlobalPlannerStepException("Optimiser failed : Points are not ordered correctly.");
      } else if (optimal_path.isApprox(2 * Eigen::MatrixXd::Ones(sampling_, 2), 1e-9)) {
        OptimisationFailed();

        log_msg_.optimization_status = 5; // Issues while checking the curvature

        throw GlobalPlannerNode::GlobalPlannerStepException("Optimiser failed : Issues while checking the curvature");
      } else if (optimal_path.isApprox(3 * Eigen::MatrixXd::Ones(sampling_, 2), 1e-9)) {
        OptimisationFailed();

        log_msg_.optimization_status = 6; // Missing a point in the path

        throw GlobalPlannerNode::GlobalPlannerStepException("Optimiser failed : Missing a point");
      } else if (optimal_path.norm() < 1e-9) {
        OptimisationFailed();

        log_msg_.optimization_status = 1; // First and last points are not the same OR optimiser failed

        throw GlobalPlannerNode::GlobalPlannerStepException(
            "Optimiser failed or first and last points are not the same");
      }

      /*** Create a message and insert the new coordinates ***/
      reference_.header = latest_reference_msg_.header;
      reference_.reference_trajectory.clear();
      for (int i = 0; i < optimal_path.rows(); ++i) {
        control_msgs::msg::ReferenceState ref_state;
        ref_state.position.x = optimal_path(i, 0);
        ref_state.position.y = optimal_path(i, 1);
        ref_state.position.z = 0;
        reference_.reference_trajectory.push_back(ref_state);
      }
    } else {
      // No optimisation!
      reference_.reference_trajectory.clear();
      reference_.header = latest_reference_msg_.header;
      for (int i = 0; i < size; ++i) {
        control_msgs::msg::ReferenceState ref_state;
        ref_state.position.x = latest_reference_msg_.middle_line[i].position.x;
        ref_state.position.y = latest_reference_msg_.middle_line[i].position.y;
        ref_state.position.z = 0;
        reference_.reference_trajectory.push_back(ref_state);
      }

      log_msg_.optimization_status = -1; // not optimised
      log_publisher_->publish(log_msg_);

      RCLCPP_WARN(this->get_logger(), "Optimiser disabled, using the original path");
    }

    /*** Publish the visualisation ***/
    EASY_BLOCK("Visualisation");

    if (params_.enable_visualisation) {
      nav_msgs::msg::Path path_msg;
      path_msg.header = latest_reference_msg_.header;
      path_msg.header.frame_id = "map";

      for (size_t i = 0; i < reference_.reference_trajectory.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = reference_.reference_trajectory[i].position.x;
        pose.pose.position.y = reference_.reference_trajectory[i].position.y;
        pose.pose.position.z = reference_.reference_trajectory[i].position.z;
        pose.header = path_msg.header;
        path_msg.poses.push_back(pose);
      }

      global_reference_viz_publisher_->publish(path_msg);
    }

    EASY_END_BLOCK;

    /*** Compute the speed profile ***/
    GetSpeedTarget();

    /*** Publish the status flag ***/
    global_map_received_ = true;

    auto flag = std_msgs::msg::Bool();
    flag.data = global_map_received_;
    global_flag_publisher_->publish(flag);
  } catch (const GlobalPlannerNode::GlobalPlannerStepException &e) {
    log_publisher_->publish(log_msg_);

    RCLCPP_ERROR(this->get_logger(), "Global Planner failed: %s", e.what());
    return;
  }
}

/*******************************************************************************
 * VelocityEstimationCallback                                                  *
 ******************************************************************************/
void GlobalPlannerNode::VelocityEstimationCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg) {
  latest_velocity_msg_ = *msg;
}

/*******************************************************************************
 * TimerCallback                                                               *
 ******************************************************************************/
void GlobalPlannerNode::TimerCallback() {
  if (global_map_received_ && params_.global_planner_frequency > 0.0) {
    if (param_listener_->is_old(params_)) {
      params_ = param_listener_->get_params();
    }

    if (params_.enable_safe_set_learning) {
      GetSpeedTarget();
    }

    // Publish the transformed reference
    PublishTransformedReference(std::nullopt);
  }
}

/*******************************************************************************
 * EstimationCallback                                                          *
 ******************************************************************************/
void GlobalPlannerNode::EstimationCallback(const autonomous_msgs::msg::ConeArray &msg) {
  if (global_map_received_ && (params_.global_planner_frequency == 0.0)) {
    if (param_listener_->is_old(params_)) {
      params_ = param_listener_->get_params();
    }

    if (params_.enable_safe_set_learning) {
      GetSpeedTarget();
    }

    // Publish the transformed reference
    PublishTransformedReference(msg);
  }
}

/*******************************************************************************
 * PublishTransformedReference                                                 *
 ******************************************************************************/
void GlobalPlannerNode::PublishTransformedReference(
    const std::optional<autonomous_msgs::msg::ConeArray> &msg_opt = std::nullopt) {
  try {
    EASY_FUNCTION(profiler::colors::Black);

    /*** Lookup the latest transform from map to base_link ***/
    geometry_msgs::msg::TransformStamped transform_map_to_base_link;
    try {
      if (msg_opt) {
        // Use timestamp from ConeArray message if available
        transform_map_to_base_link = tf_buffer_->lookupTransform("base_link", "map", msg_opt->header.stamp);
      } else {
        // Use the latest transform if no ConeArray message is provided
        transform_map_to_base_link = tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero);
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }

    /*** Prepare the updated reference message for the controller in base_link frame ***/
    control_msgs::msg::ControllerRef updated_reference_msg;
    if (msg_opt) {
      updated_reference_msg.header.stamp = msg_opt->header.stamp;
    } else {
      updated_reference_msg.header.stamp = transform_map_to_base_link.header.stamp;
    }
    updated_reference_msg.header.frame_id = "base_link";

    /*** Reserve space in the vectors for efficiency ***/
    updated_reference_msg.reference_trajectory.reserve(reference_.reference_trajectory.size());

    /*** Transform each point in the raceline from map to base_link ***/
    for (const auto &ref_state : reference_.reference_trajectory) {
      geometry_msgs::msg::PointStamped point_in, point_out;
      point_in.point.x = ref_state.position.x;
      point_in.point.y = ref_state.position.y;
      point_in.point.z = ref_state.position.z;
      point_in.header.frame_id = "map";
      if (msg_opt) {
        point_in.header.stamp = msg_opt->header.stamp; // Use the current time for the point
      }

      // Perform the transformation
      tf2::doTransform(point_in, point_out, transform_map_to_base_link);

      // Add the transformed point to the updated reference message
      geometry_msgs::msg::Point32 point_transformed;
      point_transformed.x = point_out.point.x;
      point_transformed.y = point_out.point.y;
      point_transformed.z = point_out.point.z;

      control_msgs::msg::ReferenceState reference_state;
      reference_state.position = point_transformed;
      updated_reference_msg.reference_trajectory.push_back(reference_state);
    }

    /*** Prune the path to keep only a segment around the car ***/
    int nbRefPoints = updated_reference_msg.reference_trajectory.size();
    Eigen::MatrixXd path_matrix = Eigen::MatrixXd::Zero(2, nbRefPoints);
    int count = 0;

    //  Find the point with the smallest distance from (0, 0)
    auto it = std::min_element(
        updated_reference_msg.reference_trajectory.begin(), updated_reference_msg.reference_trajectory.end(),
        [](const control_msgs::msg::ReferenceState &a, const control_msgs::msg::ReferenceState &b) {
          return std::hypot(a.position.x, a.position.y) < std::hypot(b.position.x, b.position.y);
        });

    // Get the index of the point with the smallest distance
    size_t start_index = std::distance(updated_reference_msg.reference_trajectory.begin(), it);

    // set the speed of the car to the latest speed
    driven_speeds_(start_index) = latest_velocity_msg_.vel.x;

    double max_distance = params_.path_radius;
    Eigen::MatrixX2d lateral_deviation_to_pub = Eigen::MatrixX2d::Zero(nbRefPoints, 2);
    Eigen::VectorXd speed_ref_to_pub = Eigen::VectorXd::Zero(nbRefPoints);

    size_t i = start_index;
    while (true) {
      double distance = std::hypot(updated_reference_msg.reference_trajectory[i].position.x,
                                   updated_reference_msg.reference_trajectory[i].position.y);

      if (distance < max_distance) {
        path_matrix(0, count) = updated_reference_msg.reference_trajectory[i].position.x;
        path_matrix(1, count) = updated_reference_msg.reference_trajectory[i].position.y;
        lateral_deviation_to_pub(count, 0) = lateral_deviation_(i, 0);
        lateral_deviation_to_pub(count, 1) = lateral_deviation_(i, 1);
        speed_ref_to_pub(count) = reference_.reference_trajectory[i].vx_ref;
        count++;
        i = (i + 1) % nbRefPoints; // Wrap around
      } else {
        path_matrix.conservativeResize(2, count);
        lateral_deviation_to_pub.conservativeResize(count, 2);
        speed_ref_to_pub.conservativeResize(count);
        break;
      }

      // Check if we have looped back to the starting point
      if (i == start_index) {
        break;
      }
    }

    /*** Put the path matrix into the updated reference message ***/
    updated_reference_msg.reference_trajectory.clear();
    for (int i = 0; i < path_matrix.cols(); ++i) {
      control_msgs::msg::ReferenceState ref_state;
      ref_state.position.x = path_matrix(0, i);
      ref_state.position.y = path_matrix(1, i);
      ref_state.position.z = 0.0;
      updated_reference_msg.reference_trajectory.push_back(ref_state);
    }

    /*** Fill the boundaries ***/
    for (int i = 0; i < path_matrix.cols(); ++i) {
      updated_reference_msg.reference_trajectory[i].boundary_right = lateral_deviation_to_pub(i, 1);
      updated_reference_msg.reference_trajectory[i].boundary_left = lateral_deviation_to_pub(i, 0);

      updated_reference_msg.reference_trajectory[i].vx_ref = speed_ref_to_pub(i);
    }

    /*** Publish the updated reference in the base_link frame ***/
    global_reference_publisher_->publish(updated_reference_msg);

    // publish log message
    log_publisher_->publish(log_msg_);
  } catch (const GlobalPlannerNode::GlobalPlannerStepException &e) {
    RCLCPP_ERROR(this->get_logger(), "Global Planner failed: %s", e.what());
    return;
  }
}

/*******************************************************************************
 * GetSpeedTarget                                                              *
 ******************************************************************************/
void GlobalPlannerNode::GetSpeedTarget() {
  EASY_FUNCTION(profiler::colors::Green);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Pre-process the curvature profile
   *______________________________________________*/
  Eigen::VectorXd curvature = curvature_profile_;
  Eigen::VectorXd original_curvature = curvature_profile_;

  curvature = min_curvature_opt_->ProcessCurvature(original_curvature, true, params_.curvature_smoothness,
                                                   params_.gaussian_kernel_std);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute the speed targets
   *______________________________________________*/
  Eigen::VectorXd speed_targets = min_curvature_opt_->GetSpeedTargets(
      curvature, true, params_.max_lateral_acceleration, params_.min_longitudinal_speed, params_.max_longitudinal_speed,
      params_.kappa_shift);

  if (params_.enable_safe_set_learning) {
    // update the speed targets by knowing the speeds of the car that were driven and the theoretically calculated
    // speeds

    Eigen::VectorXd new_safe_speeds = Eigen::VectorXd::Zero(driven_speeds_.size());

    // assuming we never drive zero speed exactly, we take the theoretical speeds wherever driven_speeds_ is zero
    for (int i = 0; i < driven_speeds_.size(); ++i) {
      if (driven_speeds_(i) == 0.0) {
        // we haven't driven at this point, so we take the theoretical speed target
        new_safe_speeds(i) = speed_targets(i);
      } else {
        // we have driven at this point, so we take the minimum of the driven speed and the theoretical speed target
        new_safe_speeds(i) = params_.safe_velocity_multiplier * driven_speeds_(i);
      }
    }

    // the new speed targets are calculated by taking the convex combination of the driven speeds and the theoretically
    // calculated speeds using the params_.safe_velocity_blend
    speed_targets = params_.safe_velocity_blend * new_safe_speeds + (1 - params_.safe_velocity_blend) * speed_targets;
  }

  for (int i = 0; i < speed_targets.size(); ++i) {
    reference_.reference_trajectory[i].vx_ref = speed_targets(i);
  }
}

/*******************************************************************************
 * OptimisationFailed                                                          *
 ******************************************************************************/
void GlobalPlannerNode::OptimisationFailed() {
  global_map_received_ = false;

  auto flag = std_msgs::msg::Bool();
  flag.data = global_map_received_;
  global_flag_publisher_->publish(flag);

  latest_reference_msg_.middle_line.clear();
  latest_reference_msg_.left_boundary.clear();
  latest_reference_msg_.right_boundary.clear();
}
