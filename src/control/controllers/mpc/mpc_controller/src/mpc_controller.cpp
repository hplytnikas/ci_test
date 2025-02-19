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

#include "mpc_controller/mpc_controller.hpp"

/*******************************************************************************
 * Constructor                                                                 *
 *******************************************************************************/
MpcController::MpcController(std::shared_ptr<ControllerNode> controller_node,
                             std::shared_ptr<MpcVisualisationNode> mpc_visualisation_node,
                             std::shared_ptr<mpc_controller_params::ParamListener> param_listener)
    : AbstractController(controller_node), mpc_visualisation_node_(mpc_visualisation_node),
      param_listener_(param_listener) {
  // Get the parameters
  params_ = param_listener_->get_params();

  // Set the solver type based on the config file
  solver_type_ = params_.solver_type;

  //  Get the solver object
  if (solver_type_ == "EMBOTECH") {
    mpc_solver_ptr_ = std::make_shared<EmbotechSolver>(params_.consecutive_failures_limit);
  } else if (solver_type_ == "ACADOS") {
    mpc_solver_ptr_ = std::make_shared<AcadosSolver>(params_.consecutive_failures_limit);
  } else {
    controller_node_->LogFatal("Solver type not recognized. Shutting down.");
    rclcpp::shutdown();
  }
  this->Initialize();
}

// Overloaded Constructor for when solver is passed, can be used for simulation / automatic pipeline check for Github
MpcController::MpcController(std::shared_ptr<ControllerNode> controller_node,
                             std::shared_ptr<MpcVisualisationNode> mpc_visualisation_node,
                             std::shared_ptr<mpc_controller_params::ParamListener> param_listener,
                             std::shared_ptr<MpcSolver> mpc_solver)
    : AbstractController(controller_node), mpc_visualisation_node_(mpc_visualisation_node),
      param_listener_(param_listener), mpc_solver_ptr_(mpc_solver) {
  // Get the parameters
  params_ = param_listener_->get_params();

  // Set the solver type based on the given solver
  solver_type_ = mpc_solver_ptr_->GetSolverName();

  this->Initialize();
}

/*******************************************************************************
 * Initialize the MPC controller                                               *
 *******************************************************************************/
void MpcController::Initialize() {
  EASY_FUNCTION(profiler::colors::Navy, "Initialize");

  // Get the controller node's frequency
  dt_ = 1.0 / controller_node_->GetControllerFrequency();

  //  Get the geometry object
  mpc_geometry_ptr_ = std::make_shared<MpcGeometry>(InterpolantType2D::BSPLINE);

  //  Set up the solver
  mpc_solver_ptr_->Initialize();

  // Solver Ready
  solver_status_ = MpcSolverStatus::Ready;

  // Dimensions
  N_STATE_ = mpc_solver_ptr_->GetNState();
  N_INPUT_ = mpc_solver_ptr_->GetNInput();
  N_HORIZON_ = mpc_solver_ptr_->GetNHorizon();

  // Create Interpolant for Inputs
  inputs_interpolant_1d_ = std::make_shared<LinearInterpolant1D>();

  // Control Input Interpolation
  Eigen::VectorXd t_values = Eigen::VectorXd::LinSpaced(N_HORIZON_ + 1, 0, N_HORIZON_ * mpc_solver_ptr_->GetDtMpc());
  inputs_interpolant_1d_->SetAbscissaValues(t_values);

  Eigen::VectorXd u_values = Eigen::VectorXd::Zero(N_HORIZON_ + 1);
  inputs_interpolant_1d_->SetOrdinateValues(u_values);

  // Solver Solutions
  solution_states_ = Eigen::MatrixXd::Zero(N_STATE_, N_HORIZON_ + 1);
  solution_inputs_ = Eigen::MatrixXd::Zero(N_INPUT_, N_HORIZON_);

  shifted_solver_states_ = Eigen::MatrixXd::Zero(N_STATE_, N_HORIZON_);
  shifted_solver_inputs_ = Eigen::MatrixXd::Zero(N_INPUT_, N_HORIZON_);

  // Initial State
  initial_state_ = Eigen::VectorXd::Zero(N_STATE_);

  // Measured State
  measured_state_ = Eigen::VectorXd::Zero(N_STATE_);

  // Progress and Curvature Horizon
  progress_horizon_ = Eigen::VectorXd::Zero(N_HORIZON_);
  curvature_horizon_ = Eigen::VectorXd::Zero(N_HORIZON_);

  // Trackbound Min and Max Horizons
  trackbound_min_horizon_ = Eigen::VectorXd::Zero(N_HORIZON_);
  trackbound_max_horizon_ = Eigen::VectorXd::Zero(N_HORIZON_);

  // Velocity Horizon
  velocity_horizon_ = Eigen::VectorXd::Zero(N_HORIZON_);

  // Reset the controller for the first UpdateControlCommand iteration such that we know the values in the solver
  // prediction are zeros
  resetController();

  // Init solver with parameters:
  params_ = param_listener_->get_params();
  SetSolverRuntimeParams();

  // log solver type
  controller_node_->LogInfo("Solver type: %s", solver_type_.c_str());

  // Just logging the parameters
  if (params_.enable_visualisation) {
    controller_node_->LogInfo("Visualisation enabled");
  } else {
    controller_node_->LogInfo("Visualisation disabled");
  }
  if (params_.enable_logging) {
    controller_node_->LogInfo("Logging enabled");
  } else {
    controller_node_->LogInfo("Logging disabled");
  }

  // Log the time step of the MPC solver
  controller_node_->LogInfo("MPC time step: %f [s]", mpc_solver_ptr_->GetDtMpc());
  // Node frequency
  controller_node_->LogInfo("Node time step: %f [s]", dt_);

  // Log that the controller is initialized
  controller_node_->LogInfo("MPC controller initialized. GREAT SUCCESS!");
}

/*******************************************************************************
 * Destructor                                                                  *
 *******************************************************************************/
MpcController::~MpcController() {}

/*******************************************************************************
 * updateControlCommand                                                        *
 *******************************************************************************/
vcu_msgs::msg::CarCommand MpcController::updateControlCommand() {
  EASY_FUNCTION(profiler::colors::Magenta);

  // ===========================================================================
  // === Check if controller is in fatal state =================================
  if (fatal_error_flag_) {
    // Stop the car if the controller is in a fatal state
    car_command_ = StopCar();
    return car_command_;
  }
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // ===========================================================================
  // === Timing Start ==========================================================
  auto t0 = std::chrono::high_resolution_clock::now();
  // ===========================================================================

  // ===========================================================================
  // === Local Variables =======================================================
  EASY_BLOCK("Local Variables", profiler::colors::Blue);

  auto state = ControllerNode::State();
  auto ref = ControllerNode::Reference();

  EASY_END_BLOCK; //  "Local Variables"
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // ===========================================================================
  // === Parameters ============================================================
  EASY_BLOCK("Parameters", profiler::colors::CreamWhite);

  params_ = param_listener_->get_params();

  SetSolverRuntimeParams();

  EASY_END_BLOCK; // "Parameters"
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // ===========================================================================
  // === State and Reference ===================================================
  EASY_BLOCK("State and Reference", profiler::colors::Blue);

  try { // throws an exception if the state or the reference are not available
    EASY_BLOCK("GetState", profiler::colors::White);
    state = controller_node_->GetState();
    EASY_END_BLOCK;

    EASY_BLOCK("GetReference", profiler::colors::Black);
    ref = controller_node_->GetReference();
    EASY_END_BLOCK;
  } catch (std::exception &e) {
    std::string error_message = std::string(e.what()) + "\n";
    throw ControllerNode::ControllerStepException(error_message);
  }

  EASY_END_BLOCK; // "State and Reference"
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // Fit reference path, and prepare interpolation for velocity and boundaries
  this->PrepareGeometry(ref);

  // ===========================================================================
  // === Geometric Initial State ===============================================
  EASY_BLOCK("Geometric Initial State", profiler::colors::DarkCyan);

  double s_init = 0.0;
  double n_init = 0.0;
  double mu_init = 0.0;
  // to curvilinear coordinates
  if (mpc_geometry_ptr_->GetInitialState(s_init, n_init, mu_init)) {
    throw ControllerNode::ControllerStepException("Failed to get the initial state");
  }

  EASY_END_BLOCK; // "Geometric Initial State"
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // ===========================================================================
  // === Shifted Progress Horizon ==============================================
  EASY_BLOCK("Shifted progress horizon", profiler::colors::Pink);

  // as the MPC doesn't have a state for the progress, we need to integrate it
  // from the velocities and the previous curvature values
  bool tmp_shift_s = mpc_solver_ptr_->GetShiftedProgressHorizon(progress_horizon_, curvature_horizon_);

  if (tmp_shift_s) {
    std::string error_message = "Failed to get the shifted progress horizon.\n";
    throw ControllerNode::ControllerStepException(error_message);
  }
  if (!params_.enable_delay_compensation) {
    // subtract the first element from all the elements to start at s0 = 0.0
    progress_horizon_ = progress_horizon_ - progress_horizon_[0] * Eigen::VectorXd::Ones(progress_horizon_.size());
  }
  // if we use delay compensation, we don't shift the progress horizon as the initial progress should not be 0.0 then.
  // this comes from the fact that the initial state is not the measured state but the predicted state at dt_mpc

  EASY_END_BLOCK; // "Shifted progress horizon"
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // ===========================================================================
  // === Get Horizon Parameters ================================================
  EASY_BLOCK("Horizon parameters", profiler::colors::Pink);

  EASY_BLOCK("Get curvature", profiler::colors::Pink300);
  if (mpc_geometry_ptr_->GetCurvatureHorizon(progress_horizon_, curvature_horizon_)) {
    throw ControllerNode::ControllerStepException("Failed to get the curvature horizon");
  }
  EASY_END_BLOCK; // "Get curvature"

  // Set the curvature horizon in the solver
  EASY_BLOCK("Set curvature horizon", profiler::colors::DarkCyan);
  if (mpc_solver_ptr_->SetModelParameter(MpcParamIdx::curv, curvature_horizon_)) {
    std::string error_message =
        "Failed to set the curvature horizon with size: " + std::to_string(curvature_horizon_.size()) + "\n";
    throw ControllerNode::ControllerStepException(error_message);
  }
  EASY_END_BLOCK; // "Set curvature horizon"

  EASY_BLOCK("Get Track Boundaries", profiler::colors::Pink300);
  mpc_geometry_ptr_->GetBoundaries(progress_horizon_, trackbound_min_horizon_, trackbound_max_horizon_);
  EASY_END_BLOCK; // "Get Track Boundaries"

  EASY_BLOCK("Set track boundaries", profiler::colors::DarkCyan);
  Eigen::VectorXd trackbound_min_horizon_toSolver = trackbound_min_horizon_;

  // Apply the shrinkage factor to each element of the track boundary constraint
  for (int i = 0; i < trackbound_min_horizon_toSolver.size(); ++i) {
    // trackbound_min_horizon_toSolver[i] *= std::pow(params_.trackbound_shrinkage, i);
    // trackbound_min_horizon_toSolver[i] *= params_.trackbound_shrinkage;
    trackbound_min_horizon_toSolver[i] *= exp(-params_.trackbound_shrinkage * progress_horizon_[i]);
  }

  if (mpc_solver_ptr_->SetModelParameter(MpcParamIdx::trackbound_min, trackbound_min_horizon_toSolver)) {
    std::string error_message =
        "Failed to set the trackbound_min_horizon with size: " + std::to_string(trackbound_min_horizon_.size()) + "\n";
    throw ControllerNode::ControllerStepException(error_message);
  }
  Eigen::VectorXd trackbound_max_horizon_toSolver = trackbound_max_horizon_;

  // Apply the shrinkage factor to each element of the track boundary constraint
  for (int i = 0; i < trackbound_max_horizon_toSolver.size(); ++i) {
    // trackbound_max_horizon_toSolver[i] *= std::pow(params_.trackbound_shrinkage, i);
    // trackbound_max_horizon_toSolver[i] *= params_.trackbound_shrinkage;
    trackbound_max_horizon_toSolver[i] *= exp(-params_.trackbound_shrinkage * progress_horizon_[i]);
  }

  if (mpc_solver_ptr_->SetModelParameter(MpcParamIdx::trackbound_max, trackbound_max_horizon_toSolver)) {
    std::string error_message =
        "Failed to set the trackbound_max_horizon with size: " + std::to_string(trackbound_max_horizon_.size()) + "\n";
    throw ControllerNode::ControllerStepException(error_message);
  }
  EASY_END_BLOCK; // "Set track boundaries"

  if (params_.enable_velocity_planning) {
    EASY_BLOCK("Get Velocity Horizon", profiler::colors::Pink300);
    mpc_geometry_ptr_->GetVelocityProfile(params_.terminal_vel_progress_scaling * progress_horizon_, velocity_horizon_);
    EASY_END_BLOCK; // "Get Velocity Horizon"

    EASY_BLOCK("Set max. velocities", profiler::colors::White);
    // if(mpc_solver_ptr_->SetModelParameter(MpcParamIdx::vx_max, velocity_horizon_)){
    //   std::string error_message = "Failed to set the velocity horizon with size: " +
    //   std::to_string(velocity_horizon_.size()) + "\n"; throw ControllerNode::ControllerStepException(error_message);
    // }

    double terminal_velocity_tmp = params_.terminal_vel_scaling * velocity_horizon_[velocity_horizon_.size() - 1];
    params_.vx_max_terminal = terminal_velocity_tmp;
    if (mpc_solver_ptr_->SetModelParameter(MpcParamIdx::vx_max_term, terminal_velocity_tmp)) {
      std::string error_message =
          "Failed to set the terminal velocity : " + std::to_string(terminal_velocity_tmp) + "\n";
      throw ControllerNode::ControllerStepException(error_message);
    }
    EASY_END_BLOCK; // "Set max. velocities"
  }

  EASY_END_BLOCK; // "Horizon Parameters"
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // ===========================================================================
  // === SOLVER: Set Initial State =============================================
  EASY_BLOCK("Initial state", profiler::colors::Blue);

  // Set initial state vector
  EASY_BLOCK("Fill initial state vector", profiler::colors::Pink);
  measured_state_[MpcStateIdx::n] = n_init;
  measured_state_[MpcStateIdx::mu] = mu_init;
  measured_state_[MpcStateIdx::vx] = state.dx;
  measured_state_[MpcStateIdx::vy] = state.dy;
  measured_state_[MpcStateIdx::dpsi] = state.dpsi;
  measured_state_[MpcStateIdx::ax] =
      state.ddx + params_.C_r * 9.81 + 0.5 * 1.225 * 1.2 * params_.C_d * state.dx * state.dx / params_.m;
  measured_state_[MpcStateIdx::dels] = state.delta_s;
  EASY_END_BLOCK; // "Fill initial state vector"

  // Check measured state validity
  EASY_BLOCK("Check meas. state", profiler::colors::YellowA100);
  std::string error_message_init;
  bool measured_state_valid = CheckMeasuredStateValidity(error_message_init);
  EASY_END_BLOCK; // "Check meas. state"

  if (measured_state_valid) {
    // Here we either set the initial state of the MPC problem to be the measured state
    // or, when using delay compensation, we set the initial state to be the predicted state
    // at dt_mpc
    EASY_BLOCK("Calculate initial state", profiler::colors::Pink);
    if (params_.enable_delay_compensation) {
      // using delay compensation
      // --> this function will populate the initial_state_ with the predicted state at dt_mpc
      this->IntegratePredictedInitialState();
    } else {
      // not using delay compensation
      initial_state_ = measured_state_;
    }

    EASY_BLOCK("Set init. state", profiler::colors::DarkCyan);
    if (mpc_solver_ptr_->SetInitialState(initial_state_)) {
      throw ControllerNode::ControllerStepException("Failed to set the initial state");
    }
    EASY_END_BLOCK; // "Set init. state"

    EASY_END_BLOCK; // "Initial state"
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // ===========================================================================
    // === Set the initial guess =================================================
    EASY_BLOCK("Initial guess", profiler::colors::Green);

    // as only the SQP solver needs the initial guess, the initial guess is only
    // set if the solver type is ACADOS
    // TODO(_): add initial guess for EMBOTECH if using SQP in embotech?

    this->SetInitialSolverGuess();

    EASY_END_BLOCK; // "Initial guess"
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // Increasing terminal speed if max. abs. curvature is below a certain
    // threshold and we do not enable velocity planning
    if (!params_.enable_velocity_planning) {
      double max_abs_curvature = curvature_horizon_.cwiseAbs().maxCoeff();

      // get the terminal progress to check if it is long enough
      double terminal_progress = progress_horizon_[progress_horizon_.size() - 1];

      if (max_abs_curvature < params_.low_curvature_threshold &&
          terminal_progress > params_.terminal_progress_threshold) {
        double terminal_velocity_tmp = params_.terminal_vel_scaling * params_.vx_max_terminal;
        terminal_velocity_tmp = terminal_velocity_tmp + params_.vx_max_terminal_addition_on_low_curvature;

        // change params_ to log it
        params_.vx_max_terminal = terminal_velocity_tmp;

        if (mpc_solver_ptr_->SetModelParameter(MpcParamIdx::vx_max_term, terminal_velocity_tmp)) {
          std::string error_message =
              "Failed to set the terminal velocity : " + std::to_string(terminal_velocity_tmp) + "\n";
          throw ControllerNode::ControllerStepException(error_message);
        }
      }
    }

    // ===========================================================================
    // === Solve the problem =====================================================
    EASY_BLOCK("Solving", profiler::colors::Pink);
    // solving only if the velocity is above 0.8 * vel_threshold_standstill
    if (state.dx > 0.8 * params_.vel_threshold_standstill || launch_flag_ == true) {
      mpc_solver_ptr_->Solve();
      // controller_node_->LogInfo("Solved the MPC Problem.");
    }

    EASY_END_BLOCK; // "Solving"
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // ===========================================================================
    // === Check the solver status ===============================================
    EASY_BLOCK("Check solver status", profiler::colors::CreamWhite);

    if (!CheckSolverStatus() || fatal_error_flag_) {
      fatal_error_flag_ = true;

      controller_node_->LogFatal("Fatal error flag is set. Stopping the car immediatly.");
      car_command_ = StopCar();
      return car_command_;
    }

    EASY_END_BLOCK; // "Check solver status"
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // ===========================================================================
    // === Get the solver solution ===============================================
    EASY_BLOCK("Get solver solution", profiler::colors::Pink);

    if (mpc_solver_ptr_->GetSolverSolution(solution_states_, solution_inputs_)) {
      throw ControllerNode::ControllerStepException("Failed to get the solver solution");
    }

    EASY_END_BLOCK; // "Get solver solution"
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // ===========================================================================
    // === Visualize the MPC -----------------------------------------------------
    if (params_.enable_visualisation) {
      VisualizeMpc();
    }
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // ===========================================================================
    // === Log the MPC ===========================================================
    double solvetime_ms = 0.0;
    if (mpc_solver_ptr_->GetSolverDiagnostics(MpcSolverDiagnostic::SolveTimeMS, solvetime_ms)) {
      controller_node_->LogWarn("Failed to get the solvetime");
    } else if (solvetime_ms > 15.0 || params_.enable_logging) {
      LogMpc(solvetime_ms, ref);
    }
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  } else {
    // if the measured state is not valid, we use the pure pursuit controller
    controller_node_->LogWarn("Measured state is not valid. Using Pure Pursuit as a fallback controller.");
  }

  // ===========================================================================
  // === Populate and return the car command ===================================

  // if the MPC solver threw a warning or an error,
  // we use Pure Pursuit to populate the car command
  EASY_BLOCK("Set potential fallback car command", profiler::colors::CreamWhite);

  if (launch_flag_) {
    if (((mpc_warning_counter_ > params_.num_warnings_before_fallback) && params_.pp_fallback_enabled) ||
        state.dx < params_.pp_fallback_start_speed_threshold || !measured_state_valid) {
      controller_node_->LogWarn("Using Pure Pursuit as a fallback controller.");

      using_pp_fallback_ = true;

      car_command_ = CalculatePurePursuitCommand(state, ref);
      return car_command_;
    } else if (state.dx < params_.pp_fallback_stop_speed_threshold && using_pp_fallback_) {
      controller_node_->LogWarn("Using Pure Pursuit as a fallback controller after having used it.");

      car_command_ = CalculatePurePursuitCommand(state, ref);
      return car_command_;
    } else {
      using_pp_fallback_ = false;

      if (solver_status_ == MpcSolverStatus::Warn) {
        controller_node_->LogWarn("Not using fallback but instead relying on previously sent car command.");

        throw ControllerNode::ControllerStepException("MPC solver threw a warning.");
      }
    }
  }

  EASY_END_BLOCK; // "Set potential fallback car command"

  EASY_BLOCK("Set MPC car command", profiler::colors::CreamWhite);

  double axm_1 = 0.0;
  double axm_2 = 0.0;
  double axm_3 = 0.0;

  double del_s_1 = 0.0;
  double del_s_2 = 0.0;
  double del_s_3 = 0.0;

  double dpsi_1 = 0.0;
  double dpsi_2 = 0.0;
  double dpsi_3 = 0.0;

  double dt_mpc = mpc_solver_ptr_->GetDtMpc();

  if (params_.enable_delay_compensation) {
    // using delay compensation and additionally specified interpolation times
    // to not use the predicted states without control authority
    double dt_interp_ax = params_.mpc_dt_interp_ax;
    double dt_interp_dels = params_.mpc_dt_interp_dels;
    double dt_interp_dpsi = params_.mpc_dt_interp_dpsi;

    // Calculate ax from the solution
    this->inputs_interpolant_1d_->SetOrdinateValues(solution_states_.row(MpcStateIdx::ax));
    axm_1 = this->inputs_interpolant_1d_->GetFvalFromX(dt_interp_ax);
    axm_2 = this->inputs_interpolant_1d_->GetFvalFromX(dt_interp_ax + dt_mpc);
    axm_3 = this->inputs_interpolant_1d_->GetFvalFromX(dt_interp_ax + 2 * dt_mpc);

    // Calculate the steering angle from the solution
    this->inputs_interpolant_1d_->SetOrdinateValues(solution_states_.row(MpcStateIdx::dels));
    del_s_1 = this->inputs_interpolant_1d_->GetFvalFromX(dt_interp_dels);
    del_s_2 = this->inputs_interpolant_1d_->GetFvalFromX(dt_interp_dels + dt_mpc);
    del_s_3 = this->inputs_interpolant_1d_->GetFvalFromX(dt_interp_dels + 2 * dt_mpc);

    // // Calculate the yaw rate from the solution
    this->inputs_interpolant_1d_->SetOrdinateValues(solution_states_.row(MpcStateIdx::dpsi));
    dpsi_1 = this->inputs_interpolant_1d_->GetFvalFromX(dt_interp_dpsi);
    dpsi_2 = this->inputs_interpolant_1d_->GetFvalFromX(dt_interp_dpsi + dt_mpc);
    dpsi_3 = this->inputs_interpolant_1d_->GetFvalFromX(dt_interp_dpsi + 2 * dt_mpc);
  } else {
    // Not using delay compensation
    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    double dt_solve = dt / 1e6;

    if (params_.enable_output_interpolation) {
      // Calculate ax from the solution
      this->inputs_interpolant_1d_->SetOrdinateValues(solution_states_.row(MpcStateIdx::ax));
      axm_1 = this->inputs_interpolant_1d_->GetFvalFromX(dt_solve);
      axm_2 = this->inputs_interpolant_1d_->GetFvalFromX(dt_solve + dt_mpc);
      axm_3 = this->inputs_interpolant_1d_->GetFvalFromX(dt_solve + 2 * dt_mpc);

      // Calculate the steering angle from the solution
      this->inputs_interpolant_1d_->SetOrdinateValues(solution_states_.row(MpcStateIdx::dels));
      del_s_1 = this->inputs_interpolant_1d_->GetFvalFromX(dt_solve);
      del_s_2 = this->inputs_interpolant_1d_->GetFvalFromX(dt_solve + dt_mpc);
      del_s_3 = this->inputs_interpolant_1d_->GetFvalFromX(dt_solve + 2 * dt_mpc);

      // // Calculate the yaw rate from the solution
      this->inputs_interpolant_1d_->SetOrdinateValues(solution_states_.row(MpcStateIdx::dpsi));
      dpsi_1 = this->inputs_interpolant_1d_->GetFvalFromX(dt_solve);
      dpsi_2 = this->inputs_interpolant_1d_->GetFvalFromX(dt_solve + dt_mpc);
      dpsi_3 = this->inputs_interpolant_1d_->GetFvalFromX(dt_solve + 2 * dt_mpc);

    } else {
      // This is under the assumption that the node runs at the same frequency as the MPC is discretized
      axm_1 = solution_states_(MpcStateIdx::ax, 1);
      axm_2 = solution_states_(MpcStateIdx::ax, 2);
      axm_3 = solution_states_(MpcStateIdx::ax, 3);

      del_s_1 = solution_states_(MpcStateIdx::dels, 1);
      del_s_2 = solution_states_(MpcStateIdx::dels, 2);
      del_s_3 = solution_states_(MpcStateIdx::dels, 3);

      dpsi_1 = solution_states_(MpcStateIdx::dpsi, 1);
      dpsi_2 = solution_states_(MpcStateIdx::dpsi, 2);
      dpsi_3 = solution_states_(MpcStateIdx::dpsi, 3);
    }
  }

  // Populate the car command

  if ((state.dx < params_.vel_threshold_standstill) && launch_flag_ == false) {
    // for the case when the car is standing still and we launch initially

    // calculate pure pursuit command
    car_command_ = CalculatePurePursuitCommand(state, ref);

    // overwrite the acceleration with the standstill acceleration
    car_command_.a_x[0] = params_.ax_command_standstill;
    car_command_.a_x[1] = params_.ax_command_standstill;
    car_command_.a_x[2] = params_.ax_command_standstill;
  } else {
    car_command_.a_x[0] =
        axm_1 - params_.C_r * 9.81 - 0.5 * 1.225 * 1.2 * params_.C_d * state.dx * state.dx / params_.m;
    car_command_.a_x[1] =
        axm_2 - params_.C_r * 9.81 - 0.5 * 1.225 * 1.2 * params_.C_d * state.dx * state.dx / params_.m;
    car_command_.a_x[2] =
        axm_3 - params_.C_r * 9.81 - 0.5 * 1.225 * 1.2 * params_.C_d * state.dx * state.dx / params_.m;
    // car_command_.a_x[0] = axm_1;
    // car_command_.a_x[1] = axm_2;
    // car_command_.a_x[2] = axm_3;

    car_command_.steering_angle[0] = del_s_1;
    car_command_.steering_angle[1] = del_s_2;
    car_command_.steering_angle[2] = del_s_3;

    car_command_.yaw_rate[0] = dpsi_1;
    car_command_.yaw_rate[1] = dpsi_2;
    car_command_.yaw_rate[2] = dpsi_3;

    // we have launched the car
    launch_flag_ = true;
  }

  EASY_END_BLOCK; // "Set car command"
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  return car_command_;
}

/*******************************************************************************
 * resetController                                                             *
 *******************************************************************************/
void MpcController::resetController() {
  // Reset the solver
  mpc_solver_ptr_->Reset();
}

/*******************************************************************************
 * checkSolverStatus                                                           *
 *******************************************************************************/
bool MpcController::CheckSolverStatus() {
  EASY_FUNCTION(profiler::colors::Navy, "checkSolverStatus");

  mpc_solver_ptr_->GetSolverDiagnostics(MpcSolverDiagnostic::SolverStatus, solver_status_);

  if (solver_status_ != MpcSolverStatus::Ok) {
    if (solver_status_ == MpcSolverStatus::Warn) {
      controller_node_->LogWarn(mpc_solver_ptr_->GetSolverStatusString().c_str());

      mpc_warning_counter_++;

      // print initial state of MPC
      controller_node_->LogWarn(
          "Initial state: n: %f, mu: %f, vx: %f, vy: %f, dpsi: %f, ax: %f, del_s: %f", initial_state_[MpcStateIdx::n],
          initial_state_[MpcStateIdx::mu], initial_state_[MpcStateIdx::vx], initial_state_[MpcStateIdx::vy],
          initial_state_[MpcStateIdx::dpsi], initial_state_[MpcStateIdx::ax], initial_state_[MpcStateIdx::dels]);

      return true;
    } else if (solver_status_ == MpcSolverStatus::Fatal) {
      controller_node_->LogFatal(mpc_solver_ptr_->GetSolverStatusString().c_str());

      // print initial state of MPC
      controller_node_->LogFatal(
          "Initial state: n: %f, mu: %f, vx: %f, vy: %f, dpsi: %f, ax: %f, del_s: %f", initial_state_[MpcStateIdx::n],
          initial_state_[MpcStateIdx::mu], initial_state_[MpcStateIdx::vx], initial_state_[MpcStateIdx::vy],
          initial_state_[MpcStateIdx::dpsi], initial_state_[MpcStateIdx::ax], initial_state_[MpcStateIdx::dels]);

      return false;
    }
  } else {
    mpc_warning_counter_ = 0;
  }
  return true;
}

/*******************************************************************************
 * CheckMeasuredStateValidity                                                   *
 *******************************************************************************/
bool MpcController::CheckMeasuredStateValidity(std::string &error_message) {
  EASY_FUNCTION(profiler::colors::Navy, "CheckMeasuredStateValidity");

  if (measured_state_.hasNaN()) {
    error_message = "Initial state contains NaN values.\n";
    return false;
  }

  if (measured_state_[MpcStateIdx::dels] < -0.5 || measured_state_[MpcStateIdx::dels] > 0.5) {
    error_message =
        "Initial steering angle is out of bounds. " + std::to_string(measured_state_[MpcStateIdx::dels]) + "\n";
    return false;
  }

  if (measured_state_[MpcStateIdx::ax] < -30.0 || measured_state_[MpcStateIdx::ax] > 30.0) {
    error_message =
        "Initial ax of the solver is out of bounds. " + std::to_string(measured_state_[MpcStateIdx::ax]) + "\n";
    return false;
  }

  if (measured_state_[MpcStateIdx::vx] < 0.0) {
    error_message = "Initial vx of the solver is negative. " + std::to_string(measured_state_[MpcStateIdx::vx]) + "\n";
    return false;
  }

  if (measured_state_[MpcStateIdx::vy] < -10.0 || measured_state_[MpcStateIdx::vy] > 10.0) {
    error_message =
        "Initial vy of the solver is out of bounds. " + std::to_string(measured_state_[MpcStateIdx::vy]) + "\n";
    return false;
  }

  if (measured_state_[MpcStateIdx::dpsi] < -5.0 || measured_state_[MpcStateIdx::dpsi] > 5.0) {
    error_message =
        "Initial dpsi of the solver is out of bounds. " + std::to_string(measured_state_[MpcStateIdx::dpsi]) + "\n";
    return false;
  }

  if (measured_state_[MpcStateIdx::n] < -4.0 || measured_state_[MpcStateIdx::n] > 4.0) {
    error_message =
        "Initial n of the solver is out of bounds. " + std::to_string(measured_state_[MpcStateIdx::n]) + "\n";
    return false;
  }

  if (measured_state_[MpcStateIdx::mu] < -1.5 || measured_state_[MpcStateIdx::mu] > 1.5) {
    error_message =
        "Initial mu of the solver is out of bounds. " + std::to_string(measured_state_[MpcStateIdx::mu]) + "\n";
    return false;
  }

  return true;
}

/*******************************************************************************
 * VisualizeMpc                                                                *
 *******************************************************************************/
void MpcController::VisualizeMpc() {
  EASY_FUNCTION(profiler::colors::Navy, "Visualise MPC");

  // ---------------------------------------------------------------------------
  // --- Populate and share prediction horizon ---------------------------------
  Eigen::MatrixXd poses_to_pub = Eigen::MatrixXd::Zero(3, N_HORIZON_ + 1);
  double psi_predict = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dpsi = 0.0;
  double dt_mpc = mpc_solver_ptr_->GetDtMpc();

  for (int i = 1; i <= N_HORIZON_; i++) {
    // psi_k+1 = psi_k + dpsi_k * dt
    dpsi = solution_states_.col(i - 1)[MpcStateIdx::dpsi];
    psi_predict = psi_predict + dpsi * dt_mpc;

    dx = solution_states_.col(i - 1)[MpcStateIdx::vx] * dt_mpc;
    dy = solution_states_.col(i - 1)[MpcStateIdx::vy] * dt_mpc;

    double cos_psi = std::cos(psi_predict);
    double sin_psi = std::sin(psi_predict);

    double rotated_dx = cos_psi * dx - sin_psi * dy;
    double rotated_dy = sin_psi * dx + cos_psi * dy;

    poses_to_pub(0, i) = poses_to_pub(0, i - 1) + rotated_dx;
    poses_to_pub(1, i) = poses_to_pub(1, i - 1) + rotated_dy;
    poses_to_pub(2, i) = psi_predict;
  }

  mpc_visualisation_node_->SharePredictionHorizon(poses_to_pub);

  // ---------------------------------------------------------------------------
  // --- Share fitted path -----------------------------------------------------
  mpc_visualisation_node_->ShareFittedPath(mpc_geometry_ptr_->GetFittedPath());

  // ---------------------------------------------------------------------------
  // --- Share reference projection --------------------------------------------
  Eigen::Vector2d ref_point = mpc_geometry_ptr_->GetNearestPoint();
  mpc_visualisation_node_->ShareReferenceProjection(ref_point[0], ref_point[1], -initial_state_[MpcStateIdx::mu],
                                                    initial_state_[MpcStateIdx::n]);
}

/*******************************************************************************
 * StopCar                                                                     *
 *******************************************************************************/
vcu_msgs::msg::CarCommand MpcController::StopCar() {
  vcu_msgs::msg::CarCommand car_command;

  car_command.a_x[0] = -15.0;
  car_command.a_x[1] = -15.0;
  car_command.a_x[2] = -15.0;

  car_command.steering_angle[0] = 0.0;
  car_command.steering_angle[1] = 0.0;
  car_command.steering_angle[2] = 0.0;

  car_command.yaw_rate[0] = 0.0;
  car_command.yaw_rate[1] = 0.0;
  car_command.yaw_rate[2] = 0.0;

  return car_command;
}

/*******************************************************************************
 * CalculatePurePursuitCommand                                                 *
 *******************************************************************************/
vcu_msgs::msg::CarCommand MpcController::CalculatePurePursuitCommand(const ControllerNode::State &state,
                                                                     const ControllerNode::Reference &reference) {
  EASY_FUNCTION(profiler::colors::Navy, "Calculate Pure Pursuit Command");

  vcu_msgs::msg::CarCommand car_command;

  // ==============================================================================
  // calculate the acceleration command
  double ax_cmd = params_.pp_kp_longitudinal * (params_.pp_fallback_speed_target - state.dx);
  double ax_max = params_.pp_max_acceleration_output;

  double ax_cmd_clamped = std::clamp(ax_cmd, -ax_max, ax_max);

  car_command.a_x[0] = car_command.a_x[1] = car_command.a_x[2] = ax_cmd_clamped;
  // ==============================================================================

  // ==============================================================================
  // find the lookahead point
  double lookahead_distance = std::clamp(params_.pp_lookahead_distance_factor * state.dx,
                                         params_.pp_lookahead_distance_min, params_.pp_lookahead_distance_max);

  // iterate through the reference to find the point closest to the car
  double min_distance = std::numeric_limits<double>::max();
  int closest_index = 0;
  int nbRefPoints = reference.size();

  for (size_t i = 0; i < nbRefPoints; i++) {
    double distance = std::hypot(reference[i].x, reference[i].y);
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }

  int lookahead_index = closest_index;
  // from the closest point, iterate through the rest of the reference points to find the one closest to the lookahead
  // distance
  for (size_t i = closest_index; i < nbRefPoints; i++) {
    double distance = std::hypot(reference[i].x, reference[i].y);
    if (distance > lookahead_distance) {
      lookahead_index = i;
      break;
    }
  }

  // check if we didn't find a lookahead point
  if (lookahead_index == closest_index) {
    controller_node_->LogWarn("PP lookahead index is the same as the closest index: %d\n", lookahead_index);
    // throw controller step exception
    throw ControllerNode::ControllerStepException("No lookahead point found in the reference.");
  }
  // ==============================================================================

  // ==============================================================================
  // calculate the steering angle
  const double x_lookahead = reference[lookahead_index].x + params_.l_r;
  const double y_lookahead = reference[lookahead_index].y;
  double steering_angle = std::atan2((params_.l_f + params_.l_r) * 2 * y_lookahead,
                                     (x_lookahead * x_lookahead + y_lookahead * y_lookahead));

  if (steering_angle < -params_.dels_abs_max || steering_angle > params_.dels_abs_max) {
    controller_node_->LogWarn("PP steering would be out of bounds: %f\n", steering_angle);
  }

  steering_angle = std::clamp(steering_angle, -params_.dels_abs_max, params_.dels_abs_max);

  car_command.steering_angle[0] = car_command.steering_angle[1] = car_command.steering_angle[2] = steering_angle;
  // ==============================================================================

  // ==============================================================================
  // yaw rate
  double yawrate = (state.dx * std::tan(steering_angle)) / (params_.l_f + params_.l_r);

  car_command.yaw_rate[0] = car_command.yaw_rate[1] = car_command.yaw_rate[2] = yawrate;
  // ==============================================================================

  return car_command;
}

/*******************************************************************************
 * LogMpc                                                                      *
 *******************************************************************************/
void MpcController::LogMpc(const double solvetime_ms, const ControllerNode::Reference &reference) {
  EASY_FUNCTION(profiler::colors::Navy, "Logging MPC");

  auto log_mpc_msg = control_msgs::msg::MpcLog();

  // Solver information
  log_mpc_msg.solvetime_ms = solvetime_ms;
  mpc_solver_ptr_->GetSolverDiagnostics(MpcSolverDiagnostic::ExitFlag, log_mpc_msg.solverflag);
  log_mpc_msg.solverstatus = solver_status_;
  if (solver_type_ == "ACADOS") {
    log_mpc_msg.solvertype = 1;
  } else {
    log_mpc_msg.solvertype = 0;
  }

  log_mpc_msg.dt_mpc = mpc_solver_ptr_->GetDtMpc();
  log_mpc_msg.consecutive_failures_limit = params_.consecutive_failures_limit;
  log_mpc_msg.enable_visualization = params_.enable_visualisation;
  log_mpc_msg.enable_logging = params_.enable_logging;
  log_mpc_msg.enable_profiling = params_.enable_profiling;
  log_mpc_msg.enable_output_interpolation = params_.enable_output_interpolation;
  log_mpc_msg.enable_delay_compensation = params_.enable_delay_compensation;
  log_mpc_msg.controller_frequency = controller_node_->GetControllerFrequency();
  log_mpc_msg.controller_timeout_reset = params_.controller_timeout_reset;
  log_mpc_msg.controller_timeout_fatal = params_.controller_timeout_fatal;

  log_mpc_msg.ax_command_standstill = params_.ax_command_standstill;
  log_mpc_msg.vel_threshold_standstill = params_.vel_threshold_standstill;

  log_mpc_msg.n_meas = measured_state_[MpcStateIdx::n];
  log_mpc_msg.mu_meas = measured_state_[MpcStateIdx::mu];
  log_mpc_msg.vx_meas = measured_state_[MpcStateIdx::vx];
  log_mpc_msg.vy_meas = measured_state_[MpcStateIdx::vy];
  log_mpc_msg.dpsi_meas = measured_state_[MpcStateIdx::dpsi];
  log_mpc_msg.ax_meas = measured_state_[MpcStateIdx::ax];
  log_mpc_msg.dels_meas = measured_state_[MpcStateIdx::dels];

  log_mpc_msg.n_init = initial_state_[MpcStateIdx::n];
  log_mpc_msg.mu_init = initial_state_[MpcStateIdx::mu];
  log_mpc_msg.vx_init = initial_state_[MpcStateIdx::vx];
  log_mpc_msg.vy_init = initial_state_[MpcStateIdx::vy];
  log_mpc_msg.dpsi_init = initial_state_[MpcStateIdx::dpsi];
  log_mpc_msg.ax_init = initial_state_[MpcStateIdx::ax];
  log_mpc_msg.dels_init = initial_state_[MpcStateIdx::dels];

  log_mpc_msg.model_mass = params_.m;
  log_mpc_msg.model_lr = params_.l_r;
  log_mpc_msg.model_lf = params_.l_f;
  log_mpc_msg.model_cd = params_.C_d;
  log_mpc_msg.model_cr = params_.C_r;
  log_mpc_msg.model_cl = params_.C_l;
  log_mpc_msg.model_b_tire = params_.B_tire;
  log_mpc_msg.model_c_tire = params_.C_tire;
  log_mpc_msg.model_d_tire = params_.D_tire;
  log_mpc_msg.model_iz = params_.Iz;
  log_mpc_msg.model_car_width = params_.car_width;
  log_mpc_msg.model_h_cg = params_.h_cg;
  log_mpc_msg.model_l_to_rear = params_.l_to_rear;
  log_mpc_msg.model_l_to_front = params_.l_to_front;

  log_mpc_msg.cost_q_ds = params_.q_ds;
  log_mpc_msg.cost_q_ds_terminal = params_.q_ds_terminal;
  log_mpc_msg.cost_q_n = params_.q_n;
  log_mpc_msg.cost_q_n_terminal = params_.q_n_terminal;
  log_mpc_msg.cost_q_mu = params_.q_mu;
  log_mpc_msg.cost_q_mu_terminal = params_.q_mu_terminal;
  log_mpc_msg.cost_r_dax = params_.r_dax;
  log_mpc_msg.cost_r_ddels = params_.r_ddels;

  log_mpc_msg.constr_vx_max = params_.vx_max;
  log_mpc_msg.constr_vx_max_terminal = params_.vx_max_terminal;
  log_mpc_msg.constr_axt_abs_max = params_.axt_abs_max;
  log_mpc_msg.constr_ayt_abs_max = params_.ayt_abs_max;
  log_mpc_msg.constr_dels_abs_max = params_.dels_abs_max;
  log_mpc_msg.constr_dax_min = params_.dax_min;
  log_mpc_msg.constr_dax_max = params_.dax_max;
  log_mpc_msg.constr_ddels_abs_max = params_.ddels_abs_max;
  log_mpc_msg.constr_trackbound_shrinkage = params_.trackbound_shrinkage;

  log_mpc_msg.mpc_dt_interp_ax = params_.mpc_dt_interp_ax;
  log_mpc_msg.mpc_dt_interp_dels = params_.mpc_dt_interp_dels;
  log_mpc_msg.mpc_dt_interp_dpsi = params_.mpc_dt_interp_dpsi;
  log_mpc_msg.tau_ax = params_.tau_ax;
  log_mpc_msg.tau_dels = params_.tau_dels;

  // Reserve memory in the std::vectors to avoid reallocations
  log_mpc_msg.x_refpath.reserve(reference.size());
  log_mpc_msg.y_refpath.reserve(reference.size());
  log_mpc_msg.vel_reference.reserve(reference.size());
  log_mpc_msg.boundary_min_reference.reserve(reference.size());
  log_mpc_msg.boundary_max_reference.reserve(reference.size());

  // Reference path given by planning
  for (int i = 0; i < reference.size(); ++i) {
    log_mpc_msg.x_refpath.push_back(reference[i].x);
    log_mpc_msg.y_refpath.push_back(reference[i].y);
  }
  // ref. velocities given by planning
  for (int i = 0; i < reference.size(); ++i) {
    log_mpc_msg.vel_reference.push_back(reference[i].velocity);
  }
  // boundaries given by planning
  for (int i = 0; i < reference.size(); ++i) {
    log_mpc_msg.boundary_min_reference.push_back(reference[i].bound_lateral_right);
  }
  for (int i = 0; i < reference.size(); ++i) {
    log_mpc_msg.boundary_max_reference.push_back(reference[i].bound_lateral_left);
  }

  // Reserve memory in the std::vectors to avoid reallocations
  log_mpc_msg.n_prediction.reserve(N_HORIZON_ + 1);
  log_mpc_msg.mu_prediction.reserve(N_HORIZON_ + 1);
  log_mpc_msg.vx_prediction.reserve(N_HORIZON_ + 1);
  log_mpc_msg.vy_prediction.reserve(N_HORIZON_ + 1);
  log_mpc_msg.dpsi_prediction.reserve(N_HORIZON_ + 1);
  log_mpc_msg.ax_prediction.reserve(N_HORIZON_ + 1);
  log_mpc_msg.dels_prediction.reserve(N_HORIZON_ + 1);
  log_mpc_msg.dax_prediction.reserve(N_HORIZON_);
  log_mpc_msg.ddels_prediction.reserve(N_HORIZON_);

  // Copy data from Eigen matrices/vectors to std::vectors
  for (int i = 0; i < N_HORIZON_ + 1; ++i) {
    log_mpc_msg.n_prediction.push_back(solution_states_(MpcStateIdx::n, i));
    log_mpc_msg.mu_prediction.push_back(solution_states_(MpcStateIdx::mu, i));
    log_mpc_msg.vx_prediction.push_back(solution_states_(MpcStateIdx::vx, i));
    log_mpc_msg.vy_prediction.push_back(solution_states_(MpcStateIdx::vy, i));
    log_mpc_msg.dpsi_prediction.push_back(solution_states_(MpcStateIdx::dpsi, i));
    log_mpc_msg.ax_prediction.push_back(solution_states_(MpcStateIdx::ax, i));
    log_mpc_msg.dels_prediction.push_back(solution_states_(MpcStateIdx::dels, i));
  }
  for (int i = 0; i < N_HORIZON_; ++i) {
    log_mpc_msg.dax_prediction.push_back(solution_inputs_(MpcInputIdx::dax, i));
    log_mpc_msg.ddels_prediction.push_back(solution_inputs_(MpcInputIdx::ddels, i));
  }

  // Reserve memory in the std::vectors to avoid reallocations
  log_mpc_msg.progress_horizon.reserve(N_HORIZON_);
  log_mpc_msg.curvature_horizon.reserve(N_HORIZON_);
  log_mpc_msg.trackbound_min_horizon.reserve(N_HORIZON_);
  log_mpc_msg.trackbound_max_horizon.reserve(N_HORIZON_);
  log_mpc_msg.vel_horizon.reserve(N_HORIZON_);

  for (int i = 0; i < N_HORIZON_; ++i) {
    log_mpc_msg.progress_horizon.push_back(progress_horizon_(i));
    log_mpc_msg.curvature_horizon.push_back(curvature_horizon_(i));
    log_mpc_msg.trackbound_min_horizon.push_back(trackbound_min_horizon_(i));
    log_mpc_msg.trackbound_max_horizon.push_back(trackbound_max_horizon_(i));
    log_mpc_msg.vel_horizon.push_back(velocity_horizon_(i));
  }

  // Share the log message
  // HEADER is populated in the Visualization Node!
  this->mpc_visualisation_node_->ShareMpcLogging(log_mpc_msg);
}

/*******************************************************************************
 * SetSolverRuntimeParams                                                      *
 *******************************************************************************/
void MpcController::SetSolverRuntimeParams() {
  bool tmp = false;

  // MPC Model Parameters
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::m, params_.m);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::l_f, params_.l_f);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::l_r, params_.l_r);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::C_r, params_.C_r);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::C_d, params_.C_d);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::C_l, params_.C_l);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::car_width, params_.car_width);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::B_tire, params_.B_tire);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::C_tire, params_.C_tire);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::D_tire, params_.D_tire);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::h_cg, params_.h_cg);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::Iz, params_.Iz);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::l_to_rear, params_.l_to_rear);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::l_to_front, params_.l_to_front);

  // MPC Cost Parameters
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::q_ds, params_.q_ds);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::q_n, params_.q_n);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::q_mu, params_.q_mu);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::r_dax, params_.r_dax);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::r_ddels, params_.r_ddels);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::q_ds_term, params_.q_ds_terminal);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::q_n_term, params_.q_n_terminal);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::q_mu_term, params_.q_mu_terminal);

  // MPC Constraint Parameters
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::vx_max, params_.vx_max);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::vx_max_term, params_.vx_max_terminal);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::axt_abs_max, params_.axt_abs_max);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::ayt_abs_max, params_.ayt_abs_max);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::dels_abs_max, params_.dels_abs_max);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::dax_min, params_.dax_min);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::dax_max, params_.dax_max);
  tmp = tmp || mpc_solver_ptr_->SetModelParameter(MpcParamIdx::ddels_abs_max, params_.ddels_abs_max);

  if (tmp) {
    controller_node_->LogWarn("Failed to set runtime parameters for the solver.");
  }
}

void MpcController::IntegratePredictedInitialState() {
  // Time step for integration
  double dt_mpc = mpc_solver_ptr_->GetDtMpc();
  const int steps = 10;
  const double dt = dt_mpc / steps;

  // Linear interpolation function
  auto interpolate = [](double start, double end, double ratio) { return start + (end - start) * ratio; };

  double time_constant_ax = params_.tau_ax;
  double time_constant_dels = params_.tau_dels;

  double rho = 1.225;
  double C_lift = params_.C_l;
  double a_front = 1.2;
  double D_tire_lat = params_.D_tire;
  double C_tire_lat = params_.C_tire;
  double B_tire_lat = params_.B_tire;
  double mass = params_.m;
  double grav = 9.81;
  double l_front = params_.l_f;
  double l_rear = params_.l_r;
  double Izz = params_.Iz;
  double C_roll = params_.C_r;
  double C_drag = params_.C_d;

  // Extract measured state
  double n = measured_state_[MpcStateIdx::n];
  double mu = measured_state_[MpcStateIdx::mu];
  double vx = measured_state_[MpcStateIdx::vx];
  double vy = measured_state_[MpcStateIdx::vy];
  double dpsi = measured_state_[MpcStateIdx::dpsi];
  double ax = measured_state_[MpcStateIdx::ax];
  double dels = measured_state_[MpcStateIdx::dels];

  // Predict future state using first order lag model
  for (int i = 0; i < steps; ++i) {
    // Calculate the interpolation ratio for the current step
    double ratio = static_cast<double>(i + 1) / steps;

    // Interpolate the reference values for acceleration and steering angle
    double a_x_ref = interpolate(solution_states_(MpcStateIdx::ax, 0), solution_states_(MpcStateIdx::ax, 1), ratio);
    double dels_ref =
        interpolate(solution_states_(MpcStateIdx::dels, 0), solution_states_(MpcStateIdx::dels, 1), ratio);
    // Interpolate the curvature
    double kappa_ref = interpolate(curvature_horizon_[0], curvature_horizon_[1], ratio);

    // Apply first order lag to acceleration and steering angle
    double a_x_pred = ax + (a_x_ref - ax) * (1 - exp(-dt / time_constant_ax));
    double dels_pred = dels + (dels_ref - dels) * (1 - exp(-dt / time_constant_dels));

    // Calculate vx_n
    double vx_n = std::fmax(vx, 1.0);

    // Calculate slip angles
    double alpha_f = atan2(vy + l_front * dpsi, vx_n) - dels_pred;
    double alpha_r = atan2(vy - l_rear * dpsi, vx_n);

    // Tire forces
    double F_downforce_per_axle = 0.5 * rho * C_lift * a_front * (vx * vx) / 2.0;
    double Fz_f = mass * grav * l_rear / (l_front + l_rear) + F_downforce_per_axle;
    double Fz_r = mass * grav * l_front / (l_front + l_rear) + F_downforce_per_axle;

    double Fy_f = -Fz_f * D_tire_lat * sin(C_tire_lat * atan(B_tire_lat * alpha_f));
    double Fy_r = -Fz_r * D_tire_lat * sin(C_tire_lat * atan(B_tire_lat * alpha_r));

    double Fx_f = mass * a_x_pred / 2.0;
    double Fx_r = mass * a_x_pred / 2.0;

    // Resistance forces
    double F_res = mass * grav * C_roll + 0.5 * rho * a_front * C_drag * (vx * vx);

    // Derivative of state w.r.t time
    double n_dot_expl_dyn = vx * sin(mu) + vy * cos(mu);
    double mu_dot_expl_dyn = dpsi - kappa_ref * (vx * cos(mu) - vy * sin(mu)) / (1 - n * kappa_ref);
    double vx_dot_expl_dyn = (Fx_f * cos(dels_pred) + Fx_r - Fy_f * sin(dels_pred) - F_res) / mass + vy * dpsi;
    double vy_dot_expl_dyn = (Fx_f * sin(dels_pred) + Fy_f * cos(dels_pred) + Fy_r) / mass - vx * dpsi;
    double dpsi_dot_expl_dyn =
        (Fx_f * sin(dels_pred) * l_front + Fy_f * cos(dels_pred) * l_front - Fy_r * l_rear) / Izz;

    // Integrate state
    n += n_dot_expl_dyn * dt;
    mu += mu_dot_expl_dyn * dt;
    vx += vx_dot_expl_dyn * dt;
    vy += vy_dot_expl_dyn * dt;
    dpsi += dpsi_dot_expl_dyn * dt;
    ax = a_x_pred;
    dels = dels_pred;
  }

  // Update predicted state
  initial_state_[MpcStateIdx::n] = n;
  initial_state_[MpcStateIdx::mu] = mu;
  initial_state_[MpcStateIdx::vx] = vx;
  initial_state_[MpcStateIdx::vy] = vy;
  initial_state_[MpcStateIdx::dpsi] = dpsi;
  initial_state_[MpcStateIdx::ax] = ax;
  initial_state_[MpcStateIdx::dels] = dels;
}

void MpcController::PrepareGeometry(ControllerNode::Reference &ref) {
  // ===========================================================================
  // === Fit Reference Path ====================================================
  EASY_BLOCK("Fit reference path", profiler::colors::DarkCyan);

  int nbRefPoints = ref.size();
  Eigen::MatrixXd path_matrix = Eigen::MatrixXd::Zero(2, nbRefPoints);
  Eigen::VectorXd left_boundary = Eigen::VectorXd::Zero(nbRefPoints);
  Eigen::VectorXd right_boundary = Eigen::VectorXd::Zero(nbRefPoints);
  Eigen::VectorXd velocity_profile = Eigen::VectorXd::Zero(nbRefPoints);
  int count = 0;

  for (size_t i = 0; i < nbRefPoints; i++) {
    double distance = std::hypot(ref[i].x, ref[i].y);
    double max_distance = params_.max_radius_path_points;

    if (distance < max_distance) {
      path_matrix(0, count) = ref[i].x;
      path_matrix(1, count) = ref[i].y;
      left_boundary[count] = ref[i].bound_lateral_left;
      right_boundary[count] = ref[i].bound_lateral_right;
      velocity_profile[count] = ref[i].velocity;
      count++;
    }
  }
  path_matrix.conservativeResize(2, count);
  left_boundary.conservativeResize(count);
  right_boundary.conservativeResize(count);
  velocity_profile.conservativeResize(count);

  if (mpc_geometry_ptr_->FitReferencePath(path_matrix, params_.mpc_plan_from_car,
                                          params_.mpc_from_car_pruning_distance)) {
    throw ControllerNode::ControllerStepException("Failed to fit the reference path");
  }

  EASY_END_BLOCK; // "Fit reference path"
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // ===========================================================================
  // === Set Raw Track Boundaries ==============================================
  EASY_BLOCK("Set track boundaries from Planning", profiler::colors::DarkCyan);

  if (mpc_geometry_ptr_->SetTrackBoundaries(right_boundary, left_boundary, path_matrix, params_.mpc_plan_from_car,
                                            params_.mpc_from_car_pruning_distance)) {
    throw ControllerNode::ControllerStepException("Failed to set the track boundaries");
  }

  EASY_END_BLOCK; // "Set track boundaries from Planning"
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // ===========================================================================
  // === Set Raw Velocity Profile ==============================================
  EASY_BLOCK("Set velocity profile from Planning", profiler::colors::DarkCyan);

  if (mpc_geometry_ptr_->SetVelocityProfile(velocity_profile, path_matrix)) {
    throw ControllerNode::ControllerStepException("Failed to set the velocity profile from planning.");
  }

  EASY_END_BLOCK; // "Set velocity profile from Planning"
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
}

void MpcController::SetInitialSolverGuess() {
  if (solver_type_ == "ACADOS") {
    EASY_BLOCK("Shifted solver solution", profiler::colors::Pink);
    // populate the shifted solver solution with the values from the previous
    // solver solution

    shifted_solver_states_.block(0, 0, N_STATE_, N_HORIZON_ - 1) =
        solution_states_.block(0, 2, N_STATE_, N_HORIZON_ - 1);
    shifted_solver_states_.col(N_HORIZON_ - 1) = solution_states_.col(N_HORIZON_);
    shifted_solver_inputs_.block(0, 0, N_INPUT_, N_HORIZON_ - 1) =
        solution_inputs_.block(0, 1, N_INPUT_, N_HORIZON_ - 1);
    shifted_solver_inputs_.col(N_HORIZON_ - 1) = solution_inputs_.col(N_HORIZON_ - 1);

    EASY_END_BLOCK; // "Shifted solver solution"

    EASY_BLOCK("Set init. guess", profiler::colors::DarkCyan);
    if (mpc_solver_ptr_->SetInitialGuess(shifted_solver_states_, shifted_solver_inputs_)) {
      std::string error_message = "Failed to set the initial guess with solver states solution size: " +
                                  std::to_string(shifted_solver_states_.size()) +
                                  "and solver inputs solution size: " + std::to_string(shifted_solver_inputs_.size()) +
                                  "\n";
      throw ControllerNode::ControllerStepException(error_message);
    }
    EASY_END_BLOCK; // "Set init. guess"
  }
  return;
}
