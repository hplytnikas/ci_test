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

#include "acados_solver/acados_solver.hpp"

/**
 * Destructor of the AcadosSolver class
 *
 */
AcadosSolver::~AcadosSolver() {
  // free solver
  int status = veh_dynamics_ode_acados_free(this->acados_ocp_capsule_);
  if (status) {
    printf("veh_dynamics_ode_acados_free() returned status %d. \n", status);
  }
  // free solver capsule
  status = veh_dynamics_ode_acados_free_capsule(this->acados_ocp_capsule_);
  if (status) {
    printf("veh_dynamics_ode_acados_free_capsule() returned status %d. \n", status);
  }
}

/**
 * Solve the optimization problem
 *
 * @return 0 in general, but 1 if fatal error (after too many consecutive
 * failures)
 *
 */
bool AcadosSolver::Solve() {
  // Calling the acados solver assuming all is set
  int status = veh_dynamics_ode_acados_solve(this->acados_ocp_capsule_);

  // Set the solver status
  switch (status) {
  case ACADOS_SUCCESS:
    // The solver has converged
    mpc_solver_status_ = MpcSolverStatus::Ok;
    num_consecutive_failures_ = 0;
    break;

  case ACADOS_MAXITER:
    // The solver has failed
    mpc_solver_status_ = MpcSolverStatus::Warn;
    num_consecutive_failures_++;
    break;
  case ACADOS_NAN_DETECTED:
    // The solver has failed
    mpc_solver_status_ = MpcSolverStatus::Warn;
    num_consecutive_failures_++;
    break;
  case ACADOS_MINSTEP:
    // The solver has failed
    mpc_solver_status_ = MpcSolverStatus::Warn;
    num_consecutive_failures_++;
    break;
  case ACADOS_QP_FAILURE:
    // The solver has failed
    mpc_solver_status_ = MpcSolverStatus::Warn;
    num_consecutive_failures_++;
    break;

  case ACADOS_READY:
    mpc_solver_status_ = MpcSolverStatus::Ready;
    num_consecutive_failures_ = 0;
    break;
  default:
    mpc_solver_status_ = MpcSolverStatus::Fatal;
    return 1;
  }

  if (num_consecutive_failures_ > max_consecutive_failures_) {
    mpc_solver_status_ = MpcSolverStatus::Fatal;
    return 1;
  }

  return 0;
}

/**
 * Reset the solver
 *  - Set all states of the MPC prediction horizon to zero
 *  - Set all inputs pf the MPC prediction horizon to zero
 *
 * @return 0 if successful
 */
bool AcadosSolver::Reset() {
  // Reset the solver
  // TODO(_): Reset the capsule?

  // temporary storage vectors filled with zeros (eigen does this by default)
  Eigen::VectorXd x_temp;
  x_temp.resize(NX);
  // set all values to zero
  x_temp.setZero();

  Eigen::VectorXd u_temp;
  u_temp.resize(NU);
  // set all values to zero
  u_temp.setZero();

  for (int i = 0; i <= N_HOR; i++) {
    // set states at stage i
    ocp_nlp_out_set(this->nlp_config_, this->nlp_dims_, this->nlp_out_, i, "x", x_temp.data());

    if (i < N_HOR) {
      // set inputs at stage i
      ocp_nlp_out_set(this->nlp_config_, this->nlp_dims_, this->nlp_out_, i, "u", u_temp.data());
    }
  }
  return 0;
}

/**
 * Initialize the solver by creating the necessary acados strctures
 *
 * @return 0 if successful
 */
bool AcadosSolver::Initialize() {
  // Initialize the solver
  this->acados_ocp_capsule_ = veh_dynamics_ode_acados_create_capsule();

  // Fill the capsule
  if (veh_dynamics_ode_acados_create_with_discretization(this->acados_ocp_capsule_, N_HOR, NULL)) {
    return 1;
  }

  // Get the pointers to the acados objects
  this->nlp_config_ = veh_dynamics_ode_acados_get_nlp_config(this->acados_ocp_capsule_);
  this->nlp_dims_ = veh_dynamics_ode_acados_get_nlp_dims(this->acados_ocp_capsule_);
  this->nlp_in_ = veh_dynamics_ode_acados_get_nlp_in(this->acados_ocp_capsule_);
  this->nlp_out_ = veh_dynamics_ode_acados_get_nlp_out(this->acados_ocp_capsule_);
  this->nlp_solver_ = veh_dynamics_ode_acados_get_nlp_solver(this->acados_ocp_capsule_);
  this->nlp_opts_ = veh_dynamics_ode_acados_get_nlp_opts(this->acados_ocp_capsule_);

  // Iterate through index array of initial state to set indices
  for (int ii = 0; ii < NBX0; ii++) {
    this->idxbx0_[ii] = ii;
  }
  ocp_nlp_constraints_model_set(this->nlp_config_, this->nlp_dims_, this->nlp_in_, 0, "idxbx", this->idxbx0_);

  return 0;
}

/**
 * Set the initial state of the MPC prediction horizon @n
 *
 * [_x0_|_u0_|_x1_|_u1_|_..._|_xN-1_|_uN-1_|_xN_]
 *  ^^^^________________________________________
 *
 * @param initial_state The initial state of the MPC prediction horizon
 *
 * @return 0 if successful
 */
bool AcadosSolver::SetInitialState(Eigen::VectorXd &initial_state) {
  // cehck size of initial state
  if (initial_state.size() != NX) {
    return 1;
  }
  // Set the initial state in the acados capsule using upper and lower
  // constraint bounds
  ocp_nlp_constraints_model_set(this->nlp_config_, this->nlp_dims_, this->nlp_in_, 0, "lbx", initial_state.data());
  ocp_nlp_constraints_model_set(this->nlp_config_, this->nlp_dims_, this->nlp_in_, 0, "ubx", initial_state.data());
  ocp_nlp_out_set(this->nlp_config_, this->nlp_dims_, this->nlp_out_, 0, "x", initial_state.data());

  return 0;
}

/**
 * Set the initial guess for the whole MPC prediction horizon @n
 *
 * [_x0_|_u0_|_x1_|_u1_|_..._|_xN-1_|_uN-1_|_xN_]
 *  _____^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 * It is important to put the control inputs and states in the correct order!
 *
 * @param initial_guess_states The initial guess (states from x1 on) for MPC
 * prediction horizon
 * @param initial_guess_inputs The initial guess (inputs) for MPC prediction
 * horizon
 *
 * @return 0 if successful
 */
bool AcadosSolver::SetInitialGuess(Eigen::MatrixXd &initial_guess_states, Eigen::MatrixXd &initial_guess_inputs) {
  // Check if the initial guess has the correct size for horizon
  if (initial_guess_states.cols() != static_cast<size_t>(N_HOR)) {
    return 1;
  }
  if (initial_guess_inputs.cols() != static_cast<size_t>(N_HOR)) {
    return 1;
  }
  // Check if the initial guess has the correct size for states and inputs
  if (initial_guess_states.rows() != static_cast<size_t>(NX)) {
    return 1;
  }
  if (initial_guess_inputs.rows() != static_cast<size_t>(NU)) {
    return 1;
  }

  // Set the initial guess in the acados capsule
  for (int i = 0; i < N_HOR; i++) {
    // set states at stage i+1
    ocp_nlp_out_set(this->nlp_config_, this->nlp_dims_, this->nlp_out_, i + 1, "x", initial_guess_states.col(i).data());

    // set inputs at stage i
    ocp_nlp_out_set(this->nlp_config_, this->nlp_dims_, this->nlp_out_, i, "u", initial_guess_inputs.col(i).data());
  }

  return 0;
}

/**
 * Set a solver parameter
 *
 * @param param_name Name of the parameter to set
 * @param param_value Value of the parameter to set
 *
 * @return 0 if successful
 */
bool AcadosSolver::SetSolverParameter(const std::string param_name, double param_value) {
  // Set the solver parameter
  if (param_name == "tol_stat") {
    ocp_nlp_solver_opts_set(this->nlp_config_, this->nlp_opts_, "tol_stat", &param_value);
  } else if (param_name == "tol_eq") {
    ocp_nlp_solver_opts_set(this->nlp_config_, this->nlp_opts_, "tol_eq", &param_value);
  } else if (param_name == "tol_ineq") {
    ocp_nlp_solver_opts_set(this->nlp_config_, this->nlp_opts_, "tol_ineq", &param_value);
  } else if (param_name == "tol_comp") {
    ocp_nlp_solver_opts_set(this->nlp_config_, this->nlp_opts_, "tol_comp", &param_value);
  } else {
    return 1;
  }
  return 0;
}

/**
 * Set a solver parameter
 *
 * @param param_name Name of the parameter to set
 * @param param_value Value of the parameter to set
 *
 * @return 0 if successful
 */
bool AcadosSolver::SetSolverParameter(const std::string param_name, int param_value) {
  // Set the solver parameter
  if (param_name == "qp_warm_start") {
    ocp_nlp_solver_opts_set(this->nlp_config_, this->nlp_opts_, "qp_warm_start", &param_value);
  } else if (param_name == "nlp_max_iter") {
    ocp_nlp_solver_opts_set(this->nlp_config_, this->nlp_opts_, "max_iter", &param_value);
  } else if (param_name == "rti_phase") {
    ocp_nlp_solver_opts_set(this->nlp_config_, this->nlp_opts_, "rti_phase", &param_value);
  } else {
    return 1;
  }
  return 0;
}

/**
 * Set a solver parameter
 *
 * @param param_name Name of the parameter to set
 * @param param_value Value of the parameter to set
 *
 * @return 0 if successful
 */
bool AcadosSolver::SetSolverParameter(const std::string param_name, std::string param_value) {
  // Set the solver parameter
  return 1;
}

/**
 * @brief Set double model parameter defined by its param index
 *
 * @param param_idx: the param index of the parameter
 * @param param_value: the value of the parameter
 *
 * @return 0 (false) if successful, 1 (true) otherwise
 */
bool AcadosSolver::SetModelParameter(MpcParamIdx param_idx, double param_value) {
  // int idx = (int)param_idx;
  // use static_cast instead of C-style cast
  int idx = static_cast<int>(param_idx);
  for (int i = 0; i <= N_HOR; i++) {
    veh_dynamics_ode_acados_update_params_sparse(this->acados_ocp_capsule_, i, &idx, &param_value, 1);
  }
  return 0;
}

/**
 * @brief Set vector of double model parameter defined by its param index
 *
 * @param param_idx: the param index of the parameter
 * @param param_value: the value of the parameter (a double eigen vector)
 *
 * @return 0 (false) if successful, 1 (true) otherwise
 */
bool AcadosSolver::SetModelParameter(MpcParamIdx param_idx, Eigen::VectorXd &param_value) {
  if (param_value.size() != static_cast<size_t>(N_HOR)) {
    return 1;
  }
  int idx = static_cast<int>(param_idx);
  for (int i = 0; i < N_HOR; i++) {
    veh_dynamics_ode_acados_update_params_sparse(this->acados_ocp_capsule_, i, &idx, &param_value[i], 1);
  }
  // veh_dynamics_ode_acados_update_params_sparse(this->acados_ocp_capsule_, N_HOR, &idx, &param_value[N_HOR - 1], 1);
  return 0;
}

/**
 * @brief Set double model parameter defined by its param index for a specific stage
 *
 * @param param_idx: the param index of the parameter
 * @param param_value: the value of the parameter
 * @param stage_idx: the index of the stage
 *
 * @return 0 (false) if successful, 1 (true) otherwise
 */
bool AcadosSolver::SetModelParameter(MpcParamIdx param_idx, double param_value, int stage_idx) {
  if (stage_idx < 0 || stage_idx >= N_HOR) {
    return 1;
  }
  // int idx = (int)param_idx;
  // use static_cast instead of C-style cast
  int idx = static_cast<int>(param_idx);
  veh_dynamics_ode_acados_update_params_sparse(this->acados_ocp_capsule_, stage_idx, &idx, &param_value, 1);
  return 0;
}

/**
 * Get the progress horizon (shifted by one time step) to then interpolate for
 * curavture As we need N-dimensional curvature, we need to return N values for
 * progress
 *
 * @param progress_horizon The progress horizon, returned by reference
 *
 * @return 0 if successful
 */
bool AcadosSolver::GetShiftedProgressHorizon(Eigen::VectorXd &progress_horizon,
                                             const Eigen::VectorXd &curvature_horizon) {
  // TODO(_): Check if there is a faster way to do it - maybe in the MPC Controller class using Eigen's .block()
  // function
  //  Check size of progress horizon
  if (progress_horizon.size() != static_cast<size_t>(N_HOR)) {
    return 1;
  }
  if (curvature_horizon.size() != static_cast<size_t>(N_HOR)) {
    return 1;
  }
  Eigen::VectorXd x_temp(NX);
  double s_temp = 0.0;
  double s_dot = 0.0;
  double vy = 0.0;
  double vx = 0.0;
  double mu = 0.0;
  double n = 0.0;

  // Get the progress horizon
  for (int idx = 0; idx < N_HOR; idx++) {
    // Integrate the ODE froward to get progress horizon
    // The ODE for s is:
    // ds/dt = (vx * cos(mu) - vy * sin(mu)) / (1 - n * curv)
    ocp_nlp_out_get(this->nlp_config_, this->nlp_dims_, this->nlp_out_, idx, "x", x_temp.data());
    vx = x_temp[MpcStateIdx::vx];
    mu = x_temp[MpcStateIdx::mu];
    n = x_temp[MpcStateIdx::n];
    vy = x_temp[MpcStateIdx::vy];
    s_dot = (vx * cos(mu) - vy * sin(mu)) / (1 - n * curvature_horizon[idx]);
    s_temp = s_temp + s_dot * dt_mpc_;

    // progress s
    progress_horizon[idx] = s_temp;
  }
  return 0;
}

/**
 * Get the solution of the MPC, i.e. the *whole* prediction horizon
 * [_x0_|_x1_|_..._|_xN-1_|_xN_] & [_u0_|_u1_|_..._|_uN-1_]
 *  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^____^^^^^^^^^^^^^^^^^^^^^^^
 * The states and inputs will be put in the above order.
 *
 * Important:
 * The inputs solution_states and solution_inputs will be erased and resized to the correct
 * size!!
 *
 * @param solution_states The states solution of the MPC Optimization problem
 * @param solution_inputs The inputs solution of the MPC Optimization problem
 *
 * @return 0 if successful
 */
bool AcadosSolver::GetSolverSolution(Eigen::MatrixXd &solution_states, Eigen::MatrixXd &solution_inputs) {
  solution_states.resize(NX, N_HOR + 1);
  solution_inputs.resize(NU, N_HOR);

  // get the prediction in the right format

  for (int i = 0; i <= N_HOR; i++) {
    // get states at stage i
    ocp_nlp_out_get(this->nlp_config_, this->nlp_dims_, this->nlp_out_, i, "x", solution_states.col(i).data());

    if (i < N_HOR) {
      // get inputs at stage i only for up to second last stage
      ocp_nlp_out_get(this->nlp_config_, this->nlp_dims_, this->nlp_out_, i, "u", solution_inputs.col(i).data());
    }
  }
  return 0;
}

/**
 * Get the state solution of the MPC problem at a specific stage
 *
 * @param solution_state The state solution at a specific stage
 * @param idx The index of the stage
 *
 * @return 0 if successful
 */
bool AcadosSolver::GetSolverSolutionState(Eigen::VectorXd &solution_state, const int idx) {
  // Check size of solution state
  if (solution_state.size() != static_cast<size_t>(NX)) {
    return 1;
  }
  // Get the state solution at a specific stage
  ocp_nlp_out_get(this->nlp_config_, this->nlp_dims_, this->nlp_out_, idx, "x", solution_state.data());
  return 0;
}

/**
 * Get the input solution of the MPC problem at a specific stage
 *
 * @param solution_input The input solution at a specific stage
 * @param idx The index of the stage
 *
 * @return 0 if successful
 */
bool AcadosSolver::GetSolverSolutionInput(Eigen::VectorXd &solution_input, const int idx) {
  // Check size of solution input
  if (solution_input.size() != static_cast<size_t>(NU)) {
    return 1;
  }
  // Get the input solution at a specific stage
  ocp_nlp_out_get(this->nlp_config_, this->nlp_dims_, this->nlp_out_, idx, "u", solution_input.data());
  return 0;
}

/**
 * Get the solver diagnostics of type double
 *
 * @param diagnostics_type The type of diagnostics to get, specified as an enum
 * @param diagnostic_value The value of the diagnostics, returned by reference
 *
 * @return 0 if successful
 */
bool AcadosSolver::GetSolverDiagnostics(const MpcSolverDiagnostic diagnostics_type, double &diagnostic_value) {
  // Get the solver diagnostics

  switch (diagnostics_type) {
  case SolveTimeMS:
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "time_tot", &diagnostic_value);
    diagnostic_value = diagnostic_value * 1e3;
    break;

  case StatResidual:
    ocp_nlp_eval_residuals(this->nlp_solver_, this->nlp_in_, this->nlp_out_);
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "res_stat", &diagnostic_value);
    break;

  case EqResidual:
    ocp_nlp_eval_residuals(this->nlp_solver_, this->nlp_in_, this->nlp_out_);
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "res_eq", &diagnostic_value);
    break;

  case IneqResidual:
    ocp_nlp_eval_residuals(this->nlp_solver_, this->nlp_in_, this->nlp_out_);
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "res_ineq", &diagnostic_value);
    break;

  case CompResidual:
    ocp_nlp_eval_residuals(this->nlp_solver_, this->nlp_in_, this->nlp_out_);
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "res_comp", &diagnostic_value);
    break;

  case CostValue:
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "cost_value", &diagnostic_value);
    break;

  default:
    return 1;
  }
  return 0;
}

/**
 * Get the solver diagnostics of type int
 *
 * @param diagnostics_type The type of diagnostics to get, specified as an enum
 * @param diagnostic_value The value of the diagnostics, returned by reference
 *
 * @return 0 if successful
 */
bool AcadosSolver::GetSolverDiagnostics(const MpcSolverDiagnostic diagnostics_type, int &diagnostic_value) {
  // Get the solver diagnostics
  switch (diagnostics_type) {
  case NlpIterations:
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "sqp_iter", &diagnostic_value);
    break;

  case ExitFlag:
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "status", &diagnostic_value);
    break;

  case SolverStatus:
    diagnostic_value = mpc_solver_status_;
    break;

  default:
    return 1;
  }
  return 0;
}

/**
 * Get the solver diagnostics of type double
 *
 * @param diagnostics_type The type of diagnostics to get, specified as a string
 * @param diagnostic_value The value of the diagnostics, returned by reference
 *
 * @return 0 if successful
 */
bool AcadosSolver::GetSolverDiagnostics(const std::string diagnostics_type, double &diagnostic_value) {
  // Get the solver diagnostics
  if (diagnostics_type == "solve_time_ms") {
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "time_tot", &diagnostic_value);
    diagnostic_value = diagnostic_value * 1e3;
  } else if (diagnostics_type == "time_qp_sol_ms") {
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "time_qp_sol", &diagnostic_value);
    diagnostic_value = diagnostic_value * 1e3;
  } else {
    return 1;
  }
  return 0;
}

/**
 * Get the solver diagnostics of type int
 *
 * @param diagnostics_type The type of diagnostics to get, specified as a string
 * @param diagnostic_value The value of the diagnostics, returned by reference
 *
 * @return 0 if successful
 */
bool AcadosSolver::GetSolverDiagnostics(const std::string diagnostics_type, int &diagnostic_value) {
  // Get the solver diagnostics
  if (diagnostics_type == "sqp_iter") {
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "sqp_iter", &diagnostic_value);
  } else if (diagnostics_type == "sqp_status") {
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "status", &diagnostic_value);
  } else if (diagnostics_type == "qp_iter") {
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "qp_iter", &diagnostic_value);
  } else if (diagnostics_type == "qp_status") {
    ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "qp_status", &diagnostic_value);
  } else {
    return 1;
  }
  return 0;
}

/**
 * Get the solver status as a string
 *
 * For HPIPM (QP solver) the status is one of the following:
 *  SUCCESS, // found solution satisfying accuracy tolerance
 *	MAX_ITER, // maximum iteration number reached
 *	MIN_STEP, // minimum step length reached
 *	NAN_SOL, // NaN in solution detected
 *	INCONS_EQ, // unconsistent equality constraints
 *
 * @return The solver status as a string
 */
std::string AcadosSolver::GetSolverStatusString() {
  std::string return_string;

  // depending on the solver status, append the corresponding string
  switch (mpc_solver_status_) {
  case MpcSolverStatus::Ready:
    return_string = "ACADOS Solver READY: ";
    break;
  case MpcSolverStatus::Ok:
    return_string = "ACADOS Solver OK: ";
    break;
  case MpcSolverStatus::Warn:
    return_string = "ACADOS Solver WARN: ";
    break;
  case MpcSolverStatus::Fatal:
    return_string = "ACADOS Solver FATAL: ";
    break;
  default:
    return_string = "ACADOS Solver UNKNOWN: ";
    break;
  }

  // append string corresponding to the NLP status
  int nlp_exitflag;
  ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "status", &nlp_exitflag);

  switch (nlp_exitflag) {
  case ACADOS_SUCCESS:
    return_string += "(NLP) SUCCESS";
    break;
  case ACADOS_MAXITER:
    return_string += "(NLP) MAXITER";
    break;
  case ACADOS_NAN_DETECTED:
    return_string += "(NLP) NAN_DETECTED";
    break;
  case ACADOS_MINSTEP:
    return_string += "(NLP) MINSTEP";
    break;
  case ACADOS_QP_FAILURE:
    return_string += "(NLP) QP_FAILURE";
    break;
  case ACADOS_READY:
    return_string += "(NLP) READY";
    break;
  default:
    return_string += "(NLP) UNKNOWN";
    break;
  }

  // append string corresponding to the QP status
  int qp_exitflag;
  ocp_nlp_get(this->nlp_config_, this->nlp_solver_, "qp_status", &qp_exitflag);

  switch (qp_exitflag) {
  case 0:
    return_string += " (QP) SUCCESS";
    break;
  case 1:
    return_string += " (QP) MAXITER";
    break;
  case 2:
    return_string += " (QP) MINSTEP";
    break;
  case 3:
    return_string += " (QP) NAN_SOL";
    break;
  case 4:
    return_string += " (QP) INCONS_EQ";
    break;
  default:
    return_string += " (QP) UNKNOWN";
    break;
  }

  return return_string;
}
