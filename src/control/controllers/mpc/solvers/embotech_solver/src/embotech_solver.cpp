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

#include "embotech_solver.hpp"

#ifdef __cplusplus
extern "C" {
#endif

extern solver_int32_default
embotech_adtool2forces(embotech_float *x,              // primal vars
                       embotech_float *y,              // eq. constraint multiplers
                       embotech_float *l,              // ineq. constraint multipliers
                       embotech_float *p,              // parameters
                       embotech_float *f,              // objective function (scalar)
                       embotech_float *nabla_f,        // gradient of objective function
                       embotech_float *c,              // dynamics
                       embotech_float *nabla_c,        // Jacobian of the dynamics (column major)
                       embotech_float *h,              // inequality constraints
                       embotech_float *nabla_h,        // Jacobian of inequality constraints (column major)
                       embotech_float *hess,           // Hessian (column major)
                       solver_int32_default stage,     // stage number (0 indexed)
                       solver_int32_default iteration, // iteration number of solver
                       solver_int32_default threadID   // Id of caller thread
);

#ifdef __cplusplus
}
#endif

//  --- Destructor -----------------------------------------------------------

EmbotechSolver::~EmbotechSolver() {}

// --- Methods ---------------------------------------------------------------

/**
 * Solve the optimization problem
 *
 * @return 0 in general, but 1 if fatal error (after too many consecutive
 * failures)
 *
 */
bool EmbotechSolver::Solve() {
  exitflag_ = embotech_solve(&solver_params_, &prediction_, &info_, mem_, NULL, extfunc_);

  if (exitflag_ == 1) {
    mpc_solver_status_ = MpcSolverStatus::Ok;
    num_consecutive_failures_ = 0;
  } else {
    mpc_solver_status_ = MpcSolverStatus::Warn;
    num_consecutive_failures_++;

    // set the solver output to zero
    std::fill_n(prediction_.input_horizon, N_INPUT * N_HORIZON, 0.0);
    std::fill_n(prediction_.state_horizon, N_STATE * (N_HORIZON + 1), 0.0);
  }

  if (num_consecutive_failures_ > max_consecutive_failures_) {
    mpc_solver_status_ = MpcSolverStatus::Fatal;
    return 1;
  }

  return 0;
}

/**
 * Reset the solver
 *  - Set all solutions of the MPC prediction horizon to zero
 *  - Set the exitflag to 999 (does not exist)
 *  - Set the solver status to Ready
 *
 * @return 0 if successful
 */
bool EmbotechSolver::Reset() {
  //  The prediction is set to zero
  std::fill_n(prediction_.state_horizon, N_STATE * (N_HORIZON + 1), 0.0);
  std::fill_n(prediction_.input_horizon, N_INPUT * N_HORIZON, 0.0);

  exitflag_ = 999; // flag that does not exists
  mpc_solver_status_ = MpcSolverStatus::Ready;
  return 0;
}

/**
 * Initialize the solver
 *
 * @return 0 if successful
 */
bool EmbotechSolver::Initialize() {
  mem_ = embotech_internal_mem(0);
  extfunc_ = &embotech_adtool2forces;

  exitflag_ = 999; // flag that does not exists
  return 0;
}

// --- Setters ---------------------------------------------------------------
/**
 * Set the initial state of the MPC prediction horizon
 *
 * [_x0_|_u0_|_x1_|_u1_|_..._|_xN-1_|_uN-1_|_xN_]
 *  ^^^^________________________________________
 *
 * @param initial_state The initial state of the MPC prediction horizon
 *
 * @return 0 if successful
 */
bool EmbotechSolver::SetInitialState(Eigen::VectorXd &initial_state) {
  std::memcpy(solver_params_.xinit, initial_state.data(), sizeof(solver_params_.xinit));

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
 * @param initial_guess_states The initial guess (states from x0 on) for MPC
 * prediction horizon
 * @param initial_guess_inputs The initial guess (inputs) for MPC prediction
 * horizon
 *
 * @return 0 if successful
 */
bool EmbotechSolver::SetInitialGuess(Eigen::MatrixXd &initial_guess_states, Eigen::MatrixXd &initial_guess_inputs) {
  // Check if the initial guess has the correct size
  if (initial_guess_states.cols() != (N_HORIZON + 1)) {
    std::runtime_error("The initial MPC guess for the states has the wrong cols.");
    return 1;
  }
  if (initial_guess_inputs.cols() != (N_HORIZON + 1)) {
    std::runtime_error("The initial MPC guess for the inputs has the wrong size.");
    return 1;
  }
  // Check if initial guess has the correct number of rows
  if (initial_guess_states.rows() != N_STATE) {
    std::runtime_error("The initial MPC guess for the states has the wrong rows.");
    return 1;
  }
  if (initial_guess_inputs.rows() != N_INPUT) {
    std::runtime_error("The initial MPC guess for the inputs has the wrong rows.");
    return 1;
  }

  for (size_t idx = 0; idx < (N_HORIZON + 1); idx++) {
    //  Sets the initial guess for the states
    std::memcpy(&(solver_params_.x0[idx * (N_INPUT + N_STATE + N_SLACK)]), initial_guess_states.col(idx).data(),
                sizeof(initial_guess_states.col(idx)));

    //  Sets the initial guess for the inputs
    std::memcpy(&(solver_params_.x0[idx * (N_INPUT + N_STATE + N_SLACK) + N_STATE]),
                initial_guess_inputs.col(idx).data(), sizeof(initial_guess_inputs.col(idx)));

    // Sets the initial guess for the slack variables
    if (N_SLACK > 0) {
      std::fill_n(&(solver_params_.x0[idx * (N_INPUT + N_STATE + N_SLACK) + N_STATE + N_INPUT]), N_SLACK, 0.0);
    }
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
bool EmbotechSolver::SetSolverParameter(const std::string param_name, double param_value) {
  std::runtime_error("Your parameter cannot be set. It is not a runtime parameter. \n");

  return 1;
}

/**
 * Set a solver parameter
 *
 * @param param_name Name of the parameter to set
 * @param param_value Value of the parameter to set
 *
 * @return 0 if successful
 */
bool EmbotechSolver::SetSolverParameter(const std::string param_name, int param_value) {
  std::runtime_error("Your parameter cannot be set. It is not a runtime parameter. \n");

  return 1;
}

/**
 * Set a solver parameter
 *
 * @param param_name Name of the parameter to set
 * @param param_value Value of the parameter to set
 *
 * @return 0 if successful
 */
bool EmbotechSolver::SetSolverParameter(const std::string param_name, std::string param_value) {
  std::runtime_error("Your parameter cannot be set. It is not a runtime parameter. \n");

  return 1;
}

/**
 * Set a model parameter for the whole horizon
 *
 * @param param_name Name of the parameter to set
 * @param param_value Value of the parameter to set
 *
 * @return 0 if successful
 */
bool EmbotechSolver::SetModelParameter(MpcParamIdx param_idx, double param_value) {
  // Fill the correct parameter in the solver_params_ struct
  for (int idx = 0; idx < N_HORIZON + 1; idx++) {
    solver_params_.all_parameters[idx * static_cast<int>(MpcParamIdx::N_PARAMS) + static_cast<int>(param_idx)] =
        param_value;
  }
  return 0;
}

/**
 * Set a model parameter for the whole horizon
 *
 * @param param_name Name of the parameter to set
 * @param param_value Value of the parameter to set (double vector)
 *
 * @return 0 if successful
 */
bool EmbotechSolver::SetModelParameter(MpcParamIdx param_idx, Eigen::VectorXd &param_value) {
  // check if the parameter has the correct size
  if (param_value.size() != static_cast<size_t>(N_HORIZON)) {
    std::runtime_error("The parameter has the wrong size.");
    return 1;
  }

  for (int idx = 0; idx < N_HORIZON; idx++) {
    solver_params_.all_parameters[idx * static_cast<int>(MpcParamIdx::N_PARAMS) + static_cast<int>(param_idx)] =
        param_value[idx];
  }
  solver_params_.all_parameters[N_HORIZON * static_cast<int>(MpcParamIdx::N_PARAMS) + static_cast<int>(param_idx)] =
      0.0;
  return 0;
}

/**
 * Set a model parameter for a specific stage
 *
 * @param param_name Name of the parameter to set
 * @param param_value Value of the parameter to set
 * @param stage_idx Index of the stage to set the parameter for
 *
 * @return 0 if successful
 */
bool EmbotechSolver::SetModelParameter(MpcParamIdx param_idx, double param_value, int stage_idx) {
  // check if the stage index is within the horizon
  if (stage_idx > N_HORIZON) {
    std::runtime_error("The stage index is out of bounds.");
    return 1;
  }

  solver_params_.all_parameters[stage_idx * static_cast<int>(MpcParamIdx::N_PARAMS) + static_cast<int>(param_idx)] =
      param_value;

  return 0;
}

// --- Getters ---------------------------------------------------------------
/**
 * Get the progress horizon (shifted by one time step) to then interpolate for
 * curavture As we need N-dimensional curvature, we need to return N values for
 * progress
 *
 * @param progress_horizon The progress horizon, returned by reference
 *
 * @return 0 if successful
 */
bool EmbotechSolver::GetShiftedProgressHorizon(Eigen::VectorXd &progress_horizon,
                                               const Eigen::VectorXd &curvature_horizon) {
  // check if the progress horizon has the correct size
  if (progress_horizon.size() != static_cast<size_t>(N_HORIZON)) {
    return 1;
  }
  if (curvature_horizon.size() != static_cast<size_t>(N_HORIZON)) {
    return 1;
  }

  double s_temp = 0.0;
  double s_dot = 0.0;
  double vy = 0.0;
  double vx = 0.0;
  double mu = 0.0;
  double n = 0.0;

  for (size_t idx = 1; idx < N_HORIZON + 1; idx++) {
    // Get the states form the output struct
    vx = prediction_.state_horizon[idx * N_STATE + MpcStateIdx::vx];
    vy = prediction_.state_horizon[idx * N_STATE + MpcStateIdx::vy];
    mu = prediction_.state_horizon[idx * N_STATE + MpcStateIdx::mu];
    n = prediction_.state_horizon[idx * N_STATE + MpcStateIdx::n];

    //  Get the progress
    s_dot = (vx * cos(mu) - vy * sin(mu)) / (1 - n * curvature_horizon[idx - 1]);
    s_temp = s_temp + s_dot * dt_mpc_;

    progress_horizon[idx - 1] = s_temp;
  }

  return 0;
}

/**
 * Get the solution of the MPC, i.e. the *whole* prediction horizon
 * [_x0_|_x1_|_..._|_xN_] & [_u0_|_u1_|_..._|_uN-1_]
 *  ^^^^^^^^^^^^^^^^^^^^^____^^^^^^^^^^^^^^^^^^^^^^^
 * The states and inputs will be put in the above order!
 *
 * @param solution_states The states solution of the MPC Optimization problem
 * @param solution_inputs The inputs solution of the MPC Optimization problem
 *
 * @return 0 if successful
 */
bool EmbotechSolver::GetSolverSolution(Eigen::MatrixXd &solution_states, Eigen::MatrixXd &solution_inputs) {
  solution_states.resize(N_STATE, N_HORIZON + 1);
  solution_inputs.resize(N_INPUT, N_HORIZON);

  for (size_t i = 0; i < (N_HORIZON + 1); ++i) {
    std::copy_n( //[ x0 | ... | xN-1 | xN ]
        &(prediction_.state_horizon[(i)*N_STATE]), N_STATE, solution_states.col(i).data());

    if (i < N_HORIZON) {
      std::copy_n( //[ u0 | ... | uN-1 ]
          &(prediction_.input_horizon[(i)*N_INPUT]), N_INPUT, solution_inputs.col(i).data());
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
bool EmbotechSolver::GetSolverSolutionState(Eigen::VectorXd &solution_state, const int idx) {
  std::copy_n(&prediction_.state_horizon[idx * N_STATE], N_STATE, solution_state.data());
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
bool EmbotechSolver::GetSolverSolutionInput(Eigen::VectorXd &solution_input, const int idx) {
  std::copy_n(&prediction_.input_horizon[idx * N_INPUT], N_INPUT, solution_input.data());
}

/**
 * Get the solver diagnostics of type double
 *
 * @param diagnostics_type The type of diagnostics to get, specified as a string
 * @param diagnostic_value The value of the diagnostics, returned by reference
 *
 * @return 0 if successful
 */
bool EmbotechSolver::GetSolverDiagnostics(const std::string diagnostics_type, double &diagnostic_value) {
  if (diagnostics_type == "MpcSolverDiagnosticSolveTimeMS") {
    diagnostic_value = info_.solvetime * 1000;
  } else if (diagnostics_type == "MpcSolverDiagnosticEqResidual") {
    diagnostic_value = info_.res_eq;
  } else if (diagnostics_type == "MpcSolverDiagnosticIneqResidual") {
    diagnostic_value = info_.res_ineq;
  } else if (diagnostics_type == "MpcSolverDiagnosticStatResidual") {
    diagnostic_value = info_.rsnorm;
  } else if (diagnostics_type == "MpcSolverDiagnosticCompResidual") {
    diagnostic_value = info_.rcompnorm;
  } else if (diagnostics_type == "MpcSolverDiagnosticCostValue") {
    diagnostic_value = info_.pobj;
  } else {
    std::runtime_error("The requested diagnostics type (enum) is not available.");
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
bool EmbotechSolver::GetSolverDiagnostics(const std::string diagnostics_type, int &diagnostic_value) {
  if (diagnostics_type == "MpcSolverDiagnosticExitFlag") {
    diagnostic_value = exitflag_;
  } else if (diagnostics_type == "MpcSolverDiagnosticSolverStatus") {
    diagnostic_value = mpc_solver_status_;
  } else if (diagnostics_type == "MpcSolverDiagnosticNlpIterations") {
    diagnostic_value = info_.it;
  } else {
    std::runtime_error("The requested diagnostics type (enum) is not available.");
    return 1;
  }
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
bool EmbotechSolver::GetSolverDiagnostics(const MpcSolverDiagnostic diagnostics_type, double &diagnostic_value) {
  switch (diagnostics_type) {
  case MpcSolverDiagnostic::SolveTimeMS:
    diagnostic_value = info_.solvetime * 1000;
    break;

  case MpcSolverDiagnostic::EqResidual:
    diagnostic_value = info_.res_eq;
    break;

  case MpcSolverDiagnostic::IneqResidual:
    diagnostic_value = info_.res_ineq;
    break;

  case MpcSolverDiagnostic::StatResidual:
    diagnostic_value = info_.rsnorm;
    break;

  case MpcSolverDiagnostic::CompResidual:
    diagnostic_value = info_.rcompnorm;
    break;

  case MpcSolverDiagnostic::CostValue:
    diagnostic_value = info_.pobj;
    break;

  default:
    std::runtime_error("The requested diagnostics type (enum) is not available.");
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
bool EmbotechSolver::GetSolverDiagnostics(const MpcSolverDiagnostic diagnostics_type, int &diagnostic_value) {
  switch (diagnostics_type) {
  case MpcSolverDiagnostic::ExitFlag:
    diagnostic_value = exitflag_;
    break;

  case MpcSolverDiagnostic::SolverStatus:
    diagnostic_value = mpc_solver_status_;
    break;

  case MpcSolverDiagnostic::NlpIterations:
    diagnostic_value = info_.it;
    break;

  default:
    std::runtime_error("The requested diagnostics type (enum) is not available.");
    return 1;
  }
  return 0;
}

/**
 * Get the status of the solver as a string
 *
 * @return The status of the solver as a string
 *
 */
std::string EmbotechSolver::GetSolverStatusString() {
  std::string return_string;

  // depending on the solver status, append the corresponding string
  switch (mpc_solver_status_) {
  case MpcSolverStatus::Ready:
    return_string = "EMBOTECH Solver READY: ";
    break;
  case MpcSolverStatus::Ok:
    return_string = "EMBOTECH Solver OK: ";
    break;
  case MpcSolverStatus::Warn:
    return_string = "EMBOTECH Solver WARN: ";
    break;
  case MpcSolverStatus::Fatal:
    return_string = "EMBOTECH Solver FATAL: ";
    break;
  default:
    return_string = "EMBOTECH Solver UNKNOWN: ";
    break;
  }

  // append the exitflag meaning to the string
  switch (exitflag_) {
  case 2:
    return_string += "Specified timeout set for the solver execution has been reached";
    break;
  case 3:
    return_string += "Solver execution has been terminated as the termination parameter has been set by the user";
    break;
  case 0:
    return_string += "Maximum number of iterations reached";
    break;
  case -1:
    return_string += "Infeasible problem";
    break;
  case -2:
    return_string += "Out of memory";
    break;
  case -3:
    return_string += "Deprecated";
    break;
  case -4:
    return_string += "Wrong number of inequalities input to solver";
    break;
  case -5:
    return_string += "Error occurred during matrix factorization";
    break;
  case -6:
    return_string += "NaN or INF";
    break;
  case -7:
    return_string += "The solver could not proceed";
    break;
  case -8:
    return_string += "The internal QP solver could not proceed";
    break;
  case -9:
    return_string += "The internal QP solver could not proceed";
    break;
  case -10:
    return_string += "NaN or INF";
    break;
  case -11:
    return_string += "Invalid values in problem parameters";
    break;
  case -12:
    return_string += "Specified timeout set for the solver was too small to find any point";
    break;
  case -13:
    return_string += "An error occurred in the solver while performing a linesearch";
    break;
  case -99:
    return_string += "Lock mechanism failure";
    break;
  case -100:
    return_string += "License error";
    break;
  case -101:
    return_string += "Invalid memory error";
    break;
  case -102:
    return_string += "Invalid number of threads specified";
    break;
  case -200:
    return_string += "Invalid return value (<= -100) of external functions occurred";
    break;
  case -201:
    return_string += "a negative return value ret from external function evaluation occurred";
    break;
  default:
    return_string += "Unknown error";
    break;
  }

  return return_string;
}
