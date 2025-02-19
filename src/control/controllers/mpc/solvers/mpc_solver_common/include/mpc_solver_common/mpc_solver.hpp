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

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

enum MpcSolverStatus { Ok = 0, Warn = 1, Fatal = 2, Ready = 3 };

enum MpcSolverDiagnostic {
  SolverStatus = 0,
  ExitFlag = 1,
  NlpIterations = 2,
  SolveTimeMS = 3,
  EqResidual = 4,
  IneqResidual = 5,
  StatResidual = 6,
  CompResidual = 7,
  CostValue = 8
};

// State Indices for the MPC problem
enum MpcStateIdx { n = 0, mu = 1, vx = 2, vy = 3, dpsi = 4, ax = 5, dels = 6 };

// Input Indices for the MPC problem
enum MpcInputIdx { dax = 0, ddels = 1 };

// the following struct has to be equivalent to the indices assigned in the solver generation!
typedef enum MpcParamIdx {
  m = 0,
  Iz = 1,
  h_cg = 2,
  l_f = 3,
  l_r = 4,
  l_to_rear = 5,
  l_to_front = 6,
  car_width = 7,
  C_r = 8,
  C_d = 9,
  C_l = 10,
  B_tire = 11,
  C_tire = 12,
  D_tire = 13,
  curv = 14,
  q_ds = 15,
  q_n = 16,
  q_mu = 17,
  r_dax = 18,
  r_ddels = 19,
  q_ds_term = 20,
  q_n_term = 21,
  q_mu_term = 22,
  vx_max = 23,
  vx_max_term = 24,
  axt_abs_max = 25,
  ayt_abs_max = 26,
  dels_abs_max = 27,
  dax_min = 28,
  dax_max = 29,
  ddels_abs_max = 30,
  trackbound_min = 31,
  trackbound_max = 32,
  N_PARAMS = 33
};

class MpcSolver {
protected:
  int num_consecutive_failures_{0};
  int max_consecutive_failures_;
  std::string solver_name_;
  // Discretization time step for the MPC solver
  // TODO(_): Find a better way to define this
  const double dt_mpc_ = 0.025;

public:
  /**
   * @brief Constructor
   */
  explicit MpcSolver(int max_consecutive_failures, std::string solver_name)
      : max_consecutive_failures_(max_consecutive_failures), solver_name_(solver_name) {}

  /**
   * @brief Destructor
   */
  virtual ~MpcSolver() = default;

  // === General Methods ===

  /**
   * @brief Solve the MPC problem
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool Solve() = 0;

  /**
   * @brief Reset the MPC solver
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool Reset() = 0;

  /**
   * @brief Initialize the MPC solver
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool Initialize() = 0;

  // === Setters ===

  /**
   * @brief Set the initial state for the MPC problem, x0
   *
   * [_x0_|_u0_|_x1_|_u1_|_..._|_xN-1_|_uN-1_|_xN_]
   *  ^^^^_________________________________________
   *
   * @param initial_state: the initial state of the vehicle model
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool SetInitialState(Eigen::VectorXd &initial_state) = 0;

  /**
   * @brief Set the initial guess for the whole MPC prediction horizon
   *
   * [_x0_|_u0_|_x1_|_u1_|_..._|_xN-1_|_uN-1_|_xN_]
   *  _____^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   *
   * rows are state and input variables at one specific stage
   * columns are the stages along the prediction horizon
   *
   * @param initial_guess_states: the initial guess for the states
   * @param initial_guess_inputs: the initial guess for the inputs
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool SetInitialGuess(Eigen::MatrixXd &initial_guess_states, Eigen::MatrixXd &initial_guess_inputs) = 0;

  /**
   * @brief Set double solver parameter defined by its name
   *
   * @param param_name: the name of the parameter
   * @param param_value: the value of the parameter
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool SetSolverParameter(const std::string param_name, double param_value) = 0;

  /**
   * @brief Set integer solver parameter defined by its name
   *
   * @param param_name: the name of the parameter
   * @param param_value: the value of the parameter
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool SetSolverParameter(const std::string param_name, int param_value) = 0;

  /**
   * @brief Set string solver parameter defined by its name
   *
   * @param param_name: the name of the parameter
   * @param param_value: the value of the parameter
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool SetSolverParameter(const std::string param_name, std::string param_value) = 0;

  /**
   * @brief Set double model parameter defined by its param index
   *
   * @param param_idx: the param index of the parameter
   * @param param_value: the value of the parameter
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool SetModelParameter(MpcParamIdx param_idx, double param_value) = 0;

  /**
   * @brief Set vector of double model parameter defined by its param index
   *
   * @param param_idx: the param index of the parameter
   * @param param_value: the value of the parameter (a double eigen vector)
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool SetModelParameter(MpcParamIdx param_idx, Eigen::VectorXd &param_value) = 0;

  /**
   * @brief Set double model parameter defined by its param index for a specific stage
   *
   * @param param_idx: the param index of the parameter
   * @param param_value: the value of the parameter
   * @param stage_idx: the index of the stage
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool SetModelParameter(MpcParamIdx param_idx, double param_value, int stage_idx) = 0;

  // === Getters ===

  /**
   * @brief Get number of states
   *
   * @return the number of states
   */
  virtual int GetNState() = 0;

  /**
   * @brief Get number of inputs
   *
   * @return the number of inputs
   */
  virtual int GetNInput() = 0;

  /**
   * @brief Get number of stages along prediction horizon
   *
   * @return the number of stages
   */
  virtual int GetNHorizon() = 0;

  /**
   * @brief get discretization time step for the MPC solver
   *
   * @return the discretization time step for the MPC solver
   */
  double GetDtMpc() { return dt_mpc_; }

  /**
   * @brief Get the solver name
   *
   * @return the solver name
   */
  std::string GetSolverName() { return solver_name_; }

  /**
   * @brief Get the progress horizon
   *
   * @param progress_horizon: the progress horizon
   * @param curvature_horizon: the curvature horizon
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool GetShiftedProgressHorizon(Eigen::VectorXd &progress_horizon,
                                         const Eigen::VectorXd &curvature_horizon) = 0;

  /**
   * @brief Return the solution of the MPC problem
   *
   * [_x0_|_x1_|_..._|_xN-1_|_xN_] & [_u0_|_u1_|_..._|_uN-1_]
   *  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^____^^^^^^^^^^^^^^^^^^^^^^^
   *
   * The states and inputs will be put in the above order!
   *
   * @param solution_states: states solution
   * @param solution_inputs: inputs solution
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool GetSolverSolution(Eigen::MatrixXd &solution_states, Eigen::MatrixXd &solution_inputs) = 0;

  /**
   * @brief Return the state solution of the MPC problem at a specific stage
   *
   * [_x0_|_u0_|_x1_|_u1_|_..._|_xi_|_..._|_xN-1_|_uN-1_|_xN_]
   *  __________________________^^^^_________________________
   *
   * @param solution_state: state solution
   * @param idx: the index of the stage
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool GetSolverSolutionState(Eigen::VectorXd &solution_state, const int idx) = 0;

  /**
   * @brief Return the input solution of the MPC problem at a specific stage
   *
   * [_x0_|_u0_|_x1_|_u1_|_..._|_ui_|_..._|_xN-1_|_uN-1_|_xN_]
   *  __________________________^^^^_________________________
   *
   * @param solution_input: input solution
   * @param idx: the index of the stage
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool GetSolverSolutionInput(Eigen::VectorXd &solution_input, const int idx) = 0;

  /**
   * @brief Get solver diagnostics value (double) based on the diagnostics type
   * defined by its name
   *
   * @param diagnostics_type: the name of the diagnostics type
   * @param diagnostic_value: the value of the diagnostics
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool GetSolverDiagnostics(const std::string diagnostics_type, double &diagnostic_value) = 0;

  /**
   * @brief Get solver diagnostics value (int) based on the diagnostics type
   * defined by its name
   *
   * @param diagnostics_type: the name of the diagnostics type
   * @param diagnostic_value: the value of the diagnostics
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool GetSolverDiagnostics(const std::string diagnostics_type, int &diagnostic_value) = 0;

  /**
   * @brief Get solver diagnostics value (double) based on the diagnostics type
   * defined by its enum type
   *
   * @param diagnostics_type: the enum of the diagnostics type
   * @param diagnostic_value: the value of the diagnostics
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool GetSolverDiagnostics(const MpcSolverDiagnostic diagnostics_type, double &diagnostic_value) = 0;

  /**
   * @brief Get solver diagnostics value (int) based on the diagnostics type
   * defined by its enum type
   *
   * @param diagnostics_type: the enum of the diagnostics type
   * @param diagnostic_value: the value of the diagnostics
   *
   * @return 0 (false) if successful, 1 (true) otherwise
   */
  virtual bool GetSolverDiagnostics(const MpcSolverDiagnostic diagnostics_type, int &diagnostic_value) = 0;

  /**
   * @brief Get the status of the solver as a string
   *
   * @return the status of the solver as a string
   */
  virtual std::string GetSolverStatusString() = 0;
};
