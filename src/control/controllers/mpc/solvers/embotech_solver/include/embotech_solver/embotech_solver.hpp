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

#include "codegenerated_solvers/embotech_codegen/embotech/include/embotech.h"
#include "codegenerated_solvers/embotech_codegen/embotech/include/embotech_memory.h"
#include "mpc_solver_common/mpc_solver.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>

class EmbotechSolver : public MpcSolver {
public:
  explicit EmbotechSolver(int max_consecutive_failures) : MpcSolver(max_consecutive_failures, "EMBOTECH") {}
  virtual ~EmbotechSolver();

  // === Methods ===

  bool Solve() override;

  bool Reset() override;

  bool Initialize() override;

  // === Setters ===

  bool SetInitialState(Eigen::VectorXd &initial_state) override;

  bool SetInitialGuess(Eigen::MatrixXd &initial_guess_states, Eigen::MatrixXd &initial_guess_inputs) override;

  bool SetSolverParameter(const std::string param_name, double param_value) override;

  bool SetSolverParameter(const std::string param_name, int param_value) override;

  bool SetSolverParameter(const std::string param_name, std::string param_value) override;

  bool SetModelParameter(MpcParamIdx param_idx, double param_value) override;

  bool SetModelParameter(MpcParamIdx param_idx, Eigen::VectorXd &param_value) override;

  bool SetModelParameter(MpcParamIdx param_idx, double param_value, int stage_idx) override;

  // === Getters ===
  int GetNState() { return N_STATE; }

  int GetNInput() { return N_INPUT; }

  int GetNHorizon() { return N_HORIZON; }

  bool GetShiftedProgressHorizon(Eigen::VectorXd &progress_horizon, const Eigen::VectorXd &curvature_horizon) override;

  // Gets the solution including x0:
  // [ x0 | u0 | x1 | u1 | ... | xN-1 | uN-1 | xN ]
  bool GetSolverSolution(Eigen::MatrixXd &solution_states, Eigen::MatrixXd &solution_inputs) override;

  bool GetSolverSolutionState(Eigen::VectorXd &solution_state, const int idx) override;

  bool GetSolverSolutionInput(Eigen::VectorXd &solution_input, const int idx) override;

  // diagnotics_type as string, double value
  bool GetSolverDiagnostics(const std::string diagnostics_type, double &diagnostic_value) override;

  // diagnotics_type as string, int value
  bool GetSolverDiagnostics(const std::string diagnostics_type, int &diagnostic_value) override;

  // diagnotics_type as enum, double value
  bool GetSolverDiagnostics(const MpcSolverDiagnostic diagnostics_type, double &diagnostic_value) override;

  // diagnotics_type as enum, int value
  bool GetSolverDiagnostics(const MpcSolverDiagnostic diagnostics_type, int &diagnostic_value) override;

  // get solver status as string
  std::string GetSolverStatusString() override;

private:
  //  Create the solver structure
  embotech_params solver_params_; // Defines the problem structure
  embotech_output prediction_;    // Defines the predicted output structure
  embotech_info info_;            // Defines the solver's diagnostics structure
  embotech_mem *mem_;             // Defines the memory allocation
  embotech_extfunc extfunc_;      // Defines the external function structure

  // Define the exitflag
  int exitflag_;

  // MPC solver status
  MpcSolverStatus mpc_solver_status_ = MpcSolverStatus::Ready;

  //  Number of states
  static constexpr int N_STATE = 7;

  //  Length of horizon
  static constexpr int N_HORIZON = 40;

  //  Number of inputs
  static constexpr int N_INPUT = 2;

  static constexpr int N_SLACK = 0;
};
