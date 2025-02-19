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

#include "acados/utils/types.h"
#include "codegenerated_solvers/acados_codegen/c_generated_solver_mpc/acados_solver_veh_dynamics_ode.h"
#include "mpc_solver_common/mpc_solver.hpp"
#include <array>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#define NZ VEH_DYNAMICS_ODE_NZ
#define NP VEH_DYNAMICS_ODE_NP
#define NBX0 VEH_DYNAMICS_ODE_NBX0

class AcadosSolver : public MpcSolver {
public:
  // === Constructor ===
  // construct solver with solver_name_ = ACADOS_SOLVER and max_consecutive_failures as argument
  explicit AcadosSolver(int max_consecutive_failures) : MpcSolver(max_consecutive_failures, "ACADOS") {}
  virtual ~AcadosSolver();

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

  bool SetModelParameter(MpcParamIdx param_idx, double param_value);

  bool SetModelParameter(MpcParamIdx param_idx, Eigen::VectorXd &param_value);

  bool SetModelParameter(MpcParamIdx param_idx, double param_value, int stage_idx);

  // === Getters ===
  int GetNState() { return NX; }

  int GetNInput() { return NU; }

  int GetNHorizon() { return N_HOR; }

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
  // Acados OCP capsule
  veh_dynamics_ode_solver_capsule *acados_ocp_capsule_;

  // Acados NLP configuration struct
  ocp_nlp_config *nlp_config_;

  // Acados NLP struct that contains dimensions
  ocp_nlp_dims *nlp_dims_;

  // Acados NLP input struct
  ocp_nlp_in *nlp_in_;

  // Acados NLP output struct
  ocp_nlp_out *nlp_out_;

  // Acados struct to store the
  // state/configuration for the NLP solver
  ocp_nlp_solver *nlp_solver_;

  // solver status unified enum
  MpcSolverStatus mpc_solver_status_ = Ready;

  // Acados NLP options
  void *nlp_opts_;

  // Initial State indices
  int idxbx0_[NBX0] = {0};

  // Number of states
  static constexpr int NX = VEH_DYNAMICS_ODE_NX;

  // Number of inputs
  static constexpr int NU = VEH_DYNAMICS_ODE_NU;

  // Length of horizon
  static constexpr int N_HOR = VEH_DYNAMICS_ODE_N;
};
