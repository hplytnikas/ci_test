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
#include "mpc_solver_common/mpc_solver.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <vector>

TEST(acados_solver, test_solve_with_output_and_diagnostics) {
  std::unique_ptr<MpcSolver> solver = std::make_unique<AcadosSolver>(10);
  solver->Initialize();
  const int NX = solver->GetNState();
  const int NU = solver->GetNInput();
  const int N_HOR = solver->GetNHorizon();

  Eigen::VectorXd initial_state(NX);
  initial_state[MpcStateIdx::n] = 0.0;
  initial_state[MpcStateIdx::mu] = 0.0;
  initial_state[MpcStateIdx::vx] = 0.0;
  initial_state[MpcStateIdx::vy] = 0.0;
  initial_state[MpcStateIdx::dpsi] = 0.0;
  initial_state[MpcStateIdx::ax] = 0.0;
  initial_state[MpcStateIdx::dels] = 0.0;
  solver->SetInitialState(initial_state);
  printf("Solver initialized with initial_state: \n");
  printf("n = %.3f, "
         "mu = %.3f, "
         "vx = %.3f, "
         "vy = %.3f, "
         "dpsi = %.3f, "
         "ax = %.3f, "
         "dels = %.3f\n",
         initial_state[MpcStateIdx::n], initial_state[MpcStateIdx::mu], initial_state[MpcStateIdx::vx],
         initial_state[MpcStateIdx::vy], initial_state[MpcStateIdx::dpsi], initial_state[MpcStateIdx::ax],
         initial_state[MpcStateIdx::dels]);

  // initial guess states go from i = 1 to i = N
  Eigen::MatrixXd initial_guess_states(NX, N_HOR);
  Eigen::VectorXd initial_guess_state = initial_state;

  initial_guess_states = initial_guess_state.replicate(1, N_HOR);

  printf("Solver initialized with initial_guess_state: \n");
  printf("n = %.3f, "
         "mu = %.3f, "
         "vx = %.3f, "
         "vy = %.3f, "
         "dpsi = %.3f, "
         "ax = %.3f, "
         "dels = %.3f\n",
         initial_guess_state[MpcStateIdx::n], initial_guess_state[MpcStateIdx::mu],
         initial_guess_state[MpcStateIdx::vx], initial_guess_state[MpcStateIdx::vy],
         initial_guess_state[MpcStateIdx::dpsi], initial_guess_state[MpcStateIdx::ax],
         initial_guess_state[MpcStateIdx::dels]);

  // initial guess inputs go from i = 0 to i = N-1
  Eigen::MatrixXd initial_guess_inputs(NU, N_HOR);
  Eigen::VectorXd initial_guess_input(NU);
  initial_guess_input[MpcInputIdx::dax] = 0.0;
  initial_guess_input[MpcInputIdx::ddels] = 0.0;

  initial_guess_inputs = initial_guess_input.replicate(1, N_HOR);

  printf("Solver initialized with initial_guess_input: \n");
  printf("dax = %.3f, ddels = %.3f\n", initial_guess_input[MpcInputIdx::dax], initial_guess_input[MpcInputIdx::ddels]);

  solver->SetInitialGuess(initial_guess_states, initial_guess_inputs);

  double kappa = 0.2;
  printf("Curvature : %f\n", kappa);
  solver->SetModelParameter(MpcParamIdx::curv, kappa);

  // solve method call
  solver->Solve();

  // // get solve status
  // // get solve status
  int res = 0;
  solver->GetSolverDiagnostics(MpcSolverDiagnostic::SolverStatus, res);
  int required_solver_status = Ok;
  ASSERT_EQ(res, required_solver_status);

  // get solver solution
  Eigen::MatrixXd solver_input_sol;
  Eigen::MatrixXd solver_state_sol;
  solver->GetSolverSolution(solver_state_sol, solver_input_sol);
  ASSERT_EQ(solver_input_sol.cols(), N_HOR);
  ASSERT_EQ(solver_state_sol.cols(), N_HOR + 1);

  printf("Solver solution dax_0 = %.3f\n", solver_input_sol.col(0)[MpcInputIdx::dax]);
  printf("Solver solution ddels_0 = %.3f\n", solver_input_sol.col(0)[MpcInputIdx::ddels]);

  printf("Solver input prediction: \n");
  for (int i = 0; i < N_HOR; i++) {
    printf("dax_%i = %.3f, ddels_%i = %.3f\n", i, solver_input_sol.col(i)[MpcInputIdx::dax], i,
           solver_input_sol.col(i)[MpcInputIdx::ddels]);
  }
  printf("Solver state prediction: \n");
  for (int i = 0; i < N_HOR + 1; i++) {
    printf("n_%i = %.3f, "
           "mu_%i = %.3f, "
           "vx_%i = %.3f, "
           "vy_%i = %.3f, "
           "dpsi_%i = %.3f, "
           "ax_%i = %.3f, "
           "dels_%i = %.3f\n",
           i, solver_state_sol.col(i)[MpcStateIdx::n], i, solver_state_sol.col(i)[MpcStateIdx::mu], i,
           solver_state_sol.col(i)[MpcStateIdx::vx], i, solver_state_sol.col(i)[MpcStateIdx::vy], i,
           solver_state_sol.col(i)[MpcStateIdx::dpsi], i, solver_state_sol.col(i)[MpcStateIdx::ax], i,
           solver_state_sol.col(i)[MpcStateIdx::dels]);
  }

  Eigen::VectorXd curvature_horizon(N_HOR);
  curvature_horizon.setOnes();
  curvature_horizon = curvature_horizon * kappa;
  Eigen::VectorXd progress_horizon(N_HOR);
  ASSERT_EQ(solver->GetShiftedProgressHorizon(progress_horizon, curvature_horizon), false);

  for (int i = 0; i < N_HOR; i++) {
    printf("Progress Horizon %d : %f\n", i, progress_horizon[i]);
  }

  // get solver diagnostics
  double solve_time = 0.0;
  solver->GetSolverDiagnostics(MpcSolverDiagnostic::SolveTimeMS, solve_time);
  printf("Solver solve time [ms]: %f\n", solve_time);

  int sqp_iter = 0;
  solver->GetSolverDiagnostics(MpcSolverDiagnostic::NlpIterations, sqp_iter);
  printf("Solver NLP Iterations: %i\n", sqp_iter);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
