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

#include "embotech_solver/embotech_solver.hpp"
#include "mpc_solver_common/mpc_solver.hpp"
#include <gtest/gtest.h>

// // --- Test solver initialization ----------------------------------------------
// TEST(embotech_solver, test_init) {
//   std::unique_ptr<MpcSolver> solver = std::make_unique<EmbotechSolver>(10);
//   printf("Solver object created.\n");

//   bool init_res = solver->Initialize();
//   printf("Solver object Initialized.\n");

//   ASSERT_EQ(init_res, false) << "Initialization method returned 1." << std::endl;

//   // Check that the solver_name_ is set to EMBOTECH
//   std::string solver_name;
//   solver_name = solver->GetSolverName();
//   // print solver name
//   std::cout << "Solver name: " << solver_name << std::endl;
//   ASSERT_EQ(solver_name, "EMBOTECH") << "Solver name is not EMBOTECH." << std::endl;

//   printf("Solver object will be deleted.\n");
// }

// // --- Test solver set initial states ------------------------------------------
// TEST(embotech_solver, test_set_initial_state) {
//   std::unique_ptr<MpcSolver> solver = std::make_unique<EmbotechSolver>(10);
//   solver->Initialize();

//   Eigen::VectorXd initial_state(8);
//   initial_state << 2.0, 2.0, 2.0, 2.0, 4.0, 3.0, 3.0, 2.0;

//   ASSERT_EQ(solver->SetInitialState(initial_state), false) << "SetInitialState method returned 1." << std::endl;
//   printf("--- Initial states have been set ---\n");
// }

// //  --- Test solver set initial guess
// // -------------------------------------------
// TEST(embotech_solver, test_set_initial_guess) {
//   std::unique_ptr<MpcSolver> solver = std::make_unique<EmbotechSolver>(10);
//   solver->Initialize();

//   const int N_STATE = solver->GetNState();
//   const int N_INPUT = solver->GetNInput();
//   const int N_HORIZON = solver->GetNHorizon();

//   std::cout << "N_STATE: " << N_STATE << std::endl;
//   std::cout << "N_INPUT: " << N_INPUT << std::endl;
//   std::cout << "N_HORIZON: " << N_HORIZON << std::endl;

//   // initial guess states go from i = 1 to i = N
//   Eigen::VectorXd initial_guess_state(N_STATE);
//   initial_guess_state[MpcStateIdx::s] = 0.0;
//   initial_guess_state[MpcStateIdx::n] = 0.0;
//   initial_guess_state[MpcStateIdx::mu] = 0.0;
//   initial_guess_state[MpcStateIdx::dpsi] = 0.0;
//   initial_guess_state[MpcStateIdx::vx] = 0.0;
//   initial_guess_state[MpcStateIdx::vy] = 0.0;
//   initial_guess_state[MpcStateIdx::ax] = 0.0;
//   initial_guess_state[MpcStateIdx::dels] = 0.0;

//   printf("Solver will be initialized with initial_guess_state: \n");
//   printf("s = %.3f, n = %.3f, mu = %.3f, dpsi = %.3f, vx = %.3f, vy = %.3f, ax "
//          "= %.3f, dels = %.3f\n",
//          initial_guess_state[MpcStateIdx::s], initial_guess_state[MpcStateIdx::n],
//          initial_guess_state[MpcStateIdx::mu], initial_guess_state[MpcStateIdx::dpsi],
//          initial_guess_state[MpcStateIdx::vx], initial_guess_state[MpcStateIdx::vy],
//          initial_guess_state[MpcStateIdx::ax], initial_guess_state[MpcStateIdx::dels]);

//   // initial guess inputs go from i = 0 to i = N-1
//   Eigen::VectorXd initial_guess_input(N_INPUT);
//   initial_guess_input[MpcInputIdx::dax] = 0.0;
//   initial_guess_input[MpcInputIdx::ddels] = 0.0;
//   printf("Solver will be initialized with initial_guess_input: \n");
//   printf("dax = %.3f, ddels = %.3f\n",
//          initial_guess_input[MpcInputIdx::dax], initial_guess_input[MpcInputIdx::ddels]);

//   Eigen::MatrixXd initial_guess_states(N_STATE, N_HORIZON);
//   Eigen::MatrixXd initial_guess_inputs(N_INPUT, N_HORIZON);
//   initial_guess_inputs = initial_guess_input.replicate(1, N_HORIZON);
//   initial_guess_states = initial_guess_state.replicate(1, N_HORIZON);
//   solver->SetInitialGuess(initial_guess_states, initial_guess_inputs);
//   printf("--- Initial guess has been set ---\n");
// }

// //  --- Test solver set parameters
// // ----------------------------------------------
// TEST(embotech_solver, test_set_parameters) {
//   std::unique_ptr<MpcSolver> solver = std::make_unique<EmbotechSolver>(10);
//   solver->Initialize();

//   const int N_HORIZON = solver->GetNHorizon();

//   Eigen::VectorXd curvature(N_HORIZON);

//   // set all values in the curvature vector to 0.2
//   curvature.fill(0.2);

//   printf("--- Set a runtime parameter ---\n");
//   ASSERT_EQ(solver->SetSolverParameter("parameter", 0), true);
//   ASSERT_EQ(solver->SetSolverParameter("parameter", 0.0), true);
//   ASSERT_EQ(solver->SetSolverParameter("parameter", "value"), true);

//   printf("--- Set the curvature ---\n");
//   ASSERT_EQ(solver->SetModelParameter("curvature", 0.0), false);
//   ASSERT_EQ(solver->SetModelParameter("curvature", curvature), false);
//   ASSERT_EQ(solver->SetModelParameter("curvature", 0.0, 25), false);
// }

// // --- Test solver solve -------------------------------------------------------
// TEST(embotech_solver, test_solve) {
//   std::unique_ptr<MpcSolver> solver = std::make_unique<EmbotechSolver>(10);
//   solver->Initialize();

//   const int N_STATE = solver->GetNState();
//   const int N_INPUT = solver->GetNInput();
//   const int N_HORIZON = solver->GetNHorizon();

//   Eigen::VectorXd initial_state(N_STATE);

//   solver->SetInitialState(initial_state);

//   // initial guess states go from i = 1 to i = N
//   Eigen::VectorXd initial_guess_state(N_STATE);
//   printf("Solver will be initialized with initial_guess_state: \n");
//   printf("s = %.3f, n = %.3f, mu = %.3f, dpsi = %.3f, vx = %.3f, vy = %.3f, ax "
//          "= %.3f, dels = %.3f\n",
//          initial_guess_state[MpcStateIdx::s], initial_guess_state[MpcStateIdx::n],
//          initial_guess_state[MpcStateIdx::mu], initial_guess_state[MpcStateIdx::dpsi],
//          initial_guess_state[MpcStateIdx::vx], initial_guess_state[MpcStateIdx::vy],
//          initial_guess_state[MpcStateIdx::ax], initial_guess_state[MpcStateIdx::dels]);

//   // initial guess inputs go from i = 0 to i = N-1
//   Eigen::VectorXd initial_guess_input(N_INPUT);
//   printf("Solver will be initialized with initial_guess_input: \n");
//   printf("dax = %.3f, ddels = %.3f\n", initial_guess_input[MpcInputIdx::dax],
//   initial_guess_input[MpcInputIdx::ddels]);

//   Eigen::MatrixXd initial_guess_states(N_STATE, N_HORIZON);
//   Eigen::MatrixXd initial_guess_inputs(N_INPUT, N_HORIZON);
//   initial_guess_inputs = initial_guess_input.replicate(1, N_HORIZON);
//   initial_guess_states = initial_guess_state.replicate(1, N_HORIZON);
//   solver->SetInitialGuess(initial_guess_states, initial_guess_inputs);

//   double kappa = 0.0;
//   solver->SetModelParameter("curvature", kappa);

//   printf("--- Calling the solver ---\n");
//   ASSERT_EQ(solver->Solve(), MpcSolverStatus::Ok) << "Solve method returned not OK." << std::endl;
//   printf("--- Solved ---\n");
// }

// // --- Test solver solve and get exitflag --------------------------------------
// TEST(embotech_solver, test_solve_with_exit_flag) {
//   std::unique_ptr<MpcSolver> solver = std::make_unique<EmbotechSolver>(10);
//   solver->Initialize();

//   const int N_STATE = solver->GetNState();
//   const int N_INPUT = solver->GetNInput();
//   const int N_HORIZON = solver->GetNHorizon();

//   Eigen::VectorXd initial_state(N_STATE);
//   solver->SetInitialState(initial_state);

//   double kappa = 0.0;
//   solver->SetModelParameter("curvature", kappa);

//   printf("--- Calling the solver ---\n");
//   solver->Solve();
//   printf("--- Solved ---\n");

//   int res = 0;
//   solver->GetSolverDiagnostics(MpcSolverDiagnostic::ExitFlag, res);
//   printf("--- Returned exit flag: %d ---\n", res);

//   int required_exit_flag = 1;
//   ASSERT_EQ(res, required_exit_flag);
// }

// // --- Test solver solve and get solvetime
// // --------------------------------------
// TEST(embotech_solver, test_solve_with_solvetime) {
//   std::unique_ptr<MpcSolver> solver = std::make_unique<EmbotechSolver>(10);
//   solver->Initialize();

//   const int N_STATE = solver->GetNState();
//   const int N_INPUT = solver->GetNInput();
//   const int N_HORIZON = solver->GetNHorizon();

//   Eigen::VectorXd initial_state(N_STATE);
//   solver->SetInitialState(initial_state);

//   double kappa = 0.0;
//   solver->SetModelParameter("curvature", kappa);

//   printf("--- Calling the solver ---\n");
//   solver->Solve();
//   printf("--- Solved ---\n");

//   double solvetime = 0;
//   solver->GetSolverDiagnostics(MpcSolverDiagnostic::SolveTimeMS, solvetime);
//   printf("--- Returned solvetime: %f ms ---\n", solvetime);

//   ASSERT_FALSE(solvetime == 0.0);
// }

// // --- Test solver solve and get solution --------------------------------------
// TEST(embotech_solver, test_solve_get_solution) {
//   std::unique_ptr<MpcSolver> solver = std::make_unique<EmbotechSolver>(10);
//   solver->Initialize();

//   const int N_STATE = solver->GetNState();
//   const int N_INPUT = solver->GetNInput();
//   const int N_HORIZON = solver->GetNHorizon();

//   Eigen::VectorXd initial_state(N_STATE);
//   solver->SetInitialState(initial_state);

//   double kappa = 0.0;
//   solver->SetModelParameter("curvature", kappa);

//   printf("--- Calling the solver ---\n");
//   solver->Solve();
//   printf("--- Solved ---\n");

//   Eigen::MatrixXd solution_states(N_STATE, N_HORIZON + 1);
//   Eigen::MatrixXd solution_inputs(N_INPUT, N_HORIZON);
//   solver->GetSolverSolution(solution_states, solution_inputs);

//   printf("\n--- Returned Solution ---");
//   printf("\nINPUT: Accel. rate : %f \n", solution_inputs.col(0)[MpcInputIdx::dax]);
//   printf("INPUT: Steering rate : %f \n\n", solution_inputs.col(0)[MpcInputIdx::ddels]);

//   printf("Progress : %f \n", solution_states.col(0)[MpcStateIdx::s]);
//   printf("Deviation : %f \n", solution_states.col(0)[MpcStateIdx::n]);
//   printf("Heading : %f \n", solution_states.col(0)[MpcStateIdx::mu]);
//   printf("Lon. speed : %f \n", solution_states.col(0)[MpcStateIdx::vx]);
//   printf("Lat. speed : %f \n", solution_states.col(0)[MpcStateIdx::vy]);
//   printf("Yaw rate : %f \n", solution_states.col(0)[MpcStateIdx::dpsi]);
//   printf("Acceleration : %f \n", solution_states.col(0)[MpcStateIdx::ax]);
//   printf("Steering angle : %f \n\n", solution_states.col(0)[MpcStateIdx::dels]);
// }

// // --- Test solver solve and get progress horizon
// // --------------------------------
// TEST(embotech_solver, test_solve_get_progress_horizon) {
//   std::unique_ptr<MpcSolver> solver = std::make_unique<EmbotechSolver>(10);
//   solver->Initialize();

//   const int N_STATE = solver->GetNState();
//   const int N_INPUT = solver->GetNInput();
//   const int N_HORIZON = solver->GetNHorizon();

//   Eigen::VectorXd initial_state(N_STATE);
//   solver->SetInitialState(initial_state);

//   double kappa = 0.1;
//   solver->SetModelParameter("curvature", kappa);

//   printf("--- Calling the solver ---\n");
//   ASSERT_EQ(solver->Solve(), MpcSolverStatus::Ok);
//   printf("--- Solved ---\n");

//   Eigen::VectorXd progress_horizon(N_HORIZON);
//   ASSERT_EQ(solver->GetShiftedProgressHorizon(progress_horizon), false);

//   for (int i = 0; i < N_HORIZON; i++) {
//     printf("Progress Horizon %d : %f\n", i, progress_horizon[i]);
//   }

//   ASSERT_FALSE(progress_horizon[N_HORIZON - 1] == 0.0); // Progress
// }

// --- Test solver solve complete ----------------------------------------------
TEST(embotech_solver, test_solve_complete) {
  std::unique_ptr<MpcSolver> solver = std::make_unique<EmbotechSolver>(10);
  solver->Initialize();

  const int N_HORIZON = solver->GetNHorizon();
  const int N_STATE = solver->GetNState();
  const int N_INPUT = solver->GetNInput();

  Eigen::VectorXd initial_state(N_STATE);
  // set all values to 0.1
  initial_state.fill(0.1);
  solver->SetInitialState(initial_state);

  // initial guess states go from i = 1 to i = N
  Eigen::VectorXd initial_guess_state(8);
  printf("Solver will be initialized with initial_guess_state: \n");
  printf("n = %.3f, mu = %.3f, dpsi = %.3f, vx = %.3f, vy = %.3f, ax "
         "= %.3f, dels = %.3f\n",
         initial_guess_state[MpcStateIdx::n], initial_guess_state[MpcStateIdx::mu],
         initial_guess_state[MpcStateIdx::dpsi], initial_guess_state[MpcStateIdx::vx],
         initial_guess_state[MpcStateIdx::vy], initial_guess_state[MpcStateIdx::ax],
         initial_guess_state[MpcStateIdx::dels]);

  // initial guess inputs go from i = 0 to i = N-1
  Eigen::VectorXd initial_guess_input(N_INPUT);
  printf("Solver will be initialized with initial_guess_input: \n");
  printf("dax = %.3f, ddels = %.3f\n", initial_guess_input[MpcInputIdx::dax], initial_guess_input[MpcInputIdx::ddels]);

  Eigen::MatrixXd initial_guess_states(N_STATE, N_HORIZON);
  Eigen::MatrixXd initial_guess_inputs(N_INPUT, N_HORIZON);
  initial_guess_inputs = initial_guess_input.replicate(1, N_HORIZON);
  initial_guess_states = initial_guess_state.replicate(1, N_HORIZON);
  solver->SetInitialGuess(initial_guess_states, initial_guess_inputs);

  double kappa = 0.2;
  printf("\nCurvature : %f\n\n", kappa);
  solver->SetModelParameter(MpcParamIdx::curv, kappa);

  printf("--- Calling the solver ---\n");
  ASSERT_EQ(solver->Solve(), MpcSolverStatus::Ok);
  printf("--- Solved ---\n");

  int res = 0;
  solver->GetSolverDiagnostics(MpcSolverDiagnostic::ExitFlag, res);
  int required_exit_flag = 1;
  ASSERT_EQ(res, required_exit_flag);

  int status = 0;
  solver->GetSolverDiagnostics(MpcSolverDiagnostic::SolverStatus, status);
  int required_solver_status = MpcSolverStatus::Ok;
  ASSERT_EQ(status, required_solver_status);

  int iter = 0;
  solver->GetSolverDiagnostics(MpcSolverDiagnostic::NlpIterations, iter);
  printf("\n# of iterations : %d\n", iter);

  double solve_time = 0.0;
  solver->GetSolverDiagnostics(MpcSolverDiagnostic::SolveTimeMS, solve_time);
  printf("Solve time : %f ms\n", solve_time);

  Eigen::MatrixXd solution_states(N_STATE, N_HORIZON + 1);
  Eigen::MatrixXd solution_inputs(N_INPUT, N_HORIZON);
  solver->GetSolverSolution(solution_states, solution_inputs);

  printf("\n--- Returned Solution ---");
  printf("\nINPUT: Accel. rate (0) : %f \n", solution_inputs.col(0)[MpcInputIdx::dax]);
  printf("INPUT: Steering rate (0) : %f \n\n", solution_inputs.col(0)[MpcInputIdx::ddels]);

  printf("Deviation (N) : %f \n", solution_states.col(N_HORIZON)[MpcStateIdx::n]);
  printf("Heading (N) : %f \n", solution_states.col(N_HORIZON)[MpcStateIdx::mu]);
  printf("Lon. speed (N) : %f \n", solution_states.col(N_HORIZON)[MpcStateIdx::vx]);
  printf("Lat. speed (N) : %f \n", solution_states.col(N_HORIZON)[MpcStateIdx::vy]);
  printf("Yaw rate (N) : %f \n", solution_states.col(N_HORIZON)[MpcStateIdx::dpsi]);
  printf("Acceleration (N) : %f \n", solution_states.col(N_HORIZON)[MpcStateIdx::ax]);
  printf("Steering angle (N) : %f \n\n", solution_states.col(N_HORIZON)[MpcStateIdx::dels]);

  Eigen::VectorXd progress_horizon(N_HORIZON);
  Eigen::VectorXd curvature_horizon(N_HORIZON);
  curvature_horizon.setOnes();
  curvature_horizon = curvature_horizon * kappa;
  ASSERT_EQ(solver->GetShiftedProgressHorizon(progress_horizon, curvature_horizon), false);

  for (int i = 0; i < N_HORIZON; i++) {
    printf("Progress Horizon %d : %f\n", i, progress_horizon[i]);
  }

  ASSERT_FALSE(progress_horizon[N_HORIZON - 1] == 0.0); // Progress

  for (int i = 0; i < N_HORIZON; i++) {
    printf("Deviation : %f \n", solution_states.col(i)[MpcStateIdx::n]);
    printf("Heading : %f \n", solution_states.col(i)[MpcStateIdx::mu]);
    printf("Lon. speed : %f \n", solution_states.col(i)[MpcStateIdx::vx]);
    printf("Lat. speed : %f \n", solution_states.col(i)[MpcStateIdx::vy]);
    printf("Yaw rate : %f \n", solution_states.col(i)[MpcStateIdx::dpsi]);
    printf("Acceleration : %f \n", solution_states.col(i)[MpcStateIdx::ax]);
    printf("Steering angle : %f \n\n", solution_states.col(i)[MpcStateIdx::dels]);

    printf(" ");
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
