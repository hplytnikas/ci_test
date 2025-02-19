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

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <chrono> // NOLINT
#include <ctime>
#include <easy/profiler.h>
#include <memory>
#include <string>
#include <vector>

#include "abstract_controller/abstract_controller.hpp"
#include "controller_node/controller_node.hpp"
#include "mpc_visualisation_node/mpc_visualisation_node.hpp"

// ROS Messages header files
#include "control_msgs/msg/controller_ref.hpp"
#include "control_msgs/msg/mpc_log.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "vcu_msgs/msg/car_command.hpp"

// Math related files
#include "mpc_geometry/mpc_geometry.hpp"

// Embotech Solver related files
#include "embotech_solver/embotech_solver.hpp"

// Acados Solver related files
#include "acados_solver/acados_solver.hpp"

// Parameter handler
#include "mpc_controller_params.hpp"

// interplant 1d
#include "interpolant_1d/linear_interpolant_1d.hpp"

/**
 * @brief Main class for the MPC controller
 */
class MpcController : public AbstractController {
public:
  /**
   * @brief Constructor of the MPC controller
   * The controller owns the controller node, the mpc visualisation node and the parameter listener.
   */
  MpcController(std::shared_ptr<ControllerNode> controller_node,
                std::shared_ptr<MpcVisualisationNode> mpc_visualisation_node,
                std::shared_ptr<mpc_controller_params::ParamListener> param_listener);

  /**
   * @brief Constructor of the MPC controller, overloaded to include solver
   * The controller owns the controller node, the mpc visualisation node and the parameter listener.
   */
  MpcController(std::shared_ptr<ControllerNode> controller_node,
                std::shared_ptr<MpcVisualisationNode> mpc_visualisation_node,
                std::shared_ptr<mpc_controller_params::ParamListener> param_listener,
                std::shared_ptr<MpcSolver> mpc_solver_ptr);

  /**
   * @brief Initialises the MPC controller \n
   * -> Reads the parameters from the parameter handler \n
   * -> Initialises the solver object \n
   * -> Initialises the geometry object \n
   * -> Initialises the private members \n
   */
  void Initialize();

  /**
   * @brief Destructor of the MPC controller
   */
  ~MpcController();

  /**
   * @brief updates the inputs to the car
   *
   * From a given initial state, the function computes the reference path by fitting a spline through the middle points
   * to get the curvature at each progress point. Once the initial guess, the initial state and the curvature horizon
   * are set, the function solves the MPC problem and updates the car commands.
   *
   * @return car commands message (vcu_msgs::msg::CarCommand)
   */
  vcu_msgs::msg::CarCommand updateControlCommand();

  /**
   * @brief resets the controller to its default state
   *
   * @return void
   */
  void resetController();

private:
  // --- MEMBERS ---------------------------------------------------------------
  /**
   * @brief Visualisation node object which publishes the data to be visualised
   */
  std::shared_ptr<MpcVisualisationNode> mpc_visualisation_node_;

  // --- PARAMETERS ------------------------------------------------------------
  /**
   * @brief Parameter listener object
   */
  std::shared_ptr<mpc_controller_params::ParamListener> param_listener_;

  /**
   * @brief Structure of the parameters defined by the parameter handler
   */
  mpc_controller_params::Params params_;

  /**
   * @brief Solver object which handles the solver specific functions
   */
  std::shared_ptr<MpcSolver> mpc_solver_ptr_;

  /**
   * @brief Geometry object which handles the mathematical functions
   */
  std::shared_ptr<MpcGeometry> mpc_geometry_ptr_;

  /**
   * @brief Car command message
   */
  vcu_msgs::msg::CarCommand car_command_;

  /**
   * @brief Number of states
   */
  int N_STATE_;

  /**
   * @brief Number of inputs
   */
  int N_INPUT_;

  /**
   * @brief The prediction horizon length
   */
  int N_HORIZON_;

  /**
   * @brief State matrix \n
   * [_s0_|_s1_|_..._|_sN-1_|_sN_] \n
   * [_n0_|_n1_|_..._|_nN-1_|_nN_] \n
   * [_mu0_|_mu1_|_..._|_muN-1_|_muN_] \n
   * ...
   */
  Eigen::MatrixXd solution_states_;

  /**
   * @brief Input matrix \n
   * [_dax0_|_dax1_|_..._|_daxN-1_] \n
   * [_ddels0_|_ddels1_|_..._|_ddelsN-1_] \n
   */
  Eigen::MatrixXd solution_inputs_;

  // Curvature horizon
  Eigen::VectorXd curvature_horizon_;

  // Progress horizon
  Eigen::VectorXd progress_horizon_;

  // Trackbound Min and Max Horizons
  Eigen::VectorXd trackbound_min_horizon_;
  Eigen::VectorXd trackbound_max_horizon_;

  // Velocity horizon
  Eigen::VectorXd velocity_horizon_;

  // The shifted solution states and inputs
  Eigen::MatrixXd shifted_solver_states_;
  Eigen::MatrixXd shifted_solver_inputs_;

  /**
   * @brief Initial state for the MPC problem
   */
  Eigen::VectorXd initial_state_;

  /**
   * @brief Measured state of the car
   */
  Eigen::VectorXd measured_state_;

  // Solver status
  /**
   * @brief Status of the solver from the MpcSolverStatus enum (defined in the mpc_solver.hpp file)
   */
  int solver_status_;

  /**
   * @brief Type of solver to be used (embotech or acados)
   */
  std::string solver_type_;

  /**
   * @brief Time step between two MPC calls
   */
  double dt_;

  /**
   * @brief Flag to indicate if the car has reached a threshold speed. It is used to set a different rolling resistance
   * parameter.
   */
  bool launch_flag_ = false;

  /**
   * @brief Flag to indicate if a fatal error has occurred. Once it is set to true, the controller will send a stop
   * command indefinitely.s
   */
  bool fatal_error_flag_ = false;

  bool using_pp_fallback_ = false;

  int mpc_warning_counter_ = 0;

  /**
   * @brief 1D interpolant for interpolating inputs
   */
  std::shared_ptr<LinearInterpolant1D> inputs_interpolant_1d_;

  // --- METHODS --------------------------------------------------------------
  /**
   * @brief checks the output flag of the solver
   *
   * @return 1 if the solver was successful, 0 otherwise
   */
  bool CheckSolverStatus();

  /**
   * @brief checks if the initial state is valid
   *
   * @return true if the initial state is valid, false otherwise
   */
  bool CheckMeasuredStateValidity(std::string &error_message);

  /**
   * @brief calls the publishers to visualise the prediction horizon and the reference path.
   *
   * @return void
   */
  void VisualizeMpc();

  void PrepareGeometry(ControllerNode::Reference &ref);

  void SetInitialSolverGuess();

  /**
   * @brief populate the car command in order to stop the car
   *
   * @return car command message (vcu_msgs::msg::CarCommand) with high negative acceleration and zero steering / yaw
   * rate
   */
  vcu_msgs::msg::CarCommand StopCar();

  /**
   * @brief logs the mpc data
   *
   * @param solvetime time taken to solve the mpc problem
   * @param bounds boundary message which contains the reference path and the bounds
   * @param curvature_horizon curvature horizon
   * @param progress_horizon progress horizon
   *
   * @return void
   */
  void LogMpc(const double solvetime, const ControllerNode::Reference &bounds);

  /**
   * @brief Sets solver runtime params
   *
   */
  void SetSolverRuntimeParams();

  /**
   * @brief Function to integrate for the predicted initial state
   */
  void IntegratePredictedInitialState();

  /**
   * @brief Function to calculate the fallback control command
   */
  vcu_msgs::msg::CarCommand CalculatePurePursuitCommand(const ControllerNode::State &state,
                                                        const ControllerNode::Reference &reference);
};
