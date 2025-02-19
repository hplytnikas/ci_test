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
#include <Eigen/Sparse>
#include <easy/profiler.h>
#include <memory>

// HPIPM related includes
#include <blasfeo_d_aux_ext_dep.h>
#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_dim.h>
#include <hpipm_d_dense_qp_ipm.h>
#include <hpipm_d_dense_qp_sol.h>
#include <hpipm_d_dense_qp_utils.h>

#include "interpolant_2d/bspline_interpolant_2d.hpp"

/**
 * @brief Struct to store the HPIPM arguments
 *
 * @param mode The mode of the HPIPM solver. \n
 * 0: SPEED_ABS \n
 * 1: SPEED \n
 * 2: BALANCE \n
 * 3: ROBUST \n
 * default: BALANCE
 * @param iter_max The maximum number of iterations. \n
 * @param alpha_min The minimum step length. \n
 * @param mu0 The initial value of the barrier parameter. \n
 * @param tol_stat The ttolerance for the stationarity of the KKT conditions. \n
 * @param tol_eq The tolerance for the primal and dual residuals in the equality constraints. \n
 * @param tol_ineq The tolerance for the primal and dual residuals in the inequality constraints. \n
 * @param tol_comp The tolerance for the primal and dual residuals in the complementarity constraints. \n
 * @param reg_prim The regularization parameter for the primal variables. \n
 * @param reg_dual The regularization parameter for the dual variables. \n
 * @param warm_start The warm start option. \n
 * 0: DISABLE \n
 * 1: ENABLE \n
 * @param pred_corr The predictor-corrector methods -> enhance convergence speed and solution accuracy. \n
 * 0: DISABLE \n
 * 1: ENABLE \n
 * @param split_step The split step option. If split, the step sizes for primal and dual updates are decoupled \n
 * 0: DISABLE \n
 * 1: ENABLE \n
 * @param dim_opt The dimension of the optimization problem. \n
 *
 * @note For more information, please refer to the HPIPM documentation: \n
 *        HPIPM: a high-performance quadratic programming framework
 *        for model predictive control \n
 *        Gianluca Frison∗ Moritz Diehl
 */
struct hpipm_args {
  int mode;
  int iter_max;
  double alpha_min;
  double mu0;
  double tol_stat;
  double tol_eq;
  double tol_ineq;
  double tol_comp;
  double reg_prim;
  double reg_dual;
  int warm_start;
  int pred_corr;
  int split_step;
  int dim_opt;
};

/*******************************************************************************
 * class MinCurvatureOpt                                                       *
 ******************************************************************************/

/**
 * @brief Optimises a reference path for an autonomous race car based on minimum curvature trajectory planning.
 *
 * This class implements the optimisation strategy outlined in the paper:
 * "Alexander Heilmeier, Alexander Wischnewski, Leonhard Hermansdorfer,
 * Johannes Betz, Markus Lienkamp & Boris Lohmann (2019):
 * Minimum curvature trajectory planning and control
 * for an autonomous race car,
 * Vehicle System Dynamics,
 * DOI: 10.1080/00423114.2019.1631455"
 *
 * This optimisation strategy is based on a quadratic programming problem that minimises the curvature of the reference
 * path. A QP is formulated based on the reference path and its deviation from the middle line of the track. It is
 * solved using the HPIPM solver (dense qp solver).
 *
 * Coordinates system of the car:
 * x: heading of the car
 * y: perpendicular left to the heading of the car
 *
 * @param reference_points A matrix of points (x, y) representing the reference path in Cartesian coordinates in
 * vehicle's frame.
 *
 * @return Returns an optimized reference path as a matrix of points in Cartesian coordinates.
 */
class MinCurvatureOpt {
  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PUBLIC
   *______________________________________________*/
public:
  /**
   * @brief constructor of the MPC geometry class
   */
  MinCurvatureOpt(hpipm_args args, int nb_iterations, bool first_point_fixed, bool last_point_fixed, bool closed,
                  double max_curvature, double max_distance, double car_width, double safety_margin);

  /**
   * @brief destructor of the MPC geometry class
   */
  ~MinCurvatureOpt();

  /**
   * @brief Main function to get the optimal reference path.
   *
   * @param reference_points The reference points in a 2d matrix in Cartesian coordinates, column-major. Returns the
   * matrix modified.
   * @param lateral_deviation The lateral deviation of the reference points from the middle line. Returns the vector
   * modified.
   * @param curvature_profile The curvature profile of the reference points. Returns the vector modified.
   * @param preprocess A boolean to indicate if the middle line should be preprocessed.
   *
   * @return void
   */
  void GetOptimalReferencePath(Eigen::MatrixX2d &reference_points, Eigen::MatrixX2d &lateral_deviation,
                               Eigen::VectorXd &curvature_profile, const bool &preprocess,
                               Eigen::MatrixX2d &boundary_points_left, Eigen::MatrixX2d &boundary_points_right);

  /**
   * @brief Function that preprocesses the middle line of the track. It uses a gaussian filter to smooth the curvature
   * profile
   *
   * @param curvature_profile The curvature profile of the reference points.
   * @param isClosedPath True if the path is closed, false otherwise.
   * @param smoothness The size of the gaussian filter.
   * @param sigma The standard deviation of the gaussian filter.
   *
   * @return Returns the smoothed curvature profile.
   */
  Eigen::VectorXd ProcessCurvature(const Eigen::VectorXd &curvature_profile, bool isClosedPath, const int &smoothness,
                                   const double &sigma);

  /**
   * @brief Function that computes the speed targets based on the curvature profile.
   *
   * @param curvature_profile The curvature profile of the reference points.
   * @param isClosedPath True if the path is closed, false otherwise.
   * @param ay_max The maximum lateral acceleration of the car.
   * @param vx_min The minimum longitudinal velocity of the car.
   * @param vx_max The maximum longitudinal velocity of the car.
   *
   * @return Returns the speed targets.
   */
  Eigen::VectorXd GetSpeedTargets(const Eigen::VectorXd &curvature_profile, bool isClosedPath, double ay_max,
                                  double vx_min, double vx_max, double kappa_shift);

private:
  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PRIVATE MEMBERS
   *______________________________________________*/
  /**
   * @brief The spline matrix for the QP problem.
   */
  Eigen::MatrixXd A_;

  /**
   * @brief The extraction matrix for the QP problem. Used to isolate the c coefficients.
   */
  Eigen::SparseMatrix<double> A_ex_;

  /**
   * @brief The extraction matrix for the QP problem. Used to isolate the b coefficients.
   */
  Eigen::SparseMatrix<double> A_ex_b_;

  /**
   * @brief The inverse of the A matrix.
   */
  Eigen::MatrixXd A_inv_;

  /**
   * @brief The T_c matrix for the QP problem (for more detail, look into the source code).
   */
  Eigen::MatrixXd T_c_;

  /**
   * @brief Vector containing the width of the track used for bound constraints in the solver.
   */
  Eigen::VectorXd w_tr_l_; // track width left [m]

  /**
   * @brief Vector containing the width of the track used for bound constraints in the solver.
   */
  Eigen::VectorXd w_tr_r_; // track width right [m]

  // Optimisation parameters
  /**
   * @brief The width of the car
   */
  double car_width_;

  /**
   * @brief The safety margin on each side
   */
  double safety_margin_;

  /**
   * @brief The number of iterations for the optimisation. Sort of a "manual SQP".
   */
  int nb_iterations_;

  /**
   * @brief The scaling factor for the optimisation problem. It is multiplied to the solution of the QP problem. Used
   * because the heading is assumed to be constant by the formulation of the problem, which is not true if we move by a
   * large distance the middle points.
   */
  double scaling_;

  // dimensions
  /**
   * @brief Number of points in the optimised path.
   */
  size_t n_points_;

  /**
   * @brief Number of splines in the optimised path. Equals to n_points_ - 1.
   */
  size_t n_splines_;

  /**
   * @brief Number of coefficients in the splines. Equals to 4 x n_splines_.
   */
  size_t n_coeff_;

  /**
   * @brief True if the path is closed, false otherwise.
   */
  bool closed_;

  // HPIPM related variables
  /**
   * @brief The HPIPM status of the solver.
   */
  int hpipm_status_;

  /**
   * @brief Structure containing the HPIPM arguments for the dimensions.
   */
  struct d_dense_qp_dim dim_;

  /**
   * @brief Structure containing the HPIPM arguments for the qp.
   */
  struct d_dense_qp qp_;

  /**
   * @brief Structure containing the HPIPM arguments for the solutions.
   */
  struct d_dense_qp_sol qp_sol_;

  /**
   * @brief Structure containing the HPIPM arguments for the solver.
   */
  struct d_dense_qp_ipm_arg arg_;

  /**
   * @brief Structure containing the HPIPM arguments for the workspace.
   */
  struct d_dense_qp_ipm_ws workspace_;

  /**
   * @brief Pointer to the solutions.
   */
  double *v_;

  /**
   * @brief Pointer to the memory used for the dimensions.
   */
  void *dim_mem_;

  /**
   * @brief Pointer to the memory used for the QP problem.
   */
  void *qp_mem_;

  /**
   * @brief Pointer to the memory used for the solutions.
   */
  void *qp_sol_mem_;

  /**
   * @brief Pointer to the memory used for the solver arguments.
   */
  void *ipm_arg_mem_;

  /**
   * @brief Pointer to the memory used for the solver.
   */
  void *ipm_mem_;

  /**
   * @brief Pointer to the memory used for the bound constraints' indices.
   */
  int *idxb_;

  /**
   * @brief Interpolant 2D object to interpolate the middle line of the track.
   */
  std::shared_ptr<BSplineInterpolant2D> interpolant_2d_;

  /**
   * @brief The maximum allowed curvature at each point.
   */
  double max_curvature_;

  /**
   * @brief The maximum allowed distance between two consecutive points.
   */
  double max_distance_;

  /**
   * @brief True if the the first point is fixed
   */
  bool first_point_fixed_;

  /**
   * @brief True if the the last point is fixed
   */
  bool last_point_fixed_;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * PRIVATE METHODS
   *______________________________________________*/
  /**
   * @brief Function that sets up the A, inverse of A, extraction matrix A and T_c matrices for the QP problem. It
   * relies on a specific number of points to be defined in the constructor.
   *
   * @return void
   */
  void SetupAMatrix(const bool first_point_fixed, const bool last_point_fixed);

  /**
   * @brief Function that preprocesses the middle line of the track. It uses a cubic spline to interpolate the middle
   * line after linearly upsampling.
   *
   * @param middle_line The middle line of the track in Cartesian coordinates, column-major.
   *
   * @return Return the middle line linearly resampled to a specific amount of points in a 2d matrix of points in
   * Cartesian coordinates.
   */
  Eigen::MatrixX2d MiddleLinePreprocessing(const Eigen::MatrixX2d &middle_line);

  /**
   * @brief Function that checks the validity of the path. It checks:
   * 1. If the points are ordered.
   * 2. If the curvature at the points is too high -> if it is the case, then this point is discarded (may help in case
   * of a "notch").
   * 3. If the distance between two consecutive points is not too large -> if it is the case, then the path is
   * considered too risky to use.
   *
   * @param reference_points The reference points in a 2d matrix in Cartesian coordinates, column-major.
   *
   * @return 1 if the first condition is not met, 2 if the second condition is not met (in case of spline failure), 3 if
   * the third condition is not met, 0 otherwise.
   */
  int CheckPathValidity(Eigen::MatrixX2d &reference_points);

  /**
   * @brief Function that optimises the path based on the reference points and returns the new optimised points, lateral
   * deviation and curvature profile.
   *
   * @param reference_points The reference points in a 2d matrix in Cartesian coordinates, column-major. Returns the
   * matrix modified.
   * @param curvature_profile The curvature profile of the reference points. Returns the vector modified.
   * @param preprocess A boolean to indicate if the middle line should be preprocessed.
   *
   * @return void
   */
  void OptimisePath(Eigen::MatrixX2d &reference_points, Eigen::VectorXd &curvature_profile, const bool &preprocess,
                    Eigen::MatrixX2d &boundary_points_left, Eigen::MatrixX2d &boundary_points_right);

  /**
   * @brief Function that sets up a linear system of eqautions of natural cubic splines. It defines the b matrix from
   * the reference points and solves it using the predefined inverse of A matrix.
   *
   * @param reference_points The reference points in a 2d matrix in Cartesian coordinates, column-major.
   * @param coeff_x The x coefficients of a natural cubic spline. Returns the vector modified.
   * @param coeff_y The y coefficients of a natural cubic spline. Returns the vector modified.
   *
   * @return void
   */
  void GetSplineCoefficients(const Eigen::MatrixX2d &reference_points, Eigen::MatrixX4d &coeff_x,
                             Eigen::MatrixX4d &coeff_y);

  /**
   * @brief Function that solves the optimisation problem.
   *
   * @param H The Hessian matrix of the QP problem.
   * @param f The linear term of the QP problem.
   * @param normal_vectors_normalised The normal vectors of the reference points.
   * @param reference_points The reference points.
   *
   * @return Returns the optimal reference path as a 2d matrix of points in Cartesian coordinates.
   *
   * @note Here a few other available solver options that can be used: \n
   *
   * d_dense_qp_set_all(...., &qp_): Set all the matrices of the QP problem. \n
   * d_dense_qp_set_A(A, &qp_): A matrix of the equality constraints Ax = b \n
   * d_dense_qp_set_b(b, &qp_): b matrix of the equality constraints Ax = b \n
   * d_dense_qp_set_lb_mask(lb_mask, &qp_): Mask for the lower bounds (enables to dynamically activate or deactivate the
   * single lower constraints) \n d_dense_qp_set_ub_mask(ub_mask, &qp_): Mask for the upper bounds (enables to
   * dynamically activate or deactivate the single upper constraints) \n d_dense_qp_set_C(C.data(), &qp_): C matrix of
   * the general linear inequality constraints Cx ≤ ug, lg ≤ Cx d_dense_qp_set_lg(w_tr_r.data(), &qp_): Sets the lower
   * bounds for the general linear constraints d_dense_qp_set_ug(w_tr_l.data(), &qp_): Sets the upper bounds for the
   * general linear constraints d_dense_qp_set_lg_mask(lg_mask, &qp_): Mask for the lower bounds of the general linear
   * constraints d_dense_qp_set_ug_mask(ug_mask, &qp_): Mask for the upper bounds of the general linear constraints
   * d_dense_qp_set_Zl(Zl, &qp_): Set the diagonal matrix Zl which is used in the context of soft constraints for
   * penalizing the violation of lower bounds d_dense_qp_set_Zu(Zu, &qp_): Set the diagonal matrix Zu which is used in
   * the context of soft constraints for penalizing the violation of upper bounds d_dense_qp_set_zl(zl, &qp_): Set the
   * vector zl which is used in the context of soft constraints for penalizing the violation of lower bounds
   * d_dense_qp_set_zu(zu, &qp_): Set the vector zu which is used in the context of soft constraints for penalizing the
   * violation of upper bounds d_dense_qp_set_lls(lls, &qp_): Sets the lower soft bounds for soft constraints
   * d_dense_qp_set_lus(lus, &qp_): Sets the upper soft bounds for soft constraints
   * d_dense_qp_set_lls_mask(lls_mask, &qp_): Mask for soft constraints
   * d_dense_qp_set_lus_mask(lus_mask, &qp_): Mask for upper constraints
   * d_dense_qp_set_idxs_rev(idxs_rev, &qp_): Sets a reverse mapping for the indices of slack variables introduced to
   * handle soft constraints, typically useful for mapping between different forms or representations within the QP
   * formulation
   */
  bool SolveOptimisation(Eigen::VectorXd &alpha_mincurv, Eigen::MatrixXd &H, Eigen::MatrixXd &f,
                         Eigen::MatrixX2d &normal_vectors_normalised, Eigen::MatrixX2d &reference_points);

  /**
   * @brief Function that sets up the solver arguments for the HPIPM solver.
   *
   * @param args The HPIPM arguments in a structure.
   *
   * @return false if the solver has successfully solved the problem, true otherwise.
   */
  void SetupSolverArgs(hpipm_args &args);

  Eigen::Vector2d findClosestBoundaryPoint(const Eigen::Vector2d &centerPoint, const Eigen::MatrixX2d &boundary);

  Eigen::MatrixX2d computeTrackWidths(const Eigen::MatrixX2d &optimised_points, const Eigen::MatrixX2d &left_boundaries,
                                      const Eigen::MatrixX2d &right_boundaries, bool isClosedTrack, double car_width,
                                      double safety_margin);
};
