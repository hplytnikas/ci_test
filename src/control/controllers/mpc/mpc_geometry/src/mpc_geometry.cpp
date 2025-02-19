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

#include "mpc_geometry/mpc_geometry.hpp"

MpcGeometry::MpcGeometry(InterpolantType2D interpolant_type_2d) {
  if (interpolant_type_2d == InterpolantType2D::BSPLINE) {
    refpath_interpolant_2d_ = std::make_unique<BSplineInterpolant2D>();
  } else {
    throw std::invalid_argument("Interpolant type not supported");
  }

  num_spline_points_ = 0;
  right_boundary_interpolant_ = std::make_unique<LinearInterpolant1D>();
  left_boundary_interpolant_ = std::make_unique<LinearInterpolant1D>();
  velocity_profile_interpolant_ = std::make_unique<LinearInterpolant1D>();

  // Struct for the solver arguments
  hpipm_args args{
      1,    // 0: speed_abs | 1: speed | 2: balanced | 3: robust
      15,   // iter_max
      1e-4, // alpha_min: minimum step size in the iterative solver process
      1,    // mu0: initial value ot the barrier parameter
      1e-4, // tol_stat: tolerance for the stationarity of the KKT conditions
      1e-4, // tol_eq: tolerance for the equality constraints
      1e-4, // tol_ineq: tolerance for the inequality constraints
      1e-4, // tol_comp: tolerance for the complementarity conditions
      1e-6, // reg_prim: regularization of the primal variables
      1e-6, // reg_dual: regularization of the dual variables
      0,    // warm_start: warm start of the solver
      1,    // pred_corr: predictor-corrector methods -> enhance convergence speed and solution accuracy
      1,    // split_step: if split, the step sizes for primal and dual updates are decoupled
      30    // dim_opt = # points to resample
  };

  double car_width = 1.5;
  double safety_margin = 0.5;
  int nb_iter = 5;
  bool first_point_fixed = true;
  bool last_point_fixed = true;
  bool closed = false;
  double max_curvature = 0.5;
  double max_distance = 9.0;

  local_path_optimizer_ = std::make_shared<MinCurvatureOpt>(args, nb_iter, first_point_fixed, last_point_fixed, closed,
                                                            max_curvature, max_distance, car_width, safety_margin);
}

MpcGeometry::~MpcGeometry() {} // Destructor

bool MpcGeometry::SetTrackBoundaries(const Eigen::VectorXd &rightBoundary, const Eigen::VectorXd &leftBoundary,
                                     const Eigen::Matrix<double, 2, Eigen::Dynamic> &middleLineCoordinates,
                                     bool plan_from_car, double pruning_distance) {
  // initialize the progress of the middle line to zero
  Eigen::VectorXd progress_of_middle_line = Eigen::VectorXd::Zero(middleLineCoordinates.cols());
  // discretely integrate the middle line to get the progress
  for (size_t i = 1; i < middleLineCoordinates.cols(); i++) {
    progress_of_middle_line(i) =
        progress_of_middle_line(i - 1) + (middleLineCoordinates.col(i) - middleLineCoordinates.col(i - 1)).norm();
  }

  if (rightBoundary.size() != progress_of_middle_line.size() || leftBoundary.size() != progress_of_middle_line.size()) {
    return true;
  }

  // set the right boundary interpolant
  right_boundary_interpolant_->SetAbscissaValues(progress_of_middle_line);
  right_boundary_interpolant_->SetOrdinateValues(rightBoundary);

  // set the left boundary interpolant
  left_boundary_interpolant_->SetAbscissaValues(progress_of_middle_line);
  left_boundary_interpolant_->SetOrdinateValues(leftBoundary);

  // if we plan from the car, the track boundary from the car up to pruning distance (in progress) is set to
  // +-1.5m, as the online-estimated track boundaries are then invalid
  if (plan_from_car) {
    Eigen::VectorXd right_boundary_from_car = Eigen::VectorXd::Zero(progress_of_middle_line.size());
    Eigen::VectorXd left_boundary_from_car = Eigen::VectorXd::Zero(progress_of_middle_line.size());

    for (size_t i = 0; i < progress_of_middle_line.size(); i++) {
      if (progress_of_middle_line(i) < pruning_distance) {
        right_boundary_from_car(i) = -1.5;
        left_boundary_from_car(i) = 1.5;
      } else {
        right_boundary_from_car(i) = rightBoundary(i);
        left_boundary_from_car(i) = leftBoundary(i);
      }
    }

    right_boundary_interpolant_->SetOrdinateValues(right_boundary_from_car);
    left_boundary_interpolant_->SetOrdinateValues(left_boundary_from_car);
  }

  return false;
}

bool MpcGeometry::SetVelocityProfile(const Eigen::VectorXd &velocity,
                                     const Eigen::Matrix<double, 2, Eigen::Dynamic> &middleLineCoordinates) {
  // initialize the progress of the middle line to zero
  Eigen::VectorXd progress_of_middle_line = Eigen::VectorXd::Zero(middleLineCoordinates.cols());
  // discretely integrate the middle line to get the progress
  for (size_t i = 1; i < middleLineCoordinates.cols(); i++) {
    progress_of_middle_line(i) =
        progress_of_middle_line(i - 1) + (middleLineCoordinates.col(i) - middleLineCoordinates.col(i - 1)).norm();
  }

  if (velocity.size() != progress_of_middle_line.size()) {
    return true;
  }

  // set the velocity interpolant
  velocity_profile_interpolant_->SetAbscissaValues(progress_of_middle_line);
  velocity_profile_interpolant_->SetOrdinateValues(velocity);

  return false;
}

bool MpcGeometry::FitReferencePath(const Eigen::Matrix<double, 2, Eigen::Dynamic> &middleLineCoordinates,
                                   bool plan_from_car, double pruning_distance) {
  if (plan_from_car) {
    // the middle line coordinates are given in the vehicle frame, so we need to find the nearest point to the car
    // and add the origin (car position) to the middle line coordinates and subsequently only the points in front of the
    // car (pruning_distance)

    // find nearest point (smallest norm) to the car and remember the index
    double min_norm = std::numeric_limits<double>::max();
    size_t nearest_idx = 0;
    for (size_t i = 0; i < middleLineCoordinates.cols(); i++) {
      double norm = middleLineCoordinates.col(i).norm();
      if (norm < min_norm) {
        min_norm = norm;
        nearest_idx = i;
      }
    }

    // create new middle line coordinates and only add the pionts that are in front of the car (after nearest_idx) that
    // have a distance greater than pruning_distance
    Eigen::Matrix<double, 2, Eigen::Dynamic> middleLineCoordinatesFromCar(2,
                                                                          middleLineCoordinates.cols() - nearest_idx);
    // set all to zero
    middleLineCoordinatesFromCar.setZero();
    size_t counter = 1;
    for (size_t i = nearest_idx; i < middleLineCoordinates.cols(); i++) {
      if ((middleLineCoordinates.col(i) - middleLineCoordinates.col(nearest_idx)).norm() > pruning_distance) {
        middleLineCoordinatesFromCar.col(counter) = middleLineCoordinates.col(i);
        counter++;
      }
    }
    middleLineCoordinatesFromCar.conservativeResize(2, counter);

    // transpose the middle line coordinates matrix
    Eigen::Matrix<double, Eigen::Dynamic, 2> middleLineCoordinatesFromCarT = middleLineCoordinatesFromCar.transpose();

    // Optimize local path from car using local optimizer
    Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(counter);
    Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(counter);

    // Optimize local path from car using local optimizer
    // local_path_optimizer_->GetOptimalReferencePath(middleLineCoordinatesFromCarT, lateral_deviation,
    // curvature_profile, true);

    if ((middleLineCoordinatesFromCarT(middleLineCoordinatesFromCarT.rows() - 1, 0) == 0 &&
         middleLineCoordinatesFromCarT(middleLineCoordinatesFromCarT.rows() - 1, 0) == 1) ||
        middleLineCoordinatesFromCarT.hasNaN()) {
      return true;
    }

    if (middleLineCoordinatesFromCarT.hasNaN()) {
      return true;
    } else if (middleLineCoordinatesFromCarT == Eigen::MatrixXd::Ones(counter, 2)) {
      return true;
    } else if (middleLineCoordinatesFromCarT == Eigen::MatrixXd::Ones(counter, 2) * 2) {
      return true;
    } else if (middleLineCoordinatesFromCarT == Eigen::MatrixXd::Ones(counter, 2) * 3) {
      return true;
    } else if (middleLineCoordinatesFromCarT == Eigen::MatrixXd::Zero(counter, 2)) {
      return true;
    }

    // transpose again to get the correct format
    Eigen::Matrix<double, 2, Eigen::Dynamic> middleLineCoordinatesFromCarTT = middleLineCoordinatesFromCarT.transpose();

    if (refpath_interpolant_2d_->SetControlPoints(middleLineCoordinatesFromCarTT)) {
      num_spline_points_ = 0;
      return true;
    }
  } else {
    if (refpath_interpolant_2d_->SetControlPoints(middleLineCoordinates)) {
      num_spline_points_ = 0;
      return true;
    }
  }

  if (refpath_interpolant_2d_->FitCurve(0.1)) {
    num_spline_points_ = 0;
    return true;
  }

  num_spline_points_ = refpath_interpolant_2d_->GetNumSplinePoints();

  return false;
}

bool MpcGeometry::GetInitialState(double &s, double &n, double &mu) {
  // Progress set to 0
  s = 0.0;

  refpath_interpolant_2d_->UpdateNearestIdx();

  // Lateral deviation
  n = refpath_interpolant_2d_->GetLateralDeviationFromNearestIdx();

  // Heading deviation
  Eigen::Vector2d dxy = refpath_interpolant_2d_->GetTangentFromNearestIdx();

  // if norm of dxy is zero, set mu to zero and return true
  if (dxy.norm() == 0) {
    mu = 0.0;
    return true;
  }

  mu = atan2(-dxy(1), dxy(0));

  return false;
}

Eigen::Vector2d MpcGeometry::GetNearestPoint() {
  refpath_interpolant_2d_->UpdateNearestIdx();
  return refpath_interpolant_2d_->GetNearestPoint();
}

bool MpcGeometry::GetCurvatureHorizon(const Eigen::VectorXd &progress_horizon, Eigen::VectorXd &curvature_horizon) {
  // car is in the origin
  refpath_interpolant_2d_->UpdateNearestIdx();

  double s_0 = refpath_interpolant_2d_->GetProgressOfNearestIdx();

  if (curvature_horizon.size() != progress_horizon.size()) {
    return true;
  }

  size_t n = progress_horizon.size();
  for (size_t i = 0; i < n; i++) {
    curvature_horizon(i) = refpath_interpolant_2d_->GetCurvatureFromS(progress_horizon[i] + s_0);
  }

  return false;
}

bool MpcGeometry::GetBoundaries(const Eigen::VectorXd &progress_horizon, Eigen::VectorXd &right_boundary,
                                Eigen::VectorXd &left_boundary) {
  if (right_boundary.size() != progress_horizon.size() || left_boundary.size() != progress_horizon.size()) {
    return true;
  }

  // car is in the origin
  refpath_interpolant_2d_->UpdateNearestIdx();

  double s_0 = refpath_interpolant_2d_->GetProgressOfNearestIdx();

  for (size_t i = 0; i < progress_horizon.size(); i++) {
    right_boundary(i) = right_boundary_interpolant_->GetFvalFromX(progress_horizon[i] + s_0);
    left_boundary(i) = left_boundary_interpolant_->GetFvalFromX(progress_horizon[i] + s_0);
  }

  return false;
}

Eigen::MatrixXd MpcGeometry::GetFittedPath() { return refpath_interpolant_2d_->GetInterpolatedPoints(); }

bool MpcGeometry::GetVelocityProfile(const Eigen::VectorXd &progress_horizon, Eigen::VectorXd &velocity_profile) {
  if (velocity_profile.size() != progress_horizon.size()) {
    return true;
  }

  // car is in the origin
  refpath_interpolant_2d_->UpdateNearestIdx();

  double s_0 = refpath_interpolant_2d_->GetProgressOfNearestIdx();

  for (size_t i = 0; i < progress_horizon.size(); i++) {
    velocity_profile(i) = velocity_profile_interpolant_->GetFvalFromX(progress_horizon[i] + s_0);
  }

  return false;
}
