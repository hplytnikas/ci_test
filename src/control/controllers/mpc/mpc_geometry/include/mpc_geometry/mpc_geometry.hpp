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

#include "autonomous_msgs/msg/boundary.hpp"
#include "autonomous_msgs/msg/point_with_confidence.hpp"
#include "interpolant_2d/bspline_interpolant_2d.hpp"
#include "min_curvature_opt/min_curvature_opt.hpp"

#include <Eigen/Dense>
#include <memory>

/**
 * @brief Class which is owned by the MPC controller and handles the mathematical functions.
 */
class MpcGeometry {
public:
  /**
   * @brief constructor of the MPC geometry class
   */
  explicit MpcGeometry(const InterpolantType2D interpolant_type_2d);

  /**
   * @brief destructor of the MPC geometry class
   */
  ~MpcGeometry();

  /**
   * @brief fits the reference path (only if the middle line is not empty)
   *
   * @param middleLineCoordinates middle reference points
   * @param plan_from_car if true, the reference path is planned from the car
   *
   * @return false if the reference path was fitted successfully
   */
  bool FitReferencePath(const Eigen::Matrix<double, 2, Eigen::Dynamic> &middleLineCoordinates, bool plan_from_car,
                        double pruning_distance);

  /**
   * @brief sets the track boundaries
   *
   * @param rightBoundary right track boundary
   * @param leftBoundary left track boundary
   * @param middleLineCoordinates middle reference points
   *
   * @return false if the track boundaries were set successfully
   */
  bool SetTrackBoundaries(const Eigen::VectorXd &rightBoundary, const Eigen::VectorXd &leftBoundary,
                          const Eigen::Matrix<double, 2, Eigen::Dynamic> &middleLineCoordinates, bool plan_from_car,
                          double pruning_distance);

  /**
   * @brief sets the velocity profile
   *
   * @param velocity velocity profile
   * @param middleLineCoordinates middle reference points
   *
   * @return false if the velocity profile was set successfully
   */
  bool SetVelocityProfile(const Eigen::VectorXd &velocity,
                          const Eigen::Matrix<double, 2, Eigen::Dynamic> &middleLineCoordinates);

  /**
   * @brief computes the initial state of the vehicle in the curvilinear frame given the reference path set previsouly
   *
   * @param s initial progress
   * @param n initial deviation
   * @param mu initial heading
   *
   * One can also use a C++17 feature to get the values: \n
   *
   *  std::tuple<double,double,double> getInitialState(); \n
   *  auto [s, n, mu] = getInitialState(); \n
   *
   * @return false if the initial state was computed successfully
   */
  bool GetInitialState(double &s, double &n, double &mu);

  /**
   * @brief computes the curvature at each given progress point of the reference path set previously
   *
   *
   * @param progress_horizon vector of progress points
   *
   * @return false if the curvature horizon was computed successfully
   */
  bool GetCurvatureHorizon(const Eigen::VectorXd &progress_horizon, Eigen::VectorXd &curvature_horizon);

  /**
   * @brief computes the boundaries for the given progress horizon
   *
   * @param progress_horizon vector of progress points
   * @param right_boundary vector of right track boundaries
   * @param left_boundary vector of left track boundaries
   *
   * @return false if the boundaries were computed successfully
   */
  bool GetBoundaries(const Eigen::VectorXd &progress_horizon, Eigen::VectorXd &right_boundary,
                     Eigen::VectorXd &left_boundary);

  /**
   * @brief computes the velocity profile for the given progress horizon
   *
   * @param progress_horizon vector of progress points
   * @param velocity_profile vector of velocity profile
   *
   * @return false if the velocity profile was computed successfully
   */
  bool GetVelocityProfile(const Eigen::VectorXd &progress_horizon, Eigen::VectorXd &velocity_profile);

  /**
   * @brief computes the fitted path in cartesian coordinates in vehicle frame
   *
   * @param fitted_path vector of points in cartesian coordinates in vehicle frame
   */
  Eigen::MatrixXd GetFittedPath();

  /**
   * @brief Returns number of spline points of the fitted reference path
   *
   * @return number of spline points of the fitted reference path
   */
  const int GetNumSplinePoints() { return num_spline_points_; }

  /**
   * @brief Returns the nearest point to the car which is the origin of the vehicle frame (0, 0)
   */
  Eigen::Vector2d GetNearestPoint();

private:
  /**
   * @brief Interpolant object which handles the interpolation of the reference path
   */
  std::shared_ptr<AbstractInterpolant2D> refpath_interpolant_2d_;

  /**
   * @brief Linear interpolant for the right track boundary
   */
  std::shared_ptr<AbstractInterpolant1D> right_boundary_interpolant_;

  /**
   * @brief Linear interpolant for the left track boundary
   */
  std::shared_ptr<AbstractInterpolant1D> left_boundary_interpolant_;

  /**
   * @brief Linear interpolant for the velocity profile
   */
  std::shared_ptr<AbstractInterpolant1D> velocity_profile_interpolant_;

  /**
   * @brief Number of spline points
   */
  int num_spline_points_;

  /**
   * @brief Pointer to a local path optimizer, used when planning a path from the car
   */
  std::shared_ptr<MinCurvatureOpt> local_path_optimizer_;
};
