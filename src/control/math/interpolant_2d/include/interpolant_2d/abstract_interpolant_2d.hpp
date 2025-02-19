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

#include "autonomous_msgs/msg/boundary.hpp"
#include "autonomous_msgs/msg/point_with_confidence.hpp"
#include "interpolant_1d/linear_interpolant_1d.hpp"
#include "interpolant_1d/nearest_interpolant_1d.hpp"
#include <Eigen/Dense>
#include <memory>

typedef enum { BSPLINE } InterpolantType2D;

/**
 * @brief Abstract class for 2D Interpolants
 */
class AbstractInterpolant2D {
public:
  /**
   * @brief Constructor of the Abstract 2D Interpolator
   */
  AbstractInterpolant2D() = default;

  /**
   * @brief Destructor of the Abstract 2D Interpolator
   */
  virtual ~AbstractInterpolant2D() = default;

  /**
   * @brief gets the curvature at a given progress s
   * (may raise an exception if the interpolant is not capable of computing it)
   *
   * @param s progress on the spline
   *
   * @return curvature curvature at the given progress s
   */
  virtual double GetCurvatureFromS(double s) = 0;

  /**
   * @brief sets the control points for the curve
   *
   *
   * @param control_points_to_fit control points to fit the curve
   *
   * @return returns true if fails
   */
  virtual bool SetControlPoints(const Eigen::Matrix<double, 2, Eigen::Dynamic> &control_points_to_fit) = 0;

  /**
   * @brief gets the point in cartesian coordinate by linearly interpolating between curve points given a progress s
   *
   * @param s progress on the spline
   *
   * @return point in cartesian coordinates
   */
  Eigen::Vector2d GetPointFromS(double s);

  /**
   * @brief fits a parameterized curve through the control points
   *
   * @return return 1 if fails.
   */
  virtual bool FitCurve(double t_step) = 0;

  /**
   * @brief gets the normalized tangent vector to the curve of the point at index idx (cartesian coordinates)
   *
   * @param idx index on the spline
   *
   * @return tangent vector to the curve at the point
   */
  Eigen::Vector2d GetTangentFromIdx(int idx);

  /**
   * @brief gets the points of the curve in cartesian coordinates
   *
   * @return point_vector vector of points of curve in cartesian coordinates
   */
  Eigen::MatrixXd GetInterpolatedPoints();

  /**
   * @brief Returns the 2D tangent vector at the nearest point to the car (0, 0)
   *
   * @return Eigen::Vector2d tangent vector at the nearest point to the car (0, 0)
   */
  virtual Eigen::Vector2d GetTangentFromNearestIdx() = 0;

  /**
   * @brief Returns the lateral deviation from the nearest point to the car (0, 0)
   *
   * @return double lateral deviation from the nearest point to the car (0, 0)
   */
  virtual double GetLateralDeviationFromNearestIdx() = 0;

  /**
   * @brief Returns the progress coordinate of the nearest point to the car (0, 0)
   *
   * @return double progress coordinate of the nearest point to the car (0, 0)
   */
  virtual double GetProgressOfNearestIdx() = 0;

  /**
   * @brief Updates the nearest index to the car (0, 0)
   */
  virtual void UpdateNearestIdx() = 0;

  /**
   * @brief Returns the nearest point to the car (0, 0)
   *
   * @return Eigen::Vector2d nearest point to the car (0, 0)
   */
  virtual Eigen::Vector2d GetNearestPoint() = 0;

  /**
   * @brief gets the number of spline points
   *
   * @return number of spline points
   */
  int GetNumSplinePoints();

protected:
  /**
   * @brief control points for the curve
   */
  Eigen::MatrixXd control_points_;

  /**
   * @brief interpolated points in xy coordiante system
   */
  Eigen::MatrixXd xy_interpolated_points_;

  /**
   * @brief tangent vectors to interpolated points in xy coordiante system
   */
  Eigen::MatrixXd dxy_interpolated_points_;

  /**
   * @brief second derivative vectors to interpolated points in xy coordiante system
   */
  Eigen::MatrixXd ddxy_interpolated_points_;

  /**
   * @brief progress vector of fitted curve
   */
  Eigen::VectorXd s_ref_fit_;

  /**
   * @brief number of spline points
   */
  int num_spline_points_ = 0;
};
