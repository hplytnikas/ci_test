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

#include "interpolant_1d/abstract_interpolant_1d.hpp"
#include "interpolant_1d/linear_interpolant_1d.hpp"
#include "interpolant_2d/abstract_interpolant_2d.hpp"
#include <memory>
#include <vector>

/**
 * @brief Class for 2D B-Spline Interpolants
 */
class BSplineInterpolant2D : public AbstractInterpolant2D {
public:
  /**
   * @brief Constructor of the BSplineInterpolant2D class
   */
  explicit BSplineInterpolant2D(bool closed = false);

  /**
   * @brief Destructor of the BSplineInterpolant2D class
   */
  virtual ~BSplineInterpolant2D() = default;

  /**
   * @brief fits a B-spline through the control points
   *
   * @return return 1 if fails.
   */
  bool FitCurve(double t_step) override;

  /**
   * @brief gets the curvature of the spline
   *
   * @return curvature curvature of the spline
   */
  Eigen::VectorXd GetCurvature();

  /**
   * @brief gets the curvature at a given progress s
   *
   * @param s progress on the spline
   *
   * @return curvature curvature at the given progress s
   */
  double GetCurvatureFromS(double s) override;

  /**
   * @brief sets the control points for the spline
   *
   * The first point is mirrored behind the car, then the second point will always be the actual car position (0.0,
   * 0.0), the rest of the points are added and the second to last point is mirrored on the last one.
   *
   * @param boundary_msg middle reference points and boundary points
   *
   * @return returns true if fails
   */
  bool SetControlPoints(const Eigen::Matrix<double, 2, Eigen::Dynamic> &control_points_to_fit) override;

  /**
   * @brief Returns the 2D tangent vector at the nearest point to the car (0, 0)
   *
   * @return Eigen::Vector2d tangent vector at the nearest point to the car (0, 0)
   */
  Eigen::Vector2d GetTangentFromNearestIdx() override;

  /**
   * @brief Returns the lateral deviation from the nearest point to the car (0, 0)
   *
   * @return double lateral deviation from the nearest point to the car (0, 0)
   */
  double GetLateralDeviationFromNearestIdx() override;

  /**
   * @brief Returns the progress coordinate of the nearest point to the car (0, 0)
   *
   * @return double progress coordinate of the nearest point to the car (0, 0)
   */
  double GetProgressOfNearestIdx() override;

  /**
   * @brief Updates the nearest index to the car (0, 0)
   */
  void UpdateNearestIdx() override;

  /**
   * @brief Returns the nearest point to the car (0, 0)
   *
   * @return Eigen::Vector2d nearest point to the car (0, 0)
   */
  Eigen::Vector2d GetNearestPoint() override;

private:
  /**
   * @brief 1D interpolant for interpolating curvature
   */
  std::shared_ptr<AbstractInterpolant1D> interpolant_1d_;

  /**
   * @brief Matrix for bspline coefficients
   */
  Eigen::Matrix4d bspline_coeff_;

  /**
   * @brief Vector for storing the knots of the spline
   */
  Eigen::VectorXd knots_;

  /**
   * @brief Vector for storing the curvature of the spline
   */
  Eigen::VectorXd curv_ref_spline_;

  /**
   * @brief LOCAL knot vector
   */
  Eigen::Vector4d inst_t_vec_;

  /**
   * @brief LOCAL derivative of knot vector w.r.t. t
   */
  Eigen::Vector4d inst_dt_vec_;

  /**
   * @brief LOCAL second derivative of knot vector w.r.t. t
   */
  Eigen::Vector4d inst_ddt_vec_;

  /**
   * @brief Local point to be added to spline
   */
  Eigen::Vector2d xy_point_;

  /**
   * @brief Local derivative of point to be added to spline
   */
  Eigen::Vector2d dxy_point_;

  /**
   * @brief Local second derivative of point to be added to spline
   */
  Eigen::Vector2d ddxy_point_;

  /**
   * @brief Matrix for storing instantaneous control points (2D) of the spline
   */
  Eigen::Matrix<double, 2, 4> inst_ctrl_points_;

  /**
   * @brief storage variable for fractional part of t (knot)
   */
  double t_frac_ = 0.0;

  /**
   * @brief storage variable for integer part of t (knot)
   */
  int t_int_ = 0;

  /**
   * @brief flag if the spline is closed
   */
  bool is_closed_ = false;

  int nearest_idx_to_car_ = 0;
  double s_car_wrt_nearest_ = 0.0;
};
