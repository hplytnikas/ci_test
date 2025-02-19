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

#include "interpolant_2d/bspline_interpolant_2d.hpp"

// Constructor
BSplineInterpolant2D::BSplineInterpolant2D(bool closed) {
  this->is_closed_ = closed;
  interpolant_1d_ = std::make_shared<LinearInterpolant1D>();
  this->bspline_coeff_ << -1, 3, -3, 1, 3, -6, 0, 4, -3, 3, 3, 1, 1, 0, 0, 0;
  this->bspline_coeff_ = this->bspline_coeff_ / 6.0;
}

bool BSplineInterpolant2D::SetControlPoints(const Eigen::Matrix<double, 2, Eigen::Dynamic> &control_points_to_fit) {
  // Determine the number of control points
  int num_points = control_points_to_fit.cols();

  // Resize the control_points_ matrix only once
  this->control_points_.resize(2, num_points + 2);
  // Initialize the control points to zeros
  control_points_.setZero();

  if (num_points < 2) {
    return true;
  }

  // Set control points
  control_points_.block(0, 1, 2, num_points) = control_points_to_fit;

  // Check if the path should be closed by comparing the first and last control points within a tolerance
  if (control_points_to_fit.col(0).isApprox(control_points_to_fit.col(num_points - 1), 1e-9)) {
    is_closed_ = true;
    // take the second to last control point as the first control point
    control_points_.col(0) = control_points_.col(num_points - 2);
    // Use the second control point as the last control point
    control_points_.col(num_points + 1) = control_points_.col(1);
  } else {
    is_closed_ = false;
    // Mirror second on first control point
    control_points_.col(0) = 2 * control_points_.col(1) - control_points_.col(2);
    // Mirror second last on last control point if the curve is open
    control_points_.col(num_points + 1) = 2 * control_points_.col(num_points) - control_points_.col(num_points - 1);
  }

  return false;
}

bool BSplineInterpolant2D::FitCurve(double t_step) {
  // Fit a B-spline through the control points
  int num_control_points = control_points_.cols();

  // Calculate the number of points on the spline
  int num_points = static_cast<int>((static_cast<double>(num_control_points - 3)) / t_step) + 1;
  if (is_closed_) {
    num_points += static_cast<int>(1 / t_step); // Add points to close the loop
  }

  // Resize the member variables
  s_ref_fit_.resize(num_points);
  curv_ref_spline_.resize(num_points);
  xy_interpolated_points_.resize(2, num_points);
  dxy_interpolated_points_.resize(2, num_points);
  ddxy_interpolated_points_.resize(2, num_points);

  // Set all to zero
  s_ref_fit_.setZero();
  curv_ref_spline_.setZero();
  xy_interpolated_points_.setZero();
  dxy_interpolated_points_.setZero();
  ddxy_interpolated_points_.setZero();

  // Initialize the reference length
  double s_ref_length = 0.0;

  for (int i = 0; i < num_points; ++i) {
    double t = t_step * i;
    int t_int = static_cast<int>(t);
    double t_frac = t - t_int;

    if (is_closed_) {
      // Wrap around the control points for closed path
      t_int = t_int % (num_control_points - 1);
    }

    inst_ctrl_points_ << control_points_.col(t_int), control_points_.col((t_int + 1) % num_control_points),
        control_points_.col((t_int + 2) % num_control_points), control_points_.col((t_int + 3) % num_control_points);

    inst_t_vec_ << t_frac * t_frac * t_frac, t_frac * t_frac, t_frac, 1.0;
    inst_dt_vec_ << 3.0 * t_frac * t_frac, 2.0 * t_frac, 1.0, 0.0;
    inst_ddt_vec_ << 6.0 * t_frac, 2.0, 0.0, 0.0;

    xy_interpolated_points_.col(i) = inst_ctrl_points_ * bspline_coeff_ * inst_t_vec_;
    dxy_interpolated_points_.col(i) = inst_ctrl_points_ * bspline_coeff_ * inst_dt_vec_;
    ddxy_interpolated_points_.col(i) = inst_ctrl_points_ * bspline_coeff_ * inst_ddt_vec_;

    if (i > 0) {
      s_ref_length += (xy_interpolated_points_.col(i) - xy_interpolated_points_.col(i - 1)).norm();
    } else if (is_closed_ && i == num_points - 1) {
      s_ref_length += (xy_interpolated_points_.col(i) - xy_interpolated_points_.col(0)).norm();
    }

    s_ref_fit_[i] = s_ref_length;

    double norm_dxy = dxy_interpolated_points_.col(i).norm();

    double curv_inst = (dxy_interpolated_points_(0, i) * ddxy_interpolated_points_(1, i) -
                        dxy_interpolated_points_(1, i) * ddxy_interpolated_points_(0, i)) /
                       std::pow(norm_dxy, 3);

    curv_ref_spline_[i] = curv_inst;
  }

  // Ensure the path closes correctly by manually setting the last point for closed paths
  if (is_closed_) {
    xy_interpolated_points_.col(num_points - 1) = xy_interpolated_points_.col(0);
    s_ref_length += (xy_interpolated_points_.col(num_points - 1) - xy_interpolated_points_.col(num_points - 2)).norm();
    s_ref_fit_[num_points - 1] = s_ref_length;
  }

  interpolant_1d_->SetAbscissaValues(s_ref_fit_);
  interpolant_1d_->SetOrdinateValues(curv_ref_spline_);

  num_spline_points_ = num_points;

  return false;
}

Eigen::VectorXd BSplineInterpolant2D::GetCurvature() { return curv_ref_spline_; }

double BSplineInterpolant2D::GetCurvatureFromS(const double s) {
  // get curvature by interpolating the s_ref_fit_ and curv_ref_spline_
  // vectors
  return interpolant_1d_->GetFvalFromX(s);
}

Eigen::Vector2d BSplineInterpolant2D::GetTangentFromNearestIdx() {
  // Get the tangent of the spline at the nearest point to the car
  return this->GetTangentFromIdx(nearest_idx_to_car_);
}

double BSplineInterpolant2D::GetLateralDeviationFromNearestIdx() {
  // Get the lateral deviation of the car from the spline at the nearest point
  // to the car
  Eigen::Vector2d point_on_spline = xy_interpolated_points_.col(nearest_idx_to_car_);
  Eigen::Vector2d tangent = this->GetTangentFromNearestIdx();

  // calculate the normal vector
  Eigen::Vector2d normal;
  normal << -tangent(1), tangent(0);

  // project the vector from path to car onto the normal vector
  return (-point_on_spline).dot(normal);
}

double BSplineInterpolant2D::GetProgressOfNearestIdx() {
  // Get the progress of the car along the spline at the nearest point to the car
  // check if the nearest index is valid
  if (nearest_idx_to_car_ < 0 || nearest_idx_to_car_ >= num_spline_points_) {
    return 0.0;
  }

  return s_ref_fit_[nearest_idx_to_car_];
}

Eigen::Vector2d BSplineInterpolant2D::GetNearestPoint() {
  // Get the nearest point to the car on the spline
  return xy_interpolated_points_.col(nearest_idx_to_car_);
}

void BSplineInterpolant2D::UpdateNearestIdx() {
  // Find the nearest point to the origin (car position)
  double min_dist = std::numeric_limits<double>::max();
  double dist;

  for (int i = 0; i < xy_interpolated_points_.cols(); i++) {
    dist = xy_interpolated_points_.col(i).norm();
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx_to_car_ = i;
    }
  }

  return;
}
