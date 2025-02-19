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

#include "interpolant_2d/abstract_interpolant_2d.hpp"

Eigen::Vector2d AbstractInterpolant2D::GetPointFromS(double s) {
  // check if s is within the range of s_ref_fit_
  if (s < s_ref_fit_[0] || s > s_ref_fit_[s_ref_fit_.size() - 1]) {
    throw std::invalid_argument("s out of range");
  }

  // find the index of the interval where s is located
  int idx = 0;
  while (s > s_ref_fit_[idx + 1]) {
    idx++;
  }

  // get the two points that define the interval
  Eigen::Vector2d p0 = xy_interpolated_points_.col(idx);
  Eigen::Vector2d p1 = xy_interpolated_points_.col(idx + 1);

  // interpolate between the two points
  double t = (s - s_ref_fit_[idx]) / (s_ref_fit_[idx + 1] - s_ref_fit_[idx]);
  return p0 + t * (p1 - p0);
}

Eigen::Vector2d AbstractInterpolant2D::GetTangentFromIdx(int idx) {
  // check if idx is within the range of xy_interpolated_points_
  Eigen::Vector2d tangent = Eigen::Vector2d::Zero();

  if (idx < 0 || idx >= static_cast<int>(xy_interpolated_points_.cols())) {
    return tangent;
  }
  // tangent vectors are stored in dxy_interpolated_points_
  tangent = dxy_interpolated_points_.col(idx);

  // normalize the tangent vector
  return tangent.normalized();
}

Eigen::MatrixXd AbstractInterpolant2D::GetInterpolatedPoints() { return xy_interpolated_points_; }

int AbstractInterpolant2D::GetNumSplinePoints() { return num_spline_points_; }
