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

#include "interpolant_1d/linear_interpolant_1d.hpp"

/**
 * Set the values on the abscissa axis (traditionally x-axis)
 *
 * @param x_values 1D-vector of abscissa values
 * @return void
 */
void LinearInterpolant1D::SetAbscissaValues(const Eigen::VectorXd &x_values) {
  // check that this vector is increasing
  for (int i = 0; i < x_values.size() - 1; i++) {
    if (x_values[i] > x_values[i + 1]) {
      return;
    }
  }
  x_values_ = x_values;
}

/**
 * Set the values on the ordinate axis (traditionally y-axis)
 *
 * @param f_values 1D-vector of ordinate values
 * @return void
 */
void LinearInterpolant1D::SetOrdinateValues(const Eigen::VectorXd &f_values) { f_values_ = f_values; }

/**
 * Linearly interpolate the ordinate value at a given abscissa value
 * Extrapolate by using the closest abscissa value if the given abscissa value
 * is outside the range of the abscissa values
 *
 * @param x abscissa value
 *
 * @return interpolated ordinate value
 */
double LinearInterpolant1D::GetFvalFromX(const double x) {
  // check if f_values and x_values_ have the same size
  if (f_values_.size() != x_values_.size()) {
    return 0;
  }

  // check if x_values_ is empty
  if (x_values_.size() == 0) {
    return 0;
  }

  double f = 0.0;

  // check if x is outside the range of x_values_
  if (x <= x_values_[0]) {
    f = f_values_[0];
  } else if (x >= x_values_[x_values_.size() - 1]) {
    f = f_values_[x_values_.size() - 1];
  } else {
    // linearly interpolate by finding the first x value greater than x
    int i = 0;
    while (x_values_[i] < x) {
      i++;
    }

    // linear interpolation
    f = f_values_[i - 1] +
        (f_values_[i] - f_values_[i - 1]) * (x - x_values_[i - 1]) / (x_values_[i] - x_values_[i - 1]);
  }

  return f;
}
