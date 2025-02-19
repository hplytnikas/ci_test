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
#include <Eigen/Dense>

/**
 * @brief Class for 1D Linear Interpolants
 */
class LinearInterpolant1D : public AbstractInterpolant1D {
public:
  /**
   * @brief Construct a new Linear Interpolant1D object
   */
  LinearInterpolant1D(/* args */) = default;

  /**
   * @brief Destroy the Linear Interpolant1D object
   */
  ~LinearInterpolant1D() = default;

  /**
   * @brief Set the values on the abscissa axis (traditionally x-axis)
   *
   * @param x_values 1D-vector of abscissa values
   * @return void
   */
  void SetAbscissaValues(const Eigen::VectorXd &x_values) override;

  /**
   * @brief Set the values on the ordinate axis (traditionally y-axis)
   *
   * @param f_values 1D-vector of ordinate values
   * @return void
   */
  void SetOrdinateValues(const Eigen::VectorXd &f_values) override;

  /**
   * @brief Linearly interpolate the ordinate value at a given abscissa value,
   *       using the two closest abscissa values,extrapolate by using the closest
   *      abscissa value if the given abscissa value is outside the range of the
   *     abscissa values
   *
   * @param x abscissa value
   *
   * @return interpolated ordinate value
   */
  double GetFvalFromX(const double x) override;

private:
  /* data */
};
