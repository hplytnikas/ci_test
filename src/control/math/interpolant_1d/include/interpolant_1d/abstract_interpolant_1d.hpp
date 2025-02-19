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
#include <stdexcept>

typedef enum { LINEAR, NEAREST } InterpolantType1D;

/**
 * @brief Abstract class for 1D Interpolants
 */
class AbstractInterpolant1D {
public:
  /**
   * @brief Construct a new Abstract Interpolant1D object
   */
  AbstractInterpolant1D(/* args */) = default;

  /**
   * @brief Destroy the Abstract Interpolant1D object
   */
  virtual ~AbstractInterpolant1D() = default;

  /**
   * @brief Set the values on the abscissa axis (traditionally x-axis)
   *
   * @param x_values 1D-vector of abscissa values
   * @return void
   */
  virtual void SetAbscissaValues(const Eigen::VectorXd &x_values) = 0;

  /**
   * @brief Set the values on the ordinate axis (traditionally y-axis)
   *
   * @param f_values 1D-vector of ordinate values
   * @return void
   */
  virtual void SetOrdinateValues(const Eigen::VectorXd &f_values) = 0;

  /**
   * @brief Inter(Extra)polate the ordinate value at a given abscissa value
   *
   * @param x abscissa value
   *
   * @return interpolated ordinate value
   */
  virtual double GetFvalFromX(const double x) = 0;

protected:
  /**
   * @brief values on the abscissa axis (traditionally x-axis)
   */
  Eigen::VectorXd x_values_;
  /**
   * @brief values on the ordinate axis (traditionally y-axis)
   */
  Eigen::VectorXd f_values_;

private:
  /* data */
};
