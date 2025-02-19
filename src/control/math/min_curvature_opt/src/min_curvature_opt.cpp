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

#include "min_curvature_opt/min_curvature_opt.hpp"

/*******************************************************************************
 * Constructor                                                                 *
 ******************************************************************************/
MinCurvatureOpt::MinCurvatureOpt(hpipm_args args, int nb_iterations, bool first_point_fixed, bool last_point_fixed,
                                 bool closed, double max_curvature, double max_distance, double car_width,
                                 double safety_margin) {
  n_points_ = args.dim_opt;
  closed_ = closed;
  max_curvature_ = max_curvature;
  max_distance_ = max_distance;

  nb_iterations_ = nb_iterations;
  car_width_ = car_width;
  safety_margin_ = safety_margin;

  SetupSolverArgs(args);
  SetupAMatrix(first_point_fixed, last_point_fixed);

  interpolant_2d_ = std::make_unique<BSplineInterpolant2D>(closed);
}

/*******************************************************************************
 * Destructor                                                                  *
 ******************************************************************************/
MinCurvatureOpt::~MinCurvatureOpt() {
  free(v_);
  free(dim_mem_);
  free(qp_mem_);
  free(qp_sol_mem_);
  free(ipm_arg_mem_);
  free(ipm_mem_);
  free(idxb_);
}

/*******************************************************************************
 * SetupAMatrix                                                                *
 ******************************************************************************/
void MinCurvatureOpt::SetupAMatrix(const bool first_point_fixed, const bool last_point_fixed) {
  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Define the linear equation system Ax = b
   * i : current point
   * i+1 : next point
   *
   * current point               | next point              | bounds
   * Row 1: beginning of current spline should be placed on current point (t = 0)
   * ai                          |                         = {x,y}_i
   *
   * Row 2: end of current spline should be placed on next point (t = 1)
   * ai  +   bi  +   ci  +   di  |                         = {x,y}_i+1
   *
   * Row 3: heading at end of current spline should be equal to heading at beginning of next spline (t = 1 and t = 0)
   *         bi  +  2ci  +  3di  |    - bi+1               = 0
   *
   * Row 4: curvature at end of current spline should be equal to curvature at beginning of next spline (t = 1 and t =
   *0) 2ci  +  6di  |             - 2ci+1     = 0
   *
   * End constraints
   * aN                          |                         = {x,y}_N
   * aN  +   bN  +   cN  +   dN  |                         = {x,y}_N+1
   *                2c0          |                         = 0    ------ !!!
   *                2cN          |                         = 0
   *______________________________________________*/
  n_splines_ = n_points_ - 1;
  n_coeff_ = 4 * n_splines_;

  A_ = Eigen::MatrixXd::Zero(4 * n_splines_, 4 * n_splines_);
  for (int i = 0; i < n_splines_; i++) {
    size_t j = i * 4;

    if (i < n_splines_ - 1) {
      A_.block<4, 8>(j, j) << 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 2, 3, 0, -1, 0, 0, 0, 0, 2, 6, 0, 0,
          -2, 0;
    } else {
      if (!closed_) {
        A_.block<4, 4>(j, j) << 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 2, 6;
        A_(A_.rows() - 2, 2) = 2;
      } else {
        A_.block<4, 4>(j, j) << 1, 0, 0, 0, // last point
            1, 1, 1, 1,                     // first point
            0, -1, -2, -3, 0, 0, -2, -6;
        A_(A_.rows() - 2, 1) = 1;
        A_(A_.rows() - 2, 2) = 2;
        A_(A_.rows() - 2, 3) = 3;

        A_(A_.rows() - 1, 2) = 2;
        A_(A_.rows() - 1, 3) = 6;
      }
    }
  }

  A_inv_ = A_.inverse();
  // A_inv = A.lu().inverse();
  // A_inv = A.partialPivLu().inverse();

  Eigen::MatrixXd A_ex_tmp = Eigen::MatrixXd::Zero(n_points_, 4 * n_splines_);
  Eigen::MatrixXd A_ex_b_tmp = Eigen::MatrixXd::Zero(n_points_, 4 * n_splines_);

  for (int i = 0; i < n_splines_; i++) {
    A_ex_tmp(i, i * 4 + 2) = 1;
    A_ex_b_tmp(i, i * 4 + 1) = 1;
  }

  if (!closed_) {
    A_ex_tmp(n_splines_, 4 * n_splines_ - 2) = 1;
    A_ex_tmp(n_splines_, 4 * n_splines_ - 1) = 3;

    A_ex_b_tmp(n_splines_, 4 * n_splines_ - 4) = 0;
    A_ex_b_tmp(n_splines_, 4 * n_splines_ - 3) = 1;
    A_ex_b_tmp(n_splines_, 4 * n_splines_ - 2) = 2;
    A_ex_b_tmp(n_splines_, 4 * n_splines_ - 1) = 3;
  }

  // convert to a sparse matrix
  A_ex_ = A_ex_tmp.sparseView();
  A_ex_b_ = A_ex_b_tmp.sparseView();

  A_ex_.makeCompressed();
  A_ex_b_.makeCompressed();

  // precompute T_c
  T_c_ = 2 * A_ex_ * A_inv_;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute the width of the track
   *______________________________________________*/
  first_point_fixed_ = first_point_fixed;
  last_point_fixed_ = last_point_fixed;

  w_tr_l_ = Eigen::VectorXd::Zero(n_points_);
  w_tr_r_ = Eigen::VectorXd::Zero(n_points_);
}

/*******************************************************************************
 * MiddleLinePreprocessing                                                     *
 ******************************************************************************/
Eigen::MatrixX2d MinCurvatureOpt::MiddleLinePreprocessing(const Eigen::MatrixX2d &middle_line) {
  EASY_FUNCTION(profiler::colors::Black);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Define the local variables
   *______________________________________________*/
  int n_raw_points_ = middle_line.rows();
  Eigen::MatrixX2d resampled_middle_line;
  Eigen::MatrixX2d processed_middle_line;

  double dx, dy;

  resampled_middle_line.resize(n_points_, 2);
  resampled_middle_line.setZero();
  processed_middle_line.resize(n_points_, 2);
  processed_middle_line.setZero();

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Upsample the raw middle line
   *______________________________________________*/
  int upsample_factor = 4;

  Eigen::MatrixXd control_points = middle_line.transpose();

  // Calculate the total number of points after upsampling
  int total_new_points = (n_raw_points_ - 1) * upsample_factor + 1;
  Eigen::MatrixX2d upsampled_raw_middle_line(total_new_points, 2);

  for (int i = 0; i < n_raw_points_ - 1; ++i) {
    // Original point
    upsampled_raw_middle_line.row(i * upsample_factor) = middle_line.row(i);

    // Insert additional points
    for (int j = 1; j < upsample_factor; ++j) {
      double t = static_cast<double>(j) / upsample_factor;
      upsampled_raw_middle_line.row(i * upsample_factor + j) =
          (1 - t) * middle_line.row(i) + t * middle_line.row(i + 1);
    }
  }

  // Set the last point to the last original point
  upsampled_raw_middle_line.row(total_new_points - 1) = middle_line.row(n_raw_points_ - 1);

  // If the line should be closed, connect the last point to the first point
  if (closed_) {
    upsampled_raw_middle_line.row(total_new_points - 1) = upsampled_raw_middle_line.row(0);
  }

  // Update control points and n_raw_points_
  n_raw_points_ = upsampled_raw_middle_line.rows();
  control_points.resize(2, n_raw_points_);
  control_points = upsampled_raw_middle_line.transpose();

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * B-Spline interpolation
   *______________________________________________*/
  Eigen::MatrixX2d spline_points(n_raw_points_, 2);
  double t_step = 0.1;
  int idx_factor = static_cast<int>(1 / t_step);

  if (interpolant_2d_->SetControlPoints(control_points)) {
    return middle_line;
  }

  if (interpolant_2d_->FitCurve(t_step)) {
    return middle_line;
  }

  Eigen::Matrix<double, 2, Eigen::Dynamic> interpolated_points = interpolant_2d_->GetInterpolatedPoints();

  for (int i = 0; i < n_raw_points_ - 1; i++) {
    spline_points(i, 0) = interpolated_points(0, i * idx_factor);
    spline_points(i, 1) = interpolated_points(1, i * idx_factor);
  }
  spline_points(n_raw_points_ - 1, 0) = interpolated_points(0, interpolated_points.cols() - 1);
  spline_points(n_raw_points_ - 1, 1) = interpolated_points(1, interpolated_points.cols() - 1);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Linear resampling
   *______________________________________________*/
  Eigen::VectorXd cumulative_distances(n_raw_points_);
  Eigen::VectorXd interval_lengths(n_raw_points_ - 1);

  Eigen::VectorXd x = spline_points.col(0);
  Eigen::VectorXd y = spline_points.col(1);

  // Calculate cumulative distances
  cumulative_distances(0) = 0;

  for (size_t i = 1; i < n_raw_points_; ++i) {
    dx = x(i) - x(i - 1);
    dy = y(i) - y(i - 1);
    cumulative_distances(i) = cumulative_distances(i - 1) + std::sqrt(dx * dx + dy * dy);
    interval_lengths(i - 1) = cumulative_distances(i) - cumulative_distances(i - 1);
  }

  // Generate evenly spaced distances for interpolation
  Eigen::VectorXd distances_new = Eigen::VectorXd::LinSpaced(n_points_, 0, cumulative_distances(n_raw_points_ - 1));

  // Interpolate based on cumulative distances
  for (int j = 0; j < n_points_; ++j) {
    int i =
        std::lower_bound(cumulative_distances.data(), cumulative_distances.data() + n_raw_points_, distances_new(j)) -
        cumulative_distances.data();
    i = std::max(0, i - 1);

    double t = (distances_new(j) - cumulative_distances(i)) / interval_lengths(i);
    resampled_middle_line(j, 0) = x(i) + t * (x(i + 1) - x(i));
    resampled_middle_line(j, 1) = y(i) + t * (y(i + 1) - y(i));
  }

  return resampled_middle_line;
}

/*******************************************************************************
 * GetOptimalReferencePath                                                     *
 ******************************************************************************/
void MinCurvatureOpt::GetOptimalReferencePath(Eigen::MatrixX2d &reference_points, Eigen::MatrixX2d &lateral_deviation,
                                              Eigen::VectorXd &curvature_profile, const bool &preprocess,
                                              Eigen::MatrixX2d &boundary_points_left,
                                              Eigen::MatrixX2d &boundary_points_right) {
  EASY_FUNCTION(profiler::colors::Green);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Check the validity of the path
   *______________________________________________*/
  if (closed_) {
    int check_status = CheckPathValidity(reference_points);
    switch (check_status) {
    case 0:
      break;
    case 1:
      reference_points = Eigen::MatrixXd::Ones(n_points_, 2);
      return;
    case 2:
      reference_points = Eigen::MatrixXd::Ones(n_points_, 2) * 2;
      return;
    case 3:
      reference_points = Eigen::MatrixXd::Ones(n_points_, 2) * 3;
      return;
    case 4:
      reference_points = Eigen::MatrixXd::Zero(n_points_, 2);
      return;
    default:
      break;
    }
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Optimise the path
   *______________________________________________*/
  for (int i = 0; i < nb_iterations_; i++) {
    OptimisePath(reference_points, curvature_profile, preprocess, boundary_points_left, boundary_points_right);

    if (reference_points == Eigen::MatrixXd::Zero(n_points_, 2)) {
      return;
    }
  }

  lateral_deviation =
      computeTrackWidths(reference_points, boundary_points_left, boundary_points_right, closed_, 0.0, 0.0);
}

/*******************************************************************************
 * ProcessCurvature                                                            *
 ******************************************************************************/
Eigen::VectorXd MinCurvatureOpt::ProcessCurvature(const Eigen::VectorXd &curvature_profile, bool isClosedPath,
                                                  const int &smoothness = 0, const double &sigma = 5.0) {
  if (smoothness <= 0) {
    return curvature_profile;
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Create the Gaussian kernel
   *______________________________________________*/
  Eigen::VectorXd kernel(smoothness);
  double sum = 0.0;
  int halfSize = smoothness / 2;

  for (int i = 0; i < smoothness; ++i) {
    int x = i - halfSize;
    kernel(i) = exp(-0.5 * (x * x) / (sigma * sigma)) / (sigma * sqrt(2.0 * M_PI));
    sum += kernel(i);
  }

  kernel /= sum; // Normalize the kernel

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Apply the Gaussian filter
   *______________________________________________*/
  int dataSize = curvature_profile.size();
  int kernelSize = kernel.size();
  int halfKernelSize = kernelSize / 2;
  Eigen::VectorXd result(dataSize);

  for (int i = 0; i < dataSize; ++i) {
    double sum = 0.0;
    for (int j = 0; j < kernelSize; ++j) {
      int dataIndex = i + j - halfKernelSize;

      // Handle boundary conditions
      if (isClosedPath) {
        // Wrap around
        if (dataIndex < 0) {
          dataIndex += dataSize;
        } else if (dataIndex >= dataSize) {
          dataIndex -= dataSize;
        }
      } else {
        // Clamp to boundaries
        if (dataIndex < 0) {
          dataIndex = 0;
        } else if (dataIndex >= dataSize) {
          dataIndex = dataSize - 1;
        }
      }

      sum += curvature_profile(dataIndex) * kernel(j);
    }
    result(i) = sum;
  }

  return result;
}

/*******************************************************************************
 * GetSpeedTargets                                                             *
 ******************************************************************************/
Eigen::VectorXd MinCurvatureOpt::GetSpeedTargets(const Eigen::VectorXd &curvature_profile, bool isClosedPath,
                                                 double ay_max, double vx_min, double vx_max,
                                                 double kappa_shift = 0.0) {
  Eigen::VectorXd curvature_profile_shifted = curvature_profile;
  curvature_profile_shifted = curvature_profile.array().abs() - kappa_shift;
  curvature_profile_shifted = curvature_profile_shifted.cwiseMax(0.0001);

  Eigen::VectorXd speed_targets = (ay_max / curvature_profile_shifted.array()).sqrt();

  if (isClosedPath) {
    speed_targets(speed_targets.size() - 1) = speed_targets(0);
  } else {
    speed_targets(speed_targets.size() - 1) = speed_targets(speed_targets.size() - 2);
  }

  if (speed_targets.hasNaN()) {
    speed_targets = Eigen::VectorXd::Ones(speed_targets.size()) * vx_min;
  }

  speed_targets = speed_targets.array().cwiseMax(vx_min).cwiseMin(vx_max);

  return speed_targets;
}

/*******************************************************************************
 * CheckPathValidity                                                           *
 ******************************************************************************/
int MinCurvatureOpt::CheckPathValidity(Eigen::MatrixX2d &reference_points) {
  EASY_FUNCTION(profiler::colors::DarkCyan);
  int n = reference_points.rows();
  if (n < 2) {
    return 1;
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Closed path condition
   *______________________________________________*/

  if (closed_) {
    if (reference_points.row(0) != reference_points.row(n - 1)) {
      return 4;
    }
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Consecutive duplicates
   *______________________________________________*/
  Eigen::MatrixX2d reference_points_no_duplicates(n, 2);
  reference_points_no_duplicates.row(0) = reference_points.row(0);

  int no_duplicate_counter = 1;
  for (int i = 1; i < n; ++i) {
    if (reference_points.row(i) != reference_points.row(i - 1)) {
      reference_points_no_duplicates.row(no_duplicate_counter) = reference_points.row(i);
      no_duplicate_counter++;
    }
  }

  reference_points_no_duplicates.conservativeResize(no_duplicate_counter, 2);
  n = reference_points_no_duplicates.rows();

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Points ordering
   *______________________________________________*/
  for (int i = 1; i < n; ++i) {
    Eigen::Vector2d v1 = reference_points_no_duplicates.row(i - 1);
    Eigen::Vector2d v2 = reference_points_no_duplicates.row(i);
    Eigen::Vector2d direction_vector = v2 - v1;

    if (i > 1) {
      Eigen::Vector2d previous_direction_vector =
          reference_points_no_duplicates.row(i - 1) - reference_points_no_duplicates.row(i - 2);

      double dot_product = previous_direction_vector.dot(direction_vector);
      double magnitude_old = previous_direction_vector.norm();
      double magnitude_new = direction_vector.norm();

      if (magnitude_old == 0 || magnitude_new == 0) { // duplicated points
        return 1;
      }

      double cos_theta = dot_product / (magnitude_old * magnitude_new);
      double theta = std::acos(cos_theta) * 180 / M_PI;

      if (cos_theta < 0) {
        return 1;
      }
      // if(theta > 30) {
      //   return 2;
      // }
    } else {
      if (direction_vector.norm() == 0) {
        return 1;
      }
    }
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Curvature
   *______________________________________________*/
  double t_step = 0.05;
  int idx_factor = static_cast<int>(1 / t_step);
  Eigen::MatrixX2d new_reference_points(n, 2);

  if (interpolant_2d_->SetControlPoints(reference_points.transpose())) {
    return 2;
  }

  if (interpolant_2d_->FitCurve(t_step)) {
    return 2;
  }

  Eigen::VectorXd reference_curvature = interpolant_2d_->GetCurvature();

  int counter = 0;
  for (int i = counter; i < n - 1; i++) {
    if (std::abs(reference_curvature(i * idx_factor)) < max_curvature_) {
      new_reference_points(counter, 0) = reference_points(i, 0);
      new_reference_points(counter, 1) = reference_points(i, 1);
      counter++;
    }
  }

  if (std::abs(reference_curvature(reference_curvature.size() - 1)) < max_curvature_) {
    new_reference_points(counter, 0) = reference_points(n - 1, 0);
    new_reference_points(counter, 1) = reference_points(n - 1, 1);
  }

  if (counter == 0) {
    return 2;
  }

  new_reference_points.conservativeResize(counter + 1, 2);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Distance
   *______________________________________________*/
  for (int i = 1; i < new_reference_points.rows(); i++) {
    double distance = (new_reference_points.row(i) - new_reference_points.row(i - 1)).norm();
    if (distance > max_distance_) {
      return 3;
    }
  }

  reference_points = new_reference_points;

  return 0;
}

/*******************************************************************************
 * OptimisePath                                                                *
 ******************************************************************************/
void MinCurvatureOpt::OptimisePath(Eigen::MatrixX2d &reference_points, Eigen::VectorXd &curvature_profile,
                                   const bool &preprocess, Eigen::MatrixX2d &boundary_points_left,
                                   Eigen::MatrixX2d &boundary_points_right) {
  EASY_FUNCTION(profiler::colors::CreamWhite);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Preprocess the reference points
   *______________________________________________*/
  if (preprocess) {
    reference_points = MiddleLinePreprocessing(reference_points);
  } else if ((!closed_) && (reference_points.rows() != n_points_)) {
    return;
  } else {
    reference_points = reference_points;
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute the width of the track
   *______________________________________________*/
  Eigen::MatrixX2d track_widths;

  track_widths = computeTrackWidths(reference_points, boundary_points_left, boundary_points_right, closed_, car_width_,
                                    safety_margin_);

  w_tr_l_ = track_widths.col(0);
  w_tr_r_ = track_widths.col(1);

  size_t n = track_widths.col(0).size();

  if (first_point_fixed_) {
    w_tr_l_(0) = 0.0;
    w_tr_r_(0) = 0.0;
  }
  if (last_point_fixed_) {
    w_tr_l_(n - 1) = 0.0;
    w_tr_r_(n - 1) = 0.0;
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Get the splines coefficients
   *______________________________________________*/
  EASY_BLOCK("Spline coefficients", profiler::colors::Pink);

  Eigen::MatrixX4d coeff_x(n_splines_, 4);
  coeff_x.setZero();
  Eigen::MatrixX4d coeff_y(n_splines_, 4);
  coeff_y.setZero();

  GetSplineCoefficients(reference_points, coeff_x, coeff_y);

  EASY_END_BLOCK;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute the normal vectors
   *   n = [coeff -b_y, coeff b_x]
   *   normal_vectors.col(0) = -y'
   *   normal_vectors.col(1) = x'
   *
   *   last point at t = 1 : [b + 2ct + 3dt²]->[b + 2c + 3d]
   *______________________________________________*/
  EASY_BLOCK("Normal vectors", profiler::colors::Cyan);

  Eigen::MatrixX2d normal_vectors(n_points_, 2);
  Eigen::MatrixX2d normal_vectors_normalised(n_points_, 2);

  for (size_t i = 0; i < n_points_; i++) {
    if (i < n_points_ - 1) {
      normal_vectors(i, 0) = -coeff_y(i, 1);
      normal_vectors(i, 1) = coeff_x(i, 1);
    } else {
      if (!closed_) {
        normal_vectors(i, 0) = -(coeff_y(i - 1, 1) + 2 * coeff_y(i - 1, 2) + 3 * coeff_y(i - 1, 3));
        normal_vectors(i, 1) = coeff_x(i - 1, 1) + 2 * coeff_x(i - 1, 2) + 3 * coeff_x(i - 1, 3);
      } else {
        normal_vectors(i, 0) = normal_vectors(0, 0);
        normal_vectors(i, 1) = normal_vectors(0, 1);
      }
    }
  }

  normal_vectors_normalised = normal_vectors.rowwise().normalized();

  EASY_END_BLOCK;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute the Pxx, Pxy and Pyy matrices
   *   Pxx = y'² / (x'² + y'²)³
   *   Pxy = -2 * x' * y' / (x'² + y'²)³  (minus sign will come from the -y' in the normal vector)
   *   Pyy = x'² / (x'² + y'²)³
   *______________________________________________*/
  EASY_BLOCK("P", profiler::colors::Pink);

  Eigen::VectorXd denominator =
      (normal_vectors.col(1).array().square() + normal_vectors.col(0).array().square()).cube();

  Eigen::VectorXd P_xx_temp = (normal_vectors.col(0).array().square() / denominator.array()).matrix();
  Eigen::VectorXd P_xy_temp =
      ((2 * normal_vectors.col(1).array() * normal_vectors.col(0).array()) / denominator.array()).matrix();
  Eigen::VectorXd P_yy_temp = (normal_vectors.col(1).array().square() / denominator.array()).matrix();

  Eigen::DiagonalMatrix<double, Eigen::Dynamic> P_xx(n_points_);
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> P_xy(n_points_);
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> P_yy(n_points_);

  P_xx.diagonal() = P_xx_temp;
  P_xy.diagonal() = P_xy_temp;
  P_yy.diagonal() = P_yy_temp;

  EASY_END_BLOCK;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute M_x and M_y
   *______________________________________________*/
  EASY_BLOCK("M", profiler::colors::Pink);

  Eigen::SparseMatrix<double> M_x(n_coeff_, n_points_), M_y(n_coeff_, n_points_);

  for (size_t i = 0; i < n_points_ - 1; i++) {
    M_x.insert(i * 4, i) = normal_vectors_normalised(i, 0);
    M_x.insert(i * 4 + 1, i + 1) = normal_vectors_normalised(i + 1, 0);

    M_y.insert(i * 4, i) = normal_vectors_normalised(i, 1);
    M_y.insert(i * 4 + 1, i + 1) = normal_vectors_normalised(i + 1, 1);
  }

  M_x.makeCompressed();
  M_y.makeCompressed();

  EASY_END_BLOCK;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute q_x and q_y
   *______________________________________________*/
  EASY_BLOCK("q", profiler::colors::Cyan);

  Eigen::SparseVector<double> q_x(n_coeff_), q_y(n_coeff_);

  for (size_t i = 0; i < n_points_ - 1; i++) {
    q_x.insert(i * 4) = reference_points(i, 0);
    q_x.insert(i * 4 + 1) = reference_points(i + 1, 0);

    q_y.insert(i * 4) = reference_points(i, 1);
    q_y.insert(i * 4 + 1) = reference_points(i + 1, 1);
  }

  EASY_END_BLOCK;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute T_c, T_nx and T_ny
   *______________________________________________*/
  EASY_BLOCK("T", profiler::colors::Pink);

  Eigen::MatrixXd T_nx(n_points_, n_points_), T_ny(n_points_, n_points_);

  T_nx = T_c_ * M_x;
  T_ny = T_c_ * M_y;

  EASY_END_BLOCK;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute the Hessian matrix
   *    H_x = T_nx.transpose() * P_xx * T_nx
   *    H_xy = T_ny.transpose() * P_xy * T_nx
   *    H_y = T_ny.transpose() * P_yy * T_ny
   *    H = H_x + H_xy + H_y
   *
   *    Pre-computing few products that are used multiple times
   *______________________________________________*/
  EASY_BLOCK("H", profiler::colors::Cyan);

  EASY_BLOCK("T_T", profiler::colors::Lime50);
  Eigen::MatrixXd Tnx_T = T_nx.transpose();
  Eigen::MatrixXd Tny_T = T_ny.transpose();
  EASY_END_BLOCK;

  EASY_BLOCK("T*P", profiler::colors::PaleGold);
  Eigen::MatrixXd T_nx_T_P_xx = Tnx_T * P_xx;
  Eigen::MatrixXd T_ny_T_P_xy = Tny_T * P_xy;
  Eigen::MatrixXd T_ny_T_P_yy = Tny_T * P_yy;
  EASY_END_BLOCK;

  Eigen::MatrixXd H(n_points_, n_points_);

  EASY_BLOCK("H mult", profiler::colors::Coral);
  H.noalias() = T_nx_T_P_xx * T_nx + T_ny_T_P_xy * T_nx + T_ny_T_P_yy * T_ny;
  EASY_END_BLOCK;

  EASY_BLOCK("H symm", profiler::colors::Green);
  // make H symmetric
  H = (H + H.transpose().eval()) * 0.5;
  EASY_END_BLOCK;

  EASY_END_BLOCK;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute the Jacobian matrix
   *    f_x = 2 * q_x * P_xx.transpose() * T_nx.transpose() * T_c
   *    f_y = 2 * q_y * P_yy.transpose() * T_ny.transpose() * T_c
   *    f_xy = T_ny.transpose() * P_xy.transpose() * T_c * q_x +
   *           T_nx.transpose() * P_xy.transpose() * T_c * q_y
   *    f = f_x + f_y + f_xy
   *
   *    P matrices are diagonal, so the there is no need to transpose them
   *______________________________________________*/
  EASY_BLOCK("f", profiler::colors::Pink);

  Eigen::MatrixXd f(n_coeff_, n_coeff_);

  EASY_BLOCK("f inter", profiler::colors::Indigo);
  Eigen::MatrixXd Tcq_x = T_c_ * q_x;
  Eigen::MatrixXd Tcq_y = T_c_ * q_y;
  EASY_END_BLOCK;

  EASY_BLOCK("f mult", profiler::colors::Wheat);
  f.noalias() = 2 * (T_nx_T_P_xx * Tcq_x) + 2 * (T_ny_T_P_yy * Tcq_y) + (T_ny_T_P_xy * Tcq_x) + (Tnx_T * P_xy * Tcq_y);
  EASY_END_BLOCK;

  EASY_END_BLOCK;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Set up the solver
   *______________________________________________*/
  // Solution vector
  Eigen::VectorXd alpha_mincurv(n_points_);

  if (SolveOptimisation(alpha_mincurv, H, f, normal_vectors_normalised, reference_points)) {
    reference_points = Eigen::MatrixXd::Zero(n_points_, 2);
    return;
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Compute the curvature
   *______________________________________________*/
  EASY_BLOCK("k", profiler::colors::Red500);

  // Perform q_x_tmp and q_y_tmp calculations
  Eigen::VectorXd q_x_tmp = q_x + M_x * alpha_mincurv;
  Eigen::VectorXd q_y_tmp = q_y + M_y * alpha_mincurv;

  // Perform x' and y' calculations
  Eigen::VectorXd x_prime_tmp = (A_ex_b_ * A_inv_ * q_x_tmp);
  Eigen::VectorXd y_prime_tmp = (A_ex_b_ * A_inv_ * q_y_tmp);

  // Perform x'' and y'' calculations
  Eigen::VectorXd x_prime_prime = (Tcq_x + T_nx * alpha_mincurv);
  Eigen::VectorXd y_prime_prime = (Tcq_y + T_ny * alpha_mincurv);

  // Initialize the curvature profile
  curvature_profile = Eigen::VectorXd::Zero(n_points_);

  EASY_BLOCK("curv", profiler::colors::Purple);
  for (int i = 0; i < n_points_ - 1; ++i) {
    double x_prime_ii = x_prime_tmp(i);
    double y_prime_ii = y_prime_tmp(i);
    double x_prime_prime_i = x_prime_prime(i);
    double y_prime_prime_i = y_prime_prime(i);

    curvature_profile(i) = (x_prime_ii * y_prime_prime_i - y_prime_ii * x_prime_prime_i) /
                           std::pow(std::pow(x_prime_ii, 2) + std::pow(y_prime_ii, 2), 1.5);
  }
  curvature_profile(n_points_ - 1) = 0.0;
  EASY_END_BLOCK;

  EASY_END_BLOCK;
}

/*******************************************************************************
 * GetSplineCoefficients                                                       *
 *******************************************************************************/
void MinCurvatureOpt::GetSplineCoefficients(const Eigen::MatrixX2d &reference_points, Eigen::MatrixX4d &coeff_x,
                                            Eigen::MatrixX4d &coeff_y) {
  EASY_FUNCTION(profiler::colors::Blue);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Define and fill the b matrix
   *______________________________________________*/
  Eigen::SparseMatrix<double> b(n_coeff_, 2);

  EASY_BLOCK("b", profiler::colors::Brick);

  for (size_t i = 0; i < n_splines_; i++) {
    b.insert(i * 4, 0) = reference_points(i, 0);
    b.insert(i * 4, 1) = reference_points(i, 1);
    b.insert(i * 4 + 1, 0) = reference_points(i + 1, 0);
    b.insert(i * 4 + 1, 1) = reference_points(i + 1, 1);
  }

  b.makeCompressed();

  EASY_END_BLOCK;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Solve the linear equation system
   *   As the inverse of A is already computed, we can directly multiply the inverse with b
   *______________________________________________*/
  EASY_BLOCK("Solve Ax=b", profiler::colors::SkyBlue);

  Eigen::MatrixX2d x = A_inv_ * b;

  EASY_END_BLOCK;

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * Extract the coefficients
   *______________________________________________*/
  EASY_BLOCK("extract coeff", profiler::colors::Teal);

  for (size_t i = 0; i < n_points_ - 1; i++) {
    size_t j = i * 4;

    coeff_x.row(i) << x(j, 0), x(j + 1, 0), x(j + 2, 0), x(j + 3, 0);
    coeff_y.row(i) << x(j, 1), x(j + 1, 1), x(j + 2, 1), x(j + 3, 1);
  }

  // considering the first point will be on itself
  coeff_x(0, 0) = reference_points(0, 0);
  coeff_y(0, 0) = reference_points(0, 1);

  EASY_END_BLOCK;
}

/*******************************************************************************
 * SolveOptimisation                                                           *
 *******************************************************************************/
bool MinCurvatureOpt::SolveOptimisation(Eigen::VectorXd &alpha_mincurv, Eigen::MatrixXd &H, Eigen::MatrixXd &f,
                                        Eigen::MatrixX2d &normal_vectors_normalised,
                                        Eigen::MatrixX2d &reference_points) {
  EASY_FUNCTION(profiler::colors::Green);

  /* Sets the Hessian */
  d_dense_qp_set_H(H.data(), &qp_);
  /* Sets the linear coefficient vector */
  d_dense_qp_set_g(f.data(), &qp_);
  /* Sets the indices of the decision variables x that have box constraints */
  d_dense_qp_set_idxb(idxb_, &qp_);
  /* Sets the lower bounds for the box-constrained variables */
  d_dense_qp_set_lb(w_tr_r_.data(), &qp_);
  /* Sets the upper bounds for the box-constrained variables */
  d_dense_qp_set_ub(w_tr_l_.data(), &qp_);

  if (closed_) {
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(1, n_points_);
    C(0, 0) = 1.0;
    C(0, n_points_ - 1) = -1.0;

    Eigen::VectorXd d(1);
    d << 0.0;

    /* A matrix of the equality constraints Ax = b */
    d_dense_qp_set_A(C.data(), &qp_);
    /* b matrix of the equality constraints Ax = b */
    d_dense_qp_set_b(d.data(), &qp_);
  }

  // Solve the QP problem
  d_dense_qp_ipm_solve(&qp_, &qp_sol_, &arg_, &workspace_);

  //  Get the status of the solver
  d_dense_qp_ipm_get_status(&workspace_, &hpipm_status_);

  // check the output flag
  switch (hpipm_status_) {
  case 0:
    break;
  case 1:
    return true;
    break;
  case 2:
    return true;
    break;
  case 3:
    return true;
    break;
  default:
    return true;
    break;
  }

  // extract the solution into
  d_dense_qp_sol_get_v(&qp_sol_, v_);

  // apply the solution to the reference points
  size_t n = reference_points.rows();
  for (size_t i = 0; i < n; ++i) {
    reference_points(i, 0) += *(v_ + i) * normal_vectors_normalised(i, 0);
    reference_points(i, 1) += *(v_ + i) * normal_vectors_normalised(i, 1);

    alpha_mincurv(i) = *(v_ + i);
  }

  return false;
}

/*******************************************************************************
 * SetupSolverArgs                                                             *
 *******************************************************************************/
void MinCurvatureOpt::SetupSolverArgs(hpipm_args &args) {
  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * dim
   *______________________________________________*/
  /* nv: number of variables */
  int nv = args.dim_opt;

  /* ne: number of equality constraints */
  int ne = 0;
  if (closed_) {
    ne = 1;
  }
  /* nb: number of box constraints (upper & lower bounds) */
  int nb = args.dim_opt;
  /* ng: number of linear constraints (inequalities) */
  int ng = 0;
  /* nsb: number of soft box constraints */
  int nsb = 0;
  /* nsg: number of soft general constraints */
  int nsg = 0;
  /* ns: number of slack variables */
  int ns = 0;

  idxb_ = (int *)malloc(nv * sizeof(int)); // NOLINT
  for (int i = 0; i < nv; ++i) {
    idxb_[i] = i;
  }

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * dense qp dim
   *______________________________________________*/
  hpipm_size_t dim_size = d_dense_qp_dim_memsize();
  dim_mem_ = malloc(dim_size);

  d_dense_qp_dim_create(&dim_, dim_mem_);

  d_dense_qp_dim_set_nv(nv, &dim_);
  d_dense_qp_dim_set_ne(ne, &dim_);
  d_dense_qp_dim_set_nb(nb, &dim_);
  d_dense_qp_dim_set_ng(ng, &dim_);
  d_dense_qp_dim_set_nsb(nsb, &dim_);
  d_dense_qp_dim_set_nsg(nsg, &dim_);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * dense qp
   *______________________________________________*/
  hpipm_size_t qp_size = d_dense_qp_memsize(&dim_);
  qp_mem_ = malloc(qp_size);

  d_dense_qp_create(&dim_, &qp_, qp_mem_);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * dense qp sol
   *______________________________________________*/
  hpipm_size_t qp_sol_size = d_dense_qp_sol_memsize(&dim_);
  qp_sol_mem_ = malloc(qp_sol_size);

  d_dense_qp_sol_create(&dim_, &qp_sol_, qp_sol_mem_);

  v_ = (double *)malloc(nv * sizeof(double)); // NOLINT

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * ipm arg
   *______________________________________________*/
  hpipm_size_t ipm_arg_size = d_dense_qp_ipm_arg_memsize(&dim_);
  ipm_arg_mem_ = malloc(ipm_arg_size);

  hpipm_mode mode_args;
  switch (args.mode) {
  case 0:
    mode_args = SPEED_ABS;
    break;
  case 1:
    mode_args = SPEED;
    break;
  case 2:
    mode_args = BALANCE;
    break;
  case 3:
    mode_args = ROBUST;
    break;
  default:
    mode_args = BALANCE;
    break;
  }

  d_dense_qp_ipm_arg_create(&dim_, &arg_, ipm_arg_mem_);
  d_dense_qp_ipm_arg_set_default(mode_args, &arg_);

  d_dense_qp_ipm_arg_set_mu0(&args.mu0, &arg_);
  d_dense_qp_ipm_arg_set_iter_max(&args.iter_max, &arg_);
  d_dense_qp_ipm_arg_set_alpha_min(&args.alpha_min, &arg_);
  d_dense_qp_ipm_arg_set_mu0(&args.mu0, &arg_);
  d_dense_qp_ipm_arg_set_tol_stat(&args.tol_stat, &arg_);
  d_dense_qp_ipm_arg_set_tol_eq(&args.tol_eq, &arg_);
  d_dense_qp_ipm_arg_set_tol_ineq(&args.tol_ineq, &arg_);
  d_dense_qp_ipm_arg_set_tol_comp(&args.tol_comp, &arg_);
  d_dense_qp_ipm_arg_set_reg_prim(&args.reg_prim, &arg_);
  d_dense_qp_ipm_arg_set_reg_dual(&args.reg_dual, &arg_);
  d_dense_qp_ipm_arg_set_warm_start(&args.warm_start, &arg_);
  d_dense_qp_ipm_arg_set_pred_corr(&args.pred_corr, &arg_);
  d_dense_qp_ipm_arg_set_split_step(&args.split_step, &arg_);

  /*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
   * ipm workspace
   *______________________________________________*/
  hpipm_size_t ipm_size = d_dense_qp_ipm_ws_memsize(&dim_, &arg_);
  ipm_mem_ = malloc(ipm_size);

  d_dense_qp_ipm_ws_create(&dim_, &arg_, &workspace_, ipm_mem_);
}

// Function to find the closest point in a given boundary to the center line point
Eigen::Vector2d MinCurvatureOpt::findClosestBoundaryPoint(const Eigen::Vector2d &centerPoint,
                                                          const Eigen::MatrixX2d &boundary) {
  int numBoundaries = boundary.rows();
  Eigen::Vector2d closestPoint;
  double minDistance = std::numeric_limits<double>::max();

  for (int j = 0; j < numBoundaries; ++j) {
    Eigen::Vector2d boundaryPoint = boundary.row(j);

    double distance = (boundaryPoint - centerPoint).norm();
    if (distance < minDistance) {
      minDistance = distance;
      closestPoint = boundaryPoint;
    }
  }

  return closestPoint;
}

Eigen::MatrixX2d MinCurvatureOpt::computeTrackWidths(const Eigen::MatrixX2d &optimised_points,
                                                     const Eigen::MatrixX2d &left_boundaries,
                                                     const Eigen::MatrixX2d &right_boundaries, bool isClosedTrack,
                                                     double car_width, double safety_margin) {
  int num_points = optimised_points.rows();
  Eigen::MatrixX2d trackWidths(num_points, 2);

  // Calculate the total reduction for each side due to car width and safety margin
  double reduction = (car_width / 2.0) + safety_margin;

  for (int i = 0; i < num_points; ++i) {
    // Compute direction vector
    Eigen::Vector2d direction;
    if (isClosedTrack) {
      // For closed track, use the next point for the last point calculation
      if (i == num_points - 1) {
        direction = optimised_points.row(1) - optimised_points.row(0); // Use first and second points
      } else {
        direction = optimised_points.row((i + 1) % num_points) - optimised_points.row(i);
      }
    } else {
      // Open track: end segments are forward differences
      if (i == num_points - 1) {
        direction = optimised_points.row(i) - optimised_points.row(i - 1);
      } else {
        direction = optimised_points.row(i + 1) - optimised_points.row(i);
      }
    }

    Eigen::Vector2d normal(-direction.y(), direction.x());
    normal = normal.normalized();

    // Find the closest boundary points for left and right boundaries
    Eigen::Vector2d closestLeft = findClosestBoundaryPoint(optimised_points.row(i), left_boundaries);
    Eigen::Vector2d closestRight = findClosestBoundaryPoint(optimised_points.row(i), right_boundaries);

    // Project the boundary points onto the normal to get the width
    double distance_left = std::abs((closestLeft - optimised_points.row(i).transpose()).dot(normal)) - reduction;
    double distance_right = std::abs((closestRight - optimised_points.row(i).transpose()).dot(normal)) - reduction;

    // Ensure distances are non-negative
    distance_left = std::max(0.0, distance_left);
    distance_right = std::max(0.0, distance_right);

    // Store the computed widths (left and right distances)
    trackWidths(i, 0) = distance_left;
    trackWidths(i, 1) = -distance_right;
  }

  // For closed tracks, ensure the first and last widths are the same
  if (isClosedTrack && num_points > 1) {
    trackWidths.row(num_points - 1) = trackWidths.row(0);
  }

  return trackWidths;
}
