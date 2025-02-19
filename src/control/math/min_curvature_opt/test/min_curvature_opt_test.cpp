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

#include "min_curvature_opt/dataset.hpp"
#include "min_curvature_opt/matplotlibcpp.h"
#include "min_curvature_opt/min_curvature_opt.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <gtest/gtest.h>

#include <iomanip>
#include <optional>
#include <sstream>

namespace plt = matplotlibcpp;

/*******************************************************************************
 * HELPER FUNCTIONS
 ******************************************************************************/

/* HSVtoRGB : convert HSV values to RGB values */
void HSVtoRGB(double h, double s, double v, int &r, int &g, int &b) {
  int i = static_cast<int>(h * 6);
  double f = h * 6 - i;
  double p = v * (1 - s);
  double q = v * (1 - f * s);
  double t = v * (1 - (1 - f) * s);

  switch (i % 6) {
  case 0:
    r = static_cast<int>(v * 255);
    g = static_cast<int>(t * 255);
    b = static_cast<int>(p * 255);
    break;
  case 1:
    r = static_cast<int>(q * 255);
    g = static_cast<int>(v * 255);
    b = static_cast<int>(p * 255);
    break;
  case 2:
    r = static_cast<int>(p * 255);
    g = static_cast<int>(v * 255);
    b = static_cast<int>(t * 255);
    break;
  case 3:
    r = static_cast<int>(p * 255);
    g = static_cast<int>(q * 255);
    b = static_cast<int>(v * 255);
    break;
  case 4:
    r = static_cast<int>(t * 255);
    g = static_cast<int>(p * 255);
    b = static_cast<int>(v * 255);
    break;
  case 5:
    r = static_cast<int>(v * 255);
    g = static_cast<int>(p * 255);
    b = static_cast<int>(q * 255);
    break;
  }
}

/* GetColor : compute a color corresponding to the speed */
std::string GetColor(double value, double max_speed) {
  double normalized_value = value / max_speed;
  double clamped_value = std::min(std::max(normalized_value, 0.0), 1.0); // Clamp value between 0 and 1

  // Use the clamped value to determine the hue in the HSV color space
  double hue = clamped_value; // Hue range for rainbow colors (0 to 0.8 for full spectrum)

  int r, g, b;
  HSVtoRGB(hue, 1.0, 1.0, r, g, b);

  std::stringstream ss;
  ss << "#" << std::setfill('0') << std::setw(2) << std::hex << r << std::setfill('0') << std::setw(2) << std::hex << g
     << std::setfill('0') << std::setw(2) << std::hex << b;
  return ss.str();
}

/* Normalize : normalize in the range [0, 1] from the min and max value */
Eigen::VectorXd Normalize(const Eigen::VectorXd &values) {
  double min_val = values.minCoeff();
  double max_val = values.maxCoeff();
  double range = max_val - min_val;

  if (range == 0) {
    // Handle the case where all values are the same
    return Eigen::VectorXd::Constant(values.size(), 0.5); // Set all values to the middle of the gradient
  } else {
    return (values.array() - min_val) / range;
  }
}

void PlotTrack(const std::string &title_name, const int &id, const Eigen::MatrixX2d &optimised_points,
               const std::optional<Eigen::MatrixX2d> &boundary_points_left = std::nullopt,
               const std::optional<Eigen::MatrixX2d> &boundary_points_right = std::nullopt,
               const std::optional<Eigen::MatrixX2d> &reference_points = std::nullopt,
               const std::optional<Eigen::VectorXd> &speed_targets = std::nullopt, const std::string &line_type = "o-m",
               const double &max_speed = 6.0) {
  plt::figure(id);
  plt::grid(false);

  // Extract and plot reference points if available
  if (reference_points) {
    std::vector<double> x(reference_points->rows()), y(reference_points->rows());
    for (int i = 0; i < reference_points->rows(); ++i) {
      x[i] = -reference_points->operator()(i, 1);
      y[i] = reference_points->operator()(i, 0);
    }
    plt::plot(x, y, ".:r");
  }

  // Extract and plot boundary points if available
  if (boundary_points_left) {
    std::vector<double> x_left(boundary_points_left->rows()), y_left(boundary_points_left->rows());
    for (int i = 0; i < boundary_points_left->rows(); ++i) {
      x_left[i] = -boundary_points_left->operator()(i, 1);
      y_left[i] = boundary_points_left->operator()(i, 0);
    }
    plt::plot(x_left, y_left, "<--k");
  }

  if (boundary_points_right) {
    std::vector<double> x_right(boundary_points_right->rows()), y_right(boundary_points_right->rows());
    for (int i = 0; i < boundary_points_right->rows(); ++i) {
      x_right[i] = -boundary_points_right->operator()(i, 1); // Corrected column index
      y_right[i] = boundary_points_right->operator()(i, 0);  // Corrected column index
    }
    plt::plot(x_right, y_right, ">--k");
  }

  // Extract and plot optimised points
  std::vector<double> x_opt(optimised_points.rows()), y_opt(optimised_points.rows());
  for (int i = 0; i < optimised_points.rows(); ++i) {
    x_opt[i] = -optimised_points(i, 1);
    y_opt[i] = optimised_points(i, 0);
  }

  // Plot each point with the corresponding color if speed targets are available
  if (speed_targets) {
    for (size_t i = 0; i < x_opt.size() - 1; ++i) {
      std::string color = GetColor(speed_targets->operator()(i), max_speed);
      plt::plot({x_opt[i], x_opt[i + 1]}, {y_opt[i], y_opt[i + 1]}, {{"linewidth", "4.0"}, {"color", color}});
    }
  } else {
    plt::plot(x_opt, y_opt, line_type);
  }

  plt::axis("equal");
  plt::title(title_name);
}

/* PlotSingleData : plot a single line as a 1D graph */
void PlotSingleData(const Eigen::VectorXd &curvature_profile, const int &id, const bool &ylim = true) {
  plt::figure(id);
  plt::grid(true);

  // Extract and plot reference points
  std::vector<double> y(curvature_profile.size());

  for (int i = 0; i < curvature_profile.size(); ++i) {
    y[i] = curvature_profile(i);
  }
  plt::plot(y);
  if (ylim) {
    plt::ylim(-0.4, 0.4);
  }
}

void PlotSpeedColorScale(double max_speed, int id) {
  plt::figure(id);

  // Generate a range of speeds and corresponding colors
  int num_steps = 500; // Number of steps for smooth transition
  std::vector<double> speeds(num_steps);
  std::vector<std::string> colors(num_steps);

  for (int i = 0; i < num_steps; ++i) {
    double speed = i * max_speed / (num_steps - 1);
    speeds[i] = speed;
    colors[i] = GetColor(speed, max_speed);
  }

  // Plot each speed value with the corresponding color
  for (size_t i = 0; i < speeds.size() - 1; ++i) {
    plt::plot({0, 1}, {speeds[i], speeds[i]}, {{"color", colors[i]}, {"linewidth", "8.0"}});
  }

  plt::xlim(0.4, 0.5);
  plt::ylim(0.0, max_speed);
  plt::title("Speed Color Scale");
}

// Function to compute the normal vector from a direction vector
Eigen::Vector2d computeNormal(const Eigen::Vector2d &direction) {
  Eigen::Vector2d normal(-direction.y(), direction.x());
  return normal.normalized(); // Ensure it's a unit vector
}

// Function to find the closest point in boundaries to the center line point
Eigen::Vector2d findClosestBoundaryPoint(const Eigen::Vector2d &centerPoint, const Eigen::MatrixX4d &boundaries,
                                         bool isLeft) {
  int numBoundaries = boundaries.rows();
  Eigen::Vector2d closestPoint;
  double minDistance = std::numeric_limits<double>::max();

  for (int j = 0; j < numBoundaries; ++j) {
    Eigen::Vector2d boundaryPoint = isLeft ? Eigen::Vector2d(boundaries(j, 0), boundaries(j, 1))
                                           : Eigen::Vector2d(boundaries(j, 2), boundaries(j, 3));

    double distance = (boundaryPoint - centerPoint).norm();
    if (distance < minDistance) {
      minDistance = distance;
      closestPoint = boundaryPoint;
    }
  }

  return closestPoint;
}

Eigen::MatrixX2d computeTrackWidths(const Eigen::MatrixX2d &optimised_points, const Eigen::MatrixX4d &boundaries,
                                    bool isClosedTrack, double car_width, double safety_margin) {
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

    Eigen::Vector2d normal = computeNormal(direction);

    // Find the closest boundary points for left and right boundaries
    Eigen::Vector2d closestLeft = findClosestBoundaryPoint(optimised_points.row(i), boundaries, true);
    Eigen::Vector2d closestRight = findClosestBoundaryPoint(optimised_points.row(i), boundaries, false);

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

/*******************************************************************************
 * TESTS
 *******************************************************************************/
/*TEST(min_curvature_opt, test_create_delete) {
  hpipm_args args{
      1,        // mode
      20,       // iter_max
      1e-5,     // alpha_min
      10,       // mu0
      1e-5,     // tol_stat
      1e-5,     // tol_eq
      1e-5,     // tol_ineq
      1e-5,     // tol_comp
      1e-5,     // reg_prim
      1e-5,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      NB_POINTS // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.3, 15, false, false, false, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  std::cout << ("Minimum curvature optimisation object created.\n") << std::endl;
}

TEST(min_curvature_opt, test_straight_line) {
  hpipm_args args{
      1,        // mode
      20,       // iter_max
      1e-3,     // alpha_min
      10,       // mu0
      1e-3,     // tol_stat
      1e-3,     // tol_eq
      1e-3,     // tol_ineq
      1e-3,     // tol_comp
      1e-3,     // reg_prim
      1e-3,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      NB_POINTS // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.3, 15, true, true, false, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d reference_points;
  Eigen::MatrixX2d optimised_points;
  Eigen::MatrixX4d boundary_points;
  GeneratePointsStraight(reference_points, boundary_points);

  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(NB_POINTS);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(NB_POINTS);

  optimised_points = reference_points;
  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(NB_POINTS, 2)) || optimised_points.hasNaN());

  PlotTrack("Reference + Optimised Straight Line", 100, optimised_points, boundary_points, reference_points,
            std::nullopt, "o-c");
}

TEST(min_curvature_opt, test_corner) {
  hpipm_args args{
      1,        // mode
      20,       // iter_max
      1e-3,     // alpha_min
      10,       // mu0
      1e-3,     // tol_stat
      1e-3,     // tol_eq
      1e-3,     // tol_ineq
      1e-3,     // tol_comp
      1e-3,     // reg_prim
      1e-3,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      NB_POINTS // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.3, 15, true, true, false, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(NB_POINTS);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(NB_POINTS);

  Eigen::MatrixX2d reference_points;
  Eigen::MatrixX2d optimised_points;
  Eigen::MatrixX4d boundary_points;
  GeneratePointsCorner(reference_points, boundary_points);

  optimised_points = reference_points;
  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(NB_POINTS, 2)) || optimised_points.hasNaN());

  PlotTrack("Reference + Optimised Corner", 1, optimised_points, boundary_points, reference_points, std::nullopt,
            "o-c");

  Eigen::VectorXd processed_curvature = min_curvature_opt->ProcessCurvature(curvature_profile, false, 3, 4.0);
  Eigen::VectorXd speed_targets = min_curvature_opt->GetSpeedTargets(curvature_profile, false, 5.0, 5.0, 20.0, 0.0);
  Eigen::VectorXd processed_speed_targets =
      min_curvature_opt->GetSpeedTargets(processed_curvature, false, 5.0, 5.0, 20.0, 0.0);

  PlotSingleData(speed_targets, 31, false);
  PlotSingleData(processed_speed_targets, 31, false);

  PlotSingleData(curvature_profile, 32, false);
  PlotSingleData(processed_curvature, 32, false);
  PlotSingleData(Normalize(speed_targets), 32, false);
  PlotSingleData(Normalize(processed_speed_targets), 32, false);

  PlotSingleData(curvature_profile, 33);
  PlotSingleData(processed_curvature, 33);

  PlotTrack("Trajectory with 25 m/s max || JustSmooth", 60, optimised_points, boundary_points, std::nullopt,
            speed_targets);
  PlotTrack("Trajectory with 25 m/s max || JustSmooth || Processed", 61, optimised_points, boundary_points,
            std::nullopt, processed_speed_targets);

  PlotSingleData(curvature_profile, 200);
}

/*******************************************************************************
 * Test on interpolated data                                                   *
 ******************************************************************************/
/*TEST(min_curvature_opt, test_dataset_1_2_3_4) {
  EASY_PROFILER_ENABLE;

  hpipm_args args{
      1,                   // mode
      20,                  // iter_max
      1e-6,                // alpha_min
      10,                  // mu0
      1e-6,                // tol_stat
      1e-6,                // tol_eq
      1e-6,                // tol_ineq
      1e-6,                // tol_comp
      1e-6,                // reg_prim
      1e-6,                // reg_dual
      0,                   // warm_start
      1,                   // pred_corr
      1,                   // split_step
      middle_line_1.rows() // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.3, 15, true, true, false, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  // Dataset 1 ---------------------------------------------------------------
  Eigen::MatrixX2d optimised_points_1;
  optimised_points_1 = middle_line_1;
  Eigen::VectorXd lateral_deviation_1 = Eigen::VectorXd::Zero(middle_line_1.rows());
  Eigen::VectorXd curvature_profile_1 = Eigen::VectorXd::Zero(middle_line_1.rows());

  min_curvature_opt->GetOptimalReferencePath(optimised_points_1, lateral_deviation_1, curvature_profile_1, false);

  ASSERT_FALSE((optimised_points_1 == Eigen::MatrixXd::Zero(middle_line_1.rows(), 2)) || optimised_points_1.hasNaN());

  PlotTrack("Dataset 1", 2, optimised_points_1, boundary_points_1, middle_line_1, std::nullopt, "o-m");

  // Dataset 2 ---------------------------------------------------------------
  Eigen::MatrixX2d optimised_points_2;
  optimised_points_2 = middle_line_2;
  Eigen::VectorXd lateral_deviation_2 = Eigen::VectorXd::Zero(middle_line_1.rows());
  Eigen::VectorXd curvature_profile_2 = Eigen::VectorXd::Zero(middle_line_1.rows());

  min_curvature_opt->GetOptimalReferencePath(optimised_points_2, lateral_deviation_2, curvature_profile_2, false);

  ASSERT_FALSE((optimised_points_2 == Eigen::MatrixXd::Zero(middle_line_1.rows(), 2)) || optimised_points_2.hasNaN());

  PlotTrack("Dataset 2", 3, optimised_points_2, boundary_points_2, middle_line_2, std::nullopt, "o-m");

  // Dataset 3 ---------------------------------------------------------------
  Eigen::MatrixX2d optimised_points_3;
  optimised_points_3 = middle_line_3;
  Eigen::VectorXd lateral_deviation_3 = Eigen::VectorXd::Zero(middle_line_1.rows());
  Eigen::VectorXd curvature_profile_3 = Eigen::VectorXd::Zero(middle_line_1.rows());

  min_curvature_opt->GetOptimalReferencePath(optimised_points_3, lateral_deviation_3, curvature_profile_3, false);

  ASSERT_FALSE((optimised_points_3 == Eigen::MatrixXd::Zero(middle_line_1.rows(), 2)) || optimised_points_3.hasNaN());

  PlotTrack("Dataset 3", 4, optimised_points_3, boundary_points_3, middle_line_3, std::nullopt, "o-m");

  // Dataset 4 ---------------------------------------------------------------
  Eigen::MatrixX2d optimised_points_4;
  optimised_points_4 = middle_line_4;
  Eigen::VectorXd lateral_deviation_4 = Eigen::VectorXd::Zero(middle_line_1.rows());
  Eigen::VectorXd curvature_profile_4 = Eigen::VectorXd::Zero(middle_line_1.rows());

  min_curvature_opt->GetOptimalReferencePath(optimised_points_4, lateral_deviation_4, curvature_profile_4, false);

  ASSERT_FALSE((optimised_points_4 == Eigen::MatrixXd::Zero(middle_line_1.rows(), 2)) || optimised_points_4.hasNaN());

  PlotTrack("Dataset 4", 5, optimised_points_4, boundary_points_4, middle_line_4, std::nullopt, "o-m");

  profiler::dumpBlocksToFile("MinCurvOpt_Interp.prof");
}

/*******************************************************************************
 * Tests on raw data                                                           *
 ******************************************************************************/
TEST(min_curvature_opt, test_dataset_5) {
  EASY_PROFILER_ENABLE;

  hpipm_args args{
      1,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      NB_POINTS // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 15, true, true, false, 0.5, 9.0, 1.5, 0.3);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_5;
  Eigen::MatrixX2d lateral_deviation = Eigen::MatrixX2d::Zero(NB_POINTS, 2);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(NB_POINTS);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true,
                                             boundary_points_5_left, boundary_points_5_right);

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(NB_POINTS, 2)) || optimised_points.hasNaN());

  PlotTrack("Dataset 5", 2, optimised_points, boundary_points_5_left, boundary_points_5_right, middle_line_5,
            std::nullopt, "o-g");

  PlotSingleData(1.5 - lateral_deviation.col(0).array(), 2004, false);
  PlotSingleData(-1.5 - lateral_deviation.col(1).array(), 2004, false);

  /*Eigen::VectorXd processed_curvature = min_curvature_opt->ProcessCurvature(curvature_profile, false, 5, 5.0);
  Eigen::VectorXd speed_targets = min_curvature_opt->GetSpeedTargets(curvature_profile, false, 5.0, 5.0, 20.0, 0.0);
  Eigen::VectorXd processed_speed_targets =
      min_curvature_opt->GetSpeedTargets(processed_curvature, false, 5.0, 5.0, 20.0, 0.0);

  PlotSingleData(speed_targets, 5031, false);
  PlotSingleData(processed_speed_targets, 5031, false);

  PlotSingleData(curvature_profile, 5032, false);
  PlotSingleData(processed_curvature, 5032, false);
  PlotSingleData(Normalize(speed_targets), 5032, false);
  PlotSingleData(Normalize(processed_speed_targets), 5032, false);

  PlotSingleData(curvature_profile, 5050);
  PlotSingleData(processed_curvature, 5050);

  PlotTrack("Trajectory with 25 m/s max || JustSmooth", 5060, optimised_points, boundary_points_5, std::nullopt,
            speed_targets);
  PlotTrack("Trajectory with 25 m/s max || JustSmooth || Processed", 5061, optimised_points, boundary_points_5,
            std::nullopt, processed_speed_targets);

  profiler::dumpBlocksToFile("MinCurvOpt_Raw_dataset_5.prof");*/
}

/*TEST(min_curvature_opt, test_dataset_6_Smoothandsafe) {
  EASY_PROFILER_ENABLE;

  hpipm_args args{
      1,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      NB_POINTS // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.1, 15, true, true, false, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_6;
  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(NB_POINTS);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(NB_POINTS);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(NB_POINTS, 2)) || optimised_points.hasNaN());

  PlotTrack("Dataset 6", 3, optimised_points, boundary_points_6, middle_line_6, std::nullopt, "o-k");

  profiler::dumpBlocksToFile("MinCurvOpt_Raw_dataset_6.prof");
}

TEST(min_curvature_opt, test_dataset_6_balanced) {
  EASY_PROFILER_ENABLE;

  hpipm_args args{
      1,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      NB_POINTS // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.3, 15, true, true, false, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_6;
  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(NB_POINTS);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(NB_POINTS);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(NB_POINTS, 2)) || optimised_points.hasNaN());

  PlotTrack("Dataset 6", 3, optimised_points, boundary_points_6, middle_line_6, std::nullopt, "o-b");

  profiler::dumpBlocksToFile("MinCurvOpt_Raw_dataset_6.prof");
}

TEST(min_curvature_opt, test_dataset_6_aggressive) {
  EASY_PROFILER_ENABLE;

  hpipm_args args{
      1,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      NB_POINTS // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.5, 15, true, true, false, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_6;
  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(NB_POINTS);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(NB_POINTS);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(NB_POINTS, 2)) || optimised_points.hasNaN());

  PlotTrack("Dataset 6", 3, optimised_points, boundary_points_6, middle_line_6, std::nullopt, "o-g");

  profiler::dumpBlocksToFile("MinCurvOpt_Raw_dataset_6.prof");
}

TEST(min_curvature_opt, test_dataset_6_hitCones) {
  EASY_PROFILER_ENABLE;

  hpipm_args args{
      1,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      NB_POINTS // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.7, 15, true, true, false, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_6;
  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(NB_POINTS);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(NB_POINTS);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(NB_POINTS, 2)) || optimised_points.hasNaN());

  PlotTrack("Dataset 6", 3, optimised_points, boundary_points_6, middle_line_6, std::nullopt, "o-r");

  profiler::dumpBlocksToFile("MinCurvOpt_Raw_dataset_6.prof");
}

TEST(min_curvature_opt, test_dataset_7) {
  EASY_PROFILER_ENABLE;

  hpipm_args args{
      1,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      NB_POINTS // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.3, 15, true, true, false, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_7;
  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(NB_POINTS);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(NB_POINTS);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(NB_POINTS, 2)) || optimised_points.hasNaN());

  PlotTrack("Dataset 7", 4, optimised_points, boundary_points_7, middle_line_7, std::nullopt, "o-g");

  profiler::dumpBlocksToFile("MinCurvOpt_Raw_dataset_7.prof");
}

TEST(min_curvature_opt, test_dataset_8) {
  EASY_PROFILER_ENABLE;

  hpipm_args args{
      1,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      NB_POINTS // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.3, 15, true, true, false, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_8;
  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(NB_POINTS);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(NB_POINTS);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(NB_POINTS, 2)) || optimised_points.hasNaN());

  PlotTrack("Dataset 8", 5, optimised_points, boundary_points_8, middle_line_8, std::nullopt, "o-g");

  PlotSingleData(curvature_profile, 309);

  profiler::dumpBlocksToFile("MinCurvOpt_Raw_dataset_8.prof");
}

/*******************************************************************************
 * Tests on closed track                                                       *
 ******************************************************************************/
/*TEST(min_curvature_opt, test_dataset_17_JustSmooth) {
  EASY_PROFILER_ENABLE;

  int nb_points = 400;

  hpipm_args args{
      2,        // mode
      50,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      nb_points // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.3, 25, false, false, true, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_17;
  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(nb_points);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(nb_points);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  if (optimised_points.hasNaN()) {
    std::cout << "Optimiser failed : NaN values in the path" << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Ones(nb_points, 2)) {
    std::cout << "Optimiser failed : Points are not ordered correctly (or duplicated)." << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 2) {
    std::cout << "Optimiser failed : Issues while checking the curvature" << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 3) {
    std::cout << "Optimiser failed : Missing a point" << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) {
    std::cout << "Optimiser failed" << std::endl;
  }

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) || optimised_points.hasNaN());

  PlotTrack("JustSmooth // Dataset 17", 17, optimised_points, boundary_points_17, middle_line_17, std::nullopt, "o-b");

  Eigen::VectorXd processed_curvature = min_curvature_opt->ProcessCurvature(curvature_profile, true, 5, 5.0);
  Eigen::VectorXd speed_targets = min_curvature_opt->GetSpeedTargets(curvature_profile, true, 5.0, 5.0, 20.0, 0.0);
  Eigen::VectorXd processed_speed_targets =
      min_curvature_opt->GetSpeedTargets(processed_curvature, true, 5.0, 5.0, 20.0, 0.0);

  PlotSingleData(speed_targets, 1711, false);
  PlotSingleData(processed_speed_targets, 1711, false);

  PlotSingleData(curvature_profile, 1712, false);
  PlotSingleData(processed_curvature, 1712, false);
  PlotSingleData(Normalize(speed_targets), 1712, false);
  PlotSingleData(Normalize(processed_speed_targets), 1712, false);

  PlotSingleData(curvature_profile, 1720);
  PlotSingleData(processed_curvature, 1720);

  PlotTrack("Trajectory with 25 m/s max || JustSmooth", 1717, optimised_points, boundary_points_17, std::nullopt,
            speed_targets);
  PlotTrack("Trajectory with 25 m/s max || JustSmooth || Processed", 1731, optimised_points, boundary_points_17,
            std::nullopt, processed_speed_targets);

  PlotSpeedColorScale(25.0, 4000);
}

/*TEST(min_curvature_opt, test_dataset_17_balanced) {
  EASY_PROFILER_ENABLE;

  int nb_points = 400;

  hpipm_args args{
      2,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      nb_points // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.3, 15, false, false, true, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_17;

  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(nb_points);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(nb_points);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  if(optimised_points.hasNaN()) {
    std::cout << "Optimiser failed : NaN values in the path" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2)) {
    std::cout << "Optimiser failed : Points are not ordered correctly (or duplicated)." << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 2) {
    std::cout << "Optimiser failed : Issues while checking the curvature" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 3) {
    std::cout << "Optimiser failed : Missing a point" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) {
    std::cout << "Optimiser failed" << std::endl;
  }

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) || optimised_points.hasNaN());

  PlotTrack("Balanced // Dataset 17", 17, optimised_points, boundary_points_17, middle_line_17, std::nullopt, "o-b");

  Eigen::VectorXd processed_curvature = min_curvature_opt->ProcessCurvature(curvature_profile, true, 5, 5.0);
  Eigen::VectorXd speed_targets = min_curvature_opt->GetSpeedTargets(curvature_profile, true, 5.0, 5.0, 20.0, 0.0);
  Eigen::VectorXd processed_speed_targets =
      min_curvature_opt->GetSpeedTargets(processed_curvature, true, 5.0, 5.0, 20.0, 0.0);

  PlotSingleData(speed_targets, 1711, false);
  PlotSingleData(processed_speed_targets, 1711, false);

  PlotSingleData(curvature_profile, 1712, false);
  PlotSingleData(processed_curvature, 1712, false);
  PlotSingleData(Normalize(speed_targets), 1712, false);
  PlotSingleData(Normalize(processed_speed_targets), 1712, false);

  PlotSingleData(curvature_profile, 1720);
  PlotSingleData(processed_curvature, 1720);

  PlotTrack("Trajectory with 25 m/s max || Balanced", 1717, optimised_points, boundary_points_17, std::nullopt,
            speed_targets);
  PlotTrack("Trajectory with 25 m/s max || Balanced || Processed", 1731, optimised_points, boundary_points_17,
            std::nullopt, processed_speed_targets);

  profiler::dumpBlocksToFile("MinCurvOpt_dataset_17.prof");
}

TEST(min_curvature_opt, test_dataset_17_aggressive) {
  EASY_PROFILER_ENABLE;

  int nb_points = 400;

  hpipm_args args{
      2,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      nb_points // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.4, 15, false, false, true, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_17;

  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(nb_points);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(nb_points);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  if(optimised_points.hasNaN()) {
    std::cout << "Optimiser failed : NaN values in the path" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2)) {
    std::cout << "Optimiser failed : Points are not ordered correctly (or duplicated)." << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 2) {
    std::cout << "Optimiser failed : Issues while checking the curvature" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 3) {
    std::cout << "Optimiser failed : Missing a point" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) {
    std::cout << "Optimiser failed" << std::endl;
  }

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) || optimised_points.hasNaN());

  PlotTrack("Aggressive // Dataset 17", 17, optimised_points, boundary_points_17, middle_line_17, std::nullopt, "o-g");

  Eigen::VectorXd processed_curvature = min_curvature_opt->ProcessCurvature(curvature_profile, true, 5, 5.0);
  Eigen::VectorXd speed_targets = min_curvature_opt->GetSpeedTargets(curvature_profile, true, 5.0, 5.0, 20.0, 0.0);
  Eigen::VectorXd processed_speed_targets =
      min_curvature_opt->GetSpeedTargets(processed_curvature, true, 5.0, 5.0, 20.0, 0.0);

  PlotSingleData(speed_targets, 1711, false);
  PlotSingleData(processed_speed_targets, 1711, false);

  PlotSingleData(curvature_profile, 1712, false);
  PlotSingleData(processed_curvature, 1712, false);
  PlotSingleData(Normalize(speed_targets), 1712, false);
  PlotSingleData(Normalize(processed_speed_targets), 1712, false);

  PlotSingleData(curvature_profile, 1720);
  PlotSingleData(processed_curvature, 1720);

  PlotTrack("Trajectory with 25 m/s max || Aggressive", 1717, optimised_points, boundary_points_17, std::nullopt,
            speed_targets);
  PlotTrack("Trajectory with 25 m/s max || Aggressive || Processed", 1731, optimised_points, boundary_points_17,
            std::nullopt, processed_speed_targets);

  profiler::dumpBlocksToFile("MinCurvOpt_dataset_17.prof");
}

TEST(min_curvature_opt, test_dataset_17_hitCones) {
  EASY_PROFILER_ENABLE;

  int nb_points = 400;

  hpipm_args args{
      2,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      nb_points // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.7, 15, false, false, true, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_17;

  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(nb_points);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(nb_points);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  if(optimised_points.hasNaN()) {
    std::cout << "Optimiser failed : NaN values in the path" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2)) {
    std::cout << "Optimiser failed : Points are not ordered correctly (or duplicated)." << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 2) {
    std::cout << "Optimiser failed : Issues while checking the curvature" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 3) {
    std::cout << "Optimiser failed : Missing a point" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) {
    std::cout << "Optimiser failed" << std::endl;
  }

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) || optimised_points.hasNaN());

  PlotTrack("MakesSureCarHitsCones // Dataset 17", 17, optimised_points, boundary_points_17, middle_line_17,
            std::nullopt, "o-b");

  Eigen::VectorXd processed_curvature = min_curvature_opt->ProcessCurvature(curvature_profile, true, 5, 5.0);
  Eigen::VectorXd speed_targets = min_curvature_opt->GetSpeedTargets(curvature_profile, true, 5.0, 5.0, 20.0, 0.0);
  Eigen::VectorXd processed_speed_targets =
      min_curvature_opt->GetSpeedTargets(processed_curvature, true, 5.0, 5.0, 20.0, 0.0);

  PlotSingleData(speed_targets, 1711, false);
  PlotSingleData(processed_speed_targets, 1711, false);

  PlotSingleData(curvature_profile, 1712, false);
  PlotSingleData(processed_curvature, 1712, false);
  PlotSingleData(Normalize(speed_targets), 1712, false);
  PlotSingleData(Normalize(processed_speed_targets), 1712, false);

  PlotSingleData(curvature_profile, 1720);
  PlotSingleData(processed_curvature, 1720);

  PlotTrack("Trajectory with 25 m/s max || MakesSureCarHitsCones", 1717, optimised_points, boundary_points_17,
            std::nullopt, speed_targets);
  PlotTrack("Trajectory with 25 m/s max || MakesSureCarHitsCones || Processed", 1731, optimised_points,
            boundary_points_17, std::nullopt, processed_speed_targets);
}

TEST(min_curvature_opt, test_dataset_18_balanced) {
  EASY_PROFILER_ENABLE;

  int nb_points = 150;

  hpipm_args args{
      2,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      nb_points // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.3, 15, false, false, true, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_18;

  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(nb_points);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(nb_points);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  if(optimised_points.hasNaN()) {
    std::cout << "Optimiser failed : NaN values in the path" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2)) {
    std::cout << "Optimiser failed : Points are not ordered correctly (or duplicated)." << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 2) {
    std::cout << "Optimiser failed : Issues while checking the curvature" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 3) {
    std::cout << "Optimiser failed : Missing a point" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) {
    std::cout << "Optimiser failed" << std::endl;
  }

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) || optimised_points.hasNaN());

  PlotTrack("Balanced // Dataset 18", 18, optimised_points, std::nullopt, middle_line_18, std::nullopt, "o-b");

  Eigen::VectorXd processed_curvature = min_curvature_opt->ProcessCurvature(curvature_profile, true, 5, 5.0);
  Eigen::VectorXd speed_targets = min_curvature_opt->GetSpeedTargets(curvature_profile, true, 5.0, 5.0, 20.0, 0.0);
  Eigen::VectorXd processed_speed_targets =
      min_curvature_opt->GetSpeedTargets(processed_curvature, true, 5.0, 5.0, 20.0, 0.0);

  PlotSingleData(speed_targets, 1801, false);
  // PlotSingleData(processed_speed_targets, 1801, false);

  PlotSingleData(curvature_profile, 1802, false);
  // PlotSingleData(processed_curvature, 1802, false);
  PlotSingleData(Normalize(speed_targets), 1802, false);
  // PlotSingleData(Normalize(processed_speed_targets), 1802, false);

  PlotSingleData(curvature_profile, 1803);
  PlotSingleData(processed_curvature, 1803);

  PlotTrack("Trajectory with 25 m/s max || Balanced", 1804, optimised_points, std::nullopt, std::nullopt,
            speed_targets);
  // PlotTrack("Trajectory with 25 m/s max || Balanced || Processed", 1805, optimised_points, std::nullopt,
  // std::nullopt, processed_speed_targets);

  profiler::dumpBlocksToFile("MinCurvOpt_dataset_18.prof");
}

TEST(min_curvature_opt, test_dataset_18_balanced_2) {
  EASY_PROFILER_ENABLE;

  int nb_points = 500;

  hpipm_args args{
      2,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      nb_points // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 0.3, 15, false, false, true, 0.5, 9.0);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_18;

  Eigen::VectorXd lateral_deviation = Eigen::VectorXd::Zero(nb_points);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(nb_points);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true);

  if(optimised_points.hasNaN()) {
    std::cout << "Optimiser failed : NaN values in the path" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2)) {
    std::cout << "Optimiser failed : Points are not ordered correctly (or duplicated)." << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 2) {
    std::cout << "Optimiser failed : Issues while checking the curvature" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 3) {
    std::cout << "Optimiser failed : Missing a point" << std::endl;
  } else if(optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) {
    std::cout << "Optimiser failed" << std::endl;
  }

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) || optimised_points.hasNaN());

  PlotTrack("Balanced // Dataset 18", 18, optimised_points, std::nullopt, middle_line_18, std::nullopt, "o-g");

  Eigen::VectorXd processed_curvature = min_curvature_opt->ProcessCurvature(curvature_profile, true, 5, 5.0);
  Eigen::VectorXd speed_targets = min_curvature_opt->GetSpeedTargets(curvature_profile, true, 5.0, 5.0, 20.0, 0.0);
  Eigen::VectorXd processed_speed_targets =
      min_curvature_opt->GetSpeedTargets(processed_curvature, true, 5.0, 5.0, 20.0, 0.0);

  PlotSingleData(speed_targets, 1811, false);
  // PlotSingleData(processed_speed_targets, 1811, false);

  PlotSingleData(curvature_profile, 1812, false);
  // PlotSingleData(processed_curvature, 1812, false);
  PlotSingleData(Normalize(speed_targets), 1812, false);
  // PlotSingleData(Normalize(processed_speed_targets), 1812, false);

  PlotSingleData(curvature_profile, 1813);
  PlotSingleData(processed_curvature, 1813);

  PlotTrack("Trajectory with 25 m/s max || Balanced", 1814, optimised_points, std::nullopt, std::nullopt,
            speed_targets);
  // PlotTrack("Trajectory with 25 m/s max || Balanced || Processed", 1815, optimised_points, std::nullopt,
  // std::nullopt, processed_speed_targets);

  profiler::dumpBlocksToFile("MinCurvOpt_dataset_18.prof");
}*/

TEST(min_curvature_opt, test_dataset_19_balanced) {
  EASY_PROFILER_ENABLE;

  int nb_points = 600;

  hpipm_args args{
      2,        // mode
      20,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      nb_points // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 25, false, false, true, 0.5, 9.0, 1.5, 0.5);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_17;

  Eigen::MatrixX2d lateral_deviation = Eigen::MatrixX2d::Zero(nb_points, 2);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(nb_points);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true,
                                             boundary_points_17_left, boundary_points_17_right);

  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) || optimised_points.hasNaN());

  PlotTrack("Balanced // Dataset 19", 18, optimised_points, boundary_points_17_left, boundary_points_17_right,
            middle_line_17, std::nullopt, "o-m");

  PlotSingleData(lateral_deviation.col(0), 2003, false);
  PlotSingleData(lateral_deviation.col(1), 2003, false);

  profiler::dumpBlocksToFile("MinCurvOpt_dataset_18.prof");
}

TEST(min_curvature_opt, test_dataset_20_FSeast) {
  EASY_PROFILER_ENABLE;

  int track_length = 220;
  int nb_points = static_cast<int>(std::ceil(track_length / 0.5));

  hpipm_args args{
      2,        // mode
      50,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      nb_points // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 25, false, false, true, 0.5, 9.0, 1.5, 0.5);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_20;

  // Eigen::MatrixX2d widths = computeTrackWidths(optimised_points, boundary_points_20, true, 1.5, 0.5);

  Eigen::MatrixX2d lateral_deviation = Eigen::MatrixX2d::Zero(nb_points, 2);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(nb_points);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true,
                                             boundary_points_20_left, boundary_points_20_right);

  PlotSingleData(lateral_deviation.col(0), 2001, false);
  PlotSingleData(lateral_deviation.col(1), 2001, false);

  if (optimised_points.hasNaN()) {
    std::cout << "Optimiser failed : NaN values in the path" << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Ones(nb_points, 2)) {
    std::cout << "Optimiser failed : Points are not ordered correctly (or duplicated)." << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 2) {
    std::cout << "Optimiser failed : Issues while checking the curvature" << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 3) {
    std::cout << "Optimiser failed : Missing a point" << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) {
    std::cout << "Optimiser failed" << std::endl;
  }

  ASSERT_FALSE(optimised_points.hasNaN());
  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)));

  PlotTrack("Balanced // Dataset 20", 20, optimised_points, boundary_points_20_left, boundary_points_20_right,
            middle_line_20, std::nullopt, "o-g");

  Eigen::VectorXd processed_curvature = min_curvature_opt->ProcessCurvature(curvature_profile, true, 30, 5.0);
  // Eigen::VectorXd speed_targets = min_curvature_opt->GetSpeedTargets(curvature_profile, true, 5.0, 5.0, 20.0, 0.05);
  Eigen::VectorXd processed_speed_targets =
      min_curvature_opt->GetSpeedTargets(processed_curvature, true, 0.8, 0.0001, 6.0, 0.0);
}

TEST(min_curvature_opt, test_dataset_20_FSeast_2) {
  EASY_PROFILER_ENABLE;

  int track_length = 220;
  int nb_points = static_cast<int>(std::ceil(track_length / 0.5));

  hpipm_args args{
      2,        // mode
      50,       // iter_max
      1e-6,     // alpha_min
      10,       // mu0
      1e-6,     // tol_stat
      1e-6,     // tol_eq
      1e-6,     // tol_ineq
      1e-6,     // tol_comp
      1e-6,     // reg_prim
      1e-6,     // reg_dual
      0,        // warm_start
      1,        // pred_corr
      1,        // split_step
      nb_points // dim_opt
  };

  std::unique_ptr<MinCurvatureOpt> min_curvature_opt =
      std::make_unique<MinCurvatureOpt>(args, 5, false, false, true, 0.5, 9.0, 1.5, 0.5);

  ASSERT_TRUE(min_curvature_opt != nullptr);

  Eigen::MatrixX2d optimised_points;
  optimised_points = middle_line_20;

  Eigen::MatrixX2d lateral_deviation = Eigen::MatrixX2d::Zero(nb_points, 2);
  Eigen::VectorXd curvature_profile = Eigen::VectorXd::Zero(nb_points);

  min_curvature_opt->GetOptimalReferencePath(optimised_points, lateral_deviation, curvature_profile, true,
                                             boundary_points_20_left, boundary_points_20_right);

  PlotSingleData(lateral_deviation.col(0), 2002, false);
  PlotSingleData(lateral_deviation.col(1), 2002, false);

  if (optimised_points.hasNaN()) {
    std::cout << "Optimiser failed : NaN values in the path" << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Ones(nb_points, 2)) {
    std::cout << "Optimiser failed : Points are not ordered correctly (or duplicated)." << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 2) {
    std::cout << "Optimiser failed : Issues while checking the curvature" << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Ones(nb_points, 2) * 3) {
    std::cout << "Optimiser failed : Missing a point" << std::endl;
  } else if (optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)) {
    std::cout << "Optimiser failed" << std::endl;
  }

  ASSERT_FALSE(optimised_points.hasNaN());
  ASSERT_FALSE((optimised_points == Eigen::MatrixXd::Zero(nb_points, 2)));

  PlotTrack("Balanced // Dataset 20", 20, optimised_points, boundary_points_20_left, boundary_points_20_right,
            middle_line_20, std::nullopt, "o-m");

  Eigen::VectorXd processed_curvature = min_curvature_opt->ProcessCurvature(curvature_profile, true, 30, 5.0);

  Eigen::VectorXd processed_speed_targets =
      min_curvature_opt->GetSpeedTargets(processed_curvature, true, 0.8, 0.0001, 6.0, 0.0);
}

/*******************************************************************************
 * MAIN                                                                        *
 *******************************************************************************/
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  plt::show();

  return result;
}
