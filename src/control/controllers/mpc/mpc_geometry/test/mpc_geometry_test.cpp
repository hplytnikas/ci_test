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

#include "mpc_geometry/mpc_geometry.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <vector>

void GeneratePointsCorner(Eigen::MatrixX2d &reference_points, Eigen::MatrixX4d &boundary_points) {
  const int numPoints = 11;          // Number of points
  const double boundaryOffset = 1.5; // Distance of boundary points from the middle line
  const double radius = 5.0;         // Radius of the curve
  const double pi = 3.14159265358979323846;

  reference_points.resize(numPoints, 2);
  boundary_points.resize(numPoints, 4);

  // Generate points along a quarter-circle arc
  for (int i = 0; i < numPoints; ++i) {
    double end_angle = pi / 2;
    // double angle = end_angle * (1 - (i / (double)(numPoints - 1)));
    // use static cast to avoid integer division
    double angle = end_angle * (1 - (static_cast<double>(i) / (numPoints - 1)));
    int sign = 1;

    // Divide the quarter circle into points
    double x = radius * cos(angle);
    double y = radius * sin(angle);

    reference_points(i, 0) = x;
    reference_points(i, 1) = sign * (y - radius);

    // Calculate boundary points based on the normal vector at each point
    boundary_points(i, 0) = (radius + boundaryOffset) * cos(angle);                   // Left boundary x
    boundary_points(i, 1) = sign * ((radius + boundaryOffset) * sin(angle) - radius); // Left boundary y
    boundary_points(i, 2) = (radius - boundaryOffset) * cos(angle);                   // Right boundary x
    boundary_points(i, 3) = sign * ((radius - boundaryOffset) * sin(angle) - radius); // Right boundary y
  }
}

TEST(mpc_geometry, test_create_delete) {
  InterpolantType2D interp_type_2d = InterpolantType2D::BSPLINE;
  std::unique_ptr<MpcGeometry> geometry(new MpcGeometry(interp_type_2d));
  printf("MPC Geometry object created.\n");
}

TEST(mpc_geometry, test_fit_spline_straight) {
  InterpolantType2D interp_type_2d = InterpolantType2D::BSPLINE;
  std::unique_ptr<MpcGeometry> geometry(new MpcGeometry(interp_type_2d));
  std::cout << "MPC Geometry object created.\n" << std::endl;

  Eigen::VectorXd progress_horizon(4);
  progress_horizon << 0.5, 0.7, 0.9, 1.6;

  std::cout << "Progress_horizon" << std::endl;
  for (int i = 0; i < progress_horizon.size(); i++) {
    std::cout << "Progress[" << i << "] = " << progress_horizon[i] << std::endl;
  }

  std::cout << "Creating a straight line" << std::endl;
  Eigen::Matrix<double, 2, Eigen::Dynamic> middle_line(2, 10);
  for (double j = 1.0; j < 11; j++) {
    middle_line(0, static_cast<int>(j - 1)) = j;
    middle_line(1, static_cast<int>(j - 1)) = j;
  }

  std::cout << "Middle line" << std::endl;
  for (int i = 0; i < middle_line.cols(); i++) {
    std::cout << "Middle line[" << i << "] = (" << middle_line(0, i) << ", " << middle_line(1, i) << ")" << std::endl;
  }

  ASSERT_FALSE(geometry->FitReferencePath(middle_line, false, 2.0));

  Eigen::VectorXd curvature_horizon(4);

  std::cout << "Fitting reference path" << std::endl;
  // Get fitted path
  Eigen::Matrix<double, 2, Eigen::Dynamic> fitted_path = geometry->GetFittedPath();

  // print fitted path
  std::cout << "Fitted path" << std::endl;
  for (int i = 0; i < fitted_path.cols(); i++) {
    std::cout << "Fitted path[" << i << "] = (" << fitted_path(0, i) << ", " << fitted_path(1, i) << ")" << std::endl;
  }

  ASSERT_FALSE(geometry->GetCurvatureHorizon(progress_horizon, curvature_horizon));

  std::cout << "Curvature horizon" << std::endl;
  for (int i = 0; i < curvature_horizon.size(); i++) {
    std::cout << "Curvature[" << i << "] = " << curvature_horizon[i] << std::endl;
  }

  // Get initial heading and position
  std::cout << "Getting initial state" << std::endl;
  double s, n, mu;
  ASSERT_FALSE(geometry->GetInitialState(s, n, mu));
  std::cout << "Initial state: s = " << s << ", n = " << n << ", mu = " << mu << std::endl;

  double expected_mu = -M_PI / 4.0;
  double err = std::abs(mu - expected_mu);

  ASSERT_LE(err, 1e-3);
}

TEST(mpc_geometry, test_fit_spline_circle) {
  InterpolantType2D interp_type_2d = InterpolantType2D::BSPLINE;
  std::unique_ptr<MpcGeometry> geometry(new MpcGeometry(interp_type_2d));
  printf("MPC Geometry object created.\n");

  Eigen::VectorXd progress_horizon(10);
  progress_horizon << 0.0, 0.1, 0.2, 0.4, 0.7, 1.0, 2.2, 3.4, 5.1, 6.0;

  // generate boundary message where middle line is a circular arc of radius 5
  double radius = 5.0;

  Eigen::MatrixX2d reference_points;
  Eigen::MatrixX2d optimised_points;
  Eigen::MatrixX4d boundary_points;
  GeneratePointsCorner(reference_points, boundary_points);

  std::cout << "Reference points" << std::endl;
  Eigen::Matrix<double, 2, Eigen::Dynamic> middle_line(2, 10);
  // remove 0.0
  for (int i = 1; i < 11; i++) {
    middle_line(0, i - 1) = reference_points(i, 0);
    middle_line(1, i - 1) = reference_points(i, 1);
  }

  std::cout << "Middle line" << std::endl;
  for (int i = 0; i < middle_line.cols(); i++) {
    std::cout << "Middle line[" << i << "] = (" << middle_line(0, i) << ", " << middle_line(1, i) << ")" << std::endl;
  }

  ASSERT_FALSE(geometry->FitReferencePath(middle_line, false, 2.0));
  Eigen::VectorXd curvature_horizon(10);

  // Get fitted path
  Eigen::Matrix<double, 2, Eigen::Dynamic> fitted_path = geometry->GetFittedPath();

  // print fitted path
  std::cout << "Fitted path" << std::endl;
  for (int i = 0; i < fitted_path.cols(); i++) {
    std::cout << "Fitted path[" << i << "] = (" << fitted_path(0, i) << ", " << fitted_path(1, i) << ")" << std::endl;
  }

  ASSERT_FALSE(geometry->GetCurvatureHorizon(progress_horizon, curvature_horizon));

  std::cout << "Curvature horizon" << std::endl;
  for (int i = 0; i < curvature_horizon.size(); i++) {
    std::cout << "Curvature[" << i << "] = " << curvature_horizon[i] << std::endl;
  }

  // get middle point of the curvature horizon and check if curvature is
  // approximately 0.2
  double middle_curvature = curvature_horizon[3];
  double expected_curvature = -1.0 / radius;
  double curv_err = std::abs(middle_curvature - expected_curvature);

  ASSERT_LE(curv_err, 1e-1);

  // Get initial heading and position
  printf("Getting initial state\n");
  double s, n, mu;
  geometry->GetInitialState(s, n, mu);
  printf("Initial state: s = %f, n = %f, mu = %f\n", s, n, mu);

  double expected_mu = 0.0;
  double mu_err = std::abs(mu - expected_mu);

  ASSERT_LE(mu_err, 1e-1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
