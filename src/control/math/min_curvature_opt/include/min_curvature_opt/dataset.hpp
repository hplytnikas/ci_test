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
#include <iostream>

/************************************************
 * Interpolated (middle line only) points set
 ************************************************/
extern Eigen::MatrixX2d middle_line_1;
extern Eigen::MatrixX4d boundary_points_1;

extern Eigen::MatrixX2d middle_line_2;
extern Eigen::MatrixX4d boundary_points_2;

extern Eigen::MatrixX2d middle_line_3;
extern Eigen::MatrixX4d boundary_points_3;

extern Eigen::MatrixX2d middle_line_4;
extern Eigen::MatrixX4d boundary_points_4;

/************************************************
 * Raw points set
 ************************************************/

extern Eigen::MatrixX2d middle_line_5;
extern Eigen::MatrixX2d boundary_points_5_left;
extern Eigen::MatrixX2d boundary_points_5_right;

extern Eigen::MatrixX2d middle_line_6;
extern Eigen::MatrixX4d boundary_points_6;

extern Eigen::MatrixX2d middle_line_7;
extern Eigen::MatrixX4d boundary_points_7;

extern Eigen::MatrixX2d middle_line_8;
extern Eigen::MatrixX4d boundary_points_8;

/************************************************
 * Interpolated partial (middle line only) points set
 ************************************************/
// From dataset 1
extern Eigen::MatrixX2d middle_line_9;
extern Eigen::MatrixX4d boundary_points_9;

// From dataset 2
extern Eigen::MatrixX2d middle_line_10;
extern Eigen::MatrixX4d boundary_points_10;

// From dataset 3
extern Eigen::MatrixX2d middle_line_11;
extern Eigen::MatrixX4d boundary_points_11;

// From dataset 4
extern Eigen::MatrixX2d middle_line_12;
extern Eigen::MatrixX4d boundary_points_12;

/************************************************
 * Raw partial points set
 ************************************************/
// From dataset 5
extern Eigen::MatrixX2d middle_line_13;
extern Eigen::MatrixX4d boundary_points_13;

// From dataset 6
extern Eigen::MatrixX2d middle_line_14;
extern Eigen::MatrixX4d boundary_points_14;

// From dataset 7
extern Eigen::MatrixX2d middle_line_15;
extern Eigen::MatrixX4d boundary_points_15;

// From dataset 8
extern Eigen::MatrixX2d middle_line_16;
extern Eigen::MatrixX4d boundary_points_16;

/************************************************
 * Closed track points set
 ************************************************/
extern Eigen::MatrixX2d middle_line_17;
extern Eigen::MatrixX2d boundary_points_17_left;
extern Eigen::MatrixX2d boundary_points_17_right;

extern Eigen::MatrixX2d middle_line_18;
extern Eigen::MatrixX4d boundary_points_18;

extern Eigen::MatrixX2d middle_line_19;
extern Eigen::MatrixX4d boundary_points_19;

extern Eigen::MatrixX2d middle_line_20;
extern Eigen::MatrixX2d boundary_points_20_left;
extern Eigen::MatrixX2d boundary_points_20_right;

#define NB_POINTS 140

/*******************************************************************************
 * HELPER FUNCTIONS
 *******************************************************************************/
void GeneratePointsStraight(Eigen::MatrixX2d &reference_points, Eigen::MatrixX4d &boundary_points);

void GeneratePointsCorner(Eigen::MatrixX2d &reference_points, Eigen::MatrixX4d &boundary_points);
