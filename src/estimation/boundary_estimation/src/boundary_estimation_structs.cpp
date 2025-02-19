/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023 - 2024  Authors:
 *   - Lorenzo Codeluppi <lcodeluppi@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "boundary_estimation_structs.hpp"

namespace boundary_estimation {

namespace constants {
// 1 = yolo_only, 2 = lidar_only, 4 = sensor_fusion
const int kCameraOnly = 1;
const int kLidarOnly = 2;
const int kSensorFusion = 4;
const double kMaxMidpointDistanceFromCarPosition = 1.0;
const double kMaxLastMidpointDistance = 30.0;
const double kMinimumDistanceSameBoundCones = 0.5;
const int kMinimumConsecutivesCrossedMidpoints = 3;
const double kMinimumAngleDeviationBetweenCrossedPoints = 150;
const double kPathLengthBehind = 20;
const double kPathLengthAhead = 20;
const double kMidpointDistanceThreshold = 3;
const int kMaxTempPastMipointIds = 50;
const int kMaxPointToCheckForOverlay = 4;
const double kMaxEntropyToConsiderColor = 0.2;
const double kMaxPointDistanceFirstAndLast = 0.5;
} // namespace constants

} // namespace boundary_estimation
