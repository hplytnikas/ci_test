/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2019-2023 Authors:
 *   - Lorenzo Codeluppi <lcodeluppi@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/LU>
#include <math.h>

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "autonomous_msgs/msg/boundary.hpp"
#include "autonomous_msgs/msg/point_with_confidence.hpp"
#include "boundary_estimation_graph.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace boundary_estimation {

struct KeyComparator {
  bool operator()(const std::string &lhs, const std::string &rhs) const;
};

Point_2 TransformPoint(const Point_2 &point, const tf2::Transform &stamped_transform);

std::vector<Point_2> TransformPoints(const std::vector<Point_2> &points, const tf2::Transform &stamped_transform);

std::vector<autonomous_msgs::msg::PointWithConfidence>
TransformPointsWithConfidence(const std::vector<autonomous_msgs::msg::PointWithConfidence> &points,
                              const tf2::Transform &stamped_transform);

std::vector<std::pair<std::string, Point_2>>
TransformPairPoints(const std::vector<std::pair<std::string, Point_2>> &points,
                    const tf2::Transform &stamped_transform);

std::pair<std::optional<Point_2>, std::optional<Point_2>>
TransformPastCarPosPoints(const std::pair<std::optional<Point_2>, std::optional<Point_2>> &points,
                          const tf2::Transform &stamped_transform);

std::vector<Point_2> SampleSegment(const Kernel::Segment_2 &segment, int num_samples);

int ComputeFirstPointAheadIndex(std::vector<std::pair<std::string, Point_2>> &midpoints);

std::pair<std::string, std::string> SplitId(const std::string &id);

std::vector<std::pair<std::string, Point_2>>
ComputeBoundFromMidpoints(const std::vector<std::string> &boundaries_ids,
                          const std::vector<std::pair<std::string, Point_2>> &boundary);

std::vector<std::pair<std::string, Point_2>>
ExtractSubVector(const std::vector<std::pair<std::string, Point_2>> &midpoints, const int &start_index,
                 const double &max_distance, const Headings &heading);

double PointToLineSegmentDistance(double sx, double sy, double gx, double gy, double ptx, double pty);

double PointToPointDistance(double pt1_x, double pt1_y, double pt2_x, double pt2_y);

double PointToLineSegmentMahalanobisDistance(double sx, double sy, double gx, double gy, double ptx, double pty,
                                             double cov_xx, double cov_yy, double cov_xy);

double PointToPointMahalanobisDistance(double pt1_x, double pt1_y, double pt2_x, double pt2_y, double cov_xx,
                                       double cov_yy, double cov_xy);

double AngleDifferenceRad(double angle1_rad, double angle2_rad);

double MinusPiToPi(double angle_diff);

double Min(const std::vector<double> &data);

double Max(const std::vector<double> &data);

double Mean(const std::vector<double> &data);

double RootMeanSquare(const std::vector<double> &values);

double Variance(const std::vector<double> &data);

double StandardDeviation(const std::vector<double> &data);

double GetLengthOfPath(std::vector<Point_2> &path);

double ComputeAngleDeviation(Point_2 first_point, Point_2 second_point);

double ComputeAngleChangeSecondDerivative(const std::vector<double> &path_angle_changes,
                                          const std::vector<double> &path_midpoint_distance);

std::vector<autonomous_msgs::msg::PointWithConfidence>
ParsePoint2VectorToPointWithConfidenceVector(const std::vector<std::pair<std::string, Point_2>> &point_2_vec);

std::vector<Point_2> ParsePointWithConfidenceVectorToPoint2Vector(
    const std::vector<autonomous_msgs::msg::PointWithConfidence> &point_with_conf_vec);

std::vector<geometry_msgs::msg::Point> ParsePoint2VectorToPointVector(const std::vector<Point_2> &point_2_vec);

geometry_msgs::msg::Point CgalToGeometryMsgsPoint(const Point_2 &point_2);

geometry_msgs::msg::Point32 CgalToGeometryMsgsPoint32(const Point_2 &point_2);

Point_2 GeometryMsgsToCgalPoint2(const geometry_msgs::msg::Point32 &point_msg);

std::vector<std::pair<std::string, Point_2>> ExtractPointsFromVertexHandles(const std::vector<Vertex_handle> &vertices);

std::vector<std::string>
ExtractFirstElementFromPairVector(const std::vector<std::pair<std::string, Point_2>> &pair_vector);

std::vector<Point_2>
ExtractSecondElementFromPairVector(const std::vector<std::pair<std::string, Point_2>> &pair_vector);

std::vector<std::string> ComputeVectorDifference(const std::vector<std::string> &vec1,
                                                 const std::vector<std::string> &vec2);

bool IsPointAtOrigin(const Point_2 &point);

double sigmoid(double x);

double ComputeEntropy(const std::vector<double> &probabilities);

} // namespace boundary_estimation
