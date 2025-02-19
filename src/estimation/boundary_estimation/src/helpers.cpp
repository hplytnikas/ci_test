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

#include "helpers.hpp"

namespace boundary_estimation {

bool KeyComparator::operator()(const std::string &lhs, const std::string &rhs) const {
  // Assuming keys are in the format "<first_id>-<second_id>"
  // Extracting the integer parts of the keys for comparison
  size_t dash_pos_lhs = lhs.find('-');
  size_t dash_pos_rhs = rhs.find('-');

  int id_lhs1 = std::stoi(lhs.substr(0, dash_pos_lhs));
  int id_rhs1 = std::stoi(rhs.substr(0, dash_pos_rhs));

  if (id_lhs1 < id_rhs1) {
    return true;
  }
  if (id_lhs1 > id_rhs1) {
    return false;
  }

  // First IDs are equal, compare the second IDs
  int id_lhs2 = std::stoi(lhs.substr(dash_pos_lhs + 1));
  int id_rhs2 = std::stoi(rhs.substr(dash_pos_rhs + 1));

  return id_lhs2 < id_rhs2;
}

Point_2 TransformPoint(const Point_2 &point, const tf2::Transform &stamped_transform) {
  Point_2 transformed_point;
  tf2::Transform point_transform;

  // Transform cone from global to local frame
  point_transform.setOrigin(tf2::Vector3(point.x(), point.y(), 0.0));
  point_transform = stamped_transform * point_transform;
  tf2::Vector3 point_vector = point_transform.getOrigin();
  transformed_point = Point_2(point_vector.getX(), point_vector.getY());

  return transformed_point;
}

std::pair<std::optional<Point_2>, std::optional<Point_2>>
TransformPastCarPosPoints(const std::pair<std::optional<Point_2>, std::optional<Point_2>> &points,
                          const tf2::Transform &stamped_transform) {
  std::pair<std::optional<Point_2>, std::optional<Point_2>> transformed_points(std::nullopt, std::nullopt);
  if (points.first.has_value()) {
    Point_2 transformed_point = TransformPoint(points.first.value(), stamped_transform);
    transformed_points.first = transformed_point;
  }

  if (points.second.has_value()) {
    Point_2 transformed_point = TransformPoint(points.second.value(), stamped_transform);
    transformed_points.second = transformed_point;
  }

  return transformed_points;
}

std::vector<Point_2> TransformPoints(const std::vector<Point_2> &points, const tf2::Transform &stamped_transform) {
  std::vector<Point_2> transformed_points;
  for (const auto &point : points) {
    Point_2 transformed_point = TransformPoint(point, stamped_transform);
    transformed_points.push_back(transformed_point);
  }
  return transformed_points;
}

// Function to transform a vector of autonomous_msgs::msg::PointWithConfidence
std::vector<autonomous_msgs::msg::PointWithConfidence>
TransformPointsWithConfidence(const std::vector<autonomous_msgs::msg::PointWithConfidence> &points,
                              const tf2::Transform &stamped_transform) {
  std::vector<autonomous_msgs::msg::PointWithConfidence> transformed_points;
  for (const auto &point : points) {
    // Convert geometry_msgs::msg::Point32 to tf2::Vector3
    tf2::Vector3 point_vector(point.position.x, point.position.y, point.position.z);

    // Transform the point
    tf2::Vector3 transformed_point_vector = stamped_transform * point_vector;

    // Create a new autonomous_msgs::msg::PointWithConfidence for the transformed point
    autonomous_msgs::msg::PointWithConfidence transformed_point;
    transformed_point.id_cone = point.id_cone;
    transformed_point.position.x = transformed_point_vector.x();
    transformed_point.position.y = transformed_point_vector.y();
    transformed_point.position.z = transformed_point_vector.z();
    transformed_point.confidence = point.confidence;

    // Add the transformed point to the vector
    transformed_points.push_back(transformed_point);
  }
  return transformed_points;
}

std::pair<std::string, std::string> SplitId(const std::string &id) {
  size_t pos = id.find('-');
  if (pos == std::string::npos) {
    return std::make_pair(id, "");
  }
  std::string first_id = id.substr(0, pos);
  std::string second_id = id.substr(pos + 1);
  return std::make_pair(first_id, second_id);
}

std::vector<std::pair<std::string, Point_2>>
ComputeBoundFromMidpoints(const std::vector<std::string> &boundaries_ids,
                          const std::vector<std::pair<std::string, Point_2>> &boundary) {
  std::vector<std::pair<std::string, Point_2>> final_bound;
  final_bound.reserve(boundaries_ids.size()); // Reserve space to avoid dynamic resizing
  // Create a map from string to Point_2
  std::unordered_map<std::string, Point_2> boundary_map;
  // Map to not append duplicate points
  std::unordered_map<std::string, bool> id_map;
  for (const std::pair<std::string, Point_2> &point : boundary) {
    boundary_map[point.first] = point.second;
  }

  // Iterate over the boundaries_ids and create the final_bound vector
  for (const std::string &id : boundaries_ids) {
    std::pair<std::string, std::string> ids = SplitId(id);
    if (!id_map[ids.first] && boundary_map.find(ids.first) != boundary_map.end()) {
      final_bound.push_back(std::make_pair(ids.first, boundary_map[ids.first]));
      id_map[ids.first] = true;
    }
    if (!ids.second.empty() && !id_map[ids.second] && boundary_map.find(ids.second) != boundary_map.end()) {
      final_bound.push_back(std::make_pair(ids.second, boundary_map[ids.second]));
      id_map[ids.second] = true;
    }
  }

  return final_bound;
}

std::vector<std::pair<std::string, Point_2>>
ExtractSubVector(const std::vector<std::pair<std::string, Point_2>> &midpoints, const int &start_index,
                 const double &max_distance, const Headings &heading) {
  std::vector<std::pair<std::string, Point_2>> sub_vec;
  const int midpoint_size = midpoints.size();
  int index;
  if (heading == Ahead) {
    index = start_index;
  } else {
    index = (start_index - 1 + midpoint_size) % midpoint_size;
  }
  Point_2 current_midpoint = midpoints[index].second;
  double distance = 0;
  do {
    const std::pair<std::string, Point_2> midpoint_pair = midpoints[index];
    const Point_2 next_midpoint = midpoint_pair.second;
    sub_vec.push_back(midpoint_pair);
    distance += PointToPointDistance(current_midpoint.x(), current_midpoint.y(), next_midpoint.x(), next_midpoint.y());
    if (distance > max_distance) {
      break;
    }
    current_midpoint = next_midpoint;
    if (heading == Ahead) {
      index = (index + 1) % midpoint_size;
    } else {
      index = (index - 1 + midpoint_size) % midpoint_size;
    }
  } while (index != start_index);
  return sub_vec;
}

int ComputeFirstPointAheadIndex(std::vector<std::pair<std::string, Point_2>> &midpoints) {
  double min_distance = __DBL_MAX__;
  int first_point_ahead_index = -1;
  for (int i = 0; i < midpoints.size(); ++i) {
    if (midpoints[i].second.x() < 0) {
      continue;
    }
    double distance = PointToPointDistance(midpoints[i].second.x(), midpoints[i].second.y(), 0, 0);
    if (distance < min_distance) {
      min_distance = distance;
      first_point_ahead_index = i;
    }
  }
  return first_point_ahead_index;
}

std::vector<std::pair<std::string, Point_2>>
TransformPairPoints(const std::vector<std::pair<std::string, Point_2>> &points,
                    const tf2::Transform &stamped_transform) {
  std::vector<std::pair<std::string, Point_2>> transformed_points;
  for (const auto &point : points) {
    Point_2 transformed_point;

    tf2::Transform point_transform;

    // Transform cone from global to local frame
    point_transform.setOrigin(tf2::Vector3(point.second.x(), point.second.y(), 0.0));
    point_transform = stamped_transform * point_transform;
    tf2::Vector3 point_vector = point_transform.getOrigin();
    transformed_point = Point_2(point_vector.getX(), point_vector.getY());

    transformed_points.push_back(std::make_pair(point.first, transformed_point));
  }
  return transformed_points;
}

std::vector<Point_2> SampleSegment(const Kernel::Segment_2 &segment, int num_samples) {
  std::vector<Point_2> sampled_points;

  // Get the start and end points of the segment
  Point_2 start_point = segment.target();
  Point_2 end_point = segment.source();

  // Compute the direction vector of the segment
  CGAL::Vector_2 direction = start_point - end_point;

  // Sample points along the segment using linear interpolation
  for (int i = 0; i <= num_samples; ++i) {
    double t = static_cast<double>(i) / num_samples;
    Point_2 sampled_point = start_point + t * direction;
    sampled_points.push_back(sampled_point);
  }

  return sampled_points;
}

double PointToLineSegmentDistance(double sx, double sy, double gx, double gy, double ptx, double pty) {
  CGAL::Simple_cartesian<double>::Point_2 src(sx, sy);
  CGAL::Simple_cartesian<double>::Point_2 goal(gx, gy);
  CGAL::Simple_cartesian<double>::Point_2 point(ptx, pty);

  CGAL::Simple_cartesian<double>::Segment_2 src_to_goal(src, goal);

  return (std::sqrt(CGAL::squared_distance(src_to_goal, point)));
}

double PointToPointDistance(double pt1_x, double pt1_y, double pt2_x, double pt2_y) {
  CGAL::Simple_cartesian<double>::Point_2 pt1(pt1_x, pt1_y);
  CGAL::Simple_cartesian<double>::Point_2 pt2(pt2_x, pt2_y);

  return (std::sqrt(CGAL::squared_distance(pt1, pt2)));
}

double PointToPointMahalanobisDistance(double x_difference, double y_difference, double cov_xx, double cov_yy,
                                       double cov_xy) {
  Eigen::MatrixXd point_to_point_vector(2, 1);
  // vector_x = pt2_x - pt1_x
  point_to_point_vector(0, 0) = x_difference;
  // vector_y = pt2_y - pt1_y
  point_to_point_vector(1, 0) = y_difference;

  Eigen::MatrixXd cov(2, 2);
  cov(0, 0) = cov_xx;
  cov(0, 1) = cov_xy;
  cov(1, 0) = cov_xy;
  cov(1, 1) = cov_yy;

  Eigen::MatrixXd distanceSquared;

  Eigen::FullPivLU<Eigen::Matrix<double, 2, 2>> lu(cov);
  if (lu.isInvertible()) {
    distanceSquared = point_to_point_vector.transpose() * cov.inverse() * point_to_point_vector;
  } else {
    distanceSquared = point_to_point_vector.transpose() * point_to_point_vector;
  }

  double distance = std::sqrt(distanceSquared(0, 0));

  return distance;
}

double PointToLineSegmentMahalanobisDistance(double sx, double sy, double gx, double gy, double ptx, double pty,
                                             double cov_xx, double cov_yy, double cov_xy) {
  Eigen::Vector2d src(sx, sy);
  Eigen::Vector2d goal(gx, gy);
  Eigen::Vector2d point(ptx, pty);

  Eigen::Vector2d src_to_point = point - src;
  Eigen::Vector2d src_to_goal = goal - src;

  double line_squared = src_to_goal.squaredNorm();

  if (line_squared < 0.01) {
    double distance = PointToPointMahalanobisDistance(src_to_point(0), src_to_point(1), cov_xx, cov_yy, cov_xy);
    return distance;
  }

  double t = std::max(0.0, std::min(1.0, src_to_point.dot(src_to_goal) / line_squared));

  Eigen::Vector2d projection = src + t * (src_to_goal);

  Eigen::Vector2d projection_to_point = point - projection;

  double distance =
      PointToPointMahalanobisDistance(projection_to_point(0), projection_to_point(1), cov_xx, cov_yy, cov_xy);
  return distance;
}

double AngleDifferenceRad(double angle1_rad, double angle2_rad) {
  double angle_diff = angle1_rad - angle2_rad;

  double angle_diff_new = MinusPiToPi(angle_diff);

  return angle_diff_new;
}

double MinusPiToPi(double angle_diff) {
  double angle_diff_new = angle_diff;

  while (angle_diff_new < -M_PI) {
    angle_diff_new = angle_diff_new + 2 * M_PI;
  }

  while (angle_diff_new > M_PI) {
    angle_diff_new = angle_diff_new - 2 * M_PI;
  }

  return angle_diff_new;
}

double Min(const std::vector<double> &data) { return data.empty() ? 0 : *std::min_element(data.begin(), data.end()); }

double Max(const std::vector<double> &data) { return data.empty() ? 0 : *std::max_element(data.begin(), data.end()); }

double Mean(const std::vector<double> &data) {
  return data.empty() ? 0 : std::accumulate(data.begin(), data.end(), 0.0) / data.size();
}

// Function to calculate the root mean square of a vector
double RootMeanSquare(const std::vector<double> &values) {
  if (values.empty()) return 0.0;
  double sum_of_squares =
      std::accumulate(values.begin(), values.end(), 0.0, [](double sum, double val) { return sum + val * val; });
  return std::sqrt(sum_of_squares / values.size());
}

double Variance(const std::vector<double> &data) {
  if (data.empty()) return 0;

  double x_bar = Mean(data);
  double sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0);
  return sq_sum / data.size() - x_bar * x_bar;
}

double StandardDeviation(const std::vector<double> &data) { return std::sqrt(Variance(data)); }

double GetLengthOfPath(std::vector<Point_2> &path) {
  double total_dist = 0.0;

  if (path.size() < 2) {
    return total_dist;
  } else {
    for (int i = 1; i < path.size(); ++i) {
      double dist = PointToPointDistance(path[i - 1].x(), path[i - 1].y(), path[i].x(), path[i].y());
      total_dist += dist;
    }
  }
  return total_dist;
}

double ComputeAngleDeviation(Point_2 first_point, Point_2 second_point) {
  double angle = atan2(second_point.y() - first_point.y(), second_point.x() - first_point.x());
  return angle;
}

// Function to compute the second derivative of angle changes
double ComputeAngleChangeSecondDerivative(const std::vector<double> &path_angle_changes,
                                          const std::vector<double> &path_midpoint_distance) {
  double path_size = path_angle_changes.size();
  if (path_size < 3) {
    return 0.0; // Not enough points to compute second derivatives
  }

  std::vector<double> second_derivative;
  for (int i = 1; i < path_size - 1; ++i) {
    double angle_change = path_angle_changes[i] * 180 / M_PI;
    double prev_angle_change = path_angle_changes[i - 1] * 180 / M_PI;
    double next_angle_change = path_angle_changes[i + 1] * 180 / M_PI;

    double prev_distance = path_midpoint_distance[i];
    double distance = path_midpoint_distance[i + 1];

    double second_derivative_value =
        (next_angle_change - angle_change) / distance - (angle_change - prev_angle_change) / prev_distance;
    second_derivative_value /= (distance + prev_distance) / 2.0;
    second_derivative.push_back(second_derivative_value);
  }

  return RootMeanSquare(second_derivative);
}

geometry_msgs::msg::Point32 CgalToGeometryMsgsPoint32(const Point_2 &point_2) {
  geometry_msgs::msg::Point32 point32;
  point32.x = point_2.x();
  point32.y = point_2.y();
  return point32;
}

geometry_msgs::msg::Point CgalToGeometryMsgsPoint(const Point_2 &point_2) {
  geometry_msgs::msg::Point point;
  point.x = point_2.x();
  point.y = point_2.y();
  return point;
}

Point_2 GeometryMsgsToCgalPoint2(const geometry_msgs::msg::Point32 &point_msg) {
  return Point_2(point_msg.x, point_msg.y);
}

std::vector<autonomous_msgs::msg::PointWithConfidence>
ParsePoint2VectorToPointWithConfidenceVector(const std::vector<std::pair<std::string, Point_2>> &point_2_vec) {
  std::vector<autonomous_msgs::msg::PointWithConfidence> msg_vec;

  for (std::pair<std::string, Point_2> point_2 : point_2_vec) {
    autonomous_msgs::msg::PointWithConfidence point_with_confidence;
    point_with_confidence.position = CgalToGeometryMsgsPoint32(point_2.second);
    point_with_confidence.confidence = 0.0;
    point_with_confidence.id_cone = point_2.first;
    msg_vec.push_back(point_with_confidence);
  }
  return msg_vec;
}

// Function to convert a vector of autonomous_msgs::msg::PointWithConfidence to a vector of std::pair<std::string,
// Point_2>
std::vector<Point_2> ParsePointWithConfidenceVectorToPoint2Vector(
    const std::vector<autonomous_msgs::msg::PointWithConfidence> &point_with_conf_vec) {
  std::vector<Point_2> point_2_vec;

  for (const auto &point_with_conf : point_with_conf_vec) {
    Point_2 point_2 = GeometryMsgsToCgalPoint2(point_with_conf.position);
    point_2_vec.push_back(point_2);
  }
  return point_2_vec;
}
std::vector<geometry_msgs::msg::Point> ParsePoint2VectorToPointVector(const std::vector<Point_2> &point_2_vec) {
  std::vector<geometry_msgs::msg::Point> msg_vec;

  for (Point_2 point_2 : point_2_vec) {
    geometry_msgs::msg::Point point(CgalToGeometryMsgsPoint(point_2));
    msg_vec.push_back(point);
  }
  return msg_vec;
}

std::vector<std::pair<std::string, Point_2>>
ExtractPointsFromVertexHandles(const std::vector<Vertex_handle> &vertices) {
  std::vector<std::pair<std::string, Point_2>> points;
  for (const Vertex_handle &vertex : vertices) {
    // convert the vertex info from int to string
    std::string id = std::to_string(vertex->info().id);
    points.push_back(std::make_pair(id, vertex->point()));
  }
  return points;
}

std::vector<std::string>
ExtractFirstElementFromPairVector(const std::vector<std::pair<std::string, Point_2>> &pair_vector) {
  std::vector<std::string> second_elements;
  for (const std::pair<std::string, Point_2> &pair : pair_vector) {
    second_elements.push_back(pair.first);
  }
  return second_elements;
}

std::vector<Point_2>
ExtractSecondElementFromPairVector(const std::vector<std::pair<std::string, Point_2>> &pair_vector) {
  std::vector<Point_2> second_elements;
  for (const std::pair<std::string, Point_2> &pair : pair_vector) {
    second_elements.push_back(pair.second);
  }
  return second_elements;
}

std::vector<std::string> ComputeVectorDifference(const std::vector<std::string> &vec1,
                                                 const std::vector<std::string> &vec2) {
  std::vector<std::string> difference;

  for (const auto &elem : vec1) {
    if (std::find(vec2.begin(), vec2.end(), elem) == vec2.end()) {
      difference.push_back(elem);
    }
  }

  return difference;
}

bool IsPointAtOrigin(const Point_2 &point) { return point.x() == 0.0 && point.y() == 0.0; }

double sigmoid(double x) {
  // if (x < -20.0) return 0.0;  // Avoid overflow when exp(-x) would be large
  // else if (x > 20.0) return 1.0;  // Avoid underflow when exp(-x) would be close to zero
  return 1.0 / (1.0 + exp(-x));
}

double ComputeEntropy(const std::vector<double> &probabilities) {
  double entropy = 0.0;
  for (const double &probability : probabilities) {
    entropy -= probability * std::log2(probability);
  }
  return entropy;
}
} // namespace boundary_estimation
