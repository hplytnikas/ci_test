/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Christoforos Nicolaou <cnicolaou@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#include "slam_frontend/map_loader.hpp"

namespace slam {

MapLoader::MapLoader(std::shared_ptr<SlamNode> node_handle) : node_handle_(node_handle) { LoadParameters(); }

MapLoader::~MapLoader() {}

void MapLoader::LoadParameters() {
  int mission_type = node_handle_->GetParameter<int>("mission_type", static_cast<int>(MissionType::kAutocross));
  std::string mission_str =
      std::array<std::string, 4>{"acceleration", "skidpad", "autocross", "trackdrive"}[mission_type];

  map_save_file_path_ = node_handle_->GetParameter<std::string>("map_loader.map.save_file_path",
                                                                "src/estimation/slam/slam_frontend/maps/");
  map_save_file_path_ = map_save_file_path_ + mission_str + "_" + utils::GetCurrentTimeString() +
                        ".csv"; // Map wll have same time as rosbag

  // TODO(Christoforos) If I had a suggestion to make, I would have this code automatically select the latest map
  // that was generated unless otherwise specified. One way to do that is to have a default map.csv symlink that's
  // overwritten every time the map is saved. Otherwise you'll need to manually set the map title every time/lose old
  // maps Load path will be different in each discipline param file
  map_load_file_path_ = node_handle_->GetParameter<std::string>("map_loader.map.load_file_path",
                                                                "src/estimation/slam/slam_frontend/maps/skidpad.csv");

  trajectory_save_file_path_ = node_handle_->GetParameter<std::string>("map_loader.trajectory.save_file_path",
                                                                       "src/estimation/slam/slam_frontend/maps/");
  trajectory_save_file_path_ = trajectory_save_file_path_ + "trajectory_" + utils::GetCurrentTimeString() + ".txt";

  // Acceleration Parameters
  track_first_part_length_m_ =
      node_handle_->GetParameter<double>("map_loader.map.acceleration.track_first_part_length_m", 75.0);
  track_second_part_length_m_ =
      node_handle_->GetParameter<double>("map_loader.map.acceleration.track_second_part_length_m", 75.0);
  num_boundary_cones_ = node_handle_->GetParameter<int>("map_loader.map.acceleration.num_boundary_cones", 14);
  num_boundary_orange_cones_ =
      node_handle_->GetParameter<int>("map_loader.map.acceleration.num_boundary_orange_cones", 15);
  max_track_width_y_m_ = node_handle_->GetParameter<double>("map_loader.map.acceleration.max_track_width_y_m", 6.0);
  estimated_first_cones_distance_x_m_ =
      node_handle_->GetParameter<double>("map_loader.map.acceleration.estimated_first_cones_distance_x_m", 3.0);
  estimated_big_orange_cone_distance_x_m_ =
      node_handle_->GetParameter<double>("map_loader.map.acceleration.estimated_big_orange_cone_distance_x_m", 1.0);
  min_cone_distance_x_m_ = node_handle_->GetParameter<double>("map_loader.map.acceleration.min_cone_distance_x_m", 2.0);
  max_cone_distance_x_m_ = node_handle_->GetParameter<double>("map_loader.map.acceleration.max_cone_distance_x_m", 5.5);
  perception_observation_weight_ =
      node_handle_->GetParameter<double>("map_loader.map.acceleration.perception_observation_weight", 0.1);
  min_num_detected_cones_ = node_handle_->GetParameter<int>("map_loader.map.acceleration.min_num_detected_cones", 2);

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "MapLoader: Parameters loaded.");
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "MapLoader: Map save path set to " << map_save_file_path_);
  RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                     "MapLoader: Trajectory save path set to " << trajectory_save_file_path_);
}

ConeMap MapLoader::LoadMap() const {
  ConeMap input_map;
  std::ifstream file(map_load_file_path_);

  if (!file.is_open()) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "MapLoader: Unable to open file " << map_load_file_path_);
    return input_map; // Return empty vector if file couldn't be opened
  }

  std::string line;
  while (getline(file, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    std::vector<std::string> row;

    // Extract each cell in the row
    while (getline(lineStream, cell, ',')) {
      row.push_back(cell);
    }

    // Ensure row has at least 3 cells for all values
    if (row.size() >= 3 && row[0] != "tag") {
      autonomous_msgs::msg::Cone cone;
      cone.position.x = std::stof(row[1]); // Convert string to float
      cone.position.y = std::stof(row[2]);
      cone.prob_cone = 1.0;
      cone.prob_type.blue = cone.prob_type.yellow = cone.prob_type.orange = cone.prob_type.orange_big = 0.0;

      if (row[0] == "blue") {
        cone.prob_type.blue = 1.0;
      } else if (row[0] == "yellow") {
        cone.prob_type.yellow = 1.0;
      } else if (row[0] == "orange") {
        cone.prob_type.orange = 1.0;
      } else if (row[0] == "orange_big") {
        cone.prob_type.orange_big = 1.0;
      } else if (row[0] == "none") { // lidar_only
        cone.prob_type.blue = cone.prob_type.yellow = cone.prob_type.orange = cone.prob_type.orange_big = 0.25;
      }

      input_map.push_back(cone);
    }
  }

  file.close();

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "MapLoader: Map loaded successfully from " << map_load_file_path_);
  return input_map;
}

ConeMap MapLoader::GenerateAccelerationMap(const ConeMap &cone_observations) const {
  ConeMap acceleration_map, negatives, positives;

  // Split the two boundaries and remove outliers, and calculate average y position
  double avg_distance_y_m = FilterCones(cone_observations, positives, negatives);

  if (negatives.size() < min_num_detected_cones_ || positives.size() < min_num_detected_cones_) {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                       "MapLoader: Unable to generate acceleration map. Not enough observations");
    return acceleration_map;
  }

  // Get distance between cones on boundary from number of observed cones
  double counted_cones_distance_x_m = track_first_part_length_m_ / (num_boundary_cones_ + 1);
  double counted_orange_cones_distance_x_m = track_second_part_length_m_ / num_boundary_orange_cones_;
  // Get distance between cones on boundary from perception observations
  double avg_perception_cone_distance_x_m = AverageConeDistanceX(positives, negatives);

  // Calculate the x distance at which to set the cones as a weighted average between the distance from perception
  // and the distance from the manually observed cones
  double a = perception_observation_weight_;
  double generated_cone_distance_x_m = a * avg_perception_cone_distance_x_m + (1 - a) * counted_cones_distance_x_m;

  // // Make sure that the front cones are not to far apart in x
  // if (abs(negatives.front().position.x - positives.front().position.x) > 1.5) {
  //   RCLCPP_WARN_STREAM(node_handle_->get_logger(),
  //                      "MapLoader: Unable to generate acceleration map. First cones too far apart");
  //   return acceleration_map; // Empty
  // }

  // Always start the map at 1m in front of the car
  double first_cones_x_m = 1.0; // (negatives.front().position.x + positives.front().position.x) / 2.0;

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "MapLoader: Average observed cone x distance: "
                                                     << avg_perception_cone_distance_x_m
                                                     << ", distance from num cones: " << counted_cones_distance_x_m);
  RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                     "MapLoader: Cones placed at distance x: " << generated_cone_distance_x_m
                                                               << ", distance y: " << avg_distance_y_m);

  // Generate map
  // Place cones from start to finish line
  GenerateAccelerationBoundaries(acceleration_map, first_cones_x_m, generated_cone_distance_x_m, avg_distance_y_m,
                                 num_boundary_cones_);
  // Place big orange cones
  double big_orange_cones_middle_x_m = first_cones_x_m + num_boundary_cones_ * generated_cone_distance_x_m;
  GenerateAccelerationBigOrangeCones(acceleration_map, big_orange_cones_middle_x_m, avg_distance_y_m);
  // Place remaining orange cones after finish line
  GenerateAccelerationBoundaries(acceleration_map, big_orange_cones_middle_x_m + counted_orange_cones_distance_x_m,
                                 counted_orange_cones_distance_x_m, avg_distance_y_m, num_boundary_orange_cones_);

  return acceleration_map;
}

void MapLoader::SortConesX(ConeMap &cones) const {
  std::sort(cones.begin(), cones.end(),
            [this](const autonomous_msgs::msg::Cone &a, const autonomous_msgs::msg::Cone &b) {
              return a.position.x < b.position.x;
            });
}

double MapLoader::FilterCones(const ConeMap &cone_observations, ConeMap &positives, ConeMap &negatives) const {
  double avg_negative_y = 0, avg_positive_y = 0;

  for (const auto &cone : cone_observations) {
    if (abs(cone.position.y) < 0.5 * max_track_width_y_m_ &&
        cone.position.x > estimated_first_cones_distance_x_m_) { // Filter out outliers & ignore initial orange cones
      if (cone.position.y < 0) {
        negatives.push_back(cone);
        avg_negative_y += cone.position.y;
      } else {
        positives.push_back(cone);
        avg_positive_y += cone.position.y;
      }
    }
  }

  if (negatives.size() > 0) {
    avg_negative_y /= negatives.size();
  }

  if (positives.size() > 0) {
    avg_positive_y /= positives.size();
  }

  return avg_positive_y - avg_negative_y;
}

double MapLoader::AverageConeDistanceX(ConeMap &positives, ConeMap &negatives) const {
  // Sort cones by x so that neighboring cones can be checked easily
  SortConesX(negatives);
  SortConesX(positives);

  int num_included_cones = 0;
  double total_cone_distance_x_m = 0;

  for (int i = 0; i < negatives.size() - 1; i++) {
    // Update average if distance is in accepted range
    double distance_x_m = negatives[i + 1].position.x - negatives[i].position.x;
    if (distance_x_m > min_cone_distance_x_m_ && distance_x_m < max_cone_distance_x_m_) {
      total_cone_distance_x_m += distance_x_m;
      num_included_cones++;
    }
  }

  for (int i = 0; i < positives.size() - 1; i++) {
    // Update average if distance is in accepted range
    double distance_x_m = positives[i + 1].position.x - positives[i].position.x;
    if (distance_x_m > min_cone_distance_x_m_ && distance_x_m < max_cone_distance_x_m_) {
      total_cone_distance_x_m += distance_x_m;
      num_included_cones++;
    }
  }

  if (num_included_cones > 0) {
    return total_cone_distance_x_m / num_included_cones;
  }

  // Default average
  return estimated_first_cones_distance_x_m_;
}

void MapLoader::GenerateAccelerationBoundaries(ConeMap &acceleration_map, double first_cone_position,
                                               double distance_x_m, double distance_y_m, int num_cones) const {
  for (int i = 0; i < num_cones; i++) {
    autonomous_msgs::msg::Cone left_cone;
    left_cone.position.x = first_cone_position + i * distance_x_m;
    left_cone.position.y = 0.5 * distance_y_m;
    acceleration_map.push_back(left_cone);
    autonomous_msgs::msg::Cone right_cone;
    right_cone.position.x = first_cone_position + i * distance_x_m;
    right_cone.position.y = -0.5 * distance_y_m;
    acceleration_map.push_back(right_cone);
  }
}

void MapLoader::GenerateAccelerationBigOrangeCones(ConeMap &acceleration_map, double big_orange_cones_middle_x_m,
                                                   double distance_y_m) const {
  autonomous_msgs::msg::Cone big_orange_left_cone_1;
  big_orange_left_cone_1.position.x = big_orange_cones_middle_x_m - 0.5 * estimated_big_orange_cone_distance_x_m_;
  big_orange_left_cone_1.position.y = 0.5 * distance_y_m;
  acceleration_map.push_back(big_orange_left_cone_1);
  autonomous_msgs::msg::Cone big_orange_left_cone_2;
  big_orange_left_cone_2.position.x = big_orange_cones_middle_x_m + 0.5 * estimated_big_orange_cone_distance_x_m_;
  big_orange_left_cone_2.position.y = 0.5 * distance_y_m;
  acceleration_map.push_back(big_orange_left_cone_2);
  autonomous_msgs::msg::Cone big_orange_right_cone_1;
  big_orange_right_cone_1.position.x = big_orange_cones_middle_x_m - 0.5 * estimated_big_orange_cone_distance_x_m_;
  big_orange_right_cone_1.position.y = -0.5 * distance_y_m;
  acceleration_map.push_back(big_orange_right_cone_1);
  autonomous_msgs::msg::Cone big_orange_right_cone_2;
  big_orange_right_cone_2.position.x = big_orange_cones_middle_x_m + 0.5 * estimated_big_orange_cone_distance_x_m_;
  big_orange_right_cone_2.position.y = -0.5 * distance_y_m;
  acceleration_map.push_back(big_orange_right_cone_2);
}

void MapLoader::SaveMap(const ConeMap &map_to_save) const {
  std::ofstream file(map_save_file_path_);

  if (!file.is_open()) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "MapLoader: Unable to save file " << map_save_file_path_);
    return;
  }

  file << "tag,x,y\n";
  for (const auto &cone : map_to_save) {
    if (cone.prob_type.blue >= 0.6) {
      file << "blue,";
    } else if (cone.prob_type.yellow >= 0.6) {
      file << "yellow,";
    } else if (cone.prob_type.orange >= 0.6) {
      file << "orange,";
    } else if (cone.prob_type.orange_big >= 0.6) {
      file << "orange_big,";
    } else {
      file << "none,";
    }
    file << std::fixed << std::setprecision(6) << cone.position.x << ',' << cone.position.y << '\n';
  }

  file.close();

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "MapLoader: Map saved successfully to " << map_save_file_path_);
}

void MapLoader::SaveTrajectory(const std::vector<StampedPose> &trajectory_to_save) const {
  std::ofstream file(trajectory_save_file_path_);

  if (!file.is_open()) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "MapLoader: Unable to save file " << trajectory_save_file_path_);
    return;
  }

  for (const auto &pose : trajectory_to_save) {
    // Convert to quaternion
    double qz = sin(pose.Theta() * 0.5);
    double qw = cos(pose.Theta() * 0.5);
    // time, x, y, z, qx, qy, qz, qw
    file << std::fixed << std::setprecision(9) << pose.Hash() << ' ' << pose.X() << ' ' << pose.Y() << ' ' << 0.0 << ' '
         << 0.0 << ' ' << 0.0 << ' ' << qz << ' ' << qw << '\n';
  }

  file.close();

  RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                     "MapLoader: Trajectory saved successfully to " << trajectory_save_file_path_);
}

} // end namespace slam
