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
#pragma once

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "autonomous_msgs/msg/cone.hpp"
#include "slam_common/aliases.hpp"
#include "slam_common/mission_type.hpp"
#include "slam_common/slam_node.hpp"
#include "slam_common/stamped_pose.hpp"
#include "slam_common/utils.hpp"

namespace slam {

/*
 * Class for performing save/load of cone maps.
 */
class MapLoader {
public:
  /*
   * Constructor.
   */
  explicit MapLoader(std::shared_ptr<SlamNode> node_handle);

  /*
   * Destructor.
   */
  ~MapLoader();

  // Load the skidpad map from a csv file
  ConeMap LoadMap() const;

  // Generate the map for acceleration based on the first observations
  ConeMap GenerateAccelerationMap(const ConeMap &cone_observations) const;

  // Save the map to a csv file
  void SaveMap(const ConeMap &map_to_save) const;

  // Save pose array to file
  void SaveTrajectory(const std::vector<StampedPose> &trajectory_to_save) const;

private:
  // Reference to slam node
  std::shared_ptr<SlamNode> node_handle_;

  // Parameters
  std::string map_load_file_path_;
  std::string map_save_file_path_;
  std::string trajectory_save_file_path_;
  // Acceleration Parameters
  // The track length from start to finish line in meters
  double track_first_part_length_m_;
  // The track length from finish line until end of orange cones in meters
  double track_second_part_length_m_;
  // The number of blue or yellow cones in one of the boundaries (counted manually)
  // NOT INCLUDING ORANGE CONES!
  int num_boundary_cones_;
  // The number of orange cones after finish line in one of the boundaries (counted manually)
  // NOT INCLUDING BIG ORANGE CONES!
  int num_boundary_orange_cones_;
  // Max track width (blue to yellow) distance meters
  double max_track_width_y_m_;
  // Distance from car to first blue/yellow cones (estimated manually)
  // Cones less than this distance are ignored
  double estimated_first_cones_distance_x_m_;
  // Distance between big orange cones in x (estimated manually)
  double estimated_big_orange_cone_distance_x_m_;
  // Minimum distance between blue/yellow cones in x in meters
  double min_cone_distance_x_m_;
  // Maximum distance between blue/yellow cones in x in meters
  double max_cone_distance_x_m_;
  // How much weight is put on perception measured distance
  double perception_observation_weight_;
  // How many cones should be detected (per boundary) to build a good map
  int min_num_detected_cones_;

  // Sets all parameters
  void LoadParameters();

  // Acceleration helpers
  // Sorts cones w.r.t. x coordinate
  void SortConesX(ConeMap &cones) const;
  // Split the two boundaries and remove outliers
  double FilterCones(const ConeMap &cone_observations, ConeMap &positives, ConeMap &negatives) const;
  // Calculate the average distance between cones in the x direction
  double AverageConeDistanceX(ConeMap &positives, ConeMap &negatives) const;
  // Creates the blue, yellow and orange boundaries
  void GenerateAccelerationBoundaries(ConeMap &acceleration_map, double first_cone_position, double distance_x_m,
                                      double distance_y_m, int num_cones) const;
  // Adds the big orange cones of the finish line to the map
  void GenerateAccelerationBigOrangeCones(ConeMap &acceleration_map, double big_orange_cones_middle_x_m,
                                          double distance_y_m) const;
};

} // end namespace slam
