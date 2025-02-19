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
#include "slam_frontend/nn_data_association.hpp"

#include <cmath>
#include <easy/profiler.h>

namespace slam {

NearestNeighborDataAssociation::NearestNeighborDataAssociation(std::shared_ptr<SlamNode> node_handle)
    : DataAssociation(node_handle) {
  LoadParameters();
}

NearestNeighborDataAssociation::~NearestNeighborDataAssociation() {}

void NearestNeighborDataAssociation::LoadParameters() {
  min_distance_threshold_m_ =
      node_handle_->GetParameter<double>("data_association.nearest_neighbor.min_distance_threshold_m", 1.2);

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "NearestNeighborDataAssociation: Parameters loaded.");
}

std::vector<Association> NearestNeighborDataAssociation::Associate(const ConeMap &observations,
                                                                   const ConeMap &global_map) {
  EASY_FUNCTION(profiler::colors::Green); // Time this function
  // Setup variables
  const size_t num_observed_cones = observations.size();

  // Cone associations between observed cones and global map cones
  std::vector<Association> associations;
  associations.reserve(num_observed_cones);

  // Find nearest neighbor for each observed cone
  for (size_t observed_cone_idx = 0; observed_cone_idx < num_observed_cones; ++observed_cone_idx) {
    const autonomous_msgs::msg::Cone observed_cone = observations[observed_cone_idx];

    const double observed_x = observed_cone.position.x;
    const double observed_y = observed_cone.position.y;

    // Initialize nearest neighbor variables
    size_t matched_cone_idx = -1;
    double associated_distance = std::numeric_limits<double>::max();
    const size_t num_global_map_cones = global_map.size();

    if (num_global_map_cones > 0) {
      // Match observed cone with nearest neighbor in global map (find nearest
      // global cone & distance)
      for (size_t global_map_cone_idx = 0; global_map_cone_idx < num_global_map_cones; ++global_map_cone_idx) {
        double cone_x = global_map[global_map_cone_idx].position.x;
        double cone_y = global_map[global_map_cone_idx].position.y;
        double dist = std::hypot(cone_x - observed_x, cone_y - observed_y);

        if (dist < associated_distance) {
          associated_distance = dist;
          matched_cone_idx = global_map_cone_idx;
        }
      }
    }

    int global_map_id = -1; // -1 if not associated with a cone on global map
    if (associated_distance <= min_distance_threshold_m_) {
      global_map_id = global_map[matched_cone_idx].id_cone;
    }

    // Store association
    associations.push_back(Association(observed_cone_idx, global_map_id, false));
  }

  return associations;
}

} // end namespace slam
