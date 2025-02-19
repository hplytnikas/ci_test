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

#include <cmath>
#include <memory>
#include <vector>

#include "slam_common/aliases.hpp"
#include "slam_common/association.hpp"
#include "slam_common/slam_node.hpp"
#include "slam_frontend/data_association.hpp"

namespace slam {

/*
 * Class for performing data association using the nearest neighbor algorithm.
 */
class NearestNeighborDataAssociation : public DataAssociation {
public:
  /*
   * Constructor.
   */
  explicit NearestNeighborDataAssociation(std::shared_ptr<SlamNode> node_handle);

  /*
   * Destructor.
   */
  ~NearestNeighborDataAssociation();

  // Override the pure virtual function
  std::vector<Association> Associate(const ConeMap &observations, const ConeMap &global_map) override;

private:
  // Parameters
  // Minimum distance to accept association
  double min_distance_threshold_m_;

  // Sets all parameters
  void LoadParameters();
};

} // end namespace slam
