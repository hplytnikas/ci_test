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

#include <memory>
#include <vector>

#include "slam_common/aliases.hpp"
#include "slam_common/association.hpp"
#include "slam_common/slam_node.hpp"

namespace slam {

/*
 * Base class for performing data association between cones. Inherit from this
 * class and override the method Associate for a different data association
 * algorithm.
 */
class DataAssociation {
public:
  /*
   * Constructor.
   */
  explicit DataAssociation(std::shared_ptr<SlamNode> node_handle);

  /*
   * Destructor.
   */
  virtual ~DataAssociation();

  /*!
   * Performs data association between observed cones and cones in the global
   * map. Override the following method for a different data association
   * algorithm.
   *
   * @param observations - Local map message (in the global frame!)
   * @param global_map - Global map message
   * @return An array of associations which map from the index of a local map
   * cone to the index of a global map cone, or -1 if no association was made.
   */
  virtual std::vector<Association> Associate(const ConeMap &observations, const ConeMap &global_map) = 0;

protected: // Allows derived classes to access node_handle_
  std::shared_ptr<SlamNode> node_handle_;
}; // end class DataAssociation

} // end namespace slam
