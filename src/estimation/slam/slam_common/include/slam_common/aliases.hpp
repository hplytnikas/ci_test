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

#include <vector>

#include "autonomous_msgs/msg/cone.hpp"

namespace slam {

/*
 * Data type to represent a map of Cones.
 */
typedef std::vector<autonomous_msgs::msg::Cone> ConeMap;

} // namespace slam
