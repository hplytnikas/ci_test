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

namespace slam {

/*
 * Struct that represents an association between an observed cone and a cone
 * in the global map.
 */
struct Association {
  Association(int observed_cone_idx, int global_map_id, bool is_missassociation)
      : observed_cone_index(observed_cone_idx), global_map_cone_id(global_map_id), is_misassoc(is_missassociation) {}
  int observed_cone_index;
  int global_map_cone_id;
  bool is_misassoc;
};

} // end namespace slam
