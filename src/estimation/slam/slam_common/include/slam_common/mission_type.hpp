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

// Enum for the various types of missions
enum MissionType {
  kAcceleration = 0, // Straight-line acceleration test
  kSkidpad,          // Figure-8 or circular track to test lateral acceleration
  kAutocross,        // Short and complex track, testing agility and speed
  kTrackdrive,       // Long distance track, testing endurance and overall performance
};

} // end namespace slam
