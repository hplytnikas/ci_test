/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Stanislaw Piasecki <stanislaw.piasecki@inf.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include <boost/endian/arithmetic.hpp>
#include <cstdint>

namespace vcu_comm_interface {

struct CarCommandMsg {
  CarCommandMsg() : id(static_cast<boost::endian::little_uint8_t>(6)) {}
  boost::endian::little_uint8_t id;
  boost::endian::little_int16_t steering_angle[3];
  boost::endian::little_int16_t a_x[3];
  boost::endian::little_int16_t yaw_rate[3];
};

struct FsgDataloggerMsg {
  FsgDataloggerMsg() : id(static_cast<boost::endian::little_uint8_t>(7)) {}
  boost::endian::little_uint8_t id;
  boost::endian::little_uint8_t cones_count_actual;
  boost::endian::little_uint16_t cones_count_all;
  boost::endian::little_uint8_t lap_counter;
};

struct MissionFinishedMsg {
  MissionFinishedMsg() : id(static_cast<boost::endian::little_uint8_t>(8)) {}
  boost::endian::little_uint8_t id;
  boost::endian::little_uint8_t mission_finished;
};

} // namespace vcu_comm_interface
