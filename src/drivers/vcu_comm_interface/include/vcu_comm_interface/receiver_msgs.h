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

struct ResStateMsg {
  boost::endian::little_uint8_t id;
  boost::endian::little_uint8_t switches;
};

struct StateMachineMsg {
  boost::endian::little_uint8_t id;
  boost::endian::little_uint8_t as_state;
  boost::endian::little_uint8_t ebs_state;
  boost::endian::little_uint8_t ecu_flag;
  boost::endian::little_int16_t steering_feedback;
};

struct LvBatteryMsg {
  boost::endian::little_uint8_t id;
  boost::endian::little_uint8_t cell1;
  boost::endian::little_uint8_t cell2;
  boost::endian::little_uint8_t cell3;
  boost::endian::little_uint8_t cell4;
  boost::endian::little_uint8_t cell5;
  boost::endian::little_uint8_t cell6;
  boost::endian::little_uint8_t cell7;
  boost::endian::little_uint8_t cell8;
  boost::endian::little_uint8_t cell9;
  boost::endian::little_uint8_t cell10;
  boost::endian::little_uint8_t cell11;
  boost::endian::little_uint8_t cell12;
  boost::endian::little_uint8_t cell13;
  boost::endian::little_uint8_t cell14;
};

struct EbsPressuresMsg {
  boost::endian::little_uint8_t id;
  boost::endian::little_uint8_t ebs_pressure_front;
  boost::endian::little_uint8_t ebs_pressure_rear;
  boost::endian::little_uint8_t brake_pressure_front;
  boost::endian::little_uint8_t brake_pressure_rear;
};

struct VelocityEstimationMsg {
  boost::endian::little_uint8_t id;
  // Velocities
  boost::endian::little_int16_t velocity_longitudinal;
  boost::endian::little_int16_t velocity_lateral;
  boost::endian::little_int16_t yawrate;
  // Accelerations
  boost::endian::little_int16_t acceleration_longitudinal;
  boost::endian::little_int16_t acceleration_lateral;
  boost::endian::little_int16_t acceleration_yaw;
  // Covariances
  boost::endian::little_uint16_t vel_cov_long_long;
  boost::endian::little_uint16_t vel_cov_lat_lat;
  boost::endian::little_uint16_t vel_cov_yaw_yaw;
  boost::endian::little_uint16_t vel_cov_long_lat;
  boost::endian::little_uint16_t vel_cov_long_yaw;
  boost::endian::little_uint16_t vel_cov_lat_yaw;
};

struct LLCLoggingMsg {
  boost::endian::little_uint8_t id;

  // longitudinal control
  boost::endian::little_int16_t ax_target;                // [-32, 32] m/s^2, /1024
  boost::endian::little_int16_t ax_measured;              // [-32, 32] m/s^2, /1024
  boost::endian::little_int16_t force_rolling_resistance; // /1
  boost::endian::little_int16_t force_feedforward;        // /1
  boost::endian::little_int16_t force_drag;               // /1
  boost::endian::little_int16_t force_standstill;         // /1
  boost::endian::little_int16_t force_pid;                // /1
  boost::endian::little_int16_t force_total;              // /1
};

struct LLCParametersMsg {
  boost::endian::little_uint8_t id;

  // tunning values
  boost::endian::little_uint16_t m_value;          // /1
  boost::endian::little_uint16_t q_value;          // /1
  boost::endian::little_uint16_t cda_value;        // [0, 16] /4096
  boost::endian::little_uint16_t peak_of_gaussian; // /1
  boost::endian::little_uint16_t std_of_gaussian;  // [0, 16] /4096
  boost::endian::little_uint16_t p_value;          // [0, 4096] /16
  boost::endian::little_uint16_t i_value;          // [0, 4096] /16
  boost::endian::little_uint16_t d_value;          // [0, 4096] /16
  boost::endian::little_uint16_t n_value;          // [0, 1] /65536
  boost::endian::little_uint16_t pid_limit;        // /1
};

struct TorqueDataMsg {
  boost::endian::little_uint8_t id;

  // Feedback values from inverter
  boost::endian::little_int16_t feedback_T_M_FL; // [-32, 32] Nm, /1024
  boost::endian::little_int16_t feedback_T_M_FR; // [-32, 32] Nm, /1024
  boost::endian::little_int16_t feedback_T_M_RL; // [-32, 32] Nm, /1024
  boost::endian::little_int16_t feedback_T_M_RR; // [-32, 32] Nm, /1024

  // Reference values for inverter
  boost::endian::little_int16_t reference_T_M_FL; // [-32, 32] Nm, /1024
  boost::endian::little_int16_t reference_T_M_FR; // [-32, 32] Nm, /1024
  boost::endian::little_int16_t reference_T_M_RL; // [-32, 32] Nm, /1024
  boost::endian::little_int16_t reference_T_M_RR; // [-32, 32] Nm, /1024

  // Others
  boost::endian::little_int16_t estimated_limit; // [-32, 32] Nm, /1024
  boost::endian::little_int16_t driverrequest;   // [-32, 32] Nm, /1024
};

} // namespace vcu_comm_interface
