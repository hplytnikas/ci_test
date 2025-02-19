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

#include "vcu_comm_interface/receiver.h"

namespace vcu_comm_interface {

Receiver::Receiver() : rclcpp::Node("receiver_node") {
  // Get the port that we receive the VCU messages on
  unsigned int receiver_port = this->declare_parameter("ports.receiver_port", 1027);
  if (!this->get_parameter("ports.receiver_port", receiver_port)) {
    RCLCPP_FATAL(get_logger(), "Did not specify CB port used for receiving messages from VCU!");
    rclcpp::shutdown();
  }

  // Initialize socket
  recv_socket_ = std::make_unique<Socket>((uint16_t)receiver_port, this);

  // Advertise to topics
  velocity_estimation_pub_ =
      this->create_publisher<vcu_msgs::msg::VelocityEstimation>("/vcu_msgs/velocity_estimation", 10);
  state_machine_pub_ = this->create_publisher<vcu_msgs::msg::StateMachine>("/vcu_msgs/state_machine", 1);
  lv_battery_pub_ = this->create_publisher<vcu_msgs::msg::LvBattery>("/vcu_msgs/lv_battery", 1);
  ebs_pressures_pub_ = this->create_publisher<vcu_msgs::msg::EbsPressures>("/vcu_msgs/ebs_pressures", 1);
  res_state_pub_ = this->create_publisher<vcu_msgs::msg::ResState>("/vcu_msgs/res_state", 1);
  mission_select_pub_ = this->create_publisher<vcu_msgs::msg::MissionSelect>("/vcu_msgs/mission_select", 1);
  steering_feedback_pub_ =
      this->create_publisher<autonomous_msgs::msg::DoubleStamped>("/vcu_msgs/steering_feedback", 1);
  llc_logging_pub_ = this->create_publisher<vcu_msgs::msg::LlcLogging>("/vcu_msgs/llc_logging", 1);
  llc_parameters_pub_ = this->create_publisher<vcu_msgs::msg::LlcParameters>("/vcu_msgs/llc_parameters", 1);
  torque_data_pub_ = this->create_publisher<vcu_msgs::msg::TorqueData>("/vcu_msgs/torque_data", 1);

  // Listen to UDP messages and convert them to ROS messages in an infinite loop
  while (rclcpp::ok()) {
    bool success = ListenForMessages();
    if (!success) {
      RCLCPP_WARN(get_logger(), "UDP interface received a UDP message but could not interpret it!");
    }
  }
}

bool Receiver::ListenForMessages() {
  // Create a buffer for the message with maximal expected size
  char data[recv_socket_->GetMaxMsgSize()]; // NOLINT

  // Listen to messages on the socket
  ssize_t data_size = recv_socket_->RecvMsg(data);
  if (data_size < 0) {
    RCLCPP_WARN(get_logger(), "An error occured when listening to a UDP message!");
    return false;
  }

  // Extract the ID & payload of the message
  char id = data[0];

  // Decide which interpreter to call based on ID of the received message
  switch (id) {
  case ResState:
    assert((data_size == sizeof(ResStateMsg)) && "ResStateMsg has been received but it has wrong size!");
    ResStateInterpreter(data);
    break;
  case StateMachine:
    assert((data_size == sizeof(StateMachineMsg)) && "StateMachinesMsg has been received but it has wrong size!");
    StateMachineInterpreter(data);
    break;
  case LvBattery:
    assert((data_size == sizeof(LvBatteryMsg)) && "LvAkkuMsg has been received but it has wrong size!");
    LvBatteryInterpreter(data);
    break;
  case EbsPressures:
    assert((data_size == sizeof(EbsPressuresMsg)) && "EbsPressuresMsg has been received but it has wrong size!");
    EbsPressuresInterpreter(data);
    break;
  case VelocityEstimation:
    assert((data_size == sizeof(VelocityEstimationMsg)) &&
           "VelocityEstimationMsg has been received but it has wrong size!");
    VelocityEstimationInterpreter(data);
    break;
  case LLCLogging:
    assert((data_size == sizeof(LLCLoggingMsg)) && "LLCLoggingMsg has been received but it has wrong size!");
    LLCLoggingInterpreter(data);
    break;
  case LLCParameters:
    assert((data_size == sizeof(LLCParametersMsg)) && "LLCParametersMsg has been received but it has wrong size!");
    LLCParametersInterpreter(data);
    break;
  case TorqueData:
    assert((data_size == sizeof(TorqueDataMsg)) && "TorqueDataMsg has been received but it has wrong size!");
    TorqueDataInterpreter(data);
    break;
  default:
    RCLCPP_WARN(get_logger(), "Cannot interpret a UDP message with this ID: %d!", id);
    return false;
  }

  return true;
}

void Receiver::ResStateInterpreter(char data[]) {
  vcu_msgs::msg::ResState res_state_msg;

  const ResStateMsg *msg = reinterpret_cast<const ResStateMsg *>(data);

  res_state_msg.header.stamp = now();
  res_state_msg.emergency = !BIT((uint8_t)msg->switches, 0x1); // Emergency is active when the bit is off
  res_state_msg.on_off_switch = BIT((uint8_t)msg->switches >> 1, 0x1);
  res_state_msg.push_button = BIT((uint8_t)msg->switches >> 2, 0x1);
  res_state_msg.communication_interrupted = BIT((uint8_t)msg->switches >> 3, 0x1);

  res_state_pub_->publish(res_state_msg);
}

void Receiver::StateMachineInterpreter(char data[]) {
  autonomous_msgs::msg::DoubleStamped steering_feedback_msg;
  vcu_msgs::msg::MissionSelect mission_select_msg;
  vcu_msgs::msg::StateMachine state_machine_msg;
  const StateMachineMsg *msg = reinterpret_cast<const StateMachineMsg *>(data);

  // Fill mission select message
  uint8_t mission_id = (uint8_t)msg->ecu_flag >> 1 & 0x7;
  mission_select_msg.mission_nr = mission_id;
  // RCLCPP_INFO(get_logger(), "The ecu flag is: %x", (uint8_t)msg->ecu_flag >> 1);
  // RCLCPP_INFO(get_logger(), "The ecu flag is: %x (%x)", (uint8_t)msg->ecu_flag, mission_id);
  try {
    mission_select_msg.mission_name = mission_names.at(mission_id);
  } catch (...) {
    RCLCPP_FATAL(get_logger(), "The selected mission does not exist in the list!");
  }

  // Fill machine state message
  state_machine_msg.header.stamp = now();
  state_machine_msg.as_state = msg->as_state;
  state_machine_msg.ebs_state = msg->ebs_state;
  state_machine_msg.ebs_released = BIT((uint8_t)msg->ecu_flag, 0x1);
  state_machine_msg.as_mission = mission_id;
  state_machine_msg.ins_fail = BIT((uint8_t)msg->ecu_flag >> 4, 0x1);
  state_machine_msg.imu_fail = BIT((uint8_t)msg->ecu_flag >> 5, 0x1);
  state_machine_msg.ass_fail = BIT((uint8_t)msg->ecu_flag >> 6, 0x1);

  // Fill steering feedback message
  steering_feedback_msg.header.stamp = now();
  steering_feedback_msg.data = msg->steering_feedback / -10000.0;

  steering_feedback_pub_->publish(steering_feedback_msg);
  mission_select_pub_->publish(mission_select_msg);
  state_machine_pub_->publish(state_machine_msg);
}

void Receiver::LvBatteryInterpreter(char data[]) {
  vcu_msgs::msg::LvBattery lv_battery_msg;
  const LvBatteryMsg *msg = reinterpret_cast<const LvBatteryMsg *>(data);

  lv_battery_msg.header.stamp = now();
  lv_battery_msg.cell1 = static_cast<float>(msg->cell1) / 10;
  lv_battery_msg.cell2 = static_cast<float>(msg->cell2) / 10;
  lv_battery_msg.cell3 = static_cast<float>(msg->cell3) / 10;
  lv_battery_msg.cell4 = static_cast<float>(msg->cell4) / 10;
  lv_battery_msg.cell5 = static_cast<float>(msg->cell5) / 10;
  lv_battery_msg.cell6 = static_cast<float>(msg->cell6) / 10;
  lv_battery_msg.cell7 = static_cast<float>(msg->cell7) / 10;
  lv_battery_msg.cell8 = static_cast<float>(msg->cell8) / 10;
  lv_battery_msg.cell9 = static_cast<float>(msg->cell9) / 10;
  lv_battery_msg.cell10 = static_cast<float>(msg->cell10) / 10;
  lv_battery_msg.cell11 = static_cast<float>(msg->cell11) / 10;
  lv_battery_msg.cell12 = static_cast<float>(msg->cell12) / 10;
  lv_battery_msg.cell13 = static_cast<float>(msg->cell13) / 10;
  lv_battery_msg.cell14 = static_cast<float>(msg->cell14) / 10;

  float min_cell_voltage = std::min(
      {lv_battery_msg.cell1, lv_battery_msg.cell2, lv_battery_msg.cell3, lv_battery_msg.cell4, lv_battery_msg.cell5,
       lv_battery_msg.cell6, lv_battery_msg.cell7, lv_battery_msg.cell8, lv_battery_msg.cell9, lv_battery_msg.cell10,
       lv_battery_msg.cell11, lv_battery_msg.cell12, lv_battery_msg.cell13, lv_battery_msg.cell14});

  if (min_cell_voltage <= 3.3) {
    lv_battery_msg.lv_soc = 0.0;
  } else if (3.3 < min_cell_voltage && min_cell_voltage <= 3.6) {
    lv_battery_msg.lv_soc = min_cell_voltage / 3 - 1.1;
  } else if (3.6 < min_cell_voltage && min_cell_voltage <= 3.9) {
    lv_battery_msg.lv_soc = min_cell_voltage * 2 - 7.1;
  } else if (3.9 < min_cell_voltage && min_cell_voltage <= 4.2) {
    lv_battery_msg.lv_soc = min_cell_voltage - 3.2;
  } else {
    lv_battery_msg.lv_soc = 1.0;
  }

  lv_battery_pub_->publish(lv_battery_msg);
}

void Receiver::EbsPressuresInterpreter(char data[]) {
  vcu_msgs::msg::EbsPressures ebs_pressures_msg;
  const EbsPressuresMsg *msg = reinterpret_cast<const EbsPressuresMsg *>(data);

  ebs_pressures_msg.header.stamp = now();
  ebs_pressures_msg.ebs_pressure_front = static_cast<float>(msg->ebs_pressure_front) / 10;
  ebs_pressures_msg.ebs_pressure_rear = static_cast<float>(msg->ebs_pressure_rear) / 10;
  ebs_pressures_msg.brake_pressure_front = static_cast<float>(msg->brake_pressure_front) / 10;
  ebs_pressures_msg.brake_pressure_rear = static_cast<float>(msg->brake_pressure_rear) / 10;

  ebs_pressures_pub_->publish(ebs_pressures_msg);
}

void Receiver::VelocityEstimationInterpreter(char data[]) {
  vcu_msgs::msg::VelocityEstimation velocity_estimation_msg;
  const VelocityEstimationMsg *msg = reinterpret_cast<const VelocityEstimationMsg *>(data);

  // Fill header
  velocity_estimation_msg.header.stamp = now();
  velocity_estimation_msg.header.frame_id = "vehicle";

  // Fill velocities
  velocity_estimation_msg.vel.x = static_cast<float>(msg->velocity_longitudinal) / 5e2;
  velocity_estimation_msg.vel.y = static_cast<float>(msg->velocity_lateral) / 5e2;
  velocity_estimation_msg.vel.theta = static_cast<float>(msg->yawrate) / 1e4;

  // Fill accelerations
  velocity_estimation_msg.acc.x = static_cast<float>(msg->acceleration_longitudinal) / 800;
  velocity_estimation_msg.acc.y = static_cast<float>(msg->acceleration_lateral) / 800;
  velocity_estimation_msg.acc.theta = static_cast<float>(msg->acceleration_yaw) / 800;

  // Fill covariances
  velocity_estimation_msg.vel_cov.x_x = static_cast<float>(msg->vel_cov_long_long) / 1e4;
  velocity_estimation_msg.vel_cov.y_y = static_cast<float>(msg->vel_cov_lat_lat) / 1e4;
  velocity_estimation_msg.vel_cov.theta_theta = static_cast<float>(msg->vel_cov_yaw_yaw) / 1e4;
  velocity_estimation_msg.vel_cov.x_y = static_cast<float>(msg->vel_cov_long_lat) / 1e4;
  velocity_estimation_msg.vel_cov.x_theta = static_cast<float>(msg->vel_cov_long_yaw) / 1e4;
  velocity_estimation_msg.vel_cov.y_theta = static_cast<float>(msg->vel_cov_lat_yaw) / 1e4;

  velocity_estimation_pub_->publish(velocity_estimation_msg);
}

void Receiver::LLCLoggingInterpreter(char data[]) {
  vcu_msgs::msg::LlcLogging llc_logging_msg;
  const LLCLoggingMsg *msg = reinterpret_cast<const LLCLoggingMsg *>(data);

  llc_logging_msg.header.stamp = now();
  llc_logging_msg.ax_target = static_cast<float>(msg->ax_target) / 1024;
  llc_logging_msg.ax_measured = static_cast<float>(msg->ax_measured) / 1024;
  llc_logging_msg.force_rolling_resistance = static_cast<float>(msg->force_rolling_resistance);
  llc_logging_msg.force_feedforward = static_cast<float>(msg->force_feedforward);
  llc_logging_msg.force_drag = static_cast<float>(msg->force_drag);
  llc_logging_msg.force_standstill = static_cast<float>(msg->force_standstill);
  llc_logging_msg.force_pid = static_cast<float>(msg->force_pid);
  llc_logging_msg.force_total = static_cast<float>(msg->force_total);

  llc_logging_pub_->publish(llc_logging_msg);
}

void Receiver::LLCParametersInterpreter(char data[]) {
  vcu_msgs::msg::LlcParameters llc_parameters_msg;
  const LLCParametersMsg *msg = reinterpret_cast<const LLCParametersMsg *>(data);

  llc_parameters_msg.header.stamp = now();
  llc_parameters_msg.m_value = static_cast<float>(msg->m_value);
  llc_parameters_msg.q_value = static_cast<float>(msg->q_value);
  llc_parameters_msg.cda_value = static_cast<float>(msg->cda_value) / 4096;
  llc_parameters_msg.peak_of_gaussian = static_cast<float>(msg->peak_of_gaussian);
  llc_parameters_msg.std_of_gaussian = static_cast<float>(msg->std_of_gaussian) / 4096;
  llc_parameters_msg.p_value = static_cast<float>(msg->p_value) / 16;
  llc_parameters_msg.i_value = static_cast<float>(msg->i_value) / 16;
  llc_parameters_msg.d_value = static_cast<float>(msg->d_value) / 16;
  llc_parameters_msg.n_value = static_cast<float>(msg->n_value) / 65536;
  llc_parameters_msg.pid_limit_value = static_cast<float>(msg->pid_limit);
  llc_parameters_pub_->publish(llc_parameters_msg);
}

void Receiver::TorqueDataInterpreter(char data[]) {
  vcu_msgs::msg::TorqueData torque_data_msg;
  const TorqueDataMsg *msg = reinterpret_cast<const TorqueDataMsg *>(data);

  torque_data_msg.header.stamp = now();

  torque_data_msg.feedback_t_m_fl = static_cast<float>(msg->feedback_T_M_FL) / 1024;
  torque_data_msg.feedback_t_m_fr = static_cast<float>(msg->feedback_T_M_FR) / 1024;
  torque_data_msg.feedback_t_m_rl = static_cast<float>(msg->feedback_T_M_RL) / 1024;
  torque_data_msg.feedback_t_m_rr = static_cast<float>(msg->feedback_T_M_RR) / 1024;

  torque_data_msg.reference_t_m_fl = static_cast<float>(msg->reference_T_M_FL) / 1024;
  torque_data_msg.reference_t_m_fr = static_cast<float>(msg->reference_T_M_FR) / 1024;
  torque_data_msg.reference_t_m_rl = static_cast<float>(msg->reference_T_M_RL) / 1024;
  torque_data_msg.reference_t_m_rr = static_cast<float>(msg->reference_T_M_RR) / 1024;

  torque_data_msg.estimated_limit = static_cast<float>(msg->estimated_limit) / 1024;
  torque_data_msg.driverrequest = static_cast<float>(msg->driverrequest) / 1024;

  torque_data_pub_->publish(torque_data_msg);
}

} // namespace vcu_comm_interface
