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

#include <rclcpp/rclcpp.hpp>

#include <cassert>
#include <map>
#include <memory>
#include <string>

#include <autonomous_msgs/msg/bool_stamped.hpp>
#include <autonomous_msgs/msg/double_stamped.hpp>
#include <vcu_msgs/msg/ebs_pressures.hpp>
#include <vcu_msgs/msg/llc_logging.hpp>
#include <vcu_msgs/msg/llc_parameters.hpp>
#include <vcu_msgs/msg/lv_battery.hpp>
#include <vcu_msgs/msg/mission_select.hpp>
#include <vcu_msgs/msg/res_state.hpp>
#include <vcu_msgs/msg/state_machine.hpp>
#include <vcu_msgs/msg/torque_data.hpp>
#include <vcu_msgs/msg/velocity_estimation.hpp>

#include "receiver_msgs.h"
#include "socket.h"

#define BIT(v, m) (((v) & (m)) != 0)

namespace vcu_comm_interface {

class Receiver : public rclcpp::Node {
public:
  Receiver();
  ~Receiver() {}

  // Listens for data and passes arriving packets to interpreters based on ID
  bool ListenForMessages();

private:
  // Socket
  std::unique_ptr<Socket> recv_socket_;

  // RES
  const uint8_t res_emergency_ = 0x1;
  const uint8_t res_switch_onoff_ = 0x2;
  const uint8_t res_switch_button_ = 0x4;
  const uint8_t res_comm_interrupt_ = 0x8;

  // Publishers
  rclcpp::Publisher<vcu_msgs::msg::VelocityEstimation>::SharedPtr velocity_estimation_pub_;
  rclcpp::Publisher<vcu_msgs::msg::StateMachine>::SharedPtr state_machine_pub_;
  rclcpp::Publisher<vcu_msgs::msg::LvBattery>::SharedPtr lv_battery_pub_;
  rclcpp::Publisher<vcu_msgs::msg::EbsPressures>::SharedPtr ebs_pressures_pub_;
  rclcpp::Publisher<vcu_msgs::msg::ResState>::SharedPtr res_state_pub_;
  rclcpp::Publisher<vcu_msgs::msg::MissionSelect>::SharedPtr mission_select_pub_;
  rclcpp::Publisher<autonomous_msgs::msg::DoubleStamped>::SharedPtr steering_feedback_pub_;
  rclcpp::Publisher<vcu_msgs::msg::LlcLogging>::SharedPtr llc_logging_pub_;
  rclcpp::Publisher<vcu_msgs::msg::LlcParameters>::SharedPtr llc_parameters_pub_;
  rclcpp::Publisher<vcu_msgs::msg::TorqueData>::SharedPtr torque_data_pub_;

  // Mission names
  const std::map<int, std::string> mission_names{{0, "none"},       {1, "manual"},     {2, "acceleration"},
                                                 {3, "skidpad"},    {4, "trackdrive"}, {5, "ebs_test"},
                                                 {6, "inspection"}, {7, "autocross"},  {8, "pushtest"}};

  // Message IDs
  enum MessageId {
    VelocityEstimation = 1,
    StateMachine = 2,
    LvBattery = 3,
    EbsPressures = 4,
    ResState = 5,
    LLCLogging = 6,
    LLCParameters = 7,
    TorqueData = 8,
  };

  // Interpreting functions
  void VelocityEstimationInterpreter(char data[]);
  void StateMachineInterpreter(char data[]);
  void LvBatteryInterpreter(char data[]);
  void EbsPressuresInterpreter(char data[]);
  void ResStateInterpreter(char data[]);
  void LLCLoggingInterpreter(char data[]);
  void LLCParametersInterpreter(char data[]);
  void TorqueDataInterpreter(char data[]);
};

} // namespace vcu_comm_interface
