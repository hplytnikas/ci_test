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

#include "vcu_comm_interface/socket.h"

namespace vcu_comm_interface {

Socket::Socket(uint16_t socket_port, rclcpp::Node *node) : cb_addr_{}, vcu_addr_{} {
  // Read IPs, max. message size & ports from config file

  // Declare parameters
  node->declare_parameter("ports.vcu_port", 1044);
  node->declare_parameter("max_msg_size", 25);
  node->declare_parameter("vcu_ip", "192.168.1.110");
  node->declare_parameter("cb_ip", "192.168.1.153");

  if (!node->get_parameter("ports.vcu_port", vcu_port_)) {
    RCLCPP_FATAL(node->get_logger(), "Did not specify port on which VCU receives messages!");
    rclcpp::shutdown();
  }

  if (!node->get_parameter("max_msg_size", max_msg_size_)) {
    RCLCPP_FATAL(node->get_logger(), "Did not specify max. size for exchanged UDP messages!");
    rclcpp::shutdown();
  }

  if (!node->get_parameter("vcu_ip", vcu_ip_)) {
    RCLCPP_FATAL(node->get_logger(), "Did not specify the IP of the VCU!");
    rclcpp::shutdown();
  }

  if (!node->get_parameter("cb_ip", cb_ip_)) {
    RCLCPP_FATAL(node->get_logger(), "Did not specify the IP of the CB!");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(node->get_logger(), "VCU IP: %s", vcu_ip_.c_str());
  RCLCPP_INFO(node->get_logger(), "CB IP: %s", cb_ip_.c_str());

  // Creating a UDP socket
  if ((sock_id_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_FATAL(node->get_logger(), "Failed to create a UDP socket!");
    rclcpp::shutdown();
  }

  // Filling CB address information
  cb_addr_.sin_family = AF_INET;
  cb_addr_.sin_addr.s_addr = inet_addr(cb_ip_.c_str());
  cb_addr_.sin_port = htons(socket_port);

  // Filling VCU address information
  vcu_addr_.sin_family = AF_INET;
  vcu_addr_.sin_addr.s_addr = inet_addr(vcu_ip_.c_str());
  vcu_addr_.sin_port = htons((uint16_t)vcu_port_);

  // Initialize lengths
  cb_addr_len_ = sizeof(cb_addr_);
  vcu_addr_len_ = sizeof(vcu_addr_);

  // Bind the socket with the CB address
  if (bind(sock_id_, (const struct sockaddr *)&cb_addr_, cb_addr_len_) < 0) {
    RCLCPP_FATAL(node->get_logger(), "Failed to bind UDP socket to CB address! Error: %s", strerror(errno));
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(node->get_logger(), "Socket initialized!");
  }
}

Socket::~Socket() {
  // Close the socket
  close(sock_id_);
}

bool Socket::SendMsg(const char data[], size_t data_len) {
  if (sendto(sock_id_, data, data_len, 0, (const struct sockaddr *)&vcu_addr_, vcu_addr_len_) < 0) {
    return false;
  } else {
    return true;
  }
}

ssize_t Socket::RecvMsg(char data[]) {
  return recvfrom(sock_id_, data, static_cast<size_t>(max_msg_size_), MSG_WAITALL, (struct sockaddr *)&vcu_addr_,
                  &vcu_addr_len_);
}

size_t Socket::GetMaxMsgSize() { return max_msg_size_; }

} // namespace vcu_comm_interface
