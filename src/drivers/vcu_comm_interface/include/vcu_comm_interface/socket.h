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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <cstring>
#include <string>

namespace vcu_comm_interface {

class Socket {
public:
  // Constructor for binding socket to IP's
  explicit Socket(uint16_t socket_port, rclcpp::Node *node);
  ~Socket();

  // Functions for sending and receiving data on the socket
  bool SendMsg(const char data[], size_t data_len);
  ssize_t RecvMsg(char data[]);

  size_t GetMaxMsgSize();

private:
  int sock_id_;
  sockaddr_in cb_addr_, vcu_addr_;
  socklen_t cb_addr_len_, vcu_addr_len_;

  std::string vcu_ip_;
  std::string cb_ip_;
  int vcu_port_;
  int max_msg_size_;
};
} // namespace vcu_comm_interface
