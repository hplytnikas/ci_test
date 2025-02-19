/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Romir Damle <rdamle@ethz.ch>
 *   - Emil Fahrig <efahrig@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "vcu_msgs/msg/velocity_estimation.hpp"
#include "rclcpp/rclcpp.hpp"
#include <random>

using std::placeholders::_1;

class VEFaker : public rclcpp::Node {
public:
  VEFaker() : Node("ve_faker") {
    // Declare VE parameters
    this->declare_parameter("ve_gt", false);
    this->declare_parameter("mean_vx", 0.0);
    this->declare_parameter("mean_vy", 0.0);
    this->declare_parameter("mean_vyaw", 0.0);
    this->declare_parameter("stddev_vx", 0.1);
    this->declare_parameter("stddev_vy", 0.18);
    this->declare_parameter("stddev_vyaw", 0.15);
    this->declare_parameter("mean_ax", 0.0);
    this->declare_parameter("mean_ay", 0.0);
    this->declare_parameter("stddev_ax", 0.11);
    this->declare_parameter("stddev_ay", 0.2);

    // Get VE noise parameters and log an error if not found
    if (!this->get_parameter("ve_gt", ve_gt)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find ve_gt parameter in amzsim_ve_config!");
      rclcpp::shutdown();
    }
    if (!this->get_parameter("mean_vx", mean_vx)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find mean_vx parameter in amzsim_ve_config!");
      rclcpp::shutdown();
    }
    if (!this->get_parameter("mean_vy", mean_vy)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find mean_vy parameter in amzsim_ve_config!");
      rclcpp::shutdown();
    }
    if (!this->get_parameter("mean_vyaw", mean_vy)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find mean_vyaw parameter in amzsim_ve_config!");
      rclcpp::shutdown();
    }
    if (!this->get_parameter("stddev_vx", stddev_vx)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find stddev_vx parameter in amzsim_ve_config!");
      rclcpp::shutdown();
    }
    if (!this->get_parameter("stddev_vy", stddev_vy)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find stddev_vy parameter in amzsim_ve_config!");
      rclcpp::shutdown();
    }
    if (!this->get_parameter("stddev_vyaw", stddev_vyaw)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find stddev_vyaw parameter in amzsim_ve_config!");
      rclcpp::shutdown();
    }
    if (!this->get_parameter("mean_ax", mean_ax)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find mean_ax parameter in amzsim_ve_config!");
      rclcpp::shutdown();
    }
    if (!this->get_parameter("mean_ay", mean_ay)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find mean_ay parameter in amzsim_ve_config!");
      rclcpp::shutdown();
    }
    if (!this->get_parameter("stddev_ax", stddev_ax)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find stddev_ax parameter in amzsim_ve_config!");
      rclcpp::shutdown();
    }
    if (!this->get_parameter("stddev_ay", stddev_ay)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find stddev_ay parameter in amzsim_ve_config!");
      rclcpp::shutdown();
    }

    // Initialize publisher for VE messages
    ve_pub = this->create_publisher<vcu_msgs::msg::VelocityEstimation>("/vcu_msgs/velocity_estimation", 10);

    // GT logic - add noise or not
    if (ve_gt) {
      ve_sub = this->create_subscription<vcu_msgs::msg::VelocityEstimation>(
          "/vcu_msgs/velocity_estimation_gt", 10, std::bind(&VEFaker::gt_listener_ve_talker, this, _1));
    } else {
      ve_sub = this->create_subscription<vcu_msgs::msg::VelocityEstimation>(
          "/vcu_msgs/velocity_estimation_gt", 10, std::bind(&VEFaker::gt_listener_ve_talker_noisy, this, _1));
    }
  }
  ~VEFaker() = default;

private:
  // Publish VE without noise
  void gt_listener_ve_talker(const vcu_msgs::msg::VelocityEstimation &msg) { ve_pub->publish(msg); }

  // Add noise to VE
  void gt_listener_ve_talker_noisy(const vcu_msgs::msg::VelocityEstimation &msg) {
    vcu_msgs::msg::VelocityEstimation noisy_state_msg;

    // std::default_random_engine generator;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<double> distribution_vx(mean_vx, stddev_vx);
    std::normal_distribution<double> distribution_vy(mean_vy, stddev_vy);
    std::normal_distribution<double> distribution_vyaw(mean_vyaw, stddev_vyaw);
    std::normal_distribution<double> distribution_ax(mean_ax, stddev_ax);
    std::normal_distribution<double> distribution_ay(mean_ay, stddev_ay);

    // Fixed distribution but it generates a constant random number
    auto t_now = clock.now();
    if (!msg.vel.x) {
      ve_pub->publish(msg);
    } else {
      noisy_state_msg.vel.x = msg.vel.x + distribution_vx(generator);
      noisy_state_msg.vel.y = msg.vel.y + distribution_vy(generator);
      noisy_state_msg.vel.theta = msg.vel.theta + distribution_vyaw(generator);
      noisy_state_msg.acc.x = msg.acc.x + distribution_ax(generator);
      noisy_state_msg.acc.y = msg.acc.y + distribution_ay(generator);
      noisy_state_msg.header.stamp = t_now;
      ve_pub->publish(noisy_state_msg);
    }
  }

  // Need to put in config yaml file
  bool ve_gt;
  double mean_vx;
  double mean_vy;
  double mean_vyaw;
  double stddev_vx;
  double stddev_vy;
  double stddev_vyaw;
  double mean_ax;
  double mean_ay;
  double stddev_ax;
  double stddev_ay;

  rclcpp::Publisher<vcu_msgs::msg::VelocityEstimation>::SharedPtr ve_pub;
  rclcpp::Subscription<vcu_msgs::msg::VelocityEstimation>::SharedPtr ve_sub;
  rclcpp::Clock clock;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  try {
    VEFaker::SharedPtr velocity_est = std::make_shared<VEFaker>();
    rclcpp::spin(velocity_est);
  } catch (const std::exception &e) {
  }

  rclcpp::shutdown();
  return 0;
}
