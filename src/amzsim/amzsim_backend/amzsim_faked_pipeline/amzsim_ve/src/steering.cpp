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

#include "autonomous_msgs/msg/double_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class SteeringFaker : public rclcpp::Node {
public:
  SteeringFaker() : Node("steering_faker") {
    steering_pub = this->create_publisher<autonomous_msgs::msg::DoubleStamped>("/vcu_msgs/steering_feedback", 10);
    if (ve_gt) {
      steering_sub = this->create_subscription<autonomous_msgs::msg::DoubleStamped>(
          "/vcu_msgs/steering_gt", 10, std::bind(&SteeringFaker::gt_listener_steering_talker, this, _1));
    } else {
      steering_sub = this->create_subscription<autonomous_msgs::msg::DoubleStamped>(
          "/vcu_msgs/steering_gt", 10, std::bind(&SteeringFaker::gt_listener_steering_talker_noisy, this, _1));
    }
  }

  ~SteeringFaker() = default;

private:
  bool ve_gt = true;

  void gt_listener_steering_talker(const autonomous_msgs::msg::DoubleStamped &msg) { steering_pub->publish(msg); }

  void gt_listener_steering_talker_noisy(const autonomous_msgs::msg::DoubleStamped &msg) { steering_pub->publish(msg); }

  rclcpp::Publisher<autonomous_msgs::msg::DoubleStamped>::SharedPtr steering_pub;
  rclcpp::Subscription<autonomous_msgs::msg::DoubleStamped>::SharedPtr steering_sub;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  try {
    SteeringFaker::SharedPtr steering = std::make_shared<SteeringFaker>();
    rclcpp::spin(steering);
  } catch (const std::exception &e) {
  }

  rclcpp::shutdown();
  return 0;
}
