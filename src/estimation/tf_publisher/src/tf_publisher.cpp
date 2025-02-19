/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - Giacomo Piatelli <gpiatelli@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "tf_publisher.h"

#define kIntegrationTimeout 1.0
#define kCoorEpsilon 1e-5
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)

using std::placeholders::_1;

namespace tf_publisher {

TFPublisher::TFPublisher() : Node("tf_publisher"), base_link_(this, "odom", "base_link") {
  // Subscribe to the velocity estimation topic
  velocity_est_sub_ = this->create_subscription<vcu_msgs::msg::VelocityEstimation>(
      "/vcu_msgs/velocity_estimation", 4, std::bind(&TFPublisher::VelocityEstCallback, this, _1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  base_link_.StartBaseLink();

  // Code related to the debug mode
  this->mode_ = this->declare_parameter("mode", "");

  if (strcmp(mode_.c_str(), "debug") == 0) {
    RCLCPP_INFO(this->get_logger(), "Debug mode enabled");
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tf_publisher/pose", 4);
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/tf_publisher/pose_array", 4);
    auto timer_callback = [this]() { this->base_link_.FillPoseArray(); };
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
  }
}

void TFPublisher::Base_Link::StartBaseLink() {
  // Initialize the odom->base_link transform at 0 0 0
  geometry_msgs::msg::TransformStamped tf_msg;
  x_ego_ = 0.f;
  y_ego_ = 0.f;
  theta_ego_ = 0.f;
  tf_msg.header.stamp = node_->get_clock()->now();
  tf_msg.header.frame_id = parent_frame_;
  tf_msg.child_frame_id = child_frame_;
  tf_msg.transform.translation.x = x_ego_;
  tf_msg.transform.translation.y = y_ego_;
  tf_msg.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_ego_);
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  // Publish the transform
  node_->tf_broadcaster_->sendTransform(tf_msg);
}

void TFPublisher::VelocityEstCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg) {
  base_link_.IntegrateAndSendTF(msg);
}

void TFPublisher::Base_Link::Integrate(float vx, float vy, float vtheta, float dt) {
  // Euler approximation
  x_ego_ = x_ego_ + cos(theta_ego_) * vx * dt - sin(theta_ego_) * vy * dt;
  y_ego_ = y_ego_ + sin(theta_ego_) * vx * dt + cos(theta_ego_) * vy * dt;
  theta_ego_ = theta_ego_ + vtheta * dt;

  // Constrain theta_ego_ from -pi to pi
  theta_ego_ = fmod(theta_ego_, 2 * M_PI);
  if (theta_ego_ > M_PI)
    theta_ego_ -= 2 * M_PI;
  else if (theta_ego_ < -M_PI)
    theta_ego_ += 2 * M_PI;
}

void TFPublisher::Base_Link::Integrate_RK4(float vx, float vy, float vtheta, float dt) {
  // Define the differential equations
  auto dx_dt = [this, &vy](float theta, float v) { return cos(theta) * v - sin(theta) * vy; };
  auto dy_dt = [this, &vx](float theta, float v) { return sin(theta) * vx + cos(theta) * v; };
  auto dtheta_dt = [](float vtheta) { return vtheta; };

  // RK4 integration for x
  float k1x = dt * dx_dt(theta_ego_, vx);
  float k2x = dt * dx_dt(theta_ego_ + 0.5 * dt * vtheta, vx + 0.5 * k1x);
  float k3x = dt * dx_dt(theta_ego_ + 0.5 * dt * vtheta, vx + 0.5 * k2x);
  float k4x = dt * dx_dt(theta_ego_ + dt * vtheta, vx + k3x);
  x_ego_ += (k1x + 2 * k2x + 2 * k3x + k4x) / 6;

  // RK4 integration for y
  float k1y = dt * dy_dt(theta_ego_, vy);
  float k2y = dt * dy_dt(theta_ego_ + 0.5 * dt * vtheta, vy + 0.5 * k1y);
  float k3y = dt * dy_dt(theta_ego_ + 0.5 * dt * vtheta, vy + 0.5 * k2y);
  float k4y = dt * dy_dt(theta_ego_ + dt * vtheta, vy + k3y);
  y_ego_ += (k1y + 2 * k2y + 2 * k3y + k4y) / 6;

  // RK4 integration for theta
  float k1theta = dt * dtheta_dt(vtheta);
  float k2theta = dt * dtheta_dt(vtheta + 0.5 * k1theta);
  float k3theta = dt * dtheta_dt(vtheta + 0.5 * k2theta);
  float k4theta = dt * dtheta_dt(vtheta + k3theta);
  theta_ego_ += (k1theta + 2 * k2theta + 2 * k3theta + k4theta) / 6;

  // Constrain theta_ego_ from -pi to pi
  theta_ego_ = fmod(theta_ego_, 2 * M_PI);
  if (theta_ego_ > M_PI)
    theta_ego_ -= 2 * M_PI;
  else if (theta_ego_ < -M_PI)
    theta_ego_ += 2 * M_PI;
}

void TFPublisher::Base_Link::Reset() {
  x_ego_ = 0.f;
  y_ego_ = 0.f;
  theta_ego_ = 0.f;
  time_last_integration_ = node_->get_clock()->now();
  is_first_integration_ = true;
}

void TFPublisher::Base_Link::IntegrateAndSendTF(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg) {
  if (is_first_integration_) {
    time_last_integration_ = msg->header.stamp;
    is_first_integration_ = false;

    return;
  } else {
    // Convert the message timestamp to rclcpp::Time
    rclcpp::Time msg_time(msg->header.stamp);

    // Now subtract the time_last_integration_ from msg_time
    rclcpp::Duration duration = msg_time - time_last_integration_;

    // Get the duration in seconds as a double
    double dt = duration.seconds();

    time_last_integration_ = msg->header.stamp;

    if (dt < kIntegrationTimeout) {
      // RCLCPP_INFO(node_->get_logger(), "Integrating velocity estimation msg");

      Integrate_RK4(msg->vel.x, msg->vel.y, msg->vel.theta, dt);

    } else {
      RCLCPP_WARN(node_->get_logger(), "Interruption of velocity estimation msg detected for: %f ", dt);
    }

    // Fill the transform message
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = parent_frame_;
    tf_msg.child_frame_id = child_frame_;
    tf_msg.transform.translation.x = x_ego_;
    tf_msg.transform.translation.y = y_ego_;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_ego_);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    if (strcmp(node_->mode_.c_str(), "debug") == 0) {
      // Publish the Pose message
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = msg->header.stamp;
      pose_msg.header.frame_id = parent_frame_;
      pose_msg.pose.position.x = x_ego_;
      pose_msg.pose.position.y = y_ego_;
      pose_msg.pose.position.z = 0.0;
      pose_msg.pose.orientation.x = q.x();
      pose_msg.pose.orientation.y = q.y();
      pose_msg.pose.orientation.z = q.z();
      pose_msg.pose.orientation.w = q.w();

      node_->pose_pub_->publish(pose_msg);
    }

    // RCLCPP_INFO(node_->get_logger(),
    //             "Transform data:\nHeader:\n  Stamp: %d.%d\n  Frame ID: "
    //             "%s\nChild Frame ID: %s\nTranslation:\n  x: %f\n  y: %f\n  z: "
    //             "%f\nRotation:\n  x: %f\n  y: %f\n  z: %f\n  w: %f",
    //             tf_msg.header.stamp.sec,
    //             tf_msg.header.stamp.nanosec,    // Timestamp components
    //             tf_msg.header.frame_id.c_str(), // Parent frame ID
    //             tf_msg.child_frame_id.c_str(),  // Child frame ID
    //             tf_msg.transform.translation.x, // Translation x
    //             tf_msg.transform.translation.y, // Translation y
    //             tf_msg.transform.translation.z, // Translation z
    //             tf_msg.transform.rotation.x,    // Rotation x
    //             tf_msg.transform.rotation.y,    // Rotation y
    //             tf_msg.transform.rotation.z,    // Rotation z
    //             tf_msg.transform.rotation.w);

    // Publish the transform
    node_->tf_broadcaster_->sendTransform(tf_msg);
  }
}
// Function for debugging purposes to plot the VE position
void TFPublisher::Base_Link::FillPoseArray() {
  // Fill a pose aray message for debugging
  pose_array_msg_.header.stamp = node_->get_clock()->now();
  pose_array_msg_.header.frame_id = parent_frame_;
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = x_ego_;
  pose_msg.position.y = y_ego_;
  pose_msg.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_ego_);
  pose_msg.orientation.x = q.x();
  pose_msg.orientation.y = q.y();
  pose_msg.orientation.z = q.z();
  pose_msg.orientation.w = q.w();
  pose_array_msg_.poses.push_back(pose_msg);
  pose_array_msg_.header.stamp = node_->get_clock()->now();
  pose_array_msg_.header.frame_id = parent_frame_;
  node_->pose_array_pub_->publish(pose_array_msg_);
}

} // namespace tf_publisher

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tf_publisher::TFPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
