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

#pragma once

#include "vcu_msgs/msg/velocity_estimation.hpp"
#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>
#include <string> // NOLINT
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace tf_publisher {

class TFPublisher : public rclcpp::Node {
public:
  /// Constructor for the TFPublisher node.
  TFPublisher();

private:
  // Subscriptions to the VelocityEstimation and MissionSelect topics.
  rclcpp::Subscription<vcu_msgs::msg::VelocityEstimation>::SharedPtr velocity_est_sub_;

  // Publish pose for debug
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // Publish pose array for debug
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

  /// Callback for processing velocity estimation messages.
  /// @param msg Shared pointer to the received VelocityEstimation message.
  void VelocityEstCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg);

  class Base_Link {
  public:
    /// Constructs a Base_Link object associated with the given node and frames.
    /// @param node Pointer to the parent TFPublisher node.
    /// @param parent_frame The TF frame ID of the parent frame.
    /// @param child_frame The TF frame ID of the child frame.
    explicit Base_Link(TFPublisher *node, const std::string &parent_frame, const std::string &child_frame)
        : parent_frame_(parent_frame), child_frame_(child_frame), node_(node) {
      Reset();
    }

    /// Integrate velocity estimation and send out the transform.
    /// @param msg Shared pointer to the received VelocityEstimation message.
    void IntegrateAndSendTF(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg);

    /// Take pose and fill the pose array for debug purpose
    void FillPoseArray();

    /// Reset the base link state to initial values.
    void Reset();

    /// Initialize and start the base link broadcasting.
    void StartBaseLink();

  protected:
    /// Integrate the velocity using a simple integration method.
    /// @param vx Velocity in x-direction.
    /// @param vy Velocity in y-direction.
    /// @param vtheta Angular velocity around z-axis.
    /// @param dt Time step for the integration.
    void Integrate(float vx, float vy, float vtheta, float dt);

    /// Integrate the velocity using the Runge-Kutta 4th order method.
    /// @param vx Velocity in x-direction.
    /// @param vy Velocity in y-direction.
    /// @param vtheta Angular velocity around z-axis.
    /// @param dt Time step for the integration.
    void Integrate_RK4(float vx, float vy, float vtheta, float dt);

    // The state variables for position and orientation of the base link.
    double x_ego_;
    double y_ego_;
    double theta_ego_;

    // The time of the last integration step.
    rclcpp::Time time_last_integration_;

    // Flag indicating if this is the first integration step.
    bool is_first_integration_;

    // The TF frame IDs of the parent and child frames.
    std::string parent_frame_;
    std::string child_frame_;

    // Pointer back to the upper class
    TFPublisher *node_;

    // Pose array for debug
    geometry_msgs::msg::PoseArray pose_array_msg_;
  };

  // Unique pointer to a TransformBroadcaster for broadcasting TFs.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Mode parameter
  std::string mode_;

  // Timer for debug mode to publish pose array
  rclcpp::TimerBase::SharedPtr timer_;

  Base_Link base_link_;
};

} // namespace tf_publisher
