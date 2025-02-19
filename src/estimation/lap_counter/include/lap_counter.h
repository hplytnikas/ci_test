/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2020-2024 Authors:
 *   - Trevor Phillips <tphillips@ethz.ch>
 *   - Stefan Weber <stefwebe@ethz.ch>
 *   - Yutian Han <yuthan@ethz.ch>
 *   - Jonas Wahlen <jwahlen@ethz.ch>
 *   - Alessandro Mercurio <amercurio@ethz.ch>
 *   - Giacomo Piatelli <gpiatelli@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <utility>
#include <vcu_msgs/msg/velocity_estimation.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

class LapCounter : public rclcpp::Node {
public:
  /*!
   * Constructor
   */
  LapCounter();

  // Method to get a transform between two frames
  geometry_msgs::msg::TransformStamped GetTransform(const std::string &target_frame, const std::string &source_frame);

  void LapCountCallback();

  //   /*!
  //  * True if the vehicle has moved since the last time
  //  * the lap counter was updated, and false otherwise
  //  */
  bool DidVehicleMove() const;

  // /*!
  //  * True if the vehicle's most recent pose is approximately
  //  * the same as the first pose when the vehicle started
  //  */
  bool IsCurrentPoseNearStart() const;

  // /*!
  //  * Updates the lap counter by providing it with a new pose
  //  */
  void Update(const geometry_msgs::msg::TransformStamped &pose);

  // /*!
  //  * Compute differnece between two quaternions
  //  */
  double QuaternionDifference(const geometry_msgs::msg::Quaternion &q1, const geometry_msgs::msg::Quaternion &q2) const;

  // /*!
  //  * Reset the parameters of the lap counter
  //  */
  void ResetParameters();

  // /*!
  //  * End the timer
  //  */
  void EndTimer();

  /// Callback for processing velocity estimation messages.
  /// @param msg Shared pointer to the received VelocityEstimation message.
  void VelocityEstCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg);

  // /Function to check if a line segment intersects a circle
  bool LineSegmentIntersectsCircle(double x1, double y1, double x2, double y2, double cx, double cy, double r);

  // Function to publish the circle marker
  void publishInitialMarker();

  // Callback for the timer to check if the vehicle has moved
  void PublishHasMoved();

private:
  bool did_stop_mission_;
  bool vehicle_was_near_start_;
  bool vehicle_was_near_center_;
  bool vehicle_was_near_finish_;
  bool start;
  bool has_vehicle_moved_;
  size_t lap_count_;
  geometry_msgs::msg::TransformStamped first_pose_;
  geometry_msgs::msg::TransformStamped prev_pose_;
  geometry_msgs::msg::TransformStamped current_pose_;
  std::vector<double> lap_distances_;
  double traveled_distance_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point end_time_;
  //! Lap counter publisher
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lap_counter_pub_;
  //! Lap time publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_pub_;
  //! Lap distance publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_distance_pub_;
  //! Subscriber to the vcu_msgs/velocty_estimation topic
  rclcpp::Subscription<vcu_msgs::msg::VelocityEstimation>::SharedPtr velocity_est_sub_;
  //! Publish a boolean message to see if the car has started moving
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_moving_pub_;
  //! Publish a marker to visualize the circle to intersect
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr circle_marker_pub_;
  //! Current lap starting time in milliseconds. 0 when mission not started yet
  uint64_t lap_start_time_;
  //! Counts iteration in which SLAM was skipped because of standstill_loc
  size_t iterations_count_;
  //! Offset value loaded from the parameter file to add as starting position
  double offset_;
  //! Raidus threshold to check if the vehicle is near the start
  double start_radius_;
  //! Minimum distance the vehicle can move before the lap counter is updated
  double moving_distance_condition_;
  //! Visualization of the circle to intersect
  visualization_msgs::msg::Marker circle_marker_;
  // Data structures to store the transform between frames
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_has_moved_;
  rclcpp::TimerBase::SharedPtr timer_marker_;
};
