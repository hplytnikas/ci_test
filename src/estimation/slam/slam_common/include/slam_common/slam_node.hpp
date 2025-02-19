/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Christoforos Nicolaou <cnicolaou@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#pragma once

#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "autonomous_msgs/msg/bool_stamped.hpp"
#include "autonomous_msgs/msg/cone_array.hpp"
#include "slam_common/stamped_pose.hpp"
#include "slam_common/utils.hpp"

namespace slam {

/*
 * The main node that will be running SLAM.
 */
class SlamNode : public rclcpp::Node {
public:
  /*
   * Constructor
   */
  SlamNode(const std::string &node_name, const std::string &node_namespace, const rclcpp::NodeOptions &options);
  /*
   * Destructor.
   */
  ~SlamNode();

  // Create subscribers
  void CreateConeCallbackSubscriber(std::function<void(const autonomous_msgs::msg::ConeArray)> callback);

  // Localization
  // Broadcast the initial map->odom transformation
  void BroadcastInitialPose(const rclcpp::Time timestamp);

  // Broadcast the map->odom transformation to correct the drift from raw VE
  void BroadcastEstimatedSlamPose(const StampedPose &slam_pose, const StampedPose &odom_to_base_link);

  // Publisher methods
  // Publishes the map produced by SlamFrontend.
  void PublishOnlineMap(const autonomous_msgs::msg::ConeArray &online_map) const;
  // Publishes if the acceleration map has been generated
  void PublishAccelerationMapStatus(bool acceleration_map_generated) const;

  // Visualization publishing
  void PublishGlobalMapVisualization(const visualization_msgs::msg::MarkerArray &global_map_marker) const;
  void PublishPosesVisualization(const geometry_msgs::msg::PoseArray &pose_array) const;
  void PublishRawGraphVisualization(const visualization_msgs::msg::MarkerArray &raw_graph_marker) const;
  void PublishAssociationsVisualization(const visualization_msgs::msg::MarkerArray &association_marker) const;

  // Return lap count
  inline int LapCount() const { return lap_count_; }
  // Return true if vehicle is moving
  inline bool IsVehicleMoving() const { return is_vehicle_moving_; }
  // Return true if mission has finished
  inline bool HasMissionFinished() const { return has_mission_finished_; }

  /*!
   * Query ROS TF to find the odometry which is closest in time to the given
   * `timestamp`. The frame must be specified.
   * @return A tuple, the first element is status boolean (true if successful)
   *         and the second is a StampedPose in the global frame
   */
  std::pair<bool, StampedPose> FetchTF(const rclcpp::Time &timestamp, const std::string &from_frame,
                                       const std::string &to_frame) const;

  // Parameters
  // Template function for safe parameter retrieval
  // Needs to be defined here since it is a template
  template <typename Type> Type GetParameter(const std::string &name, Type default_value) {
    Type value;
    std::stringstream ss;
    PrintValue(ss, default_value); // Use the helper function for printing
    if (!this->get_parameter(name, value)) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Parameter not found: " << name << ". Initializing with default value: " << ss.str() << ".");
      value = default_value;
    }

    return value;
  }

private:
  // Initializes Subscribers (except cone callback)
  void InitSubscribers();
  // Initializes all Publishers
  void InitPublishers();

  // Called when msg for lap count from lap counter received
  void LapCounterCallback(const std_msgs::msg::Int32::SharedPtr msg);

  // Called when msg for vehicle moving from lap counter received
  void VehicleMovingCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Called when mission finished
  void MissionFinishedCallback(const autonomous_msgs::msg::BoolStamped::SharedPtr msg);

  // Subscribers
  rclcpp::Subscription<autonomous_msgs::msg::ConeArray>::SharedPtr cone_callback_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lap_counter_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vehicle_moving_subscriber_;
  rclcpp::Subscription<autonomous_msgs::msg::BoolStamped>::SharedPtr mission_finished_subscriber_;

  // Publishers
  rclcpp::Publisher<autonomous_msgs::msg::ConeArray>::SharedPtr online_map_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr acceleration_map_status_publisher_;
  // Visualization publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_map_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr raw_graph_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr association_publisher_;

  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Lap count
  int lap_count_;
  // Indication for if vehicle is moving
  bool is_vehicle_moving_;
  // Indication for if mission has finished
  bool has_mission_finished_;

  // For parameter default printing
  // Generic print helper function
  template <typename T> std::ostream &PrintValue(std::ostream &os, const T &value) { return os << value; }
  // Specialization of the print function for std::vector
  template <typename T> std::ostream &PrintValue(std::ostream &os, const std::vector<T> &values) {
    os << "{";
    for (size_t i = 0; i < values.size(); ++i) {
      if (i != 0) os << ", ";
      os << values[i];
    }
    return os << "}";
  }
};

} // end namespace slam
