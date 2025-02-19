/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - Matteo Mazzonelli <m.mazzonelli@gmail.com>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include "visualization_msgs/msg/marker_array.hpp"
#include <autonomous_msgs/msg/cone.hpp>
#include <autonomous_msgs/msg/cone_array.hpp>
#include <autonomous_msgs/msg/cone_type_prob.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <perception_msgs/msg/pipeline_runtime.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <easy/arbitrary_value.h>
#include <easy/profiler.h>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <vector>

namespace cone_fusion {

// shorthands for message_filters synchronization policy (we use approximate
// time policy) types
using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<autonomous_msgs::msg::ConeArray, autonomous_msgs::msg::ConeArray>;
using Sync = message_filters::Synchronizer<SyncPolicy>;

enum Pipeline { camera_only = 1, lidar_only = 2, sensor_fusion = 4 };

class ConeFusion : public rclcpp::Node {
public:
  ConeFusion();

private:
  /*
   * Struct for distance/FOV configs for a specific pipeline
   */
  struct ConePositionConfig {
    double min_cone_dist_from_car_m;
    double max_cone_dist_from_car_m;
    double field_of_view_rad;
  };

  std::shared_ptr<Sync> sync_;
  bool debug_;
  bool check_color_prob_;
  float color_prob_thresh_;
  float max_cone_error_;
  float min_distance_for_neighboring_cone_m_;
  ConePositionConfig fused_config_;
  ConePositionConfig lidar_only_config_;
  ConePositionConfig camera_only_config_;
  int max_set_size_;
  float wait_threshold_;

  std::set<rclcpp::Time> timestamp_set;

  rclcpp::Time last_timestamp;

  Pipeline active_pipeline_;

  // Topic Names
  std::string topic_runtime_;
  std::string topic_camera_cone_array_;
  std::string topic_lidar_cone_array_;
  std::string topic_sf_cone_array_;
  std::string topic_output_cone_array_;

  // Subscribers
  rclcpp::Subscription<autonomous_msgs::msg::ConeArray>::SharedPtr sub_camera_cone_array_;
  rclcpp::Subscription<autonomous_msgs::msg::ConeArray>::SharedPtr sub_lidar_cone_array_;
  message_filters::Subscriber<autonomous_msgs::msg::ConeArray> sub_lidar_cone_array2_;
  message_filters::Subscriber<autonomous_msgs::msg::ConeArray> sub_sf_cone_array_;

  // Publishers
  rclcpp::Publisher<perception_msgs::msg::PipelineRuntime>::SharedPtr pub_runtime_;
  rclcpp::Publisher<autonomous_msgs::msg::ConeArray>::SharedPtr pub_output_cone_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cone_markers_;

  /**
   * Loads ROS parameters
   */
  void LoadParameters();

  /**
   * Creates subscribers for the topics that the node needs to listen to.
   */
  void SubscribeTopics();

  /**
   * Creates publishers for the topics that the node needs to publish.
   */
  void AdvertiseTopics();

  /**
   * Callback function for the camera cone array subscriber
   * @param camera_cone_msg The message containing the cones detected by the
   * camera.
   */
  void PassCameraCones(const autonomous_msgs::msg::ConeArray::SharedPtr camera_cone_msg);

  /**
   * Callback function for the LiDAR cone array subscriber
   * @param lidar_cones The message containing the cones detected by the LiDAR.
   */
  void PassLiDARCones(const autonomous_msgs::msg::ConeArray::SharedPtr lidar_cones);

  /**
   * Concatenates the cones detected by the camera and the LiDAR.
   * @param sf_cone_msg The cones detected by the camera.
   * @param lidar_cone_msg The cones detected by the LiDAR.
   */
  void ConcatenateLiDARtoFusion(const autonomous_msgs::msg::ConeArray::SharedPtr sf_cone_msg,
                                const autonomous_msgs::msg::ConeArray::SharedPtr lidar_cone_msg);

  /**
   * Check if the lidar cone is already present in the sensor fusion cones
   * @param sf_cones The cones detected by the sensor fusion.
   * @param lidar_cone The cone detected by the LiDAR.
   * @return True if the cone is new, false otherwise.
   */
  bool IsNewCone(const std::vector<autonomous_msgs::msg::Cone> &sf_cones, const autonomous_msgs::msg::Cone &lidar_cone);

  /**
   * Adds the cones that are not overlapping with the cones already present.
   * @param input_cones The cones to be added.
   * @param all_cones The cones already present.
   */
  void AddNonOverlappingCones(const std::vector<autonomous_msgs::msg::Cone> &input_cones,
                              std::vector<autonomous_msgs::msg::Cone> &all_cones);

  /**
   * Checks the color probability of the cones and keeps the probability if it is higher than a threshold.
   * If not, it sets the probability of every cone to 0.25
   * @param cones The cones to be published.
   */

  void CommonPostProcessor(const autonomous_msgs::msg::ConeArray &all_cones);

  void PostprocessObservations(const autonomous_msgs::msg::ConeArray &observations,
                               autonomous_msgs::msg::ConeArray &output);

  void PublishOnlyConfidentColors(std::vector<autonomous_msgs::msg::Cone> &cones);

  bool CheckConePositionPlausibility(const autonomous_msgs::msg::Cone &observation) const;

  bool CheckHasNeighborInRange(const autonomous_msgs::msg::Cone &observation,
                               const autonomous_msgs::msg::ConeArray &observations) const;

  /**
   * Publishes the cone markers for the visualization of the cones in RViz.
   * @param cone_array The array of cones to be visualized.
   */
  void PublishConeMarkers(const autonomous_msgs::msg::ConeArray &cone_array);

  /**
   * Publishes the pipeline runtime.
   * @param start_time Timestamp when the image/PCL was received.
   * @param end_time The time when the perception pipeline ended.
   */
  void PublishRuntime(const rclcpp::Time &start_time, const rclcpp::Time &end_time);

  bool IsCurrentPipeline(const rclcpp::Time &timestamp, Pipeline pipeline);

  /**
   * Resizes the set of timestamps keep the memory usage low.
   */
  void ResizeSet();
};

} // namespace cone_fusion
