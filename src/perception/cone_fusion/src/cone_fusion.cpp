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

#include "cone_fusion.hpp"

namespace cone_fusion {
/**
 * Constructor of ConeFusion. Extracts topic names from config, subscribes to
 * topics, and registers callback function
 */
ConeFusion::ConeFusion() : Node("cone_fusion") {
  LoadParameters();
  SubscribeTopics();
  AdvertiseTopics();
}

void ConeFusion::LoadParameters() {
  // Parameters
  this->declare_parameter<bool>("debug", true);
  this->declare_parameter<bool>("check_color_prob", true);
  this->declare_parameter<float>("color_prob_thresh", 0.5);
  this->declare_parameter<float>("max_cone_error", 0.5);
  this->declare_parameter<int>("max_set_size", 10);
  this->declare_parameter<float>("wait_threshold", 200.0);

  // Topic Names
  this->declare_parameter<std::string>("topics.sub.camera_cone", "/perception/camera/cone_array");
  this->declare_parameter<std::string>("topics.sub.lidar_cone", "/perception/lidar/cone_array");
  this->declare_parameter<std::string>("topics.sub.sf_cone", "/perception/fusion/cone_array");
  this->declare_parameter<std::string>("topics.pub.output_cone", "/perception/cone_array");
  this->declare_parameter<std::string>("topics.pub.runtime", "/perception/runtimes");

  // Post processing
  this->declare_parameter<float>("post_processing.min_distance_for_neighboring_cone_m", 6.5);

  this->declare_parameter<float>("post_processing.fused.min_cone_dist_from_car_m", 2.5);
  this->declare_parameter<float>("post_processing.fused.max_cone_dist_from_car_m", 20.0);
  this->declare_parameter<float>("post_processing.fused.field_of_view_deg", 120.0);

  this->declare_parameter<float>("post_processing.lidar_only.min_cone_dist_from_car_m", 2.5);
  this->declare_parameter<float>("post_processing.lidar_only.max_cone_dist_from_car_m", 20.0);
  this->declare_parameter<float>("post_processing.lidar_only.field_of_view_deg", 180.0);

  this->declare_parameter<float>("post_processing.camera_only.min_cone_dist_from_car_m", 2.5);
  this->declare_parameter<float>("post_processing.camera_only.max_cone_dist_from_car_m", 10.0);
  this->declare_parameter<float>("post_processing.camera_only.field_of_view_deg", 120.0);

  this->declare_parameter<std::string>("perception_mode", "sensor_fusion");

  // Parameters
  this->get_parameter("debug", debug_);
  this->get_parameter("check_color_prob", check_color_prob_);
  this->get_parameter("color_prob_thresh", color_prob_thresh_);
  this->get_parameter("max_cone_error", max_cone_error_);
  this->get_parameter("max_set_size", max_set_size_);
  this->get_parameter("wait_threshold", wait_threshold_);

  // Topic Names
  this->get_parameter("topics.sub.camera_cone", topic_camera_cone_array_);
  this->get_parameter("topics.sub.lidar_cone", topic_lidar_cone_array_);
  this->get_parameter("topics.sub.sf_cone", topic_sf_cone_array_);
  this->get_parameter("topics.pub.output_cone", topic_output_cone_array_);
  this->get_parameter("topics.pub.runtime", topic_runtime_);

  // Post processing
  this->get_parameter("post_processing.min_distance_for_neighboring_cone_m", min_distance_for_neighboring_cone_m_);
  this->get_parameter("post_processing.fused.min_cone_dist_from_car_m", fused_config_.min_cone_dist_from_car_m);
  this->get_parameter("post_processing.fused.max_cone_dist_from_car_m", fused_config_.max_cone_dist_from_car_m);
  this->get_parameter("post_processing.fused.field_of_view_deg", fused_config_.field_of_view_rad);
  fused_config_.field_of_view_rad = fused_config_.field_of_view_rad * M_PI / 180.0;

  this->get_parameter("post_processing.lidar_only.min_cone_dist_from_car_m",
                      lidar_only_config_.min_cone_dist_from_car_m);
  this->get_parameter("post_processing.lidar_only.max_cone_dist_from_car_m",
                      lidar_only_config_.max_cone_dist_from_car_m);
  this->get_parameter("post_processing.lidar_only.field_of_view_deg", lidar_only_config_.field_of_view_rad);
  lidar_only_config_.field_of_view_rad = lidar_only_config_.field_of_view_rad * M_PI / 180.0;

  this->get_parameter("post_processing.camera_only.min_cone_dist_from_car_m",
                      camera_only_config_.min_cone_dist_from_car_m);
  this->get_parameter("post_processing.camera_only.max_cone_dist_from_car_m",
                      camera_only_config_.max_cone_dist_from_car_m);
  this->get_parameter("post_processing.camera_only.field_of_view_deg", camera_only_config_.field_of_view_rad);
  camera_only_config_.field_of_view_rad = camera_only_config_.field_of_view_rad * M_PI / 180.0;

  std::string perception_mode = this->get_parameter("perception_mode").as_string();

  if (perception_mode == "camera_only")
    active_pipeline_ = camera_only;
  else if (perception_mode == "lidar_only")
    active_pipeline_ = lidar_only;
  else if (perception_mode == "sensor_fusion")
    active_pipeline_ = sensor_fusion;

  last_timestamp = this->get_clock()->now();
}

void ConeFusion::SubscribeTopics() {
  sub_camera_cone_array_ = this->create_subscription<autonomous_msgs::msg::ConeArray>(
      topic_camera_cone_array_, 10, std::bind(&ConeFusion::PassCameraCones, this, std::placeholders::_1));
  sub_lidar_cone_array_ = this->create_subscription<autonomous_msgs::msg::ConeArray>(
      topic_lidar_cone_array_, 10, std::bind(&ConeFusion::PassLiDARCones, this, std::placeholders::_1));

  sub_lidar_cone_array2_.subscribe(this, topic_lidar_cone_array_);
  sub_sf_cone_array_.subscribe(this, topic_sf_cone_array_);
  sync_.reset(new Sync(SyncPolicy(20), sub_sf_cone_array_, sub_lidar_cone_array2_));
  sync_->registerCallback(&ConeFusion::ConcatenateLiDARtoFusion, this);
}

void ConeFusion::AdvertiseTopics() {
  pub_runtime_ = this->create_publisher<perception_msgs::msg::PipelineRuntime>(topic_runtime_, 1);
  pub_output_cone_array_ = this->create_publisher<autonomous_msgs::msg::ConeArray>(topic_output_cone_array_, 1);
  pub_cone_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/cone_markers", 1);
}

void ConeFusion::PassCameraCones(const autonomous_msgs::msg::ConeArray::SharedPtr camera_cone_msg) {
  if (active_pipeline_ == camera_only) {
    EASY_FUNCTION(profiler::colors::Blue);
    rclcpp::Time sensor_time = camera_cone_msg->header.stamp;
    rclcpp::Time end_time = this->get_clock()->now();
    EASY_VALUE("Latency", (end_time - sensor_time).nanoseconds() / 1e6);
    autonomous_msgs::msg::ConeArray all_cones = *camera_cone_msg;

    if (check_color_prob_) PublishOnlyConfidentColors(all_cones.cones);

    CommonPostProcessor(all_cones);
  }
}

void ConeFusion::PassLiDARCones(const autonomous_msgs::msg::ConeArray::SharedPtr lidar_cones) {
  rclcpp::Time sensor_time = lidar_cones->header.stamp;
  rclcpp::Time end_time = this->get_clock()->now();
  if (IsCurrentPipeline(end_time, lidar_only)) {
    EASY_FUNCTION(profiler::colors::Black);
    EASY_VALUE("Latency", (end_time - sensor_time).nanoseconds() / 1e6);
    CommonPostProcessor(*lidar_cones);

    if (active_pipeline_ == lidar_only) last_timestamp = this->get_clock()->now();
    timestamp_set.insert(sensor_time);
    ResizeSet();
  }
}

void ConeFusion::ConcatenateLiDARtoFusion(const autonomous_msgs::msg::ConeArray::SharedPtr sf_cone_msg,
                                          const autonomous_msgs::msg::ConeArray::SharedPtr lidar_cone_msg) {
  rclcpp::Time sf_time = sf_cone_msg->header.stamp;
  rclcpp::Time end_time = this->get_clock()->now();
  if (IsCurrentPipeline(end_time, sensor_fusion)) {
    EASY_FUNCTION(profiler::colors::Red);
    EASY_VALUE("Latency sf", (end_time - sf_time).nanoseconds() / 1e6);

    std::vector<autonomous_msgs::msg::Cone> sf_cones = sf_cone_msg->cones;

    autonomous_msgs::msg::ConeArray filtered_sf_cones;

    AddNonOverlappingCones(sf_cone_msg->cones, filtered_sf_cones.cones);
    if (check_color_prob_) {
      PublishOnlyConfidentColors(filtered_sf_cones.cones);
    }

    autonomous_msgs::msg::ConeArray output_cones;
    output_cones.header = sf_cone_msg->header;

    autonomous_msgs::msg::ConeArray filtered_lidar_cones;

    PostprocessObservations(filtered_sf_cones, output_cones);
    PostprocessObservations(*lidar_cone_msg, filtered_lidar_cones);

    for (const auto &cone : filtered_lidar_cones.cones) {
      if (IsNewCone(output_cones.cones, cone)) {
        output_cones.cones.push_back(cone);
      }
    }
    pub_output_cone_array_->publish(output_cones);

    PublishConeMarkers(output_cones);
    PublishRuntime(sf_cone_msg->header.stamp, this->get_clock()->now());

    last_timestamp = this->get_clock()->now();
    timestamp_set.insert(sf_time);
    ResizeSet();
  }
}

void ConeFusion::CommonPostProcessor(const autonomous_msgs::msg::ConeArray &all_cones) {
  autonomous_msgs::msg::ConeArray output;
  output.header = all_cones.header;
  PostprocessObservations(all_cones, output);

  pub_output_cone_array_->publish(output);

  PublishConeMarkers(output);
  PublishRuntime(output.header.stamp, this->get_clock()->now());
}

bool ConeFusion::IsNewCone(const std::vector<autonomous_msgs::msg::Cone> &sf_cones,
                           const autonomous_msgs::msg::Cone &lidar_cone) {
  for (const auto &sf_cone : sf_cones) {
    float dist =
        sqrt(pow(sf_cone.position.x - lidar_cone.position.x, 2) + pow(sf_cone.position.y - lidar_cone.position.y, 2));
    if (dist < max_cone_error_) {
      return false;
    }
  }
  return true;
}

void ConeFusion::AddNonOverlappingCones(const std::vector<autonomous_msgs::msg::Cone> &input_cones,
                                        std::vector<autonomous_msgs::msg::Cone> &all_cones) {
  EASY_FUNCTION(profiler::colors::Green);
  for (const auto &cone1 : input_cones) {
    bool new_cone = true;
    for (auto &cone2 : all_cones) {
      float dist = sqrt(pow(cone1.position.x - cone2.position.x, 2) + pow(cone1.position.y - cone2.position.y, 2));
      if (dist < max_cone_error_) {
        new_cone = false;

        std::vector<float> probs = {cone1.prob_type.blue, cone1.prob_type.yellow, cone1.prob_type.orange,
                                    cone1.prob_type.orange_big};
        auto color1 = std::max_element(probs.begin(), probs.end()) - probs.begin();
        probs = {cone2.prob_type.blue, cone2.prob_type.yellow, cone2.prob_type.orange, cone2.prob_type.orange_big};
        auto color2 = std::max_element(probs.begin(), probs.end()) - probs.begin();

        if (color1 != color2) {
          cone2.prob_type.blue = cone2.prob_type.yellow = cone2.prob_type.orange = cone2.prob_type.orange_big = 0.25;
          cone2.pipeline = lidar_only;
        }
        continue;
      }
    }
    if (new_cone) {
      all_cones.push_back(cone1);
    }
  }
}

void ConeFusion::PublishOnlyConfidentColors(std::vector<autonomous_msgs::msg::Cone> &cones) {
  for (auto &cone : cones) {
    if (cone.prob_type.blue > color_prob_thresh_ || cone.prob_type.yellow > color_prob_thresh_ ||
        cone.prob_type.orange > color_prob_thresh_ || cone.prob_type.orange_big > color_prob_thresh_) {
      continue;
    } else {
      cone.prob_type.blue = cone.prob_type.yellow = cone.prob_type.orange = cone.prob_type.orange_big = 0.25;
      cone.pipeline = lidar_only;
    }
  }
}

void ConeFusion::PostprocessObservations(const autonomous_msgs::msg::ConeArray &observations,
                                         autonomous_msgs::msg::ConeArray &output) {
  for (auto &observation : observations.cones) {
    // Skip rejected cones
    if (!CheckConePositionPlausibility(observation)) {
      continue;
    }

    if (!CheckHasNeighborInRange(observation, observations)) {
      continue;
    }

    output.cones.push_back(observation);
  }
}

bool ConeFusion::CheckConePositionPlausibility(const autonomous_msgs::msg::Cone &observation) const {
  if (std::isnan(observation.position.x) || std::isnan(observation.position.y)) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Check cone position plausability: NaN cone position received.");
    return false;
  }

  // Set configuration based on pipeline
  const ConePositionConfig *cone_position_config = nullptr;
  if (observation.pipeline == sensor_fusion) {
    cone_position_config = &fused_config_;
  } else if (observation.pipeline == lidar_only) {
    cone_position_config = &lidar_only_config_;
  } else /*camera_only*/ {
    cone_position_config = &camera_only_config_;
  }

  // Check if the pointer is not null before using it
  if (!cone_position_config) {
    RCLCPP_ERROR(this->get_logger(), "Check cone position plausability: Null configuration pointer");
    return false;
  }

  double range = std::hypot(observation.position.x, observation.position.y);
  if (range < cone_position_config->min_cone_dist_from_car_m) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Check cone position plausability: Cone too close detected at "
                                                << std::setprecision(2) << range << "m.");
    return false;
  }

  if (range > cone_position_config->max_cone_dist_from_car_m) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Check cone position plausability: Cone out of range detected at "
                                                << std::setprecision(2) << range << "m.");
    return false;
  }

  double bearing = std::atan2(observation.position.y, observation.position.x);
  if (abs(bearing) > cone_position_config->field_of_view_rad * 0.5) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Check cone position plausability:: FOV at a bearing of "
                                                << std::setprecision(2) << (bearing * (180.0 / M_PI))
                                                << " deg detected.");
    return false;
  }

  return true;
}

bool ConeFusion::CheckHasNeighborInRange(const autonomous_msgs::msg::Cone &observation,
                                         const autonomous_msgs::msg::ConeArray &observations) const {
  for (const auto &other : observations.cones) {
    double dist = std::hypot(observation.position.x - other.position.x, observation.position.y - other.position.y);
    if (dist > 0.1 && dist < min_distance_for_neighboring_cone_m_) {
      return true;
    }
  }

  // No neighbor found within specified range
  return false;
}

void ConeFusion::PublishConeMarkers(const autonomous_msgs::msg::ConeArray &cone_array) {
  visualization_msgs::msg::MarkerArray markerArray;
  int markerId = 0;
  int max_markers = 50;

  // Remove markers from previous callback
  for (int i = 0; i < max_markers; ++i) { // assuming you won't have more than 50 markers
    visualization_msgs::msg::Marker deleteMarker;
    deleteMarker.header.frame_id = cone_array.header.frame_id;
    deleteMarker.header.stamp = cone_array.header.stamp;
    deleteMarker.ns = "cones";
    deleteMarker.id = i;
    deleteMarker.action = visualization_msgs::msg::Marker::DELETE;
    markerArray.markers.push_back(deleteMarker);
  }

  for (const auto &cone : cone_array.cones) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = cone_array.header.frame_id;
    marker.header.stamp = cone_array.header.stamp;
    marker.ns = "cones";
    marker.id = markerId++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cone.position.x;
    marker.pose.position.y = cone.position.y;
    marker.pose.position.z = 0.01; // Slightly above ground for visibility
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.4; // Specify the size of the marker
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    if (cone.pipeline == 2) {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    } else if (cone.prob_type.blue > cone.prob_type.yellow && cone.prob_type.blue > cone.prob_type.orange &&
               cone.prob_type.blue > cone.prob_type.orange_big) {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    } else if (cone.prob_type.yellow > cone.prob_type.blue && cone.prob_type.yellow > cone.prob_type.orange &&
               cone.prob_type.yellow > cone.prob_type.orange_big) {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else if (cone.prob_type.orange > cone.prob_type.blue && cone.prob_type.orange > cone.prob_type.yellow &&
               cone.prob_type.orange > cone.prob_type.orange_big) {
      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
    } else if (cone.prob_type.orange_big > cone.prob_type.blue && cone.prob_type.orange_big > cone.prob_type.yellow &&
               cone.prob_type.orange_big > cone.prob_type.orange) {
      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 1.0;
      marker.color.g = 0.3;
      marker.color.b = 0.0;
    }

    marker.color.a = 1.0; // Make the marker opaque
    markerArray.markers.push_back(marker);
  }

  pub_cone_markers_->publish(markerArray);
}

void ConeFusion::PublishRuntime(const rclcpp::Time &start_time, const rclcpp::Time &end_time) {
  perception_msgs::msg::PipelineRuntime msg;
  switch (active_pipeline_) {
  case lidar_only:
    msg.node = "perception_pipeline/lidar_only";
    break;
  case camera_only:
    msg.node = "perception_pipeline/camera_only";
    break;
  case sensor_fusion:
    msg.node = "perception_pipeline/sensor_fusion";
    break;

  default:
    break;
  }
  msg.runtime = (end_time - start_time).nanoseconds() / 1e6; // publish runtime in milliseconds
  pub_runtime_->publish(msg);
}

bool ConeFusion::IsCurrentPipeline(const rclcpp::Time &timestamp, Pipeline pipeline) {
  // If the pipeline is the active one, we always publish if the timestamp is new
  if (pipeline == active_pipeline_ && timestamp_set.find(timestamp) == timestamp_set.end()) return true;

  // If the pipeline is not the active one, we publish if the timestamp is new and the wait threshold is passed, it
  // means the current pipeline is not publishing
  if (pipeline != active_pipeline_ && timestamp_set.find(timestamp) == timestamp_set.end() &&
      (timestamp - last_timestamp).nanoseconds() / 1e6 > wait_threshold_)
    return true;

  return false;
}

void ConeFusion::ResizeSet() {
  if (timestamp_set.size() > max_set_size_) timestamp_set.erase(timestamp_set.begin());
}

} // namespace cone_fusion
