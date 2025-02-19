/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Jonas Ohnemus <johnemus@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "monitoring_pipelines/monitoring_pipelines_node.hpp"

namespace monitoring_pipelines {

// Constructor for ModuleInfo
template <class MsgType>
ModuleInfo<MsgType>::ModuleInfo(rclcpp::Node::SharedPtr &node_handle, std::string name)
    : node_handle_(node_handle), first_msg_received_(false), is_ok_(false), module_name_(name), latest_msg_time_(0.0),
      module_status_{} {

  // Declare all parameters
  node_handle_->declare_parameter(module_name_ + ".topic_name", "");
  node_handle_->declare_parameter(module_name_ + ".expected_frequency", -1.0);

  // Get the output topic name, throw an error if it is not set
  if (!node_handle_->get_parameter(module_name_ + ".topic_name", output_topic_name_)) {
    RCLCPP_ERROR(this->node_handle_->get_logger(), "[%s] output topic name not set", module_name_.c_str());
  }

  // Get the expected frequency, throw a warning if it is not set
  if (!node_handle_->get_parameter(module_name_ + ".expected_frequency", topic_nominal_freq_)) {
    RCLCPP_WARN(this->node_handle_->get_logger(), "[%s] expected frequency not set", module_name_.c_str());
  }

  // Debugging output
  RCLCPP_INFO(this->node_handle_->get_logger(), "[%s] topic_name: %s", module_name_.c_str(),
              output_topic_name_.c_str());
  RCLCPP_INFO(this->node_handle_->get_logger(), "[%s] expected_frequency: %.2f", module_name_.c_str(),
              topic_nominal_freq_);

  // Initialize the subscription
  sub_ = node_handle_->create_subscription<MsgType>(
      output_topic_name_, 1, std::bind(&ModuleInfo<MsgType>::MsgCallback, this, std::placeholders::_1));

  // Initialize the time history
  msg_recv_time_history_.clear();

  // initialize module status
  module_status_.second_since_last_msg = -1.f;
  module_status_.status_flag = 3; // start in dormant state
  module_status_.status = "Dormant";

  // Print about finished initialization, topic name and expected frequency
  RCLCPP_INFO(this->node_handle_->get_logger(), "[%s] initialized with topic name: %s, expected frequency: %.2f",
              module_name_.c_str(), output_topic_name_.c_str(), topic_nominal_freq_);
}

template <class MsgType> void ModuleInfo<MsgType>::MsgCallback(const typename MsgType::SharedPtr msg) {
  if (!first_msg_received_) first_msg_received_ = true;

  latest_msg_time_ = this->node_handle_->get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(msg_recv_time_history_mutex_);
    msg_recv_time_history_.insert(latest_msg_time_.seconds());
  }
}

void ConeDetectionModuleInfo::MsgCallback(const autonomous_msgs::msg::ConeArray::SharedPtr msg) {
  if (!first_msg_received_) first_msg_received_ = true;
  latest_msg_time_ = this->node_handle_->get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(this->msg_recv_time_history_mutex_);
    this->msg_recv_time_history_.insert(this->latest_msg_time_.seconds());
  }
  cone_count_ = msg->cones.size();
}

template <class MsgType> void ModuleInfo<MsgType>::UpdateStatus() {
  double now = this->node_handle_->get_clock()->now().seconds();

  // handle non-published topic
  module_status_.second_since_last_msg = (latest_msg_time_.seconds() > 0) ? now - latest_msg_time_.seconds() : -1.0;

  // remove history beyond predefined window (10 seconds)
  {
    std::lock_guard<std::mutex> lock(msg_recv_time_history_mutex_);
    msg_recv_time_history_.erase(msg_recv_time_history_.begin(),
                                 msg_recv_time_history_.lower_bound(now - kMsgHistoryWindow));
    // the following line is to handle system clock jumping forward
    msg_recv_time_history_.erase(msg_recv_time_history_.upper_bound(now), msg_recv_time_history_.end());

    // get hz by counting the number of timestamps within the range of history
    auto it_1sec = msg_recv_time_history_.lower_bound(now - 1.0);
    auto it_5sec = msg_recv_time_history_.lower_bound(now - 5.0);
    auto it_10sec = msg_recv_time_history_.lower_bound(now - 10.0);

    module_status_.msg_hz_1s = std::distance(it_1sec, msg_recv_time_history_.end()) / 1.0;
    module_status_.msg_hz_5s = std::distance(it_5sec, msg_recv_time_history_.end()) / 5.0;
    module_status_.msg_hz_10s = std::distance(it_10sec, msg_recv_time_history_.end()) / 10.0;

    std::tie(module_status_.msg_dt_min_1s, module_status_.msg_dt_max_1s, module_status_.msg_dt_std_1s) =
        GetDtMinMaxStd(it_1sec);
    std::tie(module_status_.msg_dt_min_5s, module_status_.msg_dt_max_5s, module_status_.msg_dt_std_5s) =
        GetDtMinMaxStd(it_5sec);
    std::tie(module_status_.msg_dt_min_10s, module_status_.msg_dt_max_10s, module_status_.msg_dt_std_10s) =
        GetDtMinMaxStd(it_10sec);
  }

  if (!first_msg_received_) {
    module_status_.status_flag = 3;
    module_status_.status = "Dormant";
    is_ok_ = false;
  } else if (module_status_.msg_hz_1s > 0.8 * topic_nominal_freq_) {
    module_status_.status_flag = 0;
    module_status_.status = "Nominal";
    is_ok_ = true;
  } else if (module_status_.msg_hz_1s > 0) {
    module_status_.status_flag = 1;
    module_status_.status = "Compromised";
    is_ok_ = false;
  } else if (first_msg_received_) {
    module_status_.status_flag = 2;
    module_status_.status = "Dead";
    is_ok_ = false;
  }
}

template <class MsgType>
std::tuple<double, double, double> ModuleInfo<MsgType>::GetDtMinMaxStd(std::set<double>::iterator start) {
  std::vector<double> time_diff;
  time_diff.resize(std::distance(start, msg_recv_time_history_.end()));
  std::adjacent_difference(start, msg_recv_time_history_.end(), time_diff.begin());

  // since adjacent_difference returns the first element in the diff array
  if (!time_diff.empty()) time_diff.erase(time_diff.begin());
  auto minmax = std::minmax_element(time_diff.begin(), time_diff.end());
  double dt_min = minmax.first == time_diff.end() ? -1.0 : *minmax.first;
  double dt_max = minmax.second == time_diff.end() ? -1.0 : *minmax.second;

  // standard deviation
  double sum = std::accumulate(time_diff.begin(), time_diff.end(), 0.0);
  double mean = sum / time_diff.size();

  std::vector<double> diff(time_diff.size());
  std::transform(time_diff.begin(), time_diff.end(), diff.begin(), [mean](double x) { return x - mean; });
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double dt_std = std::sqrt(sq_sum / time_diff.size());
  return std::make_tuple(dt_min, dt_max, dt_std);
}

/**
 * @brief Constructor for the PipelineMonitor class
 */
PipelineMonitor::PipelineMonitor() {
  node_handle_ = rclcpp::Node::make_shared("monitoring_pipelines_node");

  RCLCPP_INFO(node_handle_->get_logger(), "Monitoring node is being initialized...");

  // Declare parameters with default values
  node_handle_->declare_parameter<double>("node_rate", 0.1);
  node_handle_->declare_parameter<std::string>("pipeline_status_topic_name", "/monitoring/pipeline_status_default");

  // Get parameters
  bool node_rate_set = node_handle_->get_parameter("node_rate", monitoring_node_rate_);
  bool pipeline_status_topic_name_set =
      node_handle_->get_parameter("pipeline_status_topic_name", pipeline_status_topic_name_);

  // Debugging output
  RCLCPP_INFO(node_handle_->get_logger(), "Node rate set: %s, value: %.2f", node_rate_set ? "true" : "false",
              monitoring_node_rate_);
  RCLCPP_INFO(node_handle_->get_logger(), "Pipeline status topic name set: %s, value: %s",
              pipeline_status_topic_name_set ? "true" : "false", pipeline_status_topic_name_.c_str());

  // Initialize the publisher
  pub_pipeline_status_ =
      node_handle_->create_publisher<monitoring_msgs::msg::PipelineStatus>(pipeline_status_topic_name_, 1);

  // Initialize the timer for monitoring
  auto timer_callback = std::bind(&PipelineMonitor::MonitoringCallback, this);
  auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / monitoring_node_rate_));
  monitoring_timer_ = node_handle_->create_wall_timer(timer_period, timer_callback);

  // Initialize the modules
  InitializeModules();
}

/**
 * @brief Run the monitoring node
 */
void PipelineMonitor::RunMonitoring() { rclcpp::spin(node_handle_); }

/**
 * @brief Get the node rate of the monitoring node
 * @return The node rate
 */
double PipelineMonitor::GetNodeRate() const { return monitoring_node_rate_; }

/**
 * @brief Initialize all modules for monitoring
 */
void PipelineMonitor::InitializeModules() {
  // SENSOR modules
  sensor_driver_camera_.reset(new ModuleInfo<sensor_msgs::msg::Image>(node_handle_, "sensor_driver_camera"));
  sensor_driver_lidar_.reset(new ModuleInfo<sensor_msgs::msg::PointCloud2>(node_handle_, "sensor_driver_lidar"));
  velocity_estimation_.reset(new ModuleInfo<vcu_msgs::msg::VelocityEstimation>(node_handle_, "velocity_estimation"));
  steering_feedback_.reset(new ModuleInfo<autonomous_msgs::msg::DoubleStamped>(node_handle_, "steering_feedback"));

  // PERCEPTION modules
  camera_pipeline_.reset(new ConeDetectionModuleInfo(node_handle_, "cone_camera"));
  lidar_pipeline_.reset(new ConeDetectionModuleInfo(node_handle_, "cone_lidar"));
  fusion_pipeline_.reset(new ConeDetectionModuleInfo(node_handle_, "cone_fusion"));
  per_pipeline_.reset(new ConeDetectionModuleInfo(node_handle_, "per_cones"));

  // ESTIMATION modules
  boundary_estimation_.reset(new ModuleInfo<autonomous_msgs::msg::Boundary>(node_handle_, "boundary_estimation"));
  online_map_.reset(new ConeDetectionModuleInfo(node_handle_, "online_map"));

  // CONTROL modules
  control_.reset(new ModuleInfo<vcu_msgs::msg::CarCommand>(node_handle_, "control"));
}

/**
 * @brief Callback function for the monitoring timer, which updates the status of all modules and sends the pipeline
 * status message
 */
void PipelineMonitor::MonitoringCallback() {
  // Update the status of all modules
  pipeline_status_.header.stamp = node_handle_->get_clock()->now();

  // SENSORS
  pipeline_status_.sensor_camera_alive = sensor_driver_camera_->IsOk();
  pipeline_status_.sensor_camera = sensor_driver_camera_->GetModuleStatus();

  pipeline_status_.sensor_lidar_alive = sensor_driver_lidar_->IsOk();
  pipeline_status_.sensor_lidar = sensor_driver_lidar_->GetModuleStatus();

  pipeline_status_.velocity_estimation_alive = velocity_estimation_->IsOk();
  pipeline_status_.velocity_estimation = velocity_estimation_->GetModuleStatus();

  pipeline_status_.steering_feedback_alive = steering_feedback_->IsOk();
  pipeline_status_.steering_feedback = steering_feedback_->GetModuleStatus();

  // PERCEPTION
  pipeline_status_.camera_pipeline_alive = camera_pipeline_->IsOk();
  pipeline_status_.camera_pipeline = camera_pipeline_->GetModuleStatus();
  pipeline_status_.num_cones_camera = camera_pipeline_->GetConeCount();

  pipeline_status_.lidar_pipeline_alive = lidar_pipeline_->IsOk();
  pipeline_status_.lidar_pipeline = lidar_pipeline_->GetModuleStatus();
  pipeline_status_.num_cones_lidar = lidar_pipeline_->GetConeCount();

  pipeline_status_.fusion_pipeline_alive = fusion_pipeline_->IsOk();
  pipeline_status_.fusion_pipeline = fusion_pipeline_->GetModuleStatus();
  pipeline_status_.num_cones_fusion = fusion_pipeline_->GetConeCount();

  pipeline_status_.per_pipeline_alive = per_pipeline_->IsOk();
  pipeline_status_.per_pipeline = per_pipeline_->GetModuleStatus();
  pipeline_status_.num_cones_per = per_pipeline_->GetConeCount();

  // ESTIMATION
  pipeline_status_.boundary_estimation_alive = boundary_estimation_->IsOk();
  pipeline_status_.boundary_estimation = boundary_estimation_->GetModuleStatus();

  pipeline_status_.online_map_alive = online_map_->IsOk();
  pipeline_status_.online_map = online_map_->GetModuleStatus();
  pipeline_status_.num_cones_map = online_map_->GetConeCount();

  // CONTROL
  pipeline_status_.control_alive = control_->IsOk();
  pipeline_status_.control = control_->GetModuleStatus();

  pub_pipeline_status_->publish(pipeline_status_);
}

} // namespace monitoring_pipelines
