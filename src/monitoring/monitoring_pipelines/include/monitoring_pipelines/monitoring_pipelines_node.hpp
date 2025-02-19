/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2019-2023 Authors:
 *   - Jonas Ohnemus <johnemus@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

// Estimation messages
#include <autonomous_msgs/msg/boundary.hpp>
#include <autonomous_msgs/msg/double_stamped.hpp>
#include <vcu_msgs/msg/velocity_estimation.hpp>

// Perception messages
#include <autonomous_msgs/msg/cone_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Control messages
#include <vcu_msgs/msg/car_command.hpp>
// #include <control_msgs/msg/controller_ref.hpp>

// Monitoring messages
#include <monitoring_msgs/msg/module_status.hpp>
#include <monitoring_msgs/msg/pipeline_status.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include <chrono> // NOLINT(build/c++11)
#include <memory>
#include <mutex> // NOLINT(build/c++11)
#include <set>
#include <string>
#include <thread> // NOLINT(build/c++11)
#include <tuple>

namespace monitoring_pipelines {

// Keep a history of 10s of timestamp of monitored messages
const double kMsgHistoryWindow = 10.0;

/**
 * @brief Info Class for pipeline modules that publish something at an expected frequency
 */
template <class MsgType> class ModuleInfo {
public:
  ModuleInfo(rclcpp::Node::SharedPtr &node_handle, std::string name);

  virtual void MsgCallback(const typename MsgType::SharedPtr msg);
  monitoring_msgs::msg::ModuleStatus GetModuleStatus() {
    UpdateStatus();
    return module_status_;
  }
  bool IsOk() { return is_ok_; }

protected:
  rclcpp::Time latest_msg_time_;
  bool first_msg_received_;
  std::set<double> msg_recv_time_history_;

  rclcpp::Node::SharedPtr node_handle_;
  std::string module_name_;
  std::mutex msg_recv_time_history_mutex_;

private:
  void UpdateStatus();
  std::tuple<double, double, double> GetDtMinMaxStd(std::set<double>::iterator start);

  // flag to indicate if the module is ok
  bool is_ok_;

  // topic name, subscriber, nominal frequency
  std::string output_topic_name_;
  typename rclcpp::Subscription<MsgType>::SharedPtr sub_;
  float topic_nominal_freq_;

  // Status message
  monitoring_msgs::msg::ModuleStatus module_status_;
};

/**
 * @brief Cone Detection specific information (allows to count cones in ConeArray)
 */
class ConeDetectionModuleInfo : public ModuleInfo<autonomous_msgs::msg::ConeArray> {
public:
  explicit ConeDetectionModuleInfo(rclcpp::Node::SharedPtr &nh, const std::string &name)
      : ModuleInfo<autonomous_msgs::msg::ConeArray>(nh, name), cone_count_(0) {}

  void MsgCallback(const autonomous_msgs::msg::ConeArray::SharedPtr msg) override;
  uint8_t GetConeCount() { return cone_count_; }

private:
  uint8_t cone_count_;
};

class PipelineMonitor {
public:
  // Constructor
  PipelineMonitor();

  double GetNodeRate() const;
  void RunMonitoring();
  void MonitoringCallback();

private:
  // Node handle for ROS2 communication
  rclcpp::Node::SharedPtr node_handle_;

  // Monitoring node rate
  double monitoring_node_rate_;

  // Pipeline status object and publisher
  monitoring_msgs::msg::PipelineStatus pipeline_status_;
  rclcpp::Publisher<monitoring_msgs::msg::PipelineStatus>::SharedPtr pub_pipeline_status_;
  std::string pipeline_status_topic_name_;

  // Timer for monitoring
  rclcpp::TimerBase::SharedPtr monitoring_timer_;

  // Module info structs SENSOR
  std::unique_ptr<ModuleInfo<sensor_msgs::msg::Image>> sensor_driver_camera_;
  std::unique_ptr<ModuleInfo<sensor_msgs::msg::PointCloud2>> sensor_driver_lidar_;
  std::unique_ptr<ModuleInfo<vcu_msgs::msg::VelocityEstimation>> velocity_estimation_;
  std::unique_ptr<ModuleInfo<autonomous_msgs::msg::DoubleStamped>> steering_feedback_;

  // Module info structs PERCEPTION
  std::unique_ptr<ConeDetectionModuleInfo> camera_pipeline_;
  std::unique_ptr<ConeDetectionModuleInfo> lidar_pipeline_;
  std::unique_ptr<ConeDetectionModuleInfo> fusion_pipeline_;
  std::unique_ptr<ConeDetectionModuleInfo> per_pipeline_;

  // Module info structs ESTIMATION
  std::unique_ptr<ModuleInfo<autonomous_msgs::msg::Boundary>> boundary_estimation_;
  std::unique_ptr<ConeDetectionModuleInfo> online_map_;

  // // Module info structs PLANNING
  // std::unique_ptr<ModuleInfo<control_msgs::msg::ControllerRef>> planning_;
  // std::unique_ptr<ModuleInfo<control_msgs::msg::ControllerRef>> local_planning_;
  // std::unique_ptr<ModuleInfo<control_msgs::msg::ControllerRef>> global_planning_;

  // Module info structs CONTROL
  std::unique_ptr<ModuleInfo<vcu_msgs::msg::CarCommand>> control_;

  // initialize all modules
  void InitializeModules();
};

} // namespace monitoring_pipelines
