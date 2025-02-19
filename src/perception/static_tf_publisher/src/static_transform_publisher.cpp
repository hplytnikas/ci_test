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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

// Loads transform data from a YAML file and returns a TransformStamped message.
geometry_msgs::msg::TransformStamped LoadTransformFromFile(const std::string &filename) {
  YAML::Node extrinsics = YAML::LoadFile(filename);
  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.transform.translation.x = extrinsics["translation"]["x"].as<double>();
  transform_stamped.transform.translation.y = extrinsics["translation"]["y"].as<double>();
  transform_stamped.transform.translation.z = extrinsics["translation"]["z"].as<double>();

  transform_stamped.transform.rotation.x = extrinsics["rotation"]["qx"].as<double>();
  transform_stamped.transform.rotation.y = extrinsics["rotation"]["qy"].as<double>();
  transform_stamped.transform.rotation.z = extrinsics["rotation"]["qz"].as<double>();
  transform_stamped.transform.rotation.w = extrinsics["rotation"]["qw"].as<double>();

  transform_stamped.header.frame_id = extrinsics["target_frame"].as<std::string>();
  transform_stamped.child_frame_id = extrinsics["source_frame"].as<std::string>();

  return transform_stamped;
}

// Sets transform data from arguments and and returns a TransformStamped message.
geometry_msgs::msg::TransformStamped SetTransformFromArguments(char **argv, int first_index) {
  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.transform.translation.x = std::stod(argv[first_index]);
  transform_stamped.transform.translation.y = std::stod(argv[first_index + 1]);
  transform_stamped.transform.translation.z = std::stod(argv[first_index + 2]);

  transform_stamped.transform.rotation.x = std::stod(argv[first_index + 3]);
  transform_stamped.transform.rotation.y = std::stod(argv[first_index + 4]);
  transform_stamped.transform.rotation.z = std::stod(argv[first_index + 5]);
  transform_stamped.transform.rotation.w = std::stod(argv[first_index + 6]);

  transform_stamped.header.frame_id = argv[first_index + 7];
  transform_stamped.child_frame_id = argv[first_index + 8];

  return transform_stamped;
}

// Resolves the filename from a 'package://' format to the actual file path in the package directory
std::string GetPackageFilename(const std::string &filename) {
  /*
  Inspired by "CameraInfoManager::getPackageFileName" in
  http://docs.ros.org/en/hydro/api/camera_info_manager/html/camera__info__manager_8cpp_source.html
  */

  // Scan filename from after "package://" until next '/' and extract
  // package name.
  size_t prefix_len = std::string("package://").length();
  size_t rest = filename.find('/', prefix_len);
  std::string package(filename.substr(prefix_len, rest - prefix_len));
  // Look up the ROS package path name.
  std::string pkgPath(ament_index_cpp::get_package_share_directory(package));
  if (pkgPath.empty()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "unknown package: " << package << " (ignored)");
    return pkgPath;
  } else {
    // Construct file name from package location and remainder of URL.
    return pkgPath + filename.substr(rest);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("StaticFramePublisher");
  auto tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  tf2_msgs::msg::TFMessage current_transforms;

  // Wait for the tf_static topic to be available
  bool ret = rclcpp::wait_for_message(current_transforms, node, "tf_static", std::chrono::seconds(1));

  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  // Skip the first argument which is the node name
  int i = 1;

  // Parse the command line arguments and create the transforms
  while (i < argc) {
    // Discard the last 3 arguments (--ros-args -r __node:=perception_tf_static)
    if (std::string(argv[i]) == "--ros-args" || std::string(argv[i]) == "-r" ||
        std::string(argv[i]).find("__node:=") != std::string::npos) {
      i++;
      continue;
    }
    geometry_msgs::msg::TransformStamped transform;
    if (std::string(argv[i]).find(".yaml") != std::string::npos) {
      std::string resolved_filename = GetPackageFilename(argv[i]);
      transform = LoadTransformFromFile(resolved_filename);
      i++;
    } else {
      transform = SetTransformFromArguments(argv, i);

      // Increment by number of arguments per transform: x, y, z, qx, qy, qz,
      // qw, header, child
      i += 9;
    }
    if (!ret) {
      RCLCPP_INFO_STREAM(node->get_logger(), "Adding new transform between " << transform.header.frame_id << " and "
                                                                             << transform.child_frame_id);
      transforms.push_back(transform);
    } else {
      // In first iteration, copy existing content to new transform vector.
      if (i == 1) {
        transforms = current_transforms.transforms;
      }

      // If there already exists a transform between these frames, update it.
      bool old_transform_found = false;
      for (auto &prev_transform : transforms) {
        if (prev_transform.child_frame_id == transform.child_frame_id &&
            prev_transform.header.frame_id == transform.header.frame_id) {
          prev_transform = transform;
          old_transform_found = true;
          RCLCPP_INFO_STREAM(node->get_logger(), "Updating existing transform between " << transform.header.frame_id
                                                                                        << " and "
                                                                                        << transform.child_frame_id);
        }
      }
      if (!old_transform_found) {
        RCLCPP_INFO_STREAM(node->get_logger(), "Adding new transform between " << transform.header.frame_id << " and "
                                                                               << transform.child_frame_id);
        transforms.push_back(transform);
      }
    }
  }

  // Set the timestamp to 0, these transformations do not change over time
  for (auto &transform : transforms) {
    transform.header.stamp = rclcpp::Time(0);
  }

  // Broadcast the transforms
  tf_static_broadcaster->sendTransform(transforms);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
