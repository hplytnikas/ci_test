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

#include "amzsim_msgs/msg/state.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

using std::placeholders::_1;

class SensortfFaker : public rclcpp::Node {
public:
  SensortfFaker() : Node("sensor_tf_faker"), tfBuffer(this->get_clock()), tfListener(tfBuffer) {
    state_sub = this->create_subscription<amzsim_msgs::msg::State>("/amzsim/car/state", 10,
                                                                   std::bind(&SensortfFaker::car_pose_to_tf, this, _1));
    sensor_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  ~SensortfFaker() = default;

private:
  void car_pose_to_tf(const amzsim_msgs::msg::State::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped odom_base_link_transform;
    geometry_msgs::msg::TransformStamped map_base_link_transform;

    map_base_link_transform.header.stamp = msg->header.stamp;
    map_base_link_transform.header.frame_id = "map";
    map_base_link_transform.child_frame_id = "base_link";
    map_base_link_transform.transform.translation.x = msg->x;
    map_base_link_transform.transform.translation.y = msg->y;
    map_base_link_transform.transform.translation.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, msg->yaw);
    map_base_link_transform.transform.rotation.x = quaternion[0];
    map_base_link_transform.transform.rotation.y = quaternion[1];
    map_base_link_transform.transform.rotation.z = quaternion[2];
    map_base_link_transform.transform.rotation.w = quaternion[3];

    rclcpp::Time t = msg->header.stamp;
    rclcpp::Rate rate_sleep(1);

    geometry_msgs::msg::TransformStamped map_odom_transform;
    try {
      map_odom_transform = tfBuffer.lookupTransform("map", "odom", t);
    } catch (tf2::TransformException &ex) {
      RCLCPP_INFO(get_logger(), "%s", ex.what());
      rate_sleep.sleep();
    }

    tf2::Transform map_odom_transform_tf2;
    tf2::Transform odom_map_transform_tf2;
    tf2::Transform map_base_link_transform_tf2;

    // publish
    tf2::Transform odom_base_link_transform_tf2;

    // Find the map to baselink transform by inverting map to odom first
    tf2::convert(map_odom_transform.transform,
                 map_odom_transform_tf2); // Convert the transformed stamp
                                          // message received to tf2
    odom_map_transform_tf2 = map_odom_transform_tf2.inverse();

    tf2::convert(map_base_link_transform.transform, map_base_link_transform_tf2);

    odom_base_link_transform_tf2 = odom_map_transform_tf2 * map_base_link_transform_tf2;

    tf2::Vector3 odom_base_link_trans = odom_base_link_transform_tf2.getOrigin();
    tf2::Quaternion odom_base_link_rot = odom_base_link_transform_tf2.getRotation();

    // Transform it to the right data type (transform stamped)
    odom_base_link_transform.header.stamp = map_base_link_transform.header.stamp;
    odom_base_link_transform.header.frame_id = "odom";
    odom_base_link_transform.child_frame_id = "base_link";

    odom_base_link_transform.transform.translation.x = odom_base_link_trans.getX();
    odom_base_link_transform.transform.translation.y = odom_base_link_trans.getY();
    odom_base_link_transform.transform.translation.z = 0;

    odom_base_link_transform.transform.rotation.x = odom_base_link_rot.x();
    odom_base_link_transform.transform.rotation.y = odom_base_link_rot.y();
    odom_base_link_transform.transform.rotation.z = odom_base_link_rot.z();
    odom_base_link_transform.transform.rotation.w = odom_base_link_rot.w();
    sensor_broadcaster->sendTransform(odom_base_link_transform);
  }

  rclcpp::Subscription<amzsim_msgs::msg::State>::SharedPtr state_sub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> sensor_broadcaster;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  try {
    SensortfFaker::SharedPtr sensor_tf_faker = std::make_shared<SensortfFaker>();
    rclcpp::spin(sensor_tf_faker);
  } catch (const std::exception &e) {
  }

  rclcpp::shutdown();
  return 0;
}
