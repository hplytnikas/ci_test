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

#include "lap_counter.h"

using std::placeholders::_1;

LapCounter::LapCounter() : Node("lap_counter") {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Create a publisher for the lap count
  lap_counter_pub_ = this->create_publisher<std_msgs::msg::Int32>("lap_count", 10);

  // Create a publisher for the lap time
  lap_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", 10);

  // Create a publisher for the distance traveled
  lap_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("distance_traveled", 10);

  // Create a publisher for the moving status
  start_moving_pub_ = this->create_publisher<std_msgs::msg::Bool>("moving", 10);

  // Create a timer to check the lap count
  timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&LapCounter::LapCountCallback, this));

  // Timer for publishing the has_moved condition at 4 Hz
  timer_has_moved_ =
      this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&LapCounter::PublishHasMoved, this));

  // Timer for publishing the circle marker
  timer_marker_ =
      this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LapCounter::publishInitialMarker, this));

  // Subscribe to the velocity estimation topic
  velocity_est_sub_ = this->create_subscription<vcu_msgs::msg::VelocityEstimation>(
      "/vcu_msgs/velocity_estimation", 4, std::bind(&LapCounter::VelocityEstCallback, this, _1));

  // Create a publisher for the circle marker
  circle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lap_counter_visual", 10);

  // Initialize the lap count
  lap_count_ = 0;

  // Initialize the variable for the started distance
  traveled_distance_ = 0;

  // Initialize the variable for the start time
  start = true;

  // Initialize vehicle status to has not moved
  has_vehicle_moved_ = false;

  // Retrieve the parameter values
  this->declare_parameter<double>("offset", 6);
  this->get_parameter("offset", offset_);
  RCLCPP_INFO(this->get_logger(), "The offset is of: %f", offset_);

  this->declare_parameter<double>("start_radius", 0.5);
  this->get_parameter("start_radius", start_radius_);
  RCLCPP_INFO(this->get_logger(), "The start radius is of: %f", start_radius_);

  this->declare_parameter<double>("moving_distance_condition", 15);
  this->get_parameter("moving_distance_condition", moving_distance_condition_);
  RCLCPP_INFO(this->get_logger(), "The moving_distance_condition is of: %f", moving_distance_condition_);

  // Initialize the first pose as offset 0 0 and unit quaternion
  first_pose_.header.stamp = this->get_clock()->now();
  first_pose_.header.frame_id = "map";
  first_pose_.child_frame_id = "base_link";
  first_pose_.transform.translation.x = offset_;
  first_pose_.transform.translation.y = 0;
  first_pose_.transform.translation.z = 0;
  first_pose_.transform.rotation.x = 0;
  first_pose_.transform.rotation.y = 0;
  first_pose_.transform.rotation.z = 0;
  first_pose_.transform.rotation.w = 1;

  // Create the marker message for the circle
  circle_marker_.header.frame_id = "map";
  circle_marker_.ns = "lap_counter";
  circle_marker_.id = 0;
  circle_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
  circle_marker_.action = visualization_msgs::msg::Marker::ADD;
  circle_marker_.pose.position.x = first_pose_.transform.translation.x;
  circle_marker_.pose.position.y = first_pose_.transform.translation.y;
  circle_marker_.pose.position.z = 0;
  circle_marker_.pose.orientation.w = 1.0;
  circle_marker_.scale.x = start_radius_ * 2; // Diameter of the circle
  circle_marker_.scale.y = start_radius_ * 2; // Diameter of the circle
  circle_marker_.scale.z = 0.1;               // Small height for the cylinder to make it a circle
  circle_marker_.color.a = 0.5;               // Semi-transparent
  circle_marker_.color.r = 0.0;
  circle_marker_.color.g = 1.0;
  circle_marker_.color.b = 0.0;

  // Set marker lifetime
  circle_marker_.lifetime = rclcpp::Duration(0, 0); // Forever
}

void LapCounter::publishInitialMarker() {
  // Publish the circle marker once during initialization
  circle_marker_pub_->publish(circle_marker_);
}

// Method to get a transform between two frames

geometry_msgs::msg::TransformStamped LapCounter::GetTransform(const std::string &target_frame,
                                                              const std::string &source_frame) {
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(),
                 ex.what());
  }

  return transform_stamped;
}

void LapCounter::LapCountCallback() {
  // Get the transform between the start and current pose
  geometry_msgs::msg::TransformStamped transform_stamped = GetTransform("map", "base_link");

  Update(transform_stamped);

  if (!DidVehicleMove()) {
    return;
  }

  // Start the timer once the vehicle has moved
  if (start && traveled_distance_ > 1) {
    start_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Lap started.");
    start = false;
  }

  // Compute total distance traveled
  double distance = std::hypot(abs(current_pose_.transform.translation.x - prev_pose_.transform.translation.x),
                               abs(current_pose_.transform.translation.y - prev_pose_.transform.translation.y));

  traveled_distance_ += distance;

  // RCLCPP_INFO(this->get_logger(), "Traveled distance: %f meters", traveled_distance_);

  // Publish the distance traveled
  std_msgs::msg::Float32 distance_traveled_msg;
  distance_traveled_msg.data = traveled_distance_;
  lap_distance_pub_->publish(distance_traveled_msg);

  // Check for intersection with the starting circle
  bool intersects;
  intersects = LineSegmentIntersectsCircle(prev_pose_.transform.translation.x, prev_pose_.transform.translation.y,
                                           current_pose_.transform.translation.x, current_pose_.transform.translation.y,
                                           first_pose_.transform.translation.x, first_pose_.transform.translation.y,
                                           start_radius_);
  // RCLCPP_INFO(this->get_logger(), "Intersects: %d", intersects);

  if (vehicle_was_near_start_ && intersects && traveled_distance_ > moving_distance_condition_) {
    // Increment the lap count
    lap_count_++;
    // RCLCPP_INFO(this->get_logger(), "Lap count: %d", lap_count_);

    // Publish the new lap count
    if (lap_count_ > 0) {
      std_msgs::msg::Int32 lap_count_msg;
      lap_count_msg.data = static_cast<uint32_t>(lap_count_);
      lap_counter_pub_->publish(lap_count_msg);

      // End the timer
      EndTimer();

      // Restart parameters
      ResetParameters();
    }
  }
}

void LapCounter::VelocityEstCallback(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg) {
  // Check if the vehicle is moving
  if (msg->vel.x != 0 || msg->vel.y != 0 || msg->vel.theta != 0) {
    has_vehicle_moved_ = true;
  } else {
    has_vehicle_moved_ = false;
  }
}

bool LapCounter::DidVehicleMove() const {
  // Calculate the distance between the previous and current positions
  double distance = std::hypot(current_pose_.transform.translation.x - prev_pose_.transform.translation.x,
                               current_pose_.transform.translation.y - prev_pose_.transform.translation.y,
                               current_pose_.transform.translation.z - prev_pose_.transform.translation.z);

  // Check if the distance moved is above a certain threshold, indicating
  // movement
  if (distance > 0.001) { // Threshold of 1 mm
    return true;
  }

  double orientation_difference = QuaternionDifference(current_pose_.transform.rotation, prev_pose_.transform.rotation);

  if (orientation_difference > 0.001) { // Threshold for orientation change, adjust as needed
    return true;
  }

  // If neither position nor orientation has changed significantly, assume no
  // movement
  return false;
}

bool LapCounter::IsCurrentPoseNearStart() const {
  // Calculate the distance between the current pose and the start pose
  double distance = std::hypot(abs(current_pose_.transform.translation.x - first_pose_.transform.translation.x),
                               abs(current_pose_.transform.translation.y - first_pose_.transform.translation.y),
                               abs(current_pose_.transform.translation.z - first_pose_.transform.translation.z));

  // Check if the distance is below a certain threshold, indicating proximity to
  // the start
  if (distance < start_radius_) { // Threshold
    return true;
  }

  // If the distance is above the threshold, the vehicle is not near the start
  return false;
}

void LapCounter::Update(const geometry_msgs::msg::TransformStamped &pose) {
  prev_pose_ = current_pose_;
  current_pose_ = pose;
  if (first_pose_.transform.translation.x == offset_ && first_pose_.transform.translation.y == 0 &&
      first_pose_.transform.translation.z == 0) {
    vehicle_was_near_start_ = true;
    return;
  }
}

double LapCounter::QuaternionDifference(const geometry_msgs::msg::Quaternion &q1,
                                        const geometry_msgs::msg::Quaternion &q2) const {
  tf2::Quaternion qt1(q1.x, q1.y, q1.z, q1.w);
  tf2::Quaternion qt2(q2.x, q2.y, q2.z, q2.w);

  double dot = qt1.dot(qt2) / (qt1.length() * qt2.length());
  // Clamp dot product to ensure acos is valid
  dot = std::min(1.0, std::max(-1.0, dot));
  double angle = std::acos(dot);

  return angle;
}

void LapCounter::ResetParameters() {
  // Reset the variable for the started distance
  traveled_distance_ = 0;

  // Reset the variable for the start time
  start = true;

  // Reset the timer
  start_time_ = std::chrono::steady_clock::now();
}

void LapCounter::EndTimer() {
  // End the timer
  end_time_ = std::chrono::steady_clock::now();
  auto duration_in_seconds = std::chrono::duration<double>(end_time_ - start_time_).count();
  RCLCPP_INFO(this->get_logger(), "Lap ended. Duration: %f seconds", duration_in_seconds);

  // Publish the lap time
  std_msgs::msg::Float32 lap_time_msg;
  lap_time_msg.data = duration_in_seconds;
  lap_time_pub_->publish(lap_time_msg);
}

// Function to check if a line segment intersects a circle
bool LapCounter::LineSegmentIntersectsCircle(double x1, double y1, double x2, double y2, double cx, double cy,
                                             double r) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  double fx = x1 - cx;
  double fy = y1 - cy;

  double a = dx * dx + dy * dy;
  double b = 2 * (fx * dx + fy * dy);
  double c = (fx * fx + fy * fy) - r * r;

  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    // No intersection
    return false;
  }

  discriminant = sqrt(discriminant);

  double t1 = (-b - discriminant) / (2 * a);
  double t2 = (-b + discriminant) / (2 * a);

  bool intersects = false;
  double ix1 = 0, iy1 = 0, ix2 = 0, iy2 = 0;

  // Check if t1 is within the segment range
  if (t1 >= 0 && t1 <= 1) {
    ix1 = x1 + t1 * dx;
    iy1 = y1 + t1 * dy;
    intersects = true;
  }

  // Check if t2 is within the segment range
  if (t2 >= 0 && t2 <= 1) {
    ix2 = x1 + t2 * dx;
    iy2 = y1 + t2 * dy;
    intersects = true;
  }

  // Check if either endpoint is inside the circle
  bool endpointInside = false;
  double d1 = (x1 - cx) * (x1 - cx) + (y1 - cy) * (y1 - cy);
  double d2 = (x2 - cx) * (x2 - cx) + (y2 - cy) * (y2 - cy);

  if (d1 <= r * r || d2 <= r * r) {
    endpointInside = true;
  }

  return intersects || endpointInside;
}

void LapCounter::PublishHasMoved() {
  std_msgs::msg::Bool moving_msg;
  moving_msg.data = has_vehicle_moved_;
  start_moving_pub_->publish(moving_msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LapCounter>());
  rclcpp::shutdown();
  return 0;
}
