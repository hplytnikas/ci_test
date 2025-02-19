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

#include "amzsim_perception/perception_faked.hpp"

namespace perception_faked {

Perception_Faker::Perception_Faker(rclcpp::Node::SharedPtr nh, string csvfile)
    : node_(nh), tfBuffer(node_->get_clock()), tfListener(tfBuffer) {
  // Initialize the publisher for cone array
  cone_array_pub = node_->create_publisher<autonomous_msgs::msg::ConeArray>("/perception/cone_array", 1);

  csv_to_cone_array(csvfile);

  node_->declare_parameter("frequency", 10);
  node_->declare_parameter("fov_r_fus", 30.0);
  node_->declare_parameter("fov_r_cam", 10.0);
  node_->declare_parameter("fov_r_lid", 30.0);
  node_->declare_parameter("fov_angle_fus", 100.0);
  node_->declare_parameter("fov_angle_cam", 100.0);
  node_->declare_parameter("fov_angle_lid", 100.0);
  node_->declare_parameter("mean_x_fus", 0.0);
  node_->declare_parameter("mean_y_fus", 0.0);
  node_->declare_parameter("mean_x_cam", 0.0);
  node_->declare_parameter("mean_y_cam", 0.0);
  node_->declare_parameter("mean_x_lid", 0.0);
  node_->declare_parameter("mean_y_lid", 0.0);
  node_->declare_parameter("stddev_x_fus", 0.1);
  node_->declare_parameter("stddev_y_fus", 0.1);
  node_->declare_parameter("stddev_x_cam", 0.3);
  node_->declare_parameter("stddev_y_cam", 0.3);
  node_->declare_parameter("stddev_x_lid", 0.1);
  node_->declare_parameter("stddev_y_lid", 0.1);

  // Loads fov parameters and frequency
  load_perception_config();

  node_->declare_parameter("perception_gt", false);
  node_->declare_parameter("estimation_gt", false);
  node_->declare_parameter("pipeline_id", "fusion"); // camera_only:1, lidar_only:2, fused:4

  if (!node_->get_parameter("perception_gt", perception_gt)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("estimation_gt", estimation_gt)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("pipeline_id", pipeline_id_str)) {
    rclcpp::shutdown();
  }
}

void Perception_Faker::load_perception_config() {
  if (!node_->get_parameter("fov_r_fus", fov_r_fus_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find fov_r_fus parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("fov_angle_fus", fov_angle_fus_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find fov_angle_fus parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("frequency", frequency_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find frequency parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("mean_x_fus", mean_x_fus_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find mean_x_fus parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("mean_y_fus", mean_y_fus_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find mean_y_fus parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("stddev_x_fus", stddev_x_fus_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find stddev_x_fus parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("stddev_y_fus", stddev_y_fus_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find stddev_y_fus parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("stddev_x_cam", stddev_x_cam_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find stddev_x_cam parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("stddev_y_cam", stddev_y_cam_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find stddev_y_cam parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("stddev_x_lid", stddev_x_lid_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find stddev_x_lid parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("stddev_y_lid", stddev_y_lid_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find stddev_y_lid parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("fov_r_cam", fov_r_cam_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find fov_r_cam parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("fov_angle_cam", fov_angle_cam_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find fov_angle_cam parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("fov_r_lid", fov_r_lid_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find fov_r_lid parameter in load_sim_config()!");
    rclcpp::shutdown();
  }

  if (!node_->get_parameter("fov_angle_lid", fov_angle_lid_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find fov_angle_lid parameter in load_sim_config()!");
    rclcpp::shutdown();
  }
}

void Perception_Faker::assign_per_params() {
  // Assign perception parameters based on pipeline id
  if (pipeline_id_str == "fusion") {
    pipeline_id = 4;
  } else if (pipeline_id_str == "camera") {
    pipeline_id = 1;
  } else if (pipeline_id_str == "lidar") {
    pipeline_id = 2;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid Pipeline ID for perception! Number recieved: %d", pipeline_id);
    rclcpp::shutdown();
  }

  // Camera only
  if (pipeline_id == 1) {
    fov_r_ = fov_r_cam_;
    fov_angle_ = fov_angle_cam_;
    mean_x_ = mean_x_cam_;
    mean_y_ = mean_y_cam_;
    stddev_x_ = stddev_x_cam_;
    stddev_y_ = stddev_y_cam_;
  } else if (pipeline_id == 2) {
    fov_r_ = fov_r_lid_;
    fov_angle_ = fov_angle_lid_;
    mean_x_ = mean_x_lid_;
    mean_y_ = mean_y_lid_;
    stddev_x_ = stddev_x_lid_;
    stddev_y_ = stddev_y_lid_;
  } else if (pipeline_id == 4) {
    fov_r_ = fov_r_fus_;
    fov_angle_ = fov_angle_fus_;
    mean_x_ = mean_x_fus_;
    mean_y_ = mean_y_fus_;
    stddev_x_ = stddev_x_fus_;
    stddev_y_ = stddev_y_fus_;
  }
}

void Perception_Faker::csv_to_cone_array(std::string csvfile) {
  // Read csv and category
  // Open the CSV file

  string filepath = csvfile;

  ifstream file(filepath); // CHANGE TO GET NAME FROM GUI
  if (!file.is_open()) {
    RCLCPP_INFO(node_->get_logger(), "FILE NOT OPEN IN PERCEPTION FAKED");
    rclcpp::shutdown();
  }

  string line;

  // Read each line of the file
  getline(file, line); // Skipps the first line
  while (getline(file, line)) {
    // Create a string stream from the line
    stringstream line_stream(line);

    // Read the tag, x, and y values from the line
    string tag;
    // Stores the tag
    getline(line_stream, tag, ',');

    // Stores the cone in the vector
    string x_str, y_str;
    getline(line_stream, x_str, ',');
    getline(line_stream, y_str, ',');
    double x = stod(x_str);
    double y = stod(y_str);

    cone_struct temp;
    temp.x = x;
    temp.y = y;
    temp.tag = tag;
    array_of_cones.push_back(temp);
  }
}

void Perception_Faker::add_noise_cones(std::vector<cone_struct> &array_of_cones_in_fov) {
  // Set up a random number generator with a normal distribution
  std::random_device rd;
  std::mt19937 generator(rd());
  normal_distribution<double> dist_x(mean_x_, stddev_x_);
  normal_distribution<double> dist_y(mean_y_, stddev_y_);

  // Define all the cones
  for (size_t i = 0; i < array_of_cones_in_fov.size(); ++i) {
    double noise_x = dist_x(generator);
    double noise_y = dist_y(generator);
    // Set the position of cones
    array_of_cones_in_fov[i].x += noise_x;
    array_of_cones_in_fov[i].y += noise_y;
  }
}

autonomous_msgs::msg::ConeArray Perception_Faker::array_of_cones_to_msg(std::vector<cone_struct> array_of_cones,
                                                                        rclcpp::Time t_look) {
  // define message
  autonomous_msgs::msg::ConeArray msg;
  std::vector<autonomous_msgs::msg::Cone> array_of_cones_msg(array_of_cones.size());

  // Define all the cones
  for (size_t i = 0; i < array_of_cones.size(); ++i) {
    // Set the position of cones
    array_of_cones_msg[i].position.x = array_of_cones[i].x;
    array_of_cones_msg[i].position.y = array_of_cones[i].y;
    array_of_cones_msg[i].position.z = 0.0;
    array_of_cones_msg[i].id_cone = i;
    array_of_cones_msg[i].pipeline = pipeline_id;
    array_of_cones_msg[i].is_observed = true; // Random atm

    array_of_cones_msg[i].prob_cone = 1;
    if (pipeline_id == 2) {
      array_of_cones_msg[i].prob_type.yellow = 0.25;
      array_of_cones_msg[i].prob_type.blue = 0.25;
      array_of_cones_msg[i].prob_type.orange = 0.25;
      array_of_cones_msg[i].prob_type.orange_big = 0.25;
    } else {
      if (array_of_cones[i].tag == "blue") {
        array_of_cones_msg[i].prob_type.yellow = 0.0;
        array_of_cones_msg[i].prob_type.blue = 1.0;
        array_of_cones_msg[i].prob_type.orange = 0.0;
        array_of_cones_msg[i].prob_type.orange_big = 0.0;
      } else if (array_of_cones[i].tag == "yellow") {
        array_of_cones_msg[i].prob_type.yellow = 1.0;
        array_of_cones_msg[i].prob_type.blue = 0.0;
        array_of_cones_msg[i].prob_type.orange = 0.0;
        array_of_cones_msg[i].prob_type.orange_big = 0.0;
      } else if (array_of_cones[i].tag == "orange") {
        array_of_cones_msg[i].prob_type.yellow = 0.0;
        array_of_cones_msg[i].prob_type.blue = 0.0;
        array_of_cones_msg[i].prob_type.orange = 1.0;
        array_of_cones_msg[i].prob_type.orange_big = 0.0;
      } else if (array_of_cones[i].tag == "orange_big") {
        array_of_cones_msg[i].prob_type.yellow = 0.0;
        array_of_cones_msg[i].prob_type.blue = 0.0;
        array_of_cones_msg[i].prob_type.orange = 0.0;
        array_of_cones_msg[i].prob_type.orange_big = 1.0;
      } else if (array_of_cones[i].tag == "none") {
        array_of_cones_msg[i].prob_type.yellow = 0.25;
        array_of_cones_msg[i].prob_type.blue = 0.25;
        array_of_cones_msg[i].prob_type.orange = 0.25;
        array_of_cones_msg[i].prob_type.orange_big = 0.25;
      }
    }
  }
  // message definition
  msg.header.stamp = t_look; // STAMP IS ALMOST PUT AT BEGINNING OF THE 1/10
                             // seconds, TO CHECK, T
  // MAYBE PUT  msg.header.stamp = t where t = state.header.stamp
  msg.cones = array_of_cones_msg;
  msg.header.frame_id = "base_link_sim"; // World and no transformation

  return msg;
}

void Perception_Faker::publish_cones_in_fov() {
  // Assign perception parameters
  assign_per_params();

  // Perception runs at 10 Hz
  rclcpp::Rate rate(10);
  rclcpp::Rate rate_sleep(1);

  // Loops that runs at the same frequency as perception
  while (rclcpp::ok()) {
    // Declare the array of cone that are in the fov
    std::vector<cone_struct> array_of_cones_in_fov;

    // Buffer for the time at which the faked image is taken to stamp the
    // message later
    rclcpp::Time t_look;
    // Fetch the world->base_link transformation
    geometry_msgs::msg::TransformStamped map_base_link_sim_transform;
    try {
      map_base_link_sim_transform = tfBuffer.lookupTransform("map", "base_link_sim", tf2::TimePointZero);
      t_look = map_base_link_sim_transform.header.stamp;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
      rate_sleep.sleep();
      continue;
    }

    // Define necessary transforms
    tf2::Transform map_base_link_sim_tf2_transform;
    tf2::Transform base_link_sim_map_transform;
    tf2::Transform cone_transform;
    tf2::Transform base_link_sim_cone_transform;

    // Find the right transformation base_link->world
    tf2::convert(map_base_link_sim_transform.transform,
                 map_base_link_sim_tf2_transform); // Convert the transformed stamp
                                                   // message received to tf2
    base_link_sim_map_transform = map_base_link_sim_tf2_transform.inverse();

    // Set the cones the right rotation matrix
    tf2::Quaternion quat_identity;
    quat_identity.setRPY(0, 0, 0);
    cone_transform.setRotation(quat_identity);

    // Loop through all the cones
    for (size_t i = 0; i < array_of_cones.size(); ++i) {
      // Find x,y position of the cones in base_link frame
      cone_transform.setOrigin(tf2::Vector3(array_of_cones[i].x, array_of_cones[i].y, 0.0));
      base_link_sim_cone_transform = base_link_sim_map_transform * cone_transform;
      tf2::Vector3 base_link_sim_cone_pos = base_link_sim_cone_transform.getOrigin();

      double x_cone_base_link_sim = base_link_sim_cone_pos.getX();
      double y_cone_base_link_sim = base_link_sim_cone_pos.getY();

      // Get the distance from base_link to the cone
      double cone_distance =
          sqrt(x_cone_base_link_sim * x_cone_base_link_sim + y_cone_base_link_sim * y_cone_base_link_sim);

      // Check if the cone lies in the field of view of the car
      if (cone_distance <= fov_r_) {
        // Get the angle of the cone w.r.t. x-axis of base_link
        double angle_cone_base_link_sim = atan2(y_cone_base_link_sim, x_cone_base_link_sim) * 180 / M_PI;

        if ((angle_cone_base_link_sim < (fov_angle_ / 2)) && (angle_cone_base_link_sim > (-fov_angle_ / 2))) {
          cone_struct temp; // If yes, store the cone coordinate in base_link
                            // frame in the array
          temp.x = x_cone_base_link_sim;
          temp.y = y_cone_base_link_sim;
          temp.tag = array_of_cones[i].tag;
          // array_of_cones_in_fov.push_back(array_of_cones[i]);
          array_of_cones_in_fov.push_back(temp);
        }
      }
    }

    if (perception_gt == false) {
      add_noise_cones(array_of_cones_in_fov);
    }

    // Transforms array of cone_struct into msg
    autonomous_msgs::msg::ConeArray msg;

    // If estimation gt is true publish all the cones on the map to simulate
    // perfect SLAM
    // Print clock time
    rclcpp::Time time = node_->get_clock()->now();
    if (estimation_gt == true) {
      msg = array_of_cones_to_msg(array_of_cones, time);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } else {
      msg = array_of_cones_to_msg(array_of_cones_in_fov, time); // array_of_cones_in_fov is in base_link frame
      // sleep to simulate perception taking some time to compute stuff
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    // Publish it on perception/cone_array
    cone_array_pub->publish(msg);

    // Standard roscpp loop
    rclcpp::spin_some(node_);
    rate.sleep();
  }
}

}; // namespace perception_faked
