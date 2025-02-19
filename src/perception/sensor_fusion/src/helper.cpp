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

#include "fusion.hpp"

namespace sensor_fusion_baseline {

void SensorFusion::LoadParameters() {
  // Subscriber Topic Names
  this->declare_parameter<std::string>("topics.sub.points_compensated",
                                       "/perception/lidar_motion_compensator/compensated_pc");
  this->declare_parameter<std::string>("topics.sub.fw_bbox", "/perception/depth_estimation/bbox_depth");
  this->declare_parameter<std::string>("topics.sub.debug.fw_cam", "/sensors/forward_camera/image_color");

  // Publisher Topic Names
  this->declare_parameter<std::string>("topics.pub.runtime", "/perception/runtimes");
  this->declare_parameter<std::string>("topics.pub.debug.img_overlay",
                                       "/perception/sensor_fusion_baseline/debug/img_overlay");
  this->declare_parameter<std::string>("topics.pub.cone_array", "/perception/fusion/cone_array");
  this->declare_parameter<std::string>("topics.pub.debug.bbox_fusion",
                                       "/perception/sensor_fusion_baseline/debug/bbox_depth");
  this->declare_parameter<std::string>("intrinsics_file",
                                       "package://camera_launcher/calibration/forward_camera_castor.yaml");

  // Debug Parameter
  this->declare_parameter<bool>("debug", true);

  // Offset Parameters between camera and compensated_lidar_pc
  this->declare_parameter<int>("max_sync_offset", 30);
  this->declare_parameter<int>("min_inter_message", 70);

  // Frame IDs parameters
  this->declare_parameter<std::string>("frame_ids.camera", "pylon_camera");
  this->declare_parameter<std::string>("frame_ids.base_link", "base_link");
  this->declare_parameter<std::string>("frame_ids.reference_frame", "odom");
  this->declare_parameter<std::string>("lidar_mode", "hesai");
  this->declare_parameter<std::string>("car_mode", "dufour");

  // Image resolution parameters
  this->declare_parameter<int>("image_width", 2592);
  this->declare_parameter<int>("image_height", 352);

  // Filter Tuning Parameters
  this->declare_parameter<float>("filter_tuning.min_cone_distance", 2.0);
  this->declare_parameter<float>("filter_tuning.max_cone_distance", 30.0);
  this->declare_parameter<int>("filter_tuning.min_lidar_points", 2);
  this->declare_parameter<int>("filter_tuning.image_bottom_border", 5);
  this->declare_parameter<float>("filter_tuning.max_dist_camera_depth", 1.0);
  this->declare_parameter<float>("filter_tuning.max_dist_pixels", 40.0);
  this->declare_parameter<float>("filter_tuning.averaging_depth_offset", 0.2);

  // Subscriber Topic Names
  this->get_parameter("topics.sub.points_compensated", topic_points_compensated_);
  this->get_parameter("topics.sub.fw_bbox", topic_fw_bbox_);
  this->get_parameter("topics.sub.debug.fw_cam", topic_fw_cam_);
  this->get_parameter("topics.pub.runtime", topic_runtime_);
  this->get_parameter("topics.pub.debug.img_overlay", topic_img_overlay_);
  this->get_parameter("topics.pub.cone_array", topic_cone_array_);
  this->get_parameter("topics.pub.debug.bbox_fusion", topic_bbox_fusion_);

  this->get_parameter("intrinsics_file", intrinsics_file_);

  // Debug Parameter
  this->get_parameter("debug", debug_);

  // Offset Parameters between camera and compensated_lidar_pc
  this->get_parameter("max_sync_offset", max_sync_offset_);
  this->get_parameter("min_inter_message", min_inter_message_);

  this->get_parameter("frame_ids.camera", camera_frame_id_);
  this->get_parameter("frame_ids.base_link", base_link_frame_id_);
  this->get_parameter("frame_ids.reference_frame", reference_frame_id_);
  std::string lidar_mode_;
  this->get_parameter("lidar_mode", lidar_mode_);
  if (lidar_mode_ == "hesai") {
    lidar_frame_id_ = "hesai_lidar";
  } else if (lidar_mode_ == "ouster") {
    lidar_frame_id_ = "os_lidar";
  }

  std::string car_mode_;
  this->get_parameter("car_mode", car_mode_);
  std::string car_name = "CAR_NAME";
  size_t pos = intrinsics_file_.find(car_name);
  if (pos != std::string::npos) intrinsics_file_.replace(pos, car_name.length(), car_mode_);

  // Image resolution parameters
  this->get_parameter("image_width", image_width_);
  this->get_parameter("image_height", image_height_);

  // Filter Tuning Parameters
  this->get_parameter("filter_tuning.min_cone_distance", min_cone_distance_);
  this->get_parameter("filter_tuning.max_cone_distance", max_cone_distance_);
  this->get_parameter("filter_tuning.min_lidar_points", min_lidar_points_);
  this->get_parameter("filter_tuning.image_bottom_border", image_bottom_border_);
  this->get_parameter("filter_tuning.max_dist_camera_depth", max_dist_camera_depth_);
  this->get_parameter("filter_tuning.max_dist_pixels", max_dist_pixels_);
  this->get_parameter("filter_tuning.averaging_depth_offset", averaging_depth_offset_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// Loads all camera's intrinsics from intrinsics server, and extrinsics
// (MRH2Camera) from /tf_static
void SensorFusion::LoadIntrinsics() {
  // Load camera intrinsics from intrinsics server
  try {
    std::string resolved_filename = GetPackageFilename(intrinsics_file_);
    YAML::Node intrinsics = YAML::LoadFile(resolved_filename);
    const std::vector<float> &data = intrinsics["camera_matrix"]["data"].as<std::vector<float>>();
    std::memcpy(K_mat_.data, data.data(), 9 * sizeof(float));
    const std::vector<float> &distortion = intrinsics["distortion_coefficients"]["data"].as<std::vector<float>>();
    std::memcpy(D_mat_.data, distortion.data(), 14 * sizeof(float));
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load intrinsics -> " << e.what());
  }
  RCLCPP_INFO(this->get_logger(), "Camera intrinsics loaded from file: %s", intrinsics_file_.c_str());
}

bool SensorFusion::LoadStaticTransforms() {
  try {
    geometry_msgs::msg::TransformStamped tf_transform =
        tf_buffer_->lookupTransform(camera_frame_id_, lidar_frame_id_, tf2::TimePointZero, std::chrono::seconds(1));
    auto rotation =
        tf2::Matrix3x3(tf2::Quaternion(tf_transform.transform.rotation.x, tf_transform.transform.rotation.y,
                                       tf_transform.transform.rotation.z, tf_transform.transform.rotation.w));
    float r[] = {
        static_cast<float>(rotation[0].x()), static_cast<float>(rotation[0].y()), static_cast<float>(rotation[0].z()),
        static_cast<float>(rotation[1].x()), static_cast<float>(rotation[1].y()), static_cast<float>(rotation[1].z()),
        static_cast<float>(rotation[2].x()), static_cast<float>(rotation[2].y()), static_cast<float>(rotation[2].z())};
    cv::Mat r_mat(3, 3, CV_32F, r);
    cv::Rodrigues(r_mat, rvecR_);

    float t[] = {static_cast<float>(tf_transform.transform.translation.x),
                 static_cast<float>(tf_transform.transform.translation.y),
                 static_cast<float>(tf_transform.transform.translation.z)};
    std::memcpy(t_mat_.data, t, 3 * sizeof(float));
    return true;
  } catch (tf2::TransformException &exception) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Exception during transform lookup from [" << lidar_frame_id_ << "] to ["
                                                                                       << camera_frame_id_
                                                                                       << "]: " << exception.what());
    return false;
  }
}

void SensorFusion::LoadCalibrationParameters() {
  LoadIntrinsics();
  while (!LoadStaticTransforms()) {
    // wait for 100 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void SensorFusion::SubscribeTopics() {
  sub_pointcloud_.subscribe(this, topic_points_compensated_);
  sub_bbox_.subscribe(this, topic_fw_bbox_);
  sub_image_.subscribe(this, topic_fw_cam_);

  // SyncPolicy(20): ApproximateTime sync with queue of 20 for all the listed
  // synchronised topics Additional parameters for efficient matching of topic
  // messages
  sync_.reset(new Sync(SyncPolicy(20), sub_pointcloud_, sub_bbox_, sub_image_));

  sync_->registerCallback(&sensor_fusion_baseline::SensorFusion::DetectCones, this);
}

void SensorFusion::AdvertiseTopics() {
  pub_runtime_ = this->create_publisher<perception_msgs::msg::PipelineRuntime>(topic_runtime_, 10);
  if (debug_) {
    pub_img_overlay_ = this->create_publisher<sensor_msgs::msg::Image>(topic_img_overlay_, 10);
    pub_bbox_fusion_ = this->create_publisher<perception_msgs::msg::BoxArrayDebug>(topic_bbox_fusion_, 10);
  }
  pub_cone_array_ = this->create_publisher<autonomous_msgs::msg::ConeArray>(topic_cone_array_, 10);
  pub_cone_markers_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/fusion/cone_markers", 10);
}

autonomous_msgs::msg::Cone SensorFusion::CreateConeMsg(const cv::Point3f &cone_position,
                                                       const perception_msgs::msg::BoundingBox box) {
  autonomous_msgs::msg::Cone cone;
  cone.position.x = cone_position.x;
  cone.position.y = cone_position.y;
  cone.position.z = cone_position.z; // Gowtham: Can this be made 0, as anyway we live in 2D world
  cone.prob_cone = box.prob_cone;

  // label (color) comes from monocamera. Code for probabilities is also from
  // this repo
  cone.prob_type.blue = box.prob_type.blue;
  cone.prob_type.yellow = box.prob_type.yellow;
  cone.prob_type.orange = box.prob_type.orange;
  cone.prob_type.orange_big = box.prob_type.orange_big;
  cone.is_observed = true;
  cone.pipeline = 4;
  return cone;
}

void SensorFusion::EstimateConePosition(const cv::Mat *image_mat,
                                        const std::vector<perception_msgs::msg::BoundingBox> &bounding_boxes,
                                        const std::vector<cv::Point2f> &points_cam,
                                        const std::vector<cv::Point3f> *cloud,
                                        autonomous_msgs::msg::ConeArray *cone_array, const rclcpp::Time time_stamp,
                                        perception_msgs::msg::BoxArrayDebug &sf_boxes) {
  EASY_FUNCTION(profiler::colors::Green);
  // Implement a 1 nearest neighbor search for each point_cam in the cloud, assign for every point the index of the
  // closest bounding box
  std::vector<std::vector<int>> points_bounding_boxes(bounding_boxes.size());
  for (int i = 0; i < points_cam.size(); i++) {
    // find the index of the closest bounding box to the point
    float min_dist = std::numeric_limits<float>::max();
    int min_index = -1;
    for (int j = 0; j < bounding_boxes.size(); j++) {
      auto box = bounding_boxes[j];
      cv::Point2f box_center = cv::Point2f((box.box_x_min + box.box_x_max) / 2, (box.box_y_min + box.box_y_max) / 2);
      float dist = cv::norm(points_cam[i] - box_center);
      if (dist < min_dist && dist < max_dist_pixels_) {
        min_dist = dist;
        min_index = j;
      }
    }
    if (min_index != -1) points_bounding_boxes[min_index].push_back(i);
  }

  int bbox_id = -1;
  for (const auto &box : bounding_boxes) {
    ++bbox_id;
    auto box_width = box.box_x_max - box.box_x_min;
    auto box_height = box.box_y_max - box.box_y_min;

    std::vector<int> boxLimits{box.box_x_min, box.box_y_min, box.box_x_max, box.box_y_max};
    std::vector<int> bbox_limits{std::max(boxLimits[0], 0), std::max(boxLimits[1], 0),
                                 std::min(boxLimits[2], image_width_ - 1), std::min(boxLimits[3], image_height_ - 1)};

    // Skipping cones for which bounding box is close to the image bottom border
    if (box.box_y_max >= (image_height_ - image_bottom_border_)) {
      continue;
    }

    // Depth from yolo_camera_detector
    float distance_camera = box.depth;

    rclcpp::Time destination_time_stamp = time_stamp;
    cv::Mat MRH_to_base_link;

    // Get the correct motion compensated transformation
    if (!GetOpenCVTransform(lidar_frame_id_, base_link_frame_id_, destination_time_stamp, &MRH_to_base_link)) {
      RCLCPP_ERROR(this->get_logger(), "Unable to obtain transform in "
                                       "sensor_fusion_baseline for all points");
      return;
    }

    std::vector<cv::Point3f> cone_positions;
    std::vector<float> cone_distances;
    std::vector<unsigned int> indices_green;

    // For each point inside the bounding box transform to base_link frame and
    // store position
    for (const auto i : points_bounding_boxes[bbox_id]) {
      // transform point to a 4x1 vector
      float point_data_raw[] = {(*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z, 1.0};
      cv::Mat point_vector(4, 1, CV_32F, point_data_raw);
      // transform from MRH to egomotion
      cv::Mat point_vector_egomotion = MRH_to_base_link * point_vector;
      // transform result back into a proper opencv point
      auto p = cv::Point3f(point_vector_egomotion.at<float>(0, 0), point_vector_egomotion.at<float>(1, 0),
                           0); // AMZ lives in a 2D world
      auto distance_p = cv::norm(p);

      if (cv::abs(distance_p - distance_camera) < max_dist_camera_depth_) {
        // Keep point only if estimated distance is close to camera estimate
        if (debug_) {
          cv::circle(*image_mat, points_cam[i], 2, cv::viz::Color::green(), -1);
          indices_green.emplace_back(i);
        }
        cone_positions.push_back(p);
        cone_distances.push_back(cv::norm(p)); // store distance from egomotion to cone
      }
    }

    // Median Filtering
    if (cone_distances.size() >= min_lidar_points_) {
      std::vector<size_t> idx(cone_distances.size());
      iota(idx.begin(), idx.end(),
           0); // creates range of sequentially increasing numbers, starting
               // from 0
      std::stable_sort(idx.begin(), idx.end(),
                       [&cone_distances](size_t i1, size_t i2) { return cone_distances[i1] < cone_distances[i2]; });
      size_t median_index = idx[idx.size() / 2];
      float lidar_median_distance = cone_distances[median_index];
      if (debug_) {
        cv::putText(*image_mat, std::to_string(lidar_median_distance), cv::Point(bbox_limits[0], bbox_limits[1] - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
      }
      // Only publish cones within a min_cone_distance_ <= radius <=
      // max_cone_distance_
      if (lidar_median_distance >= min_cone_distance_ && lidar_median_distance <= max_cone_distance_) {
        // Averaging filter based on the median distance
        //-----------------------------------------------------------------------
        std::vector<cv::Point3f> cone_positions_to_average;
        // Accumulating the points close to the median distance
        for (const auto p : cone_positions) {
          if (abs(cv::norm(p) - lidar_median_distance) < averaging_depth_offset_) {
            cone_positions_to_average.push_back(p);
            if (debug_) {
              // Visualizing the points chosen to be averaged in a different
              // color
              auto index = std::find(cone_positions.begin(), cone_positions.end(), p) - cone_positions.begin();
              auto i = indices_green.at(index);
              cv::circle(*image_mat, points_cam[i], 2, cv::viz::Color::blue(), -1);
            }
          }
        }
        cv::Point3f sum = std::accumulate(cone_positions_to_average.begin(), cone_positions_to_average.end(),
                                          cv::Point3f(0.0f, 0.0f, 0.0f), std::plus<cv::Point3f>());
        cv::Point3f cone_position(sum * (1.0 / cone_positions_to_average.size())); // Divide by count to get mean
        //-----------------------------------------------------------------------

        // Create a cone message and add it to the array
        autonomous_msgs::msg::Cone cone = CreateConeMsg(cone_position, box);
        cone.id_cone = 0;
        cone_array->cones.push_back(cone);

        // New SF bbox topic with lidar depth (after the filters)
        if (debug_) {
          perception_msgs::msg::BoundingBoxDebug sf_box;
          sf_box.box_x_min = box.box_x_min;
          sf_box.box_x_max = box.box_x_max;
          sf_box.box_y_min = box.box_y_min;
          sf_box.box_y_max = box.box_y_max;
          sf_box.prob_type = box.prob_type;
          sf_box.prob_cone = box.prob_cone;
          sf_box.label = box.label;
          sf_box.x = cone_position.x;
          sf_box.y = cone_position.y;
          sf_box.depth = static_cast<float>(cv::norm(cone_position));
          sf_boxes.boxes.push_back(sf_box);
        }
      }
    }
  }
}

bool SensorFusion::GetOpenCVTransform(const std::string &src_frame, const std::string &dest_frame,
                                      const rclcpp::Time &time_stamp, cv::Mat *transform) {
  try {
    geometry_msgs::msg::TransformStamped tf_transform =
        tf_buffer_->lookupTransform(dest_frame, src_frame, time_stamp, std::chrono::milliseconds(20));
    auto rotation =
        tf2::Matrix3x3(tf2::Quaternion(tf_transform.transform.rotation.x, tf_transform.transform.rotation.y,
                                       tf_transform.transform.rotation.z, tf_transform.transform.rotation.w));
    float transform_matrix[] = {static_cast<float>(rotation[0].x()),
                                static_cast<float>(rotation[0].y()),
                                static_cast<float>(rotation[0].z()),
                                static_cast<float>(tf_transform.transform.translation.x),
                                static_cast<float>(rotation[1].x()),
                                static_cast<float>(rotation[1].y()),
                                static_cast<float>(rotation[1].z()),
                                static_cast<float>(tf_transform.transform.translation.y),
                                static_cast<float>(rotation[2].x()),
                                static_cast<float>(rotation[2].y()),
                                static_cast<float>(rotation[2].z()),
                                static_cast<float>(tf_transform.transform.translation.z),
                                0.0,
                                0.0,
                                0.0,
                                1.0};
    *transform = cv::Mat(4, 4, CV_32F, transform_matrix).clone();
    return true;
  } catch (tf2::TransformException &exception) {
    RCLCPP_ERROR(this->get_logger(), "Exception during transform lookup from [%s] to [%s]: %s", src_frame.c_str(),
                 dest_frame.c_str(), exception.what());
    return false;
  }
}

void SensorFusion::Tf2matrix(geometry_msgs::msg::TransformStamped tf, Eigen::Matrix4f &matrix) {
  Eigen::Quaterniond quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y,
                          tf.transform.rotation.z);
  Eigen::Matrix3f rotMat = quat.toRotationMatrix().cast<float>();
  matrix.col(3) << tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z, 1;
  matrix.block(0, 0, 3, 3) << rotMat;
  return;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SensorFusion::TransformCloud2CamStamp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                          const rclcpp::Time destination_time_stamp) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  geometry_msgs::msg::TransformStamped tf_to_cam;
  Eigen::Matrix4f tf_matrix;

  auto start = std::chrono::system_clock::now();
  try {
    tf_to_cam = tf_buffer_->lookupTransform(cloud->header.frame_id, destination_time_stamp, cloud->header.frame_id,
                                            pcl_conversions::fromPCL(cloud->header.stamp), reference_frame_id_);
  } catch (tf2::TransformException &exception) {
    RCLCPP_ERROR_STREAM(this->get_logger(), exception.what());
  }
  Tf2matrix(tf_to_cam, tf_matrix);
  pcl::transformPointCloud(*cloud, *transformed_cloud, tf_matrix);
  return transformed_cloud;
}

bool SensorFusion::TransformCones(const std::string &destination_frame, const rclcpp::Time destination_time,
                                  autonomous_msgs::msg::ConeArray &cone_array) {
  // TODO(Matteo): rewrite
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(destination_frame, destination_time, cone_array.header.frame_id,
                                            cone_array.header.stamp, reference_frame_id_);
  } catch (tf2::TransformException &exception) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't transform the cones to the "
                                     "lidar timestamp, reporting "
                                     "in the camera timestamp.");
    return false;
  }
  for (auto &cone : cone_array.cones) {
    geometry_msgs::msg::Point cone_pos, transformed_cone_pos;
    cone_pos.x = cone.position.x;
    cone_pos.y = cone.position.y;
    cone_pos.z = cone.position.z;
    tf2::doTransform(cone_pos, transformed_cone_pos, transform);
    cone.position.x = transformed_cone_pos.x;
    cone.position.y = transformed_cone_pos.y;
    cone.position.z = transformed_cone_pos.z;
  }
  cone_array.header.frame_id = destination_frame;
  cone_array.header.stamp = destination_time;
  return true;
}
void SensorFusion::PublishConeMarkers(const autonomous_msgs::msg::ConeArray &cone_array) {
  visualization_msgs::msg::MarkerArray markerArray;
  int markerId = 0;

  for (const auto &centroid : cone_array.cones) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_link_frame_id_; // Change to your point cloud's frame ID
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "cones";
    marker.id = markerId++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = centroid.position.x;
    marker.pose.position.y = centroid.position.y;
    marker.pose.position.z = centroid.position.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2; // Specify the size of the marker
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0; // Color the marker red
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // Make the marker opaque

    markerArray.markers.push_back(marker);
  }

  pub_cone_markers_->publish(markerArray);
}

std::string SensorFusion::GetPackageFilename(const std::string &filename) {
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

} // namespace sensor_fusion_baseline
