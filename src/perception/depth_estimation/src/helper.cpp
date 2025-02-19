/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - Matteo Mazzonelli     <m.mazzonelli@gmail.com>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "depth_estimation.hpp"

namespace depth_estimation {
void DepthEstimation::loadParameters() {
  this->declare_parameter<std::string>("topics.pub.bbox_depth", "/perception/depth_estimation/bbox_depth");
  this->declare_parameter<std::string>("topics.pub.cone_array", "/perception/camera/cone_array");

  this->declare_parameter<std::string>("topics.sub.fw_bbox", "/perception/yolo_camera_detector/forward_bbox");
  this->declare_parameter<std::string>("topics.pub.debug.img_overlay",
                                       "/perception/depth_estimation/debug/img_overlay");
  this->declare_parameter<std::string>("topics.sub.fw_cam", "/sensors/forward_camera/image_color");

  this->declare_parameter<std::string>("intrinsics_file",
                                       "package://camera_launcher/calibration/forward_camera_castor.yaml");

  // Pipeline Status Parameters
  this->declare_parameter<std::string>("weights_path", "/.amz/250_euler_13k.onnx");
  this->declare_parameter<bool>("status.cone_array", false);
  this->declare_parameter<bool>("use_gpu", false);
  this->declare_parameter<bool>("debug", true);
  this->declare_parameter<bool>("mde_on", true);

  // Frame IDs parameters
  this->declare_parameter<std::string>("frame_ids.camera", "pylon_camera");
  this->declare_parameter<std::string>("frame_ids.base_link", "base_link");
  this->declare_parameter<std::string>("car_mode", "dufour");

  // Image and cone parameters
  this->declare_parameter<int>("image_width", 2592);
  this->declare_parameter<int>("image_height", 352);
  this->declare_parameter<float>("small_cone_width", 0.228);
  this->declare_parameter<float>("small_cone_height", 0.325);
  this->declare_parameter<float>("small_cone_width_mde", 0.228);
  this->declare_parameter<float>("small_cone_height_mde", 0.325);
  this->declare_parameter<int>("distance_threshold", 10);
  this->declare_parameter<float>("big_cone_width", 0.285);
  this->declare_parameter<float>("big_cone_height", 0.505);
  this->declare_parameter<float>("big_cone_width_mde", 0.285);
  this->declare_parameter<float>("big_cone_height_mde", 0.505);
  this->declare_parameter<float>("scaling", 1.0);

  this->get_parameter("topics.pub.bbox_depth", topic_bbox_depth_);
  this->get_parameter("topics.pub.cone_array", topic_cone_array_);
  this->get_parameter("topics.sub.fw_bbox", topic_fw_bbox_);
  this->get_parameter("topics.pub.debug.img_overlay", topic_img_overlay_);
  this->get_parameter("topics.sub.fw_cam", topic_fw_cam_);

  this->get_parameter("intrinsics_file", intrinsics_file_);

  weights_path_ = std::string(std::getenv("HOME")) + this->get_parameter("weights_path").as_string();
  this->get_parameter("status.cone_array", status_cone_array_);
  this->get_parameter("use_gpu", use_gpu_);
  this->get_parameter("debug", debug_);
  this->get_parameter("mde_on", mde_on_);

  this->get_parameter("frame_ids.camera", camera_frame_id_);
  this->get_parameter("frame_ids.base_link", base_link_frame_id_);

  std::string car_mode_;
  this->get_parameter("car_mode", car_mode_);
  std::string car_name = "CAR_NAME";
  size_t pos = intrinsics_file_.find(car_name);
  if (pos != std::string::npos) intrinsics_file_.replace(pos, car_name.length(), car_mode_);

  this->get_parameter("image_width", image_width_);
  this->get_parameter("image_height", image_height_);
  this->get_parameter("small_cone_width", small_cone_width_);
  this->get_parameter("small_cone_height", small_cone_height_);
  this->get_parameter("small_cone_width_mde", small_cone_width_mde_);
  this->get_parameter("small_cone_height_mde", small_cone_height_mde_);
  this->get_parameter("distance_threshold", distance_threshold_);
  this->get_parameter("big_cone_width", big_cone_width_);
  this->get_parameter("big_cone_height", big_cone_height_);
  this->get_parameter("big_cone_width_mde", big_cone_width_mde_);
  this->get_parameter("big_cone_height_mde", big_cone_height_mde_);
  this->get_parameter("scaling", scaling_);
}

void DepthEstimation::loadCalibrationParameters() {
  try {
    std::string resolved_filename = GetPackageFilename(intrinsics_file_);
    YAML::Node intrinsics = YAML::LoadFile(resolved_filename);
    const std::vector<float> &data = intrinsics["camera_matrix"]["data"].as<std::vector<float>>();
    std::memcpy(K_mat_.data, data.data(), 9 * sizeof(float));
    const std::vector<float> &distortion = intrinsics["distortion_coefficients"]["data"].as<std::vector<float>>();
    std::memcpy(D_mat_.data, distortion.data(), 14 * sizeof(float));
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded intrinsics from " << intrinsics_file_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load intrinsics -> " << e.what());
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  try {
    geometry_msgs::msg::TransformStamped tf_transform = tf_buffer_->lookupTransform(
        base_link_frame_id_, camera_frame_id_, tf2::TimePointZero, std::chrono::seconds(10));
    auto rotation =
        tf2::Matrix3x3(tf2::Quaternion(tf_transform.transform.rotation.x, tf_transform.transform.rotation.y,
                                       tf_transform.transform.rotation.z, tf_transform.transform.rotation.w));
    double transform_matrix[] = {static_cast<double>(rotation[0].x()),
                                 static_cast<double>(rotation[0].y()),
                                 static_cast<double>(rotation[0].z()),
                                 static_cast<double>(tf_transform.transform.translation.x),
                                 static_cast<double>(rotation[1].x()),
                                 static_cast<double>(rotation[1].y()),
                                 static_cast<double>(rotation[1].z()),
                                 static_cast<double>(tf_transform.transform.translation.y),
                                 static_cast<double>(rotation[2].x()),
                                 static_cast<double>(rotation[2].y()),
                                 static_cast<double>(rotation[2].z()),
                                 static_cast<double>(tf_transform.transform.translation.z),
                                 0.0,
                                 0.0,
                                 0.0,
                                 1.0};
    transform_camera_to_base_ = cv::Mat(4, 4, CV_64F, transform_matrix).clone();
  } catch (tf2::TransformException &exception) {
    RCLCPP_ERROR(this->get_logger(), "Exception during transform lookup from [%s] to [%s]: %s",
                 camera_frame_id_.c_str(), base_link_frame_id_.c_str(), exception.what());
  }
}

void DepthEstimation::advertiseTopics() {
  pub_bbox_depth_ = this->create_publisher<perception_msgs::msg::BoxArray>(topic_bbox_depth_, 1);
  pub_cone_array_ = this->create_publisher<autonomous_msgs::msg::ConeArray>(topic_cone_array_, 1);
  pub_img_overlay_ = this->create_publisher<sensor_msgs::msg::Image>(topic_img_overlay_, 10);
  pub_cone_markers_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/camera/cone_markers", 1);
}

void DepthEstimation::subscribeTopics() {
  sub_bbox_.subscribe(this, topic_fw_bbox_);
  sub_image_.subscribe(this, topic_fw_cam_);
  sync_.reset(new Sync(SyncPolicy(20), sub_bbox_, sub_image_));

  sync_->registerCallback(&depth_estimation::DepthEstimation::BboxDepth, this);
}

void DepthEstimation::FillCones(const std::vector<perception_msgs::msg::BoundingBox> &boxes,
                                const std::vector<Pose> &pose_list, autonomous_msgs::msg::ConeArray &cones) {
  for (size_t j = 0; j < boxes.size(); j++) {
    auto box = boxes[j];
    auto pose = pose_list[j];
    int cone_counter = 0;

    // Create cone
    autonomous_msgs::msg::Cone cone;
    cone.id_cone = cone_counter;
    cone.prob_cone = box.prob_cone;
    cone.is_observed = true;
    cone.position.x = pose.first;
    cone.position.y = pose.second;
    cone.position.z = 0;

    cone.prob_type.blue = box.prob_type.blue;
    cone.prob_type.yellow = box.prob_type.yellow;
    cone.prob_type.orange = box.prob_type.orange;
    cone.prob_type.orange_big = box.prob_type.orange_big;
    cone.pipeline = 1;

    cones.cones.push_back(cone);
    cone_counter++;
  }
}

void DepthEstimation::DepthFromBoxHeight(const std::vector<int> &bounding_box, Pose &pose, double &distance,
                                         const bool is_big = false) {
  // EASY_FUNCTION(profiler::colors::Blue);
  float cone_depth;
  float box_width = bounding_box[2] - bounding_box[0];
  float box_height = bounding_box[3] - bounding_box[1];

  if (is_big) {
    cone_depth = (K_mat_.at<float>(1, 1) * big_cone_height_) / static_cast<float>(box_height);
  } else {
    cone_depth = (K_mat_.at<float>(1, 1) * small_cone_height_) / static_cast<float>(box_height);
  }

  float mid_box_x = static_cast<float>(box_width / 2.0 + bounding_box[0]);
  float xpos_mid_offset = mid_box_x - static_cast<float>(K_mat_.at<float>(0, 2));

  // x, z in camera's coordinate frame
  double x =
      static_cast<double>(xpos_mid_offset * cone_depth / K_mat_.at<float>(1, 1)); // 4 corresponds to focal_length_y
  double z = static_cast<double>(cone_depth);
  cv::Mat coords_cam = (cv::Mat_<double>(4, 1) << x, 0.0, z, 1.0);
  cv::Mat coords_base = transform_camera_to_base_ * coords_cam;
  distance = static_cast<double>(cone_depth);
  pose = std::make_pair(coords_base.at<double>(0, 0), coords_base.at<double>(1, 0));
}
} // namespace depth_estimation
