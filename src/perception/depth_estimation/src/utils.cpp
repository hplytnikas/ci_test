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

#include "utils.hpp"

#include <numeric>

namespace depth_estimation {

KeypointDetector::KeypointDetector(const std::string &weights_path, const bool &use_cuda = true) {
  env_ = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "keypoint_regression");
  sessionOptions_ = Ort::SessionOptions();

  if (use_cuda) {
    OrtCUDAProviderOptions cuda_options;
    sessionOptions_.AppendExecutionProvider_CUDA(cuda_options);
  }

  session_ = Ort::Session(env_, weights_path.c_str(), sessionOptions_);

  Ort::AllocatorWithDefaultOptions allocator;
  inputNames.push_back(session_.GetInputName(0, allocator));
  outputNames.push_back(session_.GetOutputName(0, allocator));

  // Checking input and output dimensions in ort session
  Ort::TypeInfo input_info = session_.GetInputTypeInfo(0);
  std::vector<int64_t> input_shape = input_info.GetTensorTypeAndShapeInfo().GetShape();

  if (input_shape.size() != 4) {
    RCLCPP_ERROR(rclcpp::get_logger("depth_estimation"), "Input tensor should have 4 dimensions");
  }

  Ort::TypeInfo output_info = session_.GetOutputTypeInfo(0);
  std::vector<int64_t> output_shape = output_info.GetTensorTypeAndShapeInfo().GetShape();

  if (output_shape.size() != 2 || output_shape[1] != 14) {
    RCLCPP_ERROR(rclcpp::get_logger("depth_estimation"), "Output tensor should have shape [1, 14]");
  }
}

cv::Mat KeypointDetector::PreprocessImage(cv::Mat &input_image) {
  cv::Mat image_resized;
  cv::resize(input_image, image_resized, cv::Size(80, 80));
  cv::Mat image;
  cv::cvtColor(image_resized, image, cv::COLOR_BGR2RGB);
  image.convertTo(image, CV_32FC3, 1.0 / 255.0);
  return image;
}

std::vector<float> KeypointDetector::PredictPoints(cv::Mat &input_image) {
  std::vector<int64_t> inputTensorShape{1, 3, 80, 80};
  cv::Mat preprocessedImage = PreprocessImage(input_image);
  if (preprocessedImage.rows != 80 || preprocessedImage.cols != 80) {
    RCLCPP_ERROR(rclcpp::get_logger("depth_estimation"), "Input image dimensions are wrong");
  }
  size_t inputTensorSize =
      std::accumulate(std::begin(inputTensorShape), std::end(inputTensorShape), 1, std::multiplies<>());

  std::vector<float> inputTensorValues(inputTensorSize);
  inputTensorValues.assign(preprocessedImage.begin<float>(), preprocessedImage.end<float>());
  std::vector<Ort::Value> inputTensors;

  Ort::MemoryInfo memoryInfo =
      Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

  inputTensors.push_back(Ort::Value::CreateTensor<float>(memoryInfo, inputTensorValues.data(), inputTensorSize,
                                                         inputTensorShape.data(), inputTensorShape.size()));

  std::vector<Ort::Value> outputTensors =
      session_.Run(Ort::RunOptions{nullptr}, inputNames.data(), inputTensors.data(), 1, outputNames.data(), 1);
  const float *output = outputTensors[0].GetTensorMutableData<float>();
  std::vector<float> keypoints(output, output + outputTensors[0].GetTensorTypeAndShapeInfo().GetShape()[1]);
  return keypoints;
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

} // namespace depth_estimation
