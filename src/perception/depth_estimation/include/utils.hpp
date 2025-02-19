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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <onnxruntime_cxx_api.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <easy/profiler.h>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace depth_estimation {

// Class that runs inference of ResNet using ONNX runtime API
class KeypointDetector {
public:
  explicit KeypointDetector(std::nullptr_t) {}
  KeypointDetector(const std::string &weights_path, const bool &use_cuda);
  std::vector<float> PredictPoints(cv::Mat &input_image);
  cv::Mat PreprocessImage(cv::Mat &input_image);

private:
  Ort::Env env_{nullptr};
  Ort::SessionOptions sessionOptions_{nullptr};
  Ort::Session session_{nullptr};

  std::vector<const char *> inputNames;
  std::vector<const char *> outputNames;
};

std::string GetPackageFilename(const std::string &filename);

} // namespace depth_estimation
