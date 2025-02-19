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
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>

#include <easy/profiler.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>

#include "utils.hpp"

class YOLODetector {
public:
  explicit YOLODetector(std::nullptr_t) {}
  YOLODetector(const std::string &modelPath, const bool &use_gpu, const bool &debug, const cv::Size &inputSize);
  std::vector<Detection> detect(cv::Mat &image, const cv::Size &inputSize, const float &confThreshold,
                                const float &iouThreshold, int &scaleFactor_, const int &brightness,
                                const int &contrast, const bool &enable_orange_, const bool &night_mode_);

private:
  Ort::Env env_{nullptr};
  Ort::SessionOptions sessionOptions_{nullptr};
  Ort::Session session_{nullptr};

  cv::Rect2f scaleCoords(const cv::Size &imageShape, cv::Rect2f coords, const cv::Size &imageOriginalShape,
                         bool p_Clip);
  cv::Mat preprocessing(cv::Mat &image, std::vector<int64_t> &inputTensorShape, int &scaleFactor_,
                        const int &brightness, const int &contrast, const bool &night_mode_);

  std::vector<Detection> postprocessing(const cv::Size &resizedImageShape, const cv::Size &originalImageShape,
                                        std::vector<Ort::Value> &outputTensors, const float &confThreshold,
                                        const float &iouThreshold, const bool enable_orange_);

  static void getBestClassInfo(const cv::Mat &p_Mat, const int &numClasses, float &bestConf, int &bestClassId);

  std::vector<const char *> inputNames;
  std::vector<const char *> outputNames;
  bool isDynamicInputShape{};
  cv::Size2f inputImageShape;
};
