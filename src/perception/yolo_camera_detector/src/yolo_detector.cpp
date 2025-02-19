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

#include "yolo_detector.hpp"

#include <algorithm>
#include <iostream>
#include <numeric>

YOLODetector::YOLODetector(const std::string &modelPath, const bool &isGPU, const bool &debug,
                           const cv::Size &inputSize) {
  env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "ONNX_DETECTION");
  sessionOptions_ = Ort::SessionOptions();

  std::vector<std::string> availableProviders = Ort::GetAvailableProviders();
  for (auto provider : availableProviders) {
    if (debug) {
      RCLCPP_INFO(rclcpp::get_logger("yolo_camera_detector"), "Available provider: %s", provider.c_str());
    }
  }
  auto cudaAvailable = std::find(availableProviders.begin(), availableProviders.end(), "CUDAExecutionProvider");
  OrtCUDAProviderOptions cudaOption;
  if (isGPU && (cudaAvailable == availableProviders.end())) {
    if (debug) {
      RCLCPP_INFO(rclcpp::get_logger("yolo_camera_detector"),
                  "GPU is not supported by your ONNXRuntime build. Fallback to CPU.");
    }
    if (debug) {
      RCLCPP_INFO(rclcpp::get_logger("yolo_camera_detector"), "Inference device: CPU");
    }
  } else if (isGPU && (cudaAvailable != availableProviders.end())) {
    if (debug) {
      RCLCPP_INFO(rclcpp::get_logger("yolo_camera_detector"), "Inference device: GPU");
    }
    sessionOptions_.AppendExecutionProvider_CUDA(cudaOption);
  } else if (debug) {
    RCLCPP_INFO(rclcpp::get_logger("yolo_camera_detector"), "Inference device: CPU");
  }

  session_ = Ort::Session(env_, modelPath.c_str(), sessionOptions_);

  Ort::AllocatorWithDefaultOptions allocator;

  Ort::TypeInfo inputTypeInfo = session_.GetInputTypeInfo(0);
  std::vector<int64_t> inputTensorShape = inputTypeInfo.GetTensorTypeAndShapeInfo().GetShape();
  this->isDynamicInputShape = false;
  // checking if width and height are dynamic
  if (inputTensorShape[2] == -1 && inputTensorShape[3] == -1) {
    if (debug) {
      RCLCPP_INFO(rclcpp::get_logger("yolo_camera_detector"), "Dynamic input shape");
    }
    this->isDynamicInputShape = true;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("yolo_camera_detector"),
                 "Yolo onnx model not exported with dynamic input. Check model!");
    rclcpp::shutdown();
  }

  for (auto shape : inputTensorShape) {
    if (debug) {
      RCLCPP_INFO(rclcpp::get_logger("yolo_camera_detector"), "Input shape: %ld", shape);
    }
  }

  // inputNames.push_back(session_.GetInputNameAllocated(0, allocator).get());
  // outputNames.push_back(session_.GetOutputNameAllocated(0, allocator).get());
  inputNames.push_back(session_.GetInputName(0, allocator));
  outputNames.push_back(session_.GetOutputName(0, allocator));

  this->inputImageShape = cv::Size2f(inputSize);
}

cv::Rect2f YOLODetector::scaleCoords(const cv::Size &imageShape, cv::Rect2f coords, const cv::Size &imageOriginalShape,
                                     bool p_Clip) {
  cv::Rect2f l_Result;
  float gain = std::min(imageShape.height / imageOriginalShape.height, imageShape.width / imageOriginalShape.width);

  int pad[2] = {static_cast<int>(std::round(((imageShape.width - imageOriginalShape.width * gain) / 2.0f) - 0.1f)),
                static_cast<int>(std::round(((imageShape.height - imageOriginalShape.height * gain) / 2.0f) - 0.1f))};

  l_Result.x = static_cast<int>(std::round((coords.x - pad[0]) / gain));
  l_Result.y = static_cast<int>(std::round((coords.y - pad[1]) / gain));

  l_Result.width = static_cast<int>(std::round(coords.width / gain));
  l_Result.height = static_cast<int>(std::round(coords.height / gain));

  // // clip coords, should be modified for width and height
  if (p_Clip) {
    l_Result.x = std::max(0.0f, std::min(l_Result.x, static_cast<float>(imageOriginalShape.width)));
    l_Result.y = std::max(0.0f, std::min(l_Result.y, static_cast<float>(imageOriginalShape.height)));
    l_Result.width =
        std::max(0.0f, std::min(l_Result.width, static_cast<float>(imageOriginalShape.width - l_Result.x)));
    l_Result.height =
        std::max(0.0f, std::min(l_Result.height, static_cast<float>(imageOriginalShape.height - l_Result.y)));
  }
  return l_Result;
}

void YOLODetector::getBestClassInfo(const cv::Mat &p_Mat, const int &numClasses, float &bestConf, int &bestClassId) {
  bestClassId = 0;
  bestConf = 0;

  if (p_Mat.rows && p_Mat.cols) {
    for (int i = 0; i < numClasses; i++) {
      if (p_Mat.at<float>(0, i + 4) > bestConf) {
        bestConf = p_Mat.at<float>(0, i + 4);
        bestClassId = i;
      }
    }
  }
}

cv::Mat YOLODetector::preprocessing(cv::Mat &image, std::vector<int64_t> &inputTensorShape, int &scaleFactor,
                                    const int &brightness, const int &contrast, const bool &night_mode_) {
  EASY_FUNCTION(profiler::colors::Blue);
  cv::Mat brightenedImage, resizedImage, RGBImage, floatImage, preprocessedImage;

  if (night_mode_) {
    image.convertTo(brightenedImage, -1, contrast, brightness);
    cv::resize(brightenedImage, resizedImage, cv::Size(inputTensorShape[2], inputTensorShape[3]),
               cv::InterpolationFlags::INTER_CUBIC);
  } else {
    cv::resize(image, resizedImage, cv::Size(inputTensorShape[2], inputTensorShape[3]),
               cv::InterpolationFlags::INTER_CUBIC);
  }

  cv::cvtColor(resizedImage, RGBImage, cv::COLOR_BGR2RGB);

  // ToDo - Implement input image downsampling using scaleFactor variable
  // input image width has to be multiple of max stride 32
  inputTensorShape[2] = RGBImage.rows;
  inputTensorShape[3] = RGBImage.cols;

  RGBImage.convertTo(floatImage, CV_32FC3, 1 / 255.0);

  // Performs mean subtraction, normalization and channel swapping HWC to CHW
  cv::dnn::blobFromImage(floatImage, preprocessedImage);

  return preprocessedImage;
}

std::vector<Detection> YOLODetector::postprocessing(const cv::Size &resizedImageShape,
                                                    const cv::Size &originalImageShape,
                                                    std::vector<Ort::Value> &outputTensors, const float &confThreshold,
                                                    const float &iouThreshold, const bool enable_orange_) {
  EASY_FUNCTION(profiler::colors::Orange);
  std::vector<cv::Rect> boxes;
  std::vector<cv::Rect> nms_boxes;
  std::vector<float> confs;
  std::vector<int> classIds;
  std::vector<float> y_confs;
  std::vector<float> b_confs;
  std::vector<float> os_confs;
  std::vector<float> ob_confs;

  auto *rawOutput = outputTensors[0].GetTensorData<float>();
  std::vector<int64_t> outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
  size_t count = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

  cv::Mat l_Mat =
      cv::Mat(outputShape[1], outputShape[2], CV_32FC1, const_cast<void *>(reinterpret_cast<const void *>(rawOutput)));
  cv::Mat l_Mat_t = l_Mat.t();

  // for yolov8, first 4 elements are centerX, centerY, width and height. Next n
  // elements are confidences of the n classes.
  int numClasses = l_Mat_t.cols - 4;

  // only for batch size = 1
  for (int l_Row = 0; l_Row < l_Mat_t.rows; l_Row++) {
    cv::Mat l_MatRow = l_Mat_t.row(l_Row);
    float objConf;
    int classId;

    this->getBestClassInfo(l_MatRow, numClasses, objConf, classId);

    if (objConf > confThreshold) {
      float centerX = (l_MatRow.at<float>(0, 0));
      float centerY = (l_MatRow.at<float>(0, 1));
      float width = (l_MatRow.at<float>(0, 2));
      float height = (l_MatRow.at<float>(0, 3));
      float left = centerX - width / 2;
      float top = centerY - height / 2;

      float confidence = objConf;
      cv::Rect2f l_Scaled =
          scaleCoords(resizedImageShape, cv::Rect2f(left, top, width, height), originalImageShape, true);

      // Prepare NMS filtered per class id's
      nms_boxes.emplace_back(static_cast<int>(std::round(l_Scaled.x)) + classId * 7680,
                             static_cast<int>(std::round(l_Scaled.y)) + classId * 7680,
                             static_cast<int>(std::round(l_Scaled.width)),
                             static_cast<int>(std::round(l_Scaled.height)));
      boxes.emplace_back(static_cast<int>(std::round(l_Scaled.x)), static_cast<int>(std::round(l_Scaled.y)),
                         static_cast<int>(std::round(l_Scaled.width)), static_cast<int>(std::round(l_Scaled.height)));
      // boxes.emplace_back(left, top, width, height);
      confs.emplace_back(confidence);
      classIds.emplace_back(classId);

      b_confs.emplace_back(l_MatRow.at<float>(0, 4));
      y_confs.emplace_back(l_MatRow.at<float>(0, 5));

      if (enable_orange_) {
        if (numClasses >= 3) {
          os_confs.emplace_back(l_MatRow.at<float>(0, 6));
        } else {
          os_confs.emplace_back(0);
        }
        if (numClasses == 4) {
          ob_confs.emplace_back(l_MatRow.at<float>(0, 7)); // 6
        } else {
          ob_confs.emplace_back(0);
        }
      } else {
        os_confs.emplace_back(0);
        ob_confs.emplace_back(0);
      }
    }
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(nms_boxes, confs, confThreshold, iouThreshold, indices);

  std::vector<Detection> detections;

  for (int idx : indices) {
    double minSource = 0.25;
    double maxSource = 1.0;
    double minTarget = 0.75;
    double maxTarget = 1.0;

    double maxElement = std::max({y_confs[idx], b_confs[idx], os_confs[idx], ob_confs[idx]});
    double sum = y_confs[idx] + b_confs[idx] + os_confs[idx] + ob_confs[idx];
    double normalizedValue = ((maxElement - minSource) / (maxSource - minSource)) * (maxTarget - minTarget) + minTarget;

    y_confs[idx] /= sum;
    b_confs[idx] /= sum;
    os_confs[idx] /= sum;
    ob_confs[idx] /= sum;

    Detection det;
    det.box = cv::Rect(boxes[idx]);
    det.conf = normalizedValue;
    det.classId = classIds[idx];
    det.y_conf = y_confs[idx];
    det.b_conf = b_confs[idx];
    det.os_conf = os_confs[idx];
    det.ob_conf = ob_confs[idx];
    detections.emplace_back(det);
  }

  return detections;
}

std::vector<Detection> YOLODetector::detect(cv::Mat &image, const cv::Size &inputSize, const float &confThreshold,
                                            const float &iouThreshold, int &scaleFactor_, const int &brightness,
                                            const int &contrast, const bool &enable_orange_, const bool &night_mode_) {
  EASY_FUNCTION(profiler::colors::Green);

  std::vector<int64_t> inputTensorShape{1, 3, inputSize.width, inputSize.height};

  cv::Mat preprocessedImage = preprocessing(image, inputTensorShape, scaleFactor_, brightness, contrast, night_mode_);

  // Multiplies 1*3*width*height
  size_t inputTensorSize =
      std::accumulate(std::begin(inputTensorShape), std::end(inputTensorShape), 1, std::multiplies<>());

  std::vector<float> inputTensorValues(inputTensorSize);

  inputTensorValues.assign(preprocessedImage.begin<float>(), preprocessedImage.end<float>());

  std::vector<Ort::Value> inputTensors;
  // Creates the CPU space where the input tensor will be allocated before being
  // transferred to the GPU for inference
  Ort::MemoryInfo memoryInfo =
      Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

  inputTensors.push_back(Ort::Value::CreateTensor<float>(memoryInfo, inputTensorValues.data(), inputTensorSize,
                                                         inputTensorShape.data(), inputTensorShape.size()));
  // Inference on GPU (session_ points to sessionOptions_ that sets CUDA to be
  // the inference excecution provider)
  // std::vector<Ort::Value> outputTensors = session_.Run()

  std::vector<Ort::Value> outputTensors =
      session_.Run(Ort::RunOptions{nullptr}, inputNames.data(), inputTensors.data(), 1, outputNames.data(), 1);

  cv::Size resizedShape = cv::Size(static_cast<int>(inputTensorShape[3]), static_cast<int>(inputTensorShape[2]));

  std::vector<Detection> result =
      this->postprocessing(resizedShape, image.size(), outputTensors, confThreshold, iouThreshold, enable_orange_);

  return result;
}
