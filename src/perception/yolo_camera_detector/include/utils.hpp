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
#include <perception_msgs/msg/bounding_box.hpp>
#include <perception_msgs/msg/box_array.hpp>

#include <codecvt>
#include <easy/profiler.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct Detection {
  cv::Rect box;
  float conf{};
  int classId{};
  float y_conf{};
  float b_conf{};
  float os_conf{};
  float ob_conf{};
};

namespace utils {
void drawBoundingBoxes(cv::Mat &image, std::vector<Detection> &detections, bool enable_orange_);
std::vector<std::string> loadNames(const std::string &path);
} // namespace utils
