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

#include <eigen3/Eigen/Dense>

void utils::drawBoundingBoxes(cv::Mat &image, std::vector<Detection> &detections, bool enable_orange_) {
  for (int i = 0; i < detections.size(); i++) {
    cv::Point pt;
    if (detections[i].classId == 0) {
      cv::rectangle(image, detections[i].box, cv::Scalar(255, 255, 0), 4, 8,
                    0); // blue cone BGR
      cv::circle(image, cv::Point(detections[i].box.x, detections[i].box.y), 5, cv::Scalar(255, 0, 0), 10, 0, 0);
    }
    if (detections[i].classId == 1) {
      cv::rectangle(image, detections[i].box, cv::Scalar(0, 255, 255), 4, 8,
                    0); // yellow cone BGR
      cv::circle(image, cv::Point(detections[i].box.x, detections[i].box.y), 5, cv::Scalar(255, 0, 0), 10, 0, 0);
    }
    if (enable_orange_) {
      if (detections[i].classId == 2) {
        cv::rectangle(image, detections[i].box, cv::Scalar(0, 0, 255), 4, 8,
                      0); // large orange cone BGR
        cv::circle(image, cv::Point(detections[i].box.x, detections[i].box.y), 5, cv::Scalar(255, 0, 0), 10, 0, 0);
      }
      if (detections[i].classId == 3) {
        cv::rectangle(image, detections[i].box, cv::Scalar(0, 165, 255), 4, 8,
                      0); // small orange cone BGR
        cv::circle(image, cv::Point(detections[i].box.x, detections[i].box.y), 5, cv::Scalar(255, 0, 0), 10, 0, 0);
      }
    }
  }
}

std::vector<std::string> utils::loadNames(const std::string &path) {
  // load class names
  std::vector<std::string> classNames;
  std::ifstream infile(path);
  if (infile.good()) {
    std::string line;
    while (getline(infile, line)) {
      if (line.back() == '\r') {
        line.pop_back();
      }
      classNames.emplace_back(line);
    }
    infile.close();
  } else {
    std::cerr << "ERROR: Failed to access class name path: " << path << std::endl;
  }

  return classNames;
}
