/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023 Authors:
 *   - Jonas Grütter <jgruette@ethz.ch>
 *   - Bartosz Mila <bamila@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include <string>

namespace utils {
/**
 * Prints and returns the absolute path of selected ROS package
 */
std::string getRosPackageDir(const std::string &packageName);
} // namespace utils
