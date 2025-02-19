/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Christoforos Nicolaou <cnicolaou@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include <autonomous_msgs/msg/cone.hpp>

namespace slam {

/*
 * Lightweight data structure for storing a SLAM landmark
 */
class Landmark {
public:
  /*
   * Constructor
   */
  Landmark(const autonomous_msgs::msg::Cone &cone, int association_id, double range, double bearing)
      : cone_(cone), association_id_(association_id), range_(range), bearing_(bearing) {}

  // Getter for cone message of the landmark in the global frame
  autonomous_msgs::msg::Cone Cone() const { return cone_; }

  // Getter for x location of the landmark in the global frame
  double X() const { return cone_.position.x; }

  // Getter for y location of the landmark in the global frame
  double Y() const { return cone_.position.y; }

  // Getter for x-x covariance of the landmark
  double CovarianceXX() const { return cone_.position_covariance.x_x; }

  // Getter for y-y covariance of the landmark
  double CovarianceYY() const { return cone_.position_covariance.y_y; }

  // Getter for landmark id (i.e. cone ID used for data association)
  int AssociationId() const { return association_id_; }

  // Getter for distance in meters from the landmark to the car
  double Range() const { return range_; }

  // Getter for bearing in radians of the landmark from the car
  double Bearing() const { return bearing_; }

private:
  autonomous_msgs::msg::Cone cone_;
  int association_id_;
  double range_;
  double bearing_;
  bool corrupted_;
}; // end class Landmark

} // end namespace slam
