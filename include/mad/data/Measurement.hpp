#pragma once

#include <Eigen/Dense>

namespace mad {

// Core measurement packet used by filters and trackers.
struct Measurement_t {
  double time = 0.0;
  // Local ENU position (meters) relative to the first sensor sample.
  Eigen::Vector3d sensorPosEcef = Eigen::Vector3d::Zero();
  double magneticTfc = 0.0;
  // Local ENU position (meters) for the truth target track.
  Eigen::Vector3d truthPosEcef = Eigen::Vector3d::Zero();
  Eigen::VectorXd truthState;
  bool hasTruth = false;
};

} // namespace mad

