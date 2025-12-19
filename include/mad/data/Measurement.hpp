#pragma once

#include <Eigen/Dense>

namespace mad {

// Core measurement packet used by filters and trackers.
struct Measurement_t {
  double time = 0.0;
  Eigen::Vector3d sensorPosEcef = Eigen::Vector3d::Zero();
  double magneticTfc = 0.0;
};

} // namespace mad

