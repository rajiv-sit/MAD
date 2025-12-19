#include "mad/core/EKF.hpp"

namespace mad {

EKF::EKF(int stateDim)
    : state(Vector::Zero(stateDim)),
      covariance(Matrix::Identity(stateDim, stateDim)),
      processNoise(Matrix::Identity(stateDim, stateDim) * 1e-3),
      measurementNoise(Matrix::Identity(stateDim, stateDim) * 1e-2) {}

void EKF::predict(double /*dt*/) {
  covariance = covariance + processNoise;
}

FilterOutput_t EKF::update(const FilterInput_t& input) {
  if (state.size() >= 3) {
    state(0) = input.measurement.sensorPosEcef.x();
    state(1) = input.measurement.sensorPosEcef.y();
    state(2) = input.measurement.sensorPosEcef.z();
  }
  if (state.size() >= 4) {
    state(3) = input.measurement.magneticTfc;
  }

  covariance = covariance - measurementNoise * 0.1;

  return FilterOutput_t{state, covariance};
}

} // namespace mad




