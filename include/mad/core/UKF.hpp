#pragma once

#include "mad/core/Filter.hpp"

namespace mad {

class UKF : public IFilter {
public:
  explicit UKF(int stateDim = 10);

  void predict(double dt) override;
  FilterOutput_t update(const FilterInput_t& input) override;

private:
  Vector state;
  Matrix covariance;
  Matrix processNoise;
  Matrix measurementNoise;
};

} // namespace mad




