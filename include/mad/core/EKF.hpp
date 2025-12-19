#pragma once

#include <memory>

#include "mad/core/Filter.hpp"
#include "mad/core/MadModel.hpp"

namespace mad {

class EKF : public IFilter {
public:
  EKF(std::shared_ptr<MadModel> model, int stateDim = 10);

  void predict(double dt) override;
  FilterOutput_t update(const FilterInput_t& input) override;
  void initialize(const Vector& initialState);

private:
  std::shared_ptr<MadModel> model;
  Vector state;
  Matrix covariance;
  Matrix processNoise;
  Matrix measurementNoise;
};

} // namespace mad




