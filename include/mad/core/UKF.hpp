#pragma once

#include <memory>

#include "mad/core/Filter.hpp"
#include "mad/core/MadModel.hpp"

namespace mad {

class UKF : public IFilter {
public:
  UKF(std::shared_ptr<MadModel> model, int stateDim = 10);

  void predict(double dt) override;
  FilterOutput_t update(const FilterInput_t& input) override;
  void initialize(const Vector& initialState);

private:
  std::shared_ptr<MadModel> model;
  Vector state;
  Matrix covariance;
  Matrix processNoise;
  Matrix measurementNoise;
  double alpha = 1e-3;
  double beta = 2.0;
  double kappa = 0.0;
};

} // namespace mad




