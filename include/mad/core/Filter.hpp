#pragma once

#include "mad/core/Types.hpp"
#include "mad/data/Measurement.hpp"

namespace mad {

// Bundles measurement and time delta for a single filter step.
struct FilterInput_t {
  Measurement_t measurement;
  double dt = 0.0;
};

// Filter estimate at a single time step.
struct FilterOutput_t {
  Vector state;
  Matrix covariance;
};

// Common filter interface for the MAD pipeline.
class IFilter {
public:
  virtual ~IFilter() = default;
  virtual void predict(double dt) = 0;
  virtual FilterOutput_t update(const FilterInput_t& input) = 0;
};

} // namespace mad


