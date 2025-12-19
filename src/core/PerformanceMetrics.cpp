#include "mad/core/PerformanceMetrics.hpp"

#include <cmath>

namespace mad {

void PerformanceMetrics::update(double residual, double nis, double runtimeMs, double ess) {
  lastResidualValue = residual;
  lastNisValue = nis;
  lastRuntimeMsValue = runtimeMs;
  lastEssValue = ess;

  sumAbsResidual += std::abs(residual);
  sumSqResidual += residual * residual;
  ++sampleCount;
}

double PerformanceMetrics::meanAbsResidual() const {
  if (sampleCount <= 0) {
    return 0.0;
  }
  return sumAbsResidual / sampleCount;
}

double PerformanceMetrics::rmse() const {
  if (sampleCount <= 0) {
    return 0.0;
  }
  return std::sqrt(sumSqResidual / sampleCount);
}

} // namespace mad
