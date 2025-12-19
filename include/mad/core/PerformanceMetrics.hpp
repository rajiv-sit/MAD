#pragma once

#include "mad/core/Types.hpp"

namespace mad {

// Running measurement-based performance metrics (no truth required).
class PerformanceMetrics {
public:
  void update(double residual, double nis, double runtimeMs, double ess);
  void updateTruth(const Vector& estimate, const Matrix& covariance, const Vector& truth);

  double lastResidual() const { return lastResidualValue; }
  double lastNis() const { return lastNisValue; }
  double lastRuntimeMs() const { return lastRuntimeMsValue; }
  double lastEss() const { return lastEssValue; }
  double lastTruthRmse() const { return lastTruthRmseValue; }
  double lastTruthNees() const { return lastTruthNeesValue; }

  double meanAbsResidual() const;
  double rmse() const;
  double truthRmse() const;
  double meanTruthNees() const;

private:
  double sumAbsResidual = 0.0;
  double sumSqResidual = 0.0;
  double lastResidualValue = 0.0;
  double lastNisValue = 0.0;
  double lastRuntimeMsValue = 0.0;
  double lastEssValue = 0.0;
  int sampleCount = 0;

  double sumSqTruthPos = 0.0;
  double sumTruthNees = 0.0;
  double lastTruthRmseValue = 0.0;
  double lastTruthNeesValue = 0.0;
  int truthSampleCount = 0;
};

} // namespace mad
