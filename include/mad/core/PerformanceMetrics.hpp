#pragma once

namespace mad {

// Running measurement-based performance metrics (no truth required).
class PerformanceMetrics {
public:
  void update(double residual, double nis, double runtimeMs, double ess);

  double lastResidual() const { return lastResidualValue; }
  double lastNis() const { return lastNisValue; }
  double lastRuntimeMs() const { return lastRuntimeMsValue; }
  double lastEss() const { return lastEssValue; }

  double meanAbsResidual() const;
  double rmse() const;

private:
  double sumAbsResidual = 0.0;
  double sumSqResidual = 0.0;
  double lastResidualValue = 0.0;
  double lastNisValue = 0.0;
  double lastRuntimeMsValue = 0.0;
  double lastEssValue = 0.0;
  int sampleCount = 0;
};

} // namespace mad
