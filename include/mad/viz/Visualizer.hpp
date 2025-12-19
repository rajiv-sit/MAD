#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "mad/core/Types.hpp"
#include "mad/data/Measurement.hpp"

namespace mad {

struct FilterPanelEntry_t {
  bool hasEstimate = false;
  Vector detectionState;
  double measuredMagnetic = 0.0;
  double predictedMagnetic = 0.0;
  double residual = 0.0;
  double meanAbsResidual = 0.0;
  double rmse = 0.0;
  double nis = 0.0;
  double runtimeMs = 0.0;
  double ess = 0.0;
};

struct VisualizerState_t {
  bool hasMeasurement = false;
  Measurement_t measurement;
  std::vector<float> measurementHistory;
  std::vector<float> timeHistory;
  std::unordered_map<std::string, FilterPanelEntry_t> filterEntries;
  int maxHistory = 300;
};

// ImGui-based visualization wrapper.
class Visualizer {
public:
  void start();
  void stop();
  void render();

  void setFilterNames(const std::vector<std::string>& filterNames);
  void updateMeasurement(const Measurement_t& measurement);
  void updateFilterEntry(const std::string& name,
                         const Vector& detectionState,
                         double measuredMagnetic,
                         double predictedMagnetic,
                         double residual,
                         double meanAbsResidual,
                         double rmse,
                         double nis,
                         double runtimeMs,
                         double ess);

private:
  VisualizerState_t state;
};

} // namespace mad
