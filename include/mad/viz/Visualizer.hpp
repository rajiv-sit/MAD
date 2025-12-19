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
  double truthRmse = 0.0;
  double truthNees = 0.0;
  double runtimeMs = 0.0;
  double ess = 0.0;
};

struct MagneticMapPoint_t {
  float x = 0.0f;
  float y = 0.0f;
  float value = 0.0f;
};

struct TrackPoint_t {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct VisualizerState_t {
  bool hasMeasurement = false;
  Measurement_t measurement;
  std::vector<float> measurementHistory;
  std::vector<float> timeHistory;
  std::vector<MagneticMapPoint_t> measuredMap;
  std::unordered_map<std::string, std::vector<MagneticMapPoint_t>> predictedMap;
  std::string mapFilterName;
  std::string trackerFilterName;
  float truthBaseline = 0.0f;
  Vector earthField = Vector::Zero(3);
  bool showMeasuredMap = true;
  bool showPredictedMap = true;
  bool showResidualMap = true;
  bool showTruthMap = true;
  bool showTrajectory2d = true;
  bool showTrajectory3d = false;
  std::vector<TrackPoint_t> sensorTrack;
  std::vector<TrackPoint_t> targetTrack;
  std::unordered_map<std::string, std::vector<TrackPoint_t>> estimatedTracks;
  std::unordered_map<std::string, FilterPanelEntry_t> filterEntries;
  struct MetricHistory {
    std::vector<float> rmse;
    std::vector<float> nees;
    std::vector<float> residual;
  };
  std::unordered_map<std::string, MetricHistory> metricHistories;
  int maxHistory = 300;
  bool performancePanelEnabled = true;
  bool showRmseGraph = true;
  bool showNeesGraph = true;
  bool showResidualGraph = true;
};

// ImGui-based visualization wrapper.
class Visualizer {
public:
  void start();
  void stop();
  void render();

  void setFilterNames(const std::vector<std::string>& filterNames);
  void setMapFilterName(const std::string& filterName);
  void setTrackerFilterName(const std::string& filterName);
  void setTruthBaseline(float valueTesla);
  void setEarthField(const Vector& earthFieldTesla);
  void setMapOptions(bool showMeasured, bool showPredicted, bool showResidual, bool showTruth);
  void setTrajectoryOptions(bool show2d, bool show3d);
  void setPerformancePanelEnabled(bool enabled);
  void setShowRmseGraph(bool show);
  void setShowNeesGraph(bool show);
  void setShowResidualGraph(bool show);
  void updateMeasurement(const Measurement_t& measurement);
  void updateFilterEntry(const std::string& name,
                         const Vector& detectionState,
                         double measuredMagnetic,
                         double predictedMagnetic,
                         double residual,
                         double meanAbsResidual,
                         double rmse,
                         double nis,
                         double truthRmse,
                         double truthNees,
                         double runtimeMs,
                         double ess);

private:
  VisualizerState_t state;
};

} // namespace mad
