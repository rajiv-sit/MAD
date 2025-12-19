#include "mad/viz/Visualizer.hpp"

#include <cfloat>

#include <imgui.h>

namespace mad {

void Visualizer::start() {}
void Visualizer::stop() {}

void Visualizer::setFilterNames(const std::vector<std::string>& filterNames) {
  state.filterEntries.clear();
  for (const auto& name : filterNames) {
    state.filterEntries.emplace(name, FilterPanelEntry_t{});
  }
}

void Visualizer::render() {
  ImGui::Begin("MAD Visualizer");
  if (state.hasMeasurement) {
    ImGui::Text("Measurement");
    ImGui::Text("  Time: %.3f", state.measurement.time);
    ImGui::Text("  Magnetic (T): %.6e", state.measurement.magneticTfc);
    ImGui::Text("  Sensor ECEF: [%.2f, %.2f, %.2f]",
                state.measurement.sensorPosEcef.x(),
                state.measurement.sensorPosEcef.y(),
                state.measurement.sensorPosEcef.z());
  } else {
    ImGui::Text("Measurement: n/a");
  }

  if (!state.measurementHistory.empty()) {
    ImGui::PlotLines("Measured Magnetic (T)",
                     state.measurementHistory.data(),
                     static_cast<int>(state.measurementHistory.size()),
                     0,
                     nullptr,
                     FLT_MAX,
                     FLT_MAX,
                     ImVec2(0, 80));
  }

  ImGui::Separator();

  if (!state.filterEntries.empty()) {
    ImGui::Text("Estimates");
    if (ImGui::BeginTable("estimates_table", 9, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("Filter");
      ImGui::TableSetupColumn("State[0]");
      ImGui::TableSetupColumn("State[1]");
      ImGui::TableSetupColumn("State[2]");
      ImGui::TableSetupColumn("Residual");
      ImGui::TableSetupColumn("Mean|res|");
      ImGui::TableSetupColumn("RMSE");
      ImGui::TableSetupColumn("NIS");
      ImGui::TableSetupColumn("Runtime (ms)");
      ImGui::TableHeadersRow();
      for (const auto& entry : state.filterEntries) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%s", entry.first.c_str());
        ImGui::TableNextColumn();
        if (entry.second.hasEstimate && entry.second.detectionState.size() > 0) {
          ImGui::Text("%.3f", entry.second.detectionState(0));
        } else {
          ImGui::Text("-");
        }
        ImGui::TableNextColumn();
        if (entry.second.hasEstimate && entry.second.detectionState.size() > 1) {
          ImGui::Text("%.3f", entry.second.detectionState(1));
        } else {
          ImGui::Text("-");
        }
        ImGui::TableNextColumn();
        if (entry.second.hasEstimate && entry.second.detectionState.size() > 2) {
          ImGui::Text("%.3f", entry.second.detectionState(2));
        } else {
          ImGui::Text("-");
        }
        ImGui::TableNextColumn();
        ImGui::Text("%.3e", entry.second.residual);
        ImGui::TableNextColumn();
        ImGui::Text("%.3e", entry.second.meanAbsResidual);
        ImGui::TableNextColumn();
        ImGui::Text("%.3e", entry.second.rmse);
        ImGui::TableNextColumn();
        ImGui::Text("%.3e", entry.second.nis);
        ImGui::TableNextColumn();
        ImGui::Text("%.2f", entry.second.runtimeMs);
      }
      ImGui::EndTable();
    }
  } else {
    ImGui::Text("Detection: n/a");
  }

  ImGui::Separator();

  if (!state.filterEntries.empty()) {
    ImGui::Text("Magnetic");
    if (ImGui::BeginTable("magnetic_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("Filter");
      ImGui::TableSetupColumn("Measured (T)");
      ImGui::TableSetupColumn("Predicted (T)");
      ImGui::TableSetupColumn("ESS");
      ImGui::TableHeadersRow();
      for (const auto& entry : state.filterEntries) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%s", entry.first.c_str());
        ImGui::TableNextColumn();
        ImGui::Text("%.6e", entry.second.measuredMagnetic);
        ImGui::TableNextColumn();
        ImGui::Text("%.6e", entry.second.predictedMagnetic);
        ImGui::TableNextColumn();
        if (entry.second.ess > 0.0) {
          ImGui::Text("%.1f", entry.second.ess);
        } else {
          ImGui::Text("-");
        }
      }
      ImGui::EndTable();
    }
  } else {
    ImGui::Text("Magnetic: n/a");
  }
  ImGui::End();
}

void Visualizer::updateMeasurement(const Measurement_t& measurement) {
  state.measurement = measurement;
  state.hasMeasurement = true;
  state.measurementHistory.push_back(static_cast<float>(measurement.magneticTfc));
  state.timeHistory.push_back(static_cast<float>(measurement.time));
  if (static_cast<int>(state.measurementHistory.size()) > state.maxHistory) {
    state.measurementHistory.erase(state.measurementHistory.begin());
    state.timeHistory.erase(state.timeHistory.begin());
  }
}

void Visualizer::updateFilterEntry(const std::string& name,
                                   const Vector& detectionState,
                                   double measuredMagnetic,
                                   double predictedMagnetic,
                                   double residual,
                                   double meanAbsResidual,
                                   double rmse,
                                   double nis,
                                   double runtimeMs,
                                   double ess) {
  auto& entry = state.filterEntries[name];
  entry.hasEstimate = true;
  entry.detectionState = detectionState;
  entry.measuredMagnetic = measuredMagnetic;
  entry.predictedMagnetic = predictedMagnetic;
  entry.residual = residual;
  entry.meanAbsResidual = meanAbsResidual;
  entry.rmse = rmse;
  entry.nis = nis;
  entry.runtimeMs = runtimeMs;
  entry.ess = ess;
}

} // namespace mad

