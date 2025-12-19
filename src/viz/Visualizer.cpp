#include "mad/viz/Visualizer.hpp"

#include <algorithm>
#include <cfloat>
#include <cstdio>

#include <imgui.h>

namespace mad {

void Visualizer::start() {}
void Visualizer::stop() {}

void Visualizer::setFilterNames(const std::vector<std::string>& filterNames) {
  state.filterEntries.clear();
  state.predictedMap.clear();
  state.estimatedTracks.clear();
  for (const auto& name : filterNames) {
    state.filterEntries.emplace(name, FilterPanelEntry_t{});
    state.predictedMap.emplace(name, std::vector<MagneticMapPoint_t>{});
    state.estimatedTracks.emplace(name, std::vector<TrackPoint_t>{});
  }
}

void Visualizer::setMapFilterName(const std::string& filterName) {
  state.mapFilterName = filterName;
}

void Visualizer::setTrackerFilterName(const std::string& filterName) {
  state.trackerFilterName = filterName;
}

void Visualizer::setTruthBaseline(float valueTesla) {
  state.truthBaseline = valueTesla;
}

void Visualizer::setEarthField(const Vector& earthFieldTesla) {
  state.earthField = earthFieldTesla;
}

void Visualizer::setMapOptions(bool showMeasured, bool showPredicted, bool showResidual, bool showTruth) {
  state.showMeasuredMap = showMeasured;
  state.showPredictedMap = showPredicted;
  state.showResidualMap = showResidual;
  state.showTruthMap = showTruth;
}

void Visualizer::setTrajectoryOptions(bool show2d, bool show3d) {
  state.showTrajectory2d = show2d;
  state.showTrajectory3d = show3d;
}

namespace {

ImU32 colorRamp(float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  const float r = t;
  const float g = 0.15f + 0.85f * (1.0f - std::abs(0.5f - t) * 2.0f);
  const float b = 1.0f - t;
  return IM_COL32(static_cast<int>(r * 255.0f),
                  static_cast<int>(g * 255.0f),
                  static_cast<int>(b * 255.0f),
                  220);
}

struct GridCell_t {
  float sum = 0.0f;
  int count = 0;
};

void drawMap(const char* label, const std::vector<MagneticMapPoint_t>& points) {
  ImGui::Text("%s", label);
  float width = ImGui::GetContentRegionAvail().x;
  if (width < 10.0f) {
    width = 300.0f;
  }
  const ImVec2 size = ImVec2(width, 180);
  const ImVec2 cursor = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(label, size);
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  const ImVec2 max = ImVec2(cursor.x + size.x, cursor.y + size.y);
  drawList->AddRect(cursor, max, IM_COL32(90, 90, 90, 255));

  if (points.empty()) {
    drawList->AddText(ImVec2(cursor.x + 6.0f, cursor.y + 6.0f), IM_COL32(180, 180, 180, 255), "n/a");
    return;
  }

  float minX = points.front().x;
  float maxX = points.front().x;
  float minY = points.front().y;
  float maxY = points.front().y;
  float minV = points.front().value;
  float maxV = points.front().value;
  for (const auto& p : points) {
    minX = std::min(minX, p.x);
    maxX = std::max(maxX, p.x);
    minY = std::min(minY, p.y);
    maxY = std::max(maxY, p.y);
    minV = std::min(minV, p.value);
    maxV = std::max(maxV, p.value);
  }

  const float dx = (maxX - minX);
  const float dy = (maxY - minY);
  if (dx <= 0.0f || dy <= 0.0f) {
    return;
  }

  constexpr int gridSize = 40;
  std::vector<GridCell_t> grid(gridSize * gridSize);
  for (const auto& p : points) {
    const float nx = (p.x - minX) / dx;
    const float ny = (p.y - minY) / dy;
    const int ix = std::clamp(static_cast<int>(nx * (gridSize - 1)), 0, gridSize - 1);
    const int iy = std::clamp(static_cast<int>(ny * (gridSize - 1)), 0, gridSize - 1);
    GridCell_t& cell = grid[iy * gridSize + ix];
    cell.sum += p.value;
    cell.count += 1;
  }

  float gridMin = 0.0f;
  float gridMax = 0.0f;
  bool hasValue = false;
  for (const auto& cell : grid) {
    if (cell.count == 0) {
      continue;
    }
    const float avg = cell.sum / static_cast<float>(cell.count);
    if (!hasValue) {
      gridMin = avg;
      gridMax = avg;
      hasValue = true;
    } else {
      gridMin = std::min(gridMin, avg);
      gridMax = std::max(gridMax, avg);
    }
  }

  if (!hasValue) {
    return;
  }

  const float dv = gridMax - gridMin;
  const float cellW = size.x / static_cast<float>(gridSize);
  const float cellH = size.y / static_cast<float>(gridSize);
  for (int iy = 0; iy < gridSize; ++iy) {
    for (int ix = 0; ix < gridSize; ++ix) {
      const GridCell_t& cell = grid[iy * gridSize + ix];
      if (cell.count == 0) {
        continue;
      }
      const float avg = cell.sum / static_cast<float>(cell.count);
      const float nv = (dv > 0.0f) ? (avg - gridMin) / dv : 0.5f;
      const float px0 = cursor.x + ix * cellW;
      const float py0 = cursor.y + (gridSize - 1 - iy) * cellH;
      const float px1 = px0 + cellW;
      const float py1 = py0 + cellH;
      drawList->AddRectFilled(ImVec2(px0, py0), ImVec2(px1, py1), colorRamp(nv));
    }
  }
}

void appendHistory(std::vector<float>& storage, float value, int maxSize) {
  storage.push_back(value);
  if (maxSize > 0 && static_cast<int>(storage.size()) > maxSize) {
    const int overflow = static_cast<int>(storage.size()) - maxSize;
    storage.erase(storage.begin(), storage.begin() + overflow);
  }
}

void drawUniformMap(const char* label, float baseline) {
  ImGui::Text("%s", label);
  float width = ImGui::GetContentRegionAvail().x;
  if (width < 10.0f) {
    width = 300.0f;
  }
  const ImVec2 size = ImVec2(width, 180);
  const ImVec2 cursor = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(label, size);
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  const ImVec2 max = ImVec2(cursor.x + size.x, cursor.y + size.y);
  drawList->AddRectFilled(cursor, max, colorRamp(0.5f));
  drawList->AddRect(cursor, max, IM_COL32(90, 90, 90, 255));
  char text[128];
  std::snprintf(text, sizeof(text), "Baseline |B| = %.3e T", baseline);
  drawList->AddText(ImVec2(cursor.x + 6.0f, cursor.y + 6.0f), IM_COL32(230, 230, 230, 255), text);
}

void drawEarthFieldPanel(const Vector& earthField) {
  ImGui::Text("Earth Field (Bn/Be/Bv)");
  if (earthField.size() < 3) {
    ImGui::Text("n/a");
    return;
  }
  ImGui::Text("Bn: %.3e T", earthField(0));
  ImGui::Text("Be: %.3e T", earthField(1));
  ImGui::Text("Bv: %.3e T", earthField(2));
}

std::vector<MagneticMapPoint_t> buildResidualMap(const std::vector<MagneticMapPoint_t>& measured,
                                                 const std::vector<MagneticMapPoint_t>& predicted) {
  const size_t count = std::min(measured.size(), predicted.size());
  std::vector<MagneticMapPoint_t> residuals;
  residuals.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    MagneticMapPoint_t point;
    point.x = measured[i].x;
    point.y = measured[i].y;
    point.value = measured[i].value - predicted[i].value;
    residuals.push_back(point);
  }
  return residuals;
}

void drawTrajectory2d(const char* label,
                      const std::vector<TrackPoint_t>& sensor,
                      const std::vector<TrackPoint_t>& target) {
  ImGui::Text("%s", label);
  float width = ImGui::GetContentRegionAvail().x;
  if (width < 10.0f) {
    width = 300.0f;
  }
  const ImVec2 size = ImVec2(width, 180);
  const ImVec2 cursor = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(label, size);
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  const ImVec2 max = ImVec2(cursor.x + size.x, cursor.y + size.y);
  drawList->AddRect(cursor, max, IM_COL32(90, 90, 90, 255));

  if (sensor.empty() && target.empty()) {
    drawList->AddText(ImVec2(cursor.x + 6.0f, cursor.y + 6.0f), IM_COL32(180, 180, 180, 255), "n/a");
    return;
  }

  float minX = 0.0f;
  float maxX = 0.0f;
  float minY = 0.0f;
  float maxY = 0.0f;
  bool initialized = false;
  auto expandBounds = [&](const TrackPoint_t& p) {
    if (!initialized) {
      minX = maxX = p.x;
      minY = maxY = p.y;
      initialized = true;
      return;
    }
    minX = std::min(minX, p.x);
    maxX = std::max(maxX, p.x);
    minY = std::min(minY, p.y);
    maxY = std::max(maxY, p.y);
  };
  for (const auto& p : sensor) {
    expandBounds(p);
  }
  for (const auto& p : target) {
    expandBounds(p);
  }

  const float dx = maxX - minX;
  const float dy = maxY - minY;
  if (dx <= 0.0f || dy <= 0.0f) {
    return;
  }

  auto toScreen = [&](const TrackPoint_t& p) {
    const float nx = (p.x - minX) / dx;
    const float ny = (p.y - minY) / dy;
    const float px = cursor.x + nx * size.x;
    const float py = cursor.y + (1.0f - ny) * size.y;
    return ImVec2(px, py);
  };

  for (size_t i = 1; i < sensor.size(); ++i) {
    drawList->AddLine(toScreen(sensor[i - 1]), toScreen(sensor[i]), IM_COL32(220, 80, 80, 220), 1.5f);
  }
  for (size_t i = 1; i < target.size(); ++i) {
    drawList->AddLine(toScreen(target[i - 1]), toScreen(target[i]), IM_COL32(80, 160, 255, 220), 1.5f);
  }
}

void drawTrajectory3d(const char* label,
                      const std::vector<TrackPoint_t>& sensor,
                      const std::vector<TrackPoint_t>& target) {
  ImGui::Text("%s", label);
  float width = ImGui::GetContentRegionAvail().x;
  if (width < 10.0f) {
    width = 300.0f;
  }
  const ImVec2 size = ImVec2(width, 200);
  const ImVec2 cursor = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(label, size);
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  const ImVec2 max = ImVec2(cursor.x + size.x, cursor.y + size.y);
  drawList->AddRect(cursor, max, IM_COL32(90, 90, 90, 255));

  if (sensor.empty() && target.empty()) {
    drawList->AddText(ImVec2(cursor.x + 6.0f, cursor.y + 6.0f), IM_COL32(180, 180, 180, 255), "n/a");
    return;
  }

  float minX = 0.0f;
  float maxX = 0.0f;
  float minY = 0.0f;
  float maxY = 0.0f;
  float minZ = 0.0f;
  float maxZ = 0.0f;
  bool initialized = false;
  auto expandBounds = [&](const TrackPoint_t& p) {
    if (!initialized) {
      minX = maxX = p.x;
      minY = maxY = p.y;
      minZ = maxZ = p.z;
      initialized = true;
      return;
    }
    minX = std::min(minX, p.x);
    maxX = std::max(maxX, p.x);
    minY = std::min(minY, p.y);
    maxY = std::max(maxY, p.y);
    minZ = std::min(minZ, p.z);
    maxZ = std::max(maxZ, p.z);
  };
  for (const auto& p : sensor) {
    expandBounds(p);
  }
  for (const auto& p : target) {
    expandBounds(p);
  }

  const float dx = maxX - minX;
  const float dy = maxY - minY;
  const float dz = maxZ - minZ;
  if (dx <= 0.0f || dy <= 0.0f) {
    return;
  }

  auto toScreen = [&](const TrackPoint_t& p) {
    const float nx = (p.x - minX) / dx;
    const float ny = (p.y - minY) / dy;
    const float nz = (dz > 0.0f) ? (p.z - minZ) / dz : 0.0f;
    const float isoX = (nx - ny) * 0.7f;
    const float isoY = (nx + ny) * 0.4f - nz * 0.6f;
    const float px = cursor.x + (0.5f + isoX) * size.x;
    const float py = cursor.y + (0.5f + isoY) * size.y;
    return ImVec2(px, py);
  };

  for (size_t i = 1; i < sensor.size(); ++i) {
    drawList->AddLine(toScreen(sensor[i - 1]), toScreen(sensor[i]), IM_COL32(220, 80, 80, 220), 1.5f);
  }
  for (size_t i = 1; i < target.size(); ++i) {
    drawList->AddLine(toScreen(target[i - 1]), toScreen(target[i]), IM_COL32(80, 160, 255, 220), 1.5f);
  }
}

void drawTrackerPanel(const std::vector<TrackPoint_t>& truthTrack,
                      const std::vector<TrackPoint_t>& estimatedTrack) {
  float width = ImGui::GetContentRegionAvail().x;
  if (width < 10.0f) {
    width = 300.0f;
  }
  const ImVec2 size = ImVec2(width, 220);
  const ImVec2 cursor = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton("tracker_plot", size);
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  const ImVec2 max = ImVec2(cursor.x + size.x, cursor.y + size.y);
  drawList->AddRect(cursor, max, IM_COL32(90, 90, 90, 255));

  if (truthTrack.empty() && estimatedTrack.empty()) {
    drawList->AddText(ImVec2(cursor.x + 6.0f, cursor.y + 6.0f), IM_COL32(180, 180, 180, 255), "n/a");
    return;
  }

  float minX = 0.0f;
  float maxX = 0.0f;
  float minY = 0.0f;
  float maxY = 0.0f;
  bool initialized = false;
  auto expandBounds = [&](const TrackPoint_t& p) {
    if (!initialized) {
      minX = maxX = p.x;
      minY = maxY = p.y;
      initialized = true;
      return;
    }
    minX = std::min(minX, p.x);
    maxX = std::max(maxX, p.x);
    minY = std::min(minY, p.y);
    maxY = std::max(maxY, p.y);
  };
  for (const auto& p : truthTrack) {
    expandBounds(p);
  }
  for (const auto& p : estimatedTrack) {
    expandBounds(p);
  }

  const float dx = maxX - minX;
  const float dy = maxY - minY;
  if (dx <= 0.0f || dy <= 0.0f) {
    return;
  }

  auto toScreen = [&](const TrackPoint_t& p) {
    const float nx = (p.x - minX) / dx;
    const float ny = (p.y - minY) / dy;
    const float px = cursor.x + nx * size.x;
    const float py = cursor.y + (1.0f - ny) * size.y;
    return ImVec2(px, py);
  };

  for (size_t i = 1; i < truthTrack.size(); ++i) {
    drawList->AddLine(toScreen(truthTrack[i - 1]), toScreen(truthTrack[i]), IM_COL32(80, 160, 255, 220), 2.0f);
  }
  for (size_t i = 1; i < estimatedTrack.size(); ++i) {
    drawList->AddLine(toScreen(estimatedTrack[i - 1]), toScreen(estimatedTrack[i]), IM_COL32(240, 200, 80, 220), 2.0f);
  }
}

} // namespace

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
    ImGui::Text("State (10D)");
    if (ImGui::BeginTable("state_table", 11, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("Filter");
      ImGui::TableSetupColumn("X");
      ImGui::TableSetupColumn("Vx");
      ImGui::TableSetupColumn("Y");
      ImGui::TableSetupColumn("Vy");
      ImGui::TableSetupColumn("ML");
      ImGui::TableSetupColumn("MT");
      ImGui::TableSetupColumn("MV");
      ImGui::TableSetupColumn("CLW");
      ImGui::TableSetupColumn("CTW");
      ImGui::TableSetupColumn("CVW");
      ImGui::TableHeadersRow();
  for (const auto& entry : state.filterEntries) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%s", entry.first.c_str());
        for (int i = 0; i < 10; ++i) {
          ImGui::TableNextColumn();
          if (entry.second.hasEstimate && entry.second.detectionState.size() > i) {
            ImGui::Text("%.3f", entry.second.detectionState(i));
          } else {
            ImGui::Text("-");
          }
        }
      }
      ImGui::EndTable();
    }

    ImGui::Separator();

    ImGui::Text("Metrics");
    if (ImGui::BeginTable("metrics_table", 8, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("Filter");
      ImGui::TableSetupColumn("Residual");
      ImGui::TableSetupColumn("Mean|res|");
      ImGui::TableSetupColumn("RMSE");
      ImGui::TableSetupColumn("NIS");
      ImGui::TableSetupColumn("Truth RMSE");
      ImGui::TableSetupColumn("Truth NEES");
      ImGui::TableSetupColumn("Runtime (ms)");
      ImGui::TableHeadersRow();
      for (const auto& entry : state.filterEntries) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%s", entry.first.c_str());
        ImGui::TableNextColumn();
        ImGui::Text("%.3e", entry.second.residual);
        ImGui::TableNextColumn();
        ImGui::Text("%.3e", entry.second.meanAbsResidual);
        ImGui::TableNextColumn();
        ImGui::Text("%.3e", entry.second.rmse);
        ImGui::TableNextColumn();
        ImGui::Text("%.3e", entry.second.nis);
        ImGui::TableNextColumn();
        if (entry.second.truthRmse > 0.0) {
          ImGui::Text("%.3e", entry.second.truthRmse);
        } else {
          ImGui::Text("-");
        }
        ImGui::TableNextColumn();
        if (entry.second.truthNees > 0.0) {
          ImGui::Text("%.3e", entry.second.truthNees);
        } else {
          ImGui::Text("-");
        }
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

  ImGui::Separator();

  ImGui::Text("Magnetic Map (XY, resampled)");
  if (state.showMeasuredMap) {
    drawMap("Measured", state.measuredMap);
  }
  const auto mapIt = state.predictedMap.find(state.mapFilterName);
  const std::vector<MagneticMapPoint_t>* predicted = nullptr;
  if (mapIt != state.predictedMap.end()) {
    predicted = &mapIt->second;
  }
  if (state.showPredictedMap) {
    if (predicted) {
      drawMap("Predicted", *predicted);
    } else {
      std::vector<MagneticMapPoint_t> empty;
      drawMap("Predicted", empty);
    }
  }
  if (state.showResidualMap) {
    if (predicted) {
      const auto residuals = buildResidualMap(state.measuredMap, *predicted);
      drawMap("Residual", residuals);
    } else {
      std::vector<MagneticMapPoint_t> empty;
      drawMap("Residual", empty);
    }
  }
  if (state.showTruthMap) {
    drawUniformMap("Truth (Uniform)", state.truthBaseline);
  }
  drawEarthFieldPanel(state.earthField);

  ImGui::Separator();
  if (state.showTrajectory2d) {
    drawTrajectory2d("Trajectories (2D)", state.sensorTrack, state.targetTrack);
  }
  if (state.showTrajectory3d) {
    drawTrajectory3d("Trajectories (3D)", state.sensorTrack, state.targetTrack);
  }
  ImGui::End();

  ImGui::Begin("Tracker");
  ImGui::Text("Truth vs Estimated (2D)");
  ImGui::TextColored(ImVec4(0.31f, 0.63f, 1.0f, 1.0f), "Truth");
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.94f, 0.78f, 0.31f, 1.0f), "Estimated");
  const auto trackIt = state.estimatedTracks.find(state.trackerFilterName);
  if (trackIt != state.estimatedTracks.end()) {
    drawTrackerPanel(state.targetTrack, trackIt->second);
  } else {
    std::vector<TrackPoint_t> empty;
    drawTrackerPanel(state.targetTrack, empty);
  }
  ImGui::End();

  if (state.performancePanelEnabled) {
    ImGui::Begin("Performance Metrics", &state.performancePanelEnabled);
    ImGui::Checkbox("RMSE", &state.showRmseGraph);
    ImGui::SameLine();
    ImGui::Checkbox("NEES", &state.showNeesGraph);
    ImGui::SameLine();
    ImGui::Checkbox("Residual", &state.showResidualGraph);
    ImGui::Separator();
    for (const auto& entry : state.filterEntries) {
      const auto histIt = state.metricHistories.find(entry.first);
      if (histIt == state.metricHistories.end()) {
        continue;
      }
      const auto& history = histIt->second;
      ImGui::Text("%s", entry.first.c_str());
      const ImVec2 plotSize = ImVec2(ImGui::GetContentRegionAvail().x, 80);
      if (state.showRmseGraph && !history.rmse.empty()) {
        const std::string label = "RMSE##" + entry.first;
        ImGui::PlotLines(label.c_str(),
                         history.rmse.data(),
                         static_cast<int>(history.rmse.size()),
                         0,
                         nullptr,
                         FLT_MAX,
                         FLT_MAX,
                         plotSize);
        ImGui::Text("Latest RMSE %.3e", history.rmse.back());
      }
      if (state.showNeesGraph && !history.nees.empty()) {
        const std::string label = "NEES##" + entry.first;
        ImGui::PlotLines(label.c_str(),
                         history.nees.data(),
                         static_cast<int>(history.nees.size()),
                         0,
                         nullptr,
                         FLT_MAX,
                         FLT_MAX,
                         plotSize);
        ImGui::Text("Latest NEES %.3e", history.nees.back());
      }
      if (state.showResidualGraph && !history.residual.empty()) {
        const std::string label = "Residual##" + entry.first;
        ImGui::PlotLines(label.c_str(),
                         history.residual.data(),
                         static_cast<int>(history.residual.size()),
                         0,
                         nullptr,
                         FLT_MAX,
                         FLT_MAX,
                         plotSize);
        ImGui::Text("Latest Res %.3e", history.residual.back());
      }
      ImGui::Separator();
    }
    ImGui::End();
  }
}

void Visualizer::updateMeasurement(const Measurement_t& measurement) {
  state.measurement = measurement;
  state.hasMeasurement = true;
  state.measurementHistory.push_back(static_cast<float>(measurement.magneticTfc));
  state.timeHistory.push_back(static_cast<float>(measurement.time));
  MagneticMapPoint_t point;
  point.x = static_cast<float>(measurement.sensorPosEcef.x());
  point.y = static_cast<float>(measurement.sensorPosEcef.y());
  point.value = static_cast<float>(measurement.magneticTfc);
  state.measuredMap.push_back(point);
  TrackPoint_t sensorPoint;
  sensorPoint.x = static_cast<float>(measurement.sensorPosEcef.x());
  sensorPoint.y = static_cast<float>(measurement.sensorPosEcef.y());
  sensorPoint.z = static_cast<float>(measurement.sensorPosEcef.z());
  state.sensorTrack.push_back(sensorPoint);
  if (measurement.hasTruth) {
    TrackPoint_t targetPoint;
    targetPoint.x = static_cast<float>(measurement.truthPosEcef.x());
    targetPoint.y = static_cast<float>(measurement.truthPosEcef.y());
    targetPoint.z = static_cast<float>(measurement.truthPosEcef.z());
    state.targetTrack.push_back(targetPoint);
  }
  if (static_cast<int>(state.measurementHistory.size()) > state.maxHistory) {
    state.measurementHistory.erase(state.measurementHistory.begin());
    state.timeHistory.erase(state.timeHistory.begin());
    state.measuredMap.erase(state.measuredMap.begin());
    state.sensorTrack.erase(state.sensorTrack.begin());
    if (!state.targetTrack.empty()) {
      state.targetTrack.erase(state.targetTrack.begin());
    }
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
                                   double truthRmse,
                                   double truthNees,
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
  entry.truthRmse = truthRmse;
  entry.truthNees = truthNees;
  entry.runtimeMs = runtimeMs;
  entry.ess = ess;

  if (state.hasMeasurement) {
    MagneticMapPoint_t point;
    point.x = static_cast<float>(state.measurement.sensorPosEcef.x());
    point.y = static_cast<float>(state.measurement.sensorPosEcef.y());
    point.value = static_cast<float>(predictedMagnetic);
    auto& buffer = state.predictedMap[name];
    buffer.push_back(point);
    if (static_cast<int>(buffer.size()) > state.maxHistory) {
      buffer.erase(buffer.begin());
    }
  }

  if (state.hasMeasurement && detectionState.size() >= 3) {
    TrackPoint_t point;
    point.x = static_cast<float>(detectionState(0));
    point.y = static_cast<float>(detectionState(2));
    point.z = 0.0f;
    auto& track = state.estimatedTracks[name];
    track.push_back(point);
    if (static_cast<int>(track.size()) > state.maxHistory) {
      track.erase(track.begin());
    }
  }

  auto& metricsHistory = state.metricHistories[name];
  appendHistory(metricsHistory.rmse, static_cast<float>(rmse), state.maxHistory);
  appendHistory(metricsHistory.nees, static_cast<float>(truthNees), state.maxHistory);
  appendHistory(metricsHistory.residual, static_cast<float>(residual), state.maxHistory);
}

void Visualizer::setPerformancePanelEnabled(bool enabled) {
  state.performancePanelEnabled = enabled;
}

void Visualizer::setShowRmseGraph(bool show) {
  state.showRmseGraph = show;
}

void Visualizer::setShowNeesGraph(bool show) {
  state.showNeesGraph = show;
}

void Visualizer::setShowResidualGraph(bool show) {
  state.showResidualGraph = show;
}

} // namespace mad

