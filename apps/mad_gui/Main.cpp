#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <nlohmann/json.hpp>

#include "mad/core/FilterFactory.hpp"
#include "mad/core/MadModel.hpp"
#include "mad/core/PerformanceMetrics.hpp"
#include "mad/core/ParticleFilter.hpp"
#include "mad/data/AsciiDataSource.hpp"
#include "mad/viz/Visualizer.hpp"

namespace {

void glfwErrorCallback(int error, const char* description) {
  fmt::print("GLFW Error {}: {}\n", error, description ? description : "unknown");
}

mad::Vector parseVector3(const nlohmann::json& node, const mad::Vector& fallback) {
  if (!node.is_array() || node.size() != 3) {
    return fallback;
  }
  mad::Vector value(3);
  value(0) = node[0].get<double>();
  value(1) = node[1].get<double>();
  value(2) = node[2].get<double>();
  return value;
}

mad::ObservationModel_e parseObservationModel(const std::string& value) {
  if (value == "body") {
    return mad::ObservationModel_e::kBodyFrame;
  }
  return mad::ObservationModel_e::kDipole;
}

std::string resolveDataPath(const std::filesystem::path& baseDir, const std::string& path);

struct AppState_t {
  std::shared_ptr<mad::AsciiDataSource> dataSource;
  std::shared_ptr<mad::MadModel> model;
  std::unordered_map<std::string, std::shared_ptr<mad::IFilter>> filters;
  std::unordered_map<std::string, mad::PerformanceMetrics> metrics;
  std::unordered_map<std::string, bool> selectedFilters;
  bool hasMoreData = true;
  double lastTime = 0.0;
  double measurementNoiseVar = 0.0;
};

std::vector<std::string> availableFilterTypes() {
  return {"ekf", "ukf", "sir", "sis", "apf", "rpf", "adaptive", "robust", "gmf", "ekf-pf", "ukf-pf"};
}

std::shared_ptr<mad::IFilter> createFilterInstance(const std::string& type,
                                                   const nlohmann::json& config,
                                                   const std::shared_ptr<mad::MadModel>& model) {
  nlohmann::json filterNode = config.value("filter", nlohmann::json::object());
  filterNode["type"] = type;
  return mad::createFilter(filterNode, model);
}

void resetAppState(AppState_t& appState,
                   const nlohmann::json& config,
                   const std::filesystem::path& configDir) {
  const nlohmann::json datasetNode = config.value("dataset", nlohmann::json::object());
  const std::string datasetPath = datasetNode.value("path", "data/raw/mad_data.asc");
  const std::string resolvedDatasetPath = resolveDataPath(configDir, datasetPath);
  appState.dataSource = std::make_shared<mad::AsciiDataSource>(resolvedDatasetPath);
  appState.filters.clear();
  appState.metrics.clear();
  appState.lastTime = 0.0;
  appState.hasMoreData = appState.dataSource->good();

  for (const auto& entry : appState.selectedFilters) {
    if (!entry.second) {
      continue;
    }
    auto filter = createFilterInstance(entry.first, config, appState.model);
    if (filter) {
      appState.filters[entry.first] = filter;
      appState.metrics[entry.first] = mad::PerformanceMetrics{};
    }
  }
}

void stepOnce(AppState_t& appState, mad::Visualizer& visualizer) {
  if (!appState.dataSource || !appState.model) {
    appState.hasMoreData = false;
    return;
  }
  if (appState.filters.empty()) {
    return;
  }

  mad::Measurement_t measurement;
  if (!appState.dataSource->next(measurement)) {
    appState.hasMoreData = false;
    return;
  }

  const double dt = measurement.time - appState.lastTime;
  appState.model->setMeasurementContext(measurement);
  visualizer.updateMeasurement(measurement);

  for (const auto& entry : appState.selectedFilters) {
    if (!entry.second) {
      continue;
    }
    auto it = appState.filters.find(entry.first);
    if (it == appState.filters.end()) {
      continue;
    }
    auto& filter = it->second;
    const auto start = std::chrono::steady_clock::now();
    filter->predict(dt);
    mad::FilterInput_t input{measurement, dt};
    mad::FilterOutput_t output = filter->update(input);
    const auto end = std::chrono::steady_clock::now();
    const double runtimeMs = std::chrono::duration<double, std::milli>(end - start).count();

    const mad::Vector predicted = appState.model->predictMeasurement(output.state);
    const double predictedMagnetic = predicted.size() > 0 ? predicted(0) : 0.0;
    const double residual = measurement.magneticTfc - predictedMagnetic;
    const double nis = appState.measurementNoiseVar > 0.0
                           ? (residual * residual) / appState.measurementNoiseVar
                           : 0.0;

    double ess = 0.0;
    if (auto pf = std::dynamic_pointer_cast<mad::ParticleFilterBase>(filter)) {
      ess = pf->effectiveSampleSize();
    }

    auto& metrics = appState.metrics[entry.first];
    metrics.update(residual, nis, runtimeMs, ess);
    visualizer.updateFilterEntry(entry.first,
                                 output.state,
                                 measurement.magneticTfc,
                                 predictedMagnetic,
                                 residual,
                                 metrics.meanAbsResidual(),
                                 metrics.rmse(),
                                 nis,
                                 runtimeMs,
                                 ess);
  }

  appState.lastTime = measurement.time;
}

std::string resolveConfigPath(const std::string& path) {
  std::filesystem::path candidate(path);
  if (std::filesystem::exists(candidate)) {
    return candidate.string();
  }

  const std::filesystem::path cwd = std::filesystem::current_path();
  candidate = cwd / path;
  if (std::filesystem::exists(candidate)) {
    return candidate.string();
  }

  candidate = cwd / ".." / path;
  if (std::filesystem::exists(candidate)) {
    return candidate.string();
  }

  candidate = cwd / ".." / ".." / path;
  if (std::filesystem::exists(candidate)) {
    return candidate.string();
  }

  return path;
}

std::string resolveDataPath(const std::filesystem::path& baseDir, const std::string& path) {
  std::filesystem::path candidate(path);
  if (candidate.is_absolute() && std::filesystem::exists(candidate)) {
    return candidate.string();
  }

  if (!baseDir.empty()) {
    candidate = baseDir / path;
    if (std::filesystem::exists(candidate)) {
      return candidate.string();
    }
  }

  return resolveConfigPath(path);
}

nlohmann::json loadJson(const std::string& path) {
  const std::string resolved = resolveConfigPath(path);
  std::ifstream file(resolved);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open config file: " + resolved);
  }
  nlohmann::json config;
  file >> config;
  return config;
}

} // namespace

int main() {
  try {
  glfwSetErrorCallback(glfwErrorCallback);
  if (!glfwInit()) {
    fmt::print("Failed to init GLFW\n");
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWwindow* window = glfwCreateWindow(1280, 720, "MAD Visualizer", nullptr, nullptr);
  if (!window) {
    fmt::print("Failed to create GLFW window\n");
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    fmt::print("Failed to init GLAD\n");
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  mad::Visualizer viz;
  viz.start();

  const std::string configPath = "config/default.json";
  const std::string resolvedConfigPath = resolveConfigPath(configPath);
  nlohmann::json config = loadJson(resolvedConfigPath);
  const std::filesystem::path configDir = std::filesystem::path(resolvedConfigPath).parent_path();

  const nlohmann::json modelNode = config.value("model", nlohmann::json::object());
  mad::Vector earthField(3);
  earthField << 25000e-9, -3400e-9, 36000e-9;
  earthField = parseVector3(modelNode.value("earthField", nlohmann::json::array()), earthField);
  const double processNoiseVar = modelNode.value("processNoiseVar", 0.05);
  const double measurementNoiseVar = modelNode.value("measurementNoiseVar", 1.0e-18);
  const std::string observationModel = modelNode.value("observationModel", "dipole");

  AppState_t appState;
  appState.model = std::make_shared<mad::MadModel>(earthField,
                                                   processNoiseVar,
                                                   measurementNoiseVar,
                                                   parseObservationModel(observationModel));
  appState.measurementNoiseVar = measurementNoiseVar;
  for (const auto& type : availableFilterTypes()) {
    appState.selectedFilters[type] = false;
  }
  appState.selectedFilters["ukf"] = true;
  resetAppState(appState, config, configDir);

  bool run = true;
  bool step = false;
  bool selectionChanged = true;

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Controls");
    if (ImGui::Button(run ? "Pause" : "Run")) {
      run = !run;
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
      step = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
      resetAppState(appState, config, configDir);
      selectionChanged = true;
    }
    ImGui::Text("Data: %s", appState.hasMoreData ? "running" : "complete");
    ImGui::Separator();
    ImGui::Text("Filters");
    for (const auto& type : availableFilterTypes()) {
      bool enabled = appState.selectedFilters[type];
      if (ImGui::Checkbox(type.c_str(), &enabled)) {
        appState.selectedFilters[type] = enabled;
        selectionChanged = true;
      }
    }
    ImGui::End();

    if (selectionChanged) {
      resetAppState(appState, config, configDir);
      std::vector<std::string> activeFilters;
      for (const auto& entry : appState.selectedFilters) {
        if (entry.second) {
          activeFilters.push_back(entry.first);
        }
      }
      viz.setFilterNames(activeFilters);
      selectionChanged = false;
    }

    if ((run || step) && appState.hasMoreData) {
      stepOnce(appState, viz);
      step = false;
    }

    viz.render();

    ImGui::Render();
    int displayW = 0;
    int displayH = 0;
    glfwGetFramebufferSize(window, &displayW, &displayH);
    glViewport(0, 0, displayW, displayH);
    glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  viz.stop();

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
  } catch (const std::exception& ex) {
    fmt::print("Fatal error: {}\n", ex.what());
    return 1;
  }
}
