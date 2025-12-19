#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <nlohmann/json.hpp>

#include "mad/core/EKF.hpp"
#include "mad/core/FilterFactory.hpp"
#include "mad/core/Logger.hpp"
#include "mad/core/MadModel.hpp"
#include "mad/core/PerformanceMetrics.hpp"
#include "mad/core/ParticleFilter.hpp"
#include "mad/core/UKF.hpp"
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

void playAlertSound(const std::string& path) {
#ifdef _WIN32
  PlaySoundA(path.c_str(), nullptr, SND_FILENAME | SND_ASYNC);
#else
  (void)path;
#endif
}

struct AppState_t {
  std::shared_ptr<mad::AsciiDataSource> dataSource;
  std::shared_ptr<mad::MadModel> model;
  std::unordered_map<std::string, std::shared_ptr<mad::IFilter>> filters;
  std::unordered_map<std::string, mad::PerformanceMetrics> metrics;
  std::unordered_map<std::string, bool> selectedFilters;
  bool hasMoreData = true;
  double lastTime = 0.0;
  double measurementNoiseVar = 0.0;
  double measurementScale = 1.0;
  bool autoScale = false;
  std::size_t maxScans = 0;
  mad::Measurement_t pendingMeasurement;
  bool hasPendingMeasurement = false;
  mad::Measurement_t nextMeasurement;
  bool hasNextMeasurement = false;
  double alertThresholdTesla = 5e-9;
  double alertBaselineTesla = 0.0;
  int alertCooldownMs = 800;
  bool alertEnabled = false;
  bool alertAllFilters = false;
  std::string alertSoundPath;
  std::chrono::steady_clock::time_point lastAlertTime{};
  bool logEnabled = true;
  std::string logLevel = "info";
  std::size_t stepCount = 0;
  int maxScansSlider = 500;
  bool maxScansUnlimited = false;
  bool performancePanelEnabled = true;
  bool showRmseGraph = true;
  bool showNeesGraph = true;
  bool showResidualGraph = true;
  float trackerZoom = 1.0f;
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
  appState.maxScans = datasetNode.value("maxScans", 0);
  if (appState.maxScans > 0) {
    appState.maxScansSlider = static_cast<int>(appState.maxScans);
    appState.maxScansUnlimited = false;
  } else {
    appState.maxScansUnlimited = true;
  }
  const std::string resolvedDatasetPath = resolveDataPath(configDir, datasetPath);
  appState.dataSource = std::make_shared<mad::AsciiDataSource>(resolvedDatasetPath);
  if (auto logger = mad::Logger::Get()) {
    logger->info("Loading dataset: {}", resolvedDatasetPath);
  }
  appState.filters.clear();
  appState.metrics.clear();
  appState.lastTime = 0.0;
  appState.hasPendingMeasurement = false;
  appState.hasNextMeasurement = false;
  appState.stepCount = 0;
  appState.hasMoreData = appState.dataSource->good();
  if (!appState.hasMoreData) {
    if (auto logger = mad::Logger::Get()) {
      logger->error("Failed to open dataset: {}", resolvedDatasetPath);
    }
  }

  for (const auto& entry : appState.selectedFilters) {
    if (!entry.second) {
      continue;
    }
    auto filter = createFilterInstance(entry.first, config, appState.model);
    if (filter) {
      appState.filters[entry.first] = filter;
      appState.metrics[entry.first] = mad::PerformanceMetrics{};
      if (auto logger = mad::Logger::Get()) {
        logger->info("Enabled filter: {}", entry.first);
      }
    } else {
      if (auto logger = mad::Logger::Get()) {
        logger->warn("Failed to create filter: {}", entry.first);
      }
    }
  }

  mad::Measurement_t firstMeasurement;
  if (appState.dataSource && appState.dataSource->next(firstMeasurement)) {
    mad::Measurement_t secondMeasurement;
    if (appState.dataSource->next(secondMeasurement)) {
      const double dt = secondMeasurement.time - firstMeasurement.time;
      if (dt > 0.0 && firstMeasurement.truthState.size() == 10 && secondMeasurement.truthState.size() == 10) {
        firstMeasurement.truthState(1) = (secondMeasurement.truthState(0) - firstMeasurement.truthState(0)) / dt;
        firstMeasurement.truthState(3) = (secondMeasurement.truthState(2) - firstMeasurement.truthState(2)) / dt;
      }
      appState.nextMeasurement = secondMeasurement;
      appState.hasNextMeasurement = true;
    }
    appState.pendingMeasurement = firstMeasurement;
    appState.hasPendingMeasurement = true;
    appState.lastTime = firstMeasurement.time;
    if (appState.autoScale && firstMeasurement.hasTruth) {
      appState.model->setMeasurementContext(firstMeasurement);
      appState.model->setMeasurementScale(1.0);
      const mad::Vector truthPred = appState.model->predictMeasurement(firstMeasurement.truthState);
      if (truthPred.size() > 0 && std::abs(truthPred(0)) > 0.0) {
        const double scale = firstMeasurement.magneticTfc / truthPred(0);
        appState.model->setMeasurementScale(scale);
        if (auto logger = mad::Logger::Get()) {
          logger->info("Auto measurementScale set to {:.3e}", scale);
        }
      } else if (auto logger = mad::Logger::Get()) {
        logger->warn("Auto measurementScale skipped (zero truth prediction).");
      }
    }
    if (firstMeasurement.hasTruth && firstMeasurement.truthState.size() == 10) {
      for (auto& entry : appState.filters) {
        if (auto ekf = std::dynamic_pointer_cast<mad::EKF>(entry.second)) {
          ekf->initialize(firstMeasurement.truthState);
        } else if (auto ukf = std::dynamic_pointer_cast<mad::UKF>(entry.second)) {
          ukf->initialize(firstMeasurement.truthState);
        } else if (auto pf = std::dynamic_pointer_cast<mad::ParticleFilterBase>(entry.second)) {
          pf->initialize(firstMeasurement.truthState);
        }
      }
    }
  }
}

void stepOnce(AppState_t& appState, mad::Visualizer& visualizer, const std::string& mapFilterName) {
  if (!appState.dataSource || !appState.model) {
    appState.hasMoreData = false;
    if (auto logger = mad::Logger::Get()) {
      logger->warn("Data source or model missing, stopping.");
    }
    return;
  }
  if (appState.filters.empty()) {
    return;
  }
  if (appState.maxScans > 0 && appState.stepCount >= appState.maxScans) {
    appState.hasMoreData = false;
    if (auto logger = mad::Logger::Get()) {
      logger->info("Reached max scans {}", appState.maxScans);
    }
    return;
  }

  mad::Measurement_t measurement;
  if (appState.hasPendingMeasurement) {
    measurement = appState.pendingMeasurement;
    appState.hasPendingMeasurement = false;
  } else if (appState.hasNextMeasurement) {
    measurement = appState.nextMeasurement;
    appState.hasNextMeasurement = false;
  } else {
    if (!appState.dataSource->next(measurement)) {
      appState.hasMoreData = false;
      return;
    }
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
    if (measurement.hasTruth && measurement.truthState.size() == output.state.size()) {
      metrics.updateTruth(output.state, output.covariance, measurement.truthState);
      if ((appState.stepCount % 50) == 0) {
        if (auto logger = mad::Logger::Get()) {
          const double dx = output.state(0) - measurement.truthState(0);
          const double dy = output.state(2) - measurement.truthState(2);
          double truthPred = 0.0;
          const mad::Vector truthMeasurement = appState.model->predictMeasurement(measurement.truthState);
          if (truthMeasurement.size() > 0) {
            truthPred = truthMeasurement(0);
          }
          logger->debug("Step {} [{}] truth dx {:.3f} dy {:.3f} meas {:.3e} pred {:.3e} truthPred {:.3e}",
                        appState.stepCount,
                        entry.first,
                        dx,
                        dy,
                        measurement.magneticTfc,
                        predictedMagnetic,
                        truthPred);
        }
      }
    }
    visualizer.updateFilterEntry(entry.first,
                                 output.state,
                                 measurement.magneticTfc,
                                 predictedMagnetic,
                                 residual,
                                 metrics.meanAbsResidual(),
                                 metrics.rmse(),
                                 nis,
                                 metrics.lastTruthRmse(),
                                 metrics.lastTruthNees(),
                                 runtimeMs,
                                 ess);

    const bool shouldCheckAlert = appState.alertAllFilters || entry.first == mapFilterName;
    if (appState.alertEnabled && shouldCheckAlert) {
      const double predictedTotal = predictedMagnetic + appState.alertBaselineTesla;
      const double delta = std::abs(predictedTotal - appState.alertBaselineTesla);
      const auto now = std::chrono::steady_clock::now();
      const auto since = std::chrono::duration_cast<std::chrono::milliseconds>(now - appState.lastAlertTime).count();
      if (delta > appState.alertThresholdTesla && since >= appState.alertCooldownMs) {
        if (!appState.alertSoundPath.empty()) {
          playAlertSound(appState.alertSoundPath);
        }
        if (auto logger = mad::Logger::Get()) {
          logger->warn("Alert: |B| delta {:.3e} T (threshold {:.3e})", delta, appState.alertThresholdTesla);
        }
        appState.lastAlertTime = now;
      }
    }
  }

  appState.lastTime = measurement.time;
  ++appState.stepCount;
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

void applyLoggingConfig(AppState_t& appState, const nlohmann::json& config) {
  const nlohmann::json loggingNode = config.value("logging", nlohmann::json::object());
  appState.logEnabled = loggingNode.value("enabled", true);
  appState.logLevel = loggingNode.value("level", "info");
  const auto fileNode = loggingNode.value("file", nlohmann::json::object());
  mad::FileSinkConfig_t fileConfig;
  fileConfig.enabled = fileNode.value("enabled", true);
  fileConfig.path = fileNode.value("path", "logs/mad.log");
  fileConfig.maxSizeBytes = fileNode.value("maxSizeBytes", static_cast<std::size_t>(5 * 1024 * 1024));
  fileConfig.maxFiles = fileNode.value("maxFiles", static_cast<std::size_t>(3));
  mad::Logger::ConfigureFileSink(fileConfig);
  const auto classNode = loggingNode.value("classLogs", nlohmann::json::object());
  mad::ClassSinkConfig_t classConfig;
  classConfig.enabled = classNode.value("enabled", true);
  classConfig.directory = classNode.value("directory", "logs/classes");
  classConfig.maxSizeBytes = classNode.value("maxSizeBytes", static_cast<std::size_t>(5 * 1024 * 1024));
  classConfig.maxFiles = classNode.value("maxFiles", static_cast<std::size_t>(3));
  mad::Logger::ConfigureClassSink(classConfig);
  mad::Logger::SetEnabled(appState.logEnabled);
  mad::Logger::SetLevel(mad::Logger::ParseLevel(appState.logLevel));
}

void syncPerformanceUi(mad::Visualizer& viz, const AppState_t& appState) {
  viz.setPerformancePanelEnabled(appState.performancePanelEnabled);
  viz.setShowRmseGraph(appState.showRmseGraph);
  viz.setShowNeesGraph(appState.showNeesGraph);
  viz.setShowResidualGraph(appState.showResidualGraph);
}
} // namespace

int main() {
  try {
    mad::Logger::Initialize();
    const std::string configPath = "config/default.json";
    const std::string resolvedConfigPath = resolveConfigPath(configPath);
    nlohmann::json config = loadJson(resolvedConfigPath);
    const std::filesystem::path configDir = std::filesystem::path(resolvedConfigPath).parent_path();
    const std::string imguiIniPath = (configDir / "imgui.ini").string();
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) {
    if (auto logger = mad::Logger::Get()) {
      logger->error("Failed to init GLFW.");
    }
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWwindow* window = glfwCreateWindow(1280, 720, "MAD Visualizer", nullptr, nullptr);
  if (!window) {
    if (auto logger = mad::Logger::Get()) {
      logger->error("Failed to create GLFW window.");
    }
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    if (auto logger = mad::Logger::Get()) {
      logger->error("Failed to init GLAD.");
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.IniFilename = imguiIniPath.c_str();

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  mad::Visualizer viz;
  viz.start();

  const nlohmann::json modelNode = config.value("model", nlohmann::json::object());
  const nlohmann::json vizNode = config.value("viz", nlohmann::json::object());
  const nlohmann::json alertNode = vizNode.value("alert", nlohmann::json::object());
  mad::Vector earthField(3);
  earthField << 25000e-9, -3400e-9, 36000e-9;
  earthField = parseVector3(modelNode.value("earthField", nlohmann::json::array()), earthField);
  const double processNoiseVar = modelNode.value("processNoiseVar", 0.05);
  const double measurementNoiseVar = modelNode.value("measurementNoiseVar", 1.0e-18);
  const double measurementScale = modelNode.value("measurementScale", 1.0);
  const std::string observationModel = modelNode.value("observationModel", "dipole");

  AppState_t appState;
  applyLoggingConfig(appState, config);
  if (auto logger = mad::Logger::Get()) {
    logger->info("MAD GUI starting.");
    logger->info("Loaded config: {}", resolvedConfigPath);
  }
  appState.model = std::make_shared<mad::MadModel>(earthField,
                                                   processNoiseVar,
                                                   measurementNoiseVar,
                                                   parseObservationModel(observationModel),
                                                   measurementScale);
  appState.measurementNoiseVar = measurementNoiseVar;
  appState.measurementScale = measurementScale;
  appState.autoScale = measurementScale <= 0.0;
  appState.alertEnabled = alertNode.value("enabled", false);
  appState.alertThresholdTesla = alertNode.value("thresholdTesla", 5e-9);
  appState.alertCooldownMs = alertNode.value("cooldownMs", 800);
  appState.alertAllFilters = alertNode.value("scope", "map") == "all";
  appState.alertBaselineTesla = earthField.norm();
  appState.alertSoundPath = resolveDataPath(configDir, alertNode.value("soundPath", "sound/button-11.wav"));
  const std::string defaultFilter = config.value("filter", nlohmann::json::object()).value("type", "ekf");
  for (const auto& type : availableFilterTypes()) {
    appState.selectedFilters[type] = (type == defaultFilter);
  }
  viz.setTruthBaseline(static_cast<float>(earthField.norm()));
  viz.setEarthField(earthField);
  resetAppState(appState, config, configDir);
  syncPerformanceUi(viz, appState);

  bool run = true;
  bool step = false;
  bool selectionChanged = true;
  std::vector<std::string> activeFilters;
  std::string mapFilterName;
  std::string trackerFilterName;
  bool showMeasuredMap = true;
  bool showPredictedMap = true;
  bool showResidualMap = true;
  bool showTruthMap = true;
  bool showTrajectory2d = true;
  bool showTrajectory3d = false;

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
      syncPerformanceUi(viz, appState);
    }
    ImGui::Text("Data: %s", appState.hasMoreData ? "running" : "complete");
    if (ImGui::Checkbox("Unlimited Scans", &appState.maxScansUnlimited)) {
      if (auto logger = mad::Logger::Get()) {
        logger->info("Max scans {}", appState.maxScansUnlimited ? 0 : appState.maxScans);
      }
    }
    ImGui::SameLine();
    if (appState.maxScansUnlimited) {
      ImGui::BeginDisabled();
    }
    if (ImGui::SliderInt("Max Scans", &appState.maxScansSlider, 1, 10000)) {
      if (!appState.maxScansUnlimited) {
        appState.maxScans = static_cast<std::size_t>(appState.maxScansSlider);
        if (auto logger = mad::Logger::Get()) {
          logger->info("Max scans set to {}", appState.maxScans);
        }
      }
    }
    if (appState.maxScansUnlimited) {
      ImGui::EndDisabled();
    }
    if (appState.maxScansUnlimited) {
      appState.maxScans = 0;
    } else {
      appState.maxScans = static_cast<std::size_t>(appState.maxScansSlider);
    }
    ImGui::Separator();
    if (ImGui::Checkbox("Performance Panel", &appState.performancePanelEnabled)) {
      viz.setPerformancePanelEnabled(appState.performancePanelEnabled);
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Show RMSE", &appState.showRmseGraph)) {
      viz.setShowRmseGraph(appState.showRmseGraph);
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Show NEES", &appState.showNeesGraph)) {
      viz.setShowNeesGraph(appState.showNeesGraph);
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Show Residual", &appState.showResidualGraph)) {
      viz.setShowResidualGraph(appState.showResidualGraph);
    }
    ImGui::Separator();
    ImGui::Text("Logging");
    if (ImGui::Checkbox("Enabled##logging", &appState.logEnabled)) {
      mad::Logger::SetEnabled(appState.logEnabled);
      if (auto logger = mad::Logger::Get()) {
        logger->info("Logging enabled from UI.");
      }
    }
    const char* levels[] = {"trace", "debug", "info", "warn", "error", "critical", "off"};
    int currentLevel = 2;
    for (int i = 0; i < static_cast<int>(std::size(levels)); ++i) {
      if (appState.logLevel == levels[i]) {
        currentLevel = i;
        break;
      }
    }
    ImGui::SameLine();
    if (ImGui::BeginCombo("Level##logging", levels[currentLevel])) {
      for (int i = 0; i < static_cast<int>(std::size(levels)); ++i) {
        const bool isSelected = (i == currentLevel);
        if (ImGui::Selectable(levels[i], isSelected)) {
          appState.logLevel = levels[i];
          mad::Logger::SetLevel(mad::Logger::ParseLevel(appState.logLevel));
          if (auto logger = mad::Logger::Get()) {
            logger->info("Log level set to {} from UI.", appState.logLevel);
          }
        }
        if (isSelected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    ImGui::Separator();
    ImGui::Text("Filters");
    for (const auto& type : availableFilterTypes()) {
      bool enabled = appState.selectedFilters[type];
      if (ImGui::Checkbox(type.c_str(), &enabled)) {
        appState.selectedFilters[type] = enabled;
        selectionChanged = true;
      }
    }
    ImGui::Separator();
    ImGui::Text("Map Filter");
    if (activeFilters.empty()) {
      ImGui::Text("n/a");
    } else {
      int currentIndex = 0;
      for (size_t i = 0; i < activeFilters.size(); ++i) {
        if (activeFilters[i] == mapFilterName) {
          currentIndex = static_cast<int>(i);
          break;
        }
      }
      if (ImGui::BeginCombo("##map_filter", activeFilters[currentIndex].c_str())) {
        for (size_t i = 0; i < activeFilters.size(); ++i) {
          const bool isSelected = (static_cast<int>(i) == currentIndex);
          if (ImGui::Selectable(activeFilters[i].c_str(), isSelected)) {
            mapFilterName = activeFilters[i];
            viz.setMapFilterName(mapFilterName);
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
    }
    ImGui::Separator();
    ImGui::Text("Tracker Filter");
    if (activeFilters.empty()) {
      ImGui::Text("n/a");
    } else {
      int currentIndex = 0;
      for (size_t i = 0; i < activeFilters.size(); ++i) {
        if (activeFilters[i] == trackerFilterName) {
          currentIndex = static_cast<int>(i);
          break;
        }
      }
      if (ImGui::BeginCombo("##tracker_filter", activeFilters[currentIndex].c_str())) {
        for (size_t i = 0; i < activeFilters.size(); ++i) {
          const bool isSelected = (static_cast<int>(i) == currentIndex);
          if (ImGui::Selectable(activeFilters[i].c_str(), isSelected)) {
            trackerFilterName = activeFilters[i];
            viz.setTrackerFilterName(trackerFilterName);
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
    }
    ImGui::Separator();
    ImGui::Text("Map Layers");
    ImGui::Checkbox("Measured", &showMeasuredMap);
    ImGui::SameLine();
    ImGui::Checkbox("Predicted", &showPredictedMap);
    ImGui::Checkbox("Residual", &showResidualMap);
    ImGui::SameLine();
    ImGui::Checkbox("Truth", &showTruthMap);
    viz.setMapOptions(showMeasuredMap, showPredictedMap, showResidualMap, showTruthMap);
    ImGui::Separator();
    ImGui::Text("Trajectories");
    ImGui::Checkbox("2D", &showTrajectory2d);
    ImGui::SameLine();
    ImGui::Checkbox("3D", &showTrajectory3d);
    viz.setTrajectoryOptions(showTrajectory2d, showTrajectory3d);
    ImGui::Separator();
    ImGui::Text("Tracker Zoom");
    ImGui::SliderFloat("Zoom##tracker", &appState.trackerZoom, 0.5f, 5.0f, "%.2f");
    viz.setTrackerZoom(appState.trackerZoom);
    ImGui::End();

    if (selectionChanged) {
      resetAppState(appState, config, configDir);
      activeFilters.clear();
      for (const auto& entry : appState.selectedFilters) {
        if (entry.second) {
          activeFilters.push_back(entry.first);
        }
      }
      if (!activeFilters.empty()) {
        if (mapFilterName.empty() ||
            std::find(activeFilters.begin(), activeFilters.end(), mapFilterName) == activeFilters.end()) {
          mapFilterName = activeFilters.front();
        }
        viz.setMapFilterName(mapFilterName);
        if (trackerFilterName.empty() ||
            std::find(activeFilters.begin(), activeFilters.end(), trackerFilterName) == activeFilters.end()) {
          trackerFilterName = activeFilters.front();
        }
        viz.setTrackerFilterName(trackerFilterName);
      } else {
        mapFilterName.clear();
        viz.setMapFilterName(mapFilterName);
        trackerFilterName.clear();
        viz.setTrackerFilterName(trackerFilterName);
      }
      viz.setFilterNames(activeFilters);
      selectionChanged = false;
    }

    if ((run || step) && appState.hasMoreData) {
      stepOnce(appState, viz, mapFilterName);
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
  ImGui::SaveIniSettingsToDisk(imguiIniPath.c_str());
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
  } catch (const std::exception& ex) {
    if (auto logger = mad::Logger::Get()) {
      logger->critical("Fatal error: {}", ex.what());
    } else {
      fmt::print("Fatal error: {}\n", ex.what());
    }
    return 1;
  }
}
