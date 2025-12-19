#include <fmt/core.h>

#include <fstream>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "mad/core/FilterFactory.hpp"
#include "mad/core/Logger.hpp"
#include "mad/core/MadModel.hpp"
#include "mad/data/AsciiDataSource.hpp"
#include "mad/pipeline/Pipeline.hpp"
#include "mad/tracking/NoopTracker.hpp"

namespace {

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

struct CliOptions_t {
  std::string configPath;
  std::string datasetPath;
  std::string filterType;
  std::string observationModel;
  bool showHelp = false;
};

CliOptions_t parseArgs(int argc, char** argv) {
  CliOptions_t options;
  options.configPath = "config/default.json";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      options.showHelp = true;
      return options;
    } else if (arg == "--config" && i + 1 < argc) {
      options.configPath = argv[++i];
    } else if (arg == "--dataset" && i + 1 < argc) {
      options.datasetPath = argv[++i];
    } else if (arg == "--filter" && i + 1 < argc) {
      options.filterType = argv[++i];
    } else if (arg == "--obs-model" && i + 1 < argc) {
      options.observationModel = argv[++i];
    }
  }
  return options;
}

nlohmann::json loadJson(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open config file: " + path);
  }
  nlohmann::json config;
  file >> config;
  return config;
}

void applyLoggingConfig(const nlohmann::json& config) {
  const nlohmann::json loggingNode = config.value("logging", nlohmann::json::object());
  const bool enabled = loggingNode.value("enabled", true);
  const std::string level = loggingNode.value("level", "info");
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
  mad::Logger::SetEnabled(enabled);
  mad::Logger::SetLevel(mad::Logger::ParseLevel(level));
}

} // namespace

int main(int argc, char** argv) {
  mad::Logger::Initialize();
  const CliOptions_t cliOptions = parseArgs(argc, argv);
  if (cliOptions.showHelp) {
    fmt::print("Usage: mad_cli [--config <path>] [--dataset <path>] [--filter <type>] [--obs-model <dipole|body>]\n");
    return 0;
  }

  nlohmann::json config = loadJson(cliOptions.configPath);
  applyLoggingConfig(config);
  if (auto logger = mad::Logger::Get()) {
    logger->info("mad_cli using config: {}", cliOptions.configPath);
  }
  const nlohmann::json datasetNode = config.value("dataset", nlohmann::json::object());
  std::string datasetPath = datasetNode.value("path", "data/raw/mad_data.asc");
  const std::size_t maxScans = datasetNode.value("maxScans", 0);
  if (!cliOptions.datasetPath.empty()) {
    datasetPath = cliOptions.datasetPath;
  }

  const nlohmann::json modelNode = config.value("model", nlohmann::json::object());
  mad::Vector earthField(3);
  earthField << 25000e-9, -3400e-9, 36000e-9;
  earthField = parseVector3(modelNode.value("earthField", nlohmann::json::array()), earthField);
  const double processNoiseVar = modelNode.value("processNoiseVar", 0.05);
  const double measurementNoiseVar = modelNode.value("measurementNoiseVar", 1.0e-18);

  std::string observationModel = modelNode.value("observationModel", "dipole");
  if (!cliOptions.observationModel.empty()) {
    observationModel = cliOptions.observationModel;
  }

  double measurementScale = modelNode.value("measurementScale", 1.0);

  auto model = std::make_shared<mad::MadModel>(earthField,
                                               processNoiseVar,
                                               measurementNoiseVar,
                                               parseObservationModel(observationModel),
                                               measurementScale);
  if (measurementScale <= 0.0) {
    auto probeSource = std::make_shared<mad::AsciiDataSource>(datasetPath);
    mad::Measurement_t first;
    if (probeSource && probeSource->next(first) && first.hasTruth) {
      model->setMeasurementContext(first);
      model->setMeasurementScale(1.0);
      const mad::Vector truthPred = model->predictMeasurement(first.truthState);
      if (truthPred.size() > 0 && std::abs(truthPred(0)) > 0.0) {
        measurementScale = first.magneticTfc / truthPred(0);
        model->setMeasurementScale(measurementScale);
        if (auto logger = mad::Logger::Get()) {
          logger->info("Auto measurementScale set to {:.3e}", measurementScale);
        }
      }
    }
  }

  auto dataSource = std::make_shared<mad::AsciiDataSource>(datasetPath);
  if (!dataSource->good()) {
    if (auto logger = mad::Logger::Get()) {
      logger->error("Failed to open data source: {}", datasetPath);
    } else {
      fmt::print("Failed to open data source.\n");
    }
    return 1;
  }
  if (auto logger = mad::Logger::Get()) {
    logger->info("Dataset: {}", datasetPath);
  }

  nlohmann::json filterNode = config.value("filter", nlohmann::json::object());
  if (!cliOptions.filterType.empty()) {
    filterNode["type"] = cliOptions.filterType;
  }
  auto filter = mad::createFilter(filterNode, model);
  if (!filter) {
    if (auto logger = mad::Logger::Get()) {
      logger->error("Failed to create filter from config.");
    } else {
      fmt::print("Failed to create filter from config.\n");
    }
    return 1;
  }

  auto tracker = std::make_shared<mad::NoopTracker>();
  mad::Pipeline pipeline(dataSource, filter, tracker, maxScans);
  pipeline.run();

  if (auto logger = mad::Logger::Get()) {
    logger->info("mad_cli done");
  } else {
    fmt::print("mad_cli done\n");
  }
  return 0;
}
