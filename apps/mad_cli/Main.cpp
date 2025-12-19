#include <fmt/core.h>

#include <fstream>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "mad/core/FilterFactory.hpp"
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

} // namespace

int main(int argc, char** argv) {
  const CliOptions_t cliOptions = parseArgs(argc, argv);
  if (cliOptions.showHelp) {
    fmt::print("Usage: mad_cli [--config <path>] [--dataset <path>] [--filter <type>] [--obs-model <dipole|body>]\n");
    return 0;
  }
  fmt::print("mad_cli using config: {}\n", cliOptions.configPath);

  nlohmann::json config = loadJson(cliOptions.configPath);
  const nlohmann::json datasetNode = config.value("dataset", nlohmann::json::object());
  std::string datasetPath = datasetNode.value("path", "data/raw/mad_data.asc");
  if (!cliOptions.datasetPath.empty()) {
    datasetPath = cliOptions.datasetPath;
  }

  auto dataSource = std::make_shared<mad::AsciiDataSource>(datasetPath);
  if (!dataSource->good()) {
    fmt::print("Failed to open data source.\n");
    return 1;
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

  auto model = std::make_shared<mad::MadModel>(earthField,
                                               processNoiseVar,
                                               measurementNoiseVar,
                                               parseObservationModel(observationModel));

  nlohmann::json filterNode = config.value("filter", nlohmann::json::object());
  if (!cliOptions.filterType.empty()) {
    filterNode["type"] = cliOptions.filterType;
  }
  auto filter = mad::createFilter(filterNode, model);
  if (!filter) {
    fmt::print("Failed to create filter from config.\n");
    return 1;
  }

  auto tracker = std::make_shared<mad::NoopTracker>();
  mad::Pipeline pipeline(dataSource, filter, tracker);
  pipeline.run();

  fmt::print("mad_cli done\n");
  return 0;
}
