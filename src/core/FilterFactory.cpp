#include "mad/core/FilterFactory.hpp"

#include <string>

#include "mad/core/EKF.hpp"
#include "mad/core/Logger.hpp"
#include "mad/core/MadModel.hpp"
#include "mad/core/UKF.hpp"

namespace mad {

namespace {

std::string getString(const nlohmann::json& node, const std::string& key, const std::string& fallback) {
  if (!node.is_object()) {
    return fallback;
  }
  auto it = node.find(key);
  if (it == node.end() || !it->is_string()) {
    return fallback;
  }
  return it->get<std::string>();
}

int getInt(const nlohmann::json& node, const std::string& key, int fallback) {
  if (!node.is_object()) {
    return fallback;
  }
  auto it = node.find(key);
  if (it == node.end() || !it->is_number_integer()) {
    return fallback;
  }
  return it->get<int>();
}

double getDouble(const nlohmann::json& node, const std::string& key, double fallback) {
  if (!node.is_object()) {
    return fallback;
  }
  auto it = node.find(key);
  if (it == node.end() || !it->is_number()) {
    return fallback;
  }
  return it->get<double>();
}

ResamplePolicy_e parseResamplePolicy(const nlohmann::json& node, ResamplePolicy_e fallback) {
  if (!node.is_string()) {
    return fallback;
  }
  const std::string value = node.get<std::string>();
  if (value == "never") {
    return ResamplePolicy_e::kNever;
  }
  if (value == "always") {
    return ResamplePolicy_e::kAlways;
  }
  return ResamplePolicy_e::kAdaptive;
}

ResampleMethod_e parseResampleMethod(const nlohmann::json& node, ResampleMethod_e fallback) {
  if (!node.is_string()) {
    return fallback;
  }
  const std::string value = node.get<std::string>();
  if (value == "multinomial") {
    return ResampleMethod_e::kMultinomial;
  }
  return ResampleMethod_e::kSystematic;
}

ParticleFilterOptions_t parseParticleOptions(const nlohmann::json& node) {
  ParticleFilterOptions_t options;
  if (!node.is_object()) {
    return options;
  }

  options.numParticles = getInt(node, "numParticles", options.numParticles);
  options.stateDimension = getInt(node, "stateDimension", options.stateDimension);
  options.resamplePolicy = parseResamplePolicy(node.value("resamplePolicy", nlohmann::json{}), options.resamplePolicy);
  options.resampleMethod = parseResampleMethod(node.value("resampleMethod", nlohmann::json{}), options.resampleMethod);
  options.essThresholdRatio = getDouble(node, "essThresholdRatio", options.essThresholdRatio);
  options.regularizationBandwidth = getDouble(node, "regularizationBandwidth", options.regularizationBandwidth);
  options.studentTDof = getDouble(node, "studentTDof", options.studentTDof);

  auto mixIt = node.find("mixture");
  if (mixIt != node.end() && mixIt->is_array()) {
    for (const auto& compNode : *mixIt) {
      GaussianMixtureComponent_t component;
      component.weight = getDouble(compNode, "weight", component.weight);
      auto covIt = compNode.find("covariance");
      if (covIt != compNode.end() && covIt->is_array()) {
        const int rows = static_cast<int>(covIt->size());
        const int cols = rows > 0 ? static_cast<int>((*covIt)[0].size()) : 0;
        component.covariance = Matrix::Zero(rows, cols);
        for (int r = 0; r < rows; ++r) {
          for (int c = 0; c < cols; ++c) {
            component.covariance(r, c) = (*covIt)[r][c].get<double>();
          }
        }
      }
      options.mixture.push_back(component);
    }
  }

  auto proposalIt = node.find("proposalCovariance");
  if (proposalIt != node.end() && proposalIt->is_array()) {
    const int rows = static_cast<int>(proposalIt->size());
    const int cols = rows > 0 ? static_cast<int>((*proposalIt)[0].size()) : 0;
    options.proposalCovariance = Matrix::Zero(rows, cols);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        options.proposalCovariance(r, c) = (*proposalIt)[r][c].get<double>();
      }
    }
  }

  return options;
}

} // namespace

std::shared_ptr<IFilter> createFilter(const nlohmann::json& filterNode,
                                      const std::shared_ptr<StateSpaceModel>& model) {
  const std::string type = getString(filterNode, "type", "sir");
  const nlohmann::json params = filterNode.is_object() ? filterNode.value("params", nlohmann::json{}) : nlohmann::json{};

  if (type == "ekf") {
    if (!model) {
      if (auto logger = Logger::Get()) {
        logger->error("FilterFactory: EKF requires a model.");
      }
      return nullptr;
    }
    auto madModel = std::dynamic_pointer_cast<MadModel>(model);
    if (!madModel) {
      if (auto logger = Logger::Get()) {
        logger->error("FilterFactory: EKF requires MadModel.");
      }
      return nullptr;
    }
    if (auto logger = Logger::Get()) {
      logger->info("FilterFactory: creating EKF.");
    }
    return std::make_shared<EKF>(madModel);
  }
  if (type == "ukf") {
    if (!model) {
      if (auto logger = Logger::Get()) {
        logger->error("FilterFactory: UKF requires a model.");
      }
      return nullptr;
    }
    auto madModel = std::dynamic_pointer_cast<MadModel>(model);
    if (!madModel) {
      if (auto logger = Logger::Get()) {
        logger->error("FilterFactory: UKF requires MadModel.");
      }
      return nullptr;
    }
    if (auto logger = Logger::Get()) {
      logger->info("FilterFactory: creating UKF.");
    }
    return std::make_shared<UKF>(madModel);
  }

  ParticleFilterOptions_t options = parseParticleOptions(params);
  if (options.stateDimension == 0) {
    options.stateDimension = 10;
  }
  if (!model) {
    if (auto logger = Logger::Get()) {
      logger->error("FilterFactory: particle filters require a model.");
    }
    return nullptr;
  }

  if (type == "sis") {
    if (auto logger = Logger::Get()) {
      logger->info("FilterFactory: creating SIS PF.");
    }
    return std::make_shared<SISParticleFilter>(model, options);
  }
  if (type == "apf") {
    if (auto logger = Logger::Get()) {
      logger->info("FilterFactory: creating APF.");
    }
    return std::make_shared<AuxiliaryParticleFilter>(model, options);
  }
  if (type == "rpf") {
    if (auto logger = Logger::Get()) {
      logger->info("FilterFactory: creating RPF.");
    }
    return std::make_shared<RegularizedParticleFilter>(model, options);
  }
  if (type == "adaptive") {
    if (auto logger = Logger::Get()) {
      logger->info("FilterFactory: creating adaptive PF.");
    }
    return std::make_shared<AdaptiveResamplingParticleFilter>(model, options);
  }
  if (type == "robust") {
    if (auto logger = Logger::Get()) {
      logger->info("FilterFactory: creating robust PF.");
    }
    return std::make_shared<RobustParticleFilter>(model, options);
  }
  if (type == "gmf") {
    if (auto logger = Logger::Get()) {
      logger->info("FilterFactory: creating Gaussian mixture PF.");
    }
    return std::make_shared<GaussianMixtureParticleFilter>(model, options);
  }
  if (type == "ekf-pf") {
    if (auto logger = Logger::Get()) {
      logger->info("FilterFactory: creating EKF-PF.");
    }
    return std::make_shared<EkfParticleFilter>(model, options);
  }
  if (type == "ukf-pf") {
    if (auto logger = Logger::Get()) {
      logger->info("FilterFactory: creating UKF-PF.");
    }
    return std::make_shared<UkfParticleFilter>(model, options);
  }

  if (auto logger = Logger::Get()) {
    logger->warn("FilterFactory: unknown type '{}', defaulting to SIR PF.", type);
  }
  return std::make_shared<SIRParticleFilter>(model, options);
}

} // namespace mad
