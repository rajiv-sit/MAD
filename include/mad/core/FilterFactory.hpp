#pragma once

#include <memory>

#include <nlohmann/json.hpp>

#include "mad/core/Filter.hpp"
#include "mad/core/ParticleFilter.hpp"

namespace mad {

// Creates a filter instance based on JSON configuration.
std::shared_ptr<IFilter> createFilter(const nlohmann::json& filterNode,
                                      const std::shared_ptr<StateSpaceModel>& model);

} // namespace mad
