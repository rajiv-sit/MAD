#include <memory>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "mad/core/FilterFactory.hpp"
#include "mad/core/EKF.hpp"
#include "mad/core/UKF.hpp"
#include "mad/core/MadModel.hpp"
#include "mad/core/ParticleFilter.hpp"

namespace {

std::shared_ptr<mad::MadModel> makeModel() {
  mad::Vector earthField(3);
  earthField << 2.5e-5, -3.4e-6, 3.6e-5;
  return std::make_shared<mad::MadModel>(earthField, 0.0, 1e-18, mad::ObservationModel_e::kDipole);
}

} // namespace

TEST(FilterFactoryTests, CreatesEkfAndUkf) {
  const auto model = makeModel();

  nlohmann::json ekfNode;
  ekfNode["type"] = "ekf";
  auto ekfFilter = mad::createFilter(ekfNode, model);
  EXPECT_NE(ekfFilter, nullptr);
  EXPECT_NE(dynamic_cast<mad::EKF*>(ekfFilter.get()), nullptr);

  nlohmann::json ukfNode;
  ukfNode["type"] = "ukf";
  auto ukfFilter = mad::createFilter(ukfNode, model);
  EXPECT_NE(ukfFilter, nullptr);
  EXPECT_NE(dynamic_cast<mad::UKF*>(ukfFilter.get()), nullptr);
}

TEST(FilterFactoryTests, CreatesParticleFilters) {
  const auto model = makeModel();

  nlohmann::json node;
  node["type"] = "sir";
  auto sirFilter = mad::createFilter(node, model);
  EXPECT_NE(dynamic_cast<mad::SIRParticleFilter*>(sirFilter.get()), nullptr);

  node["type"] = "robust";
  auto robustFilter = mad::createFilter(node, model);
  EXPECT_NE(dynamic_cast<mad::RobustParticleFilter*>(robustFilter.get()), nullptr);

  node["type"] = "ekf-pf";
  auto ekfPfFilter = mad::createFilter(node, model);
  EXPECT_NE(dynamic_cast<mad::EkfParticleFilter*>(ekfPfFilter.get()), nullptr);
}
