#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "mad/core/FilterFactory.hpp"
#include "mad/core/MadModel.hpp"
#include "mad/data/AsciiDataSource.hpp"

namespace {

mad::Measurement_t loadFirstMeasurement() {
  mad::AsciiDataSource dataSource(MAD_REAL_DATA_PATH);
  mad::Measurement_t measurement;
  const bool ok = dataSource.next(measurement);
  EXPECT_TRUE(ok);
  return measurement;
}

std::shared_ptr<mad::MadModel> makeModel(const mad::Measurement_t& measurement) {
  mad::Vector earthField(3);
  earthField << 2.5e-5, -3.4e-6, 3.6e-5;
  auto model = std::make_shared<mad::MadModel>(earthField, 0.05, 1e-18, mad::ObservationModel_e::kDipole);
  model->setMeasurementContext(measurement);
  return model;
}

nlohmann::json makeFilterConfig(const std::string& type) {
  nlohmann::json config;
  config["type"] = type;
  nlohmann::json params;
  params["stateDimension"] = 10;
  params["numParticles"] = 64;
  params["essThresholdRatio"] = 0.5;
  params["resamplePolicy"] = "adaptive";
  config["params"] = params;
  return config;
}

mad::Matrix numericJacobian(const mad::MadModel& model, const mad::Vector& state, double eps = 1e-4) {
  const mad::Vector base = model.predictMeasurement(state);
  const int rows = static_cast<int>(base.size());
  const int cols = static_cast<int>(state.size());
  mad::Matrix jac(rows, cols);
  for (int i = 0; i < cols; ++i) {
    mad::Vector pert = state;
    pert(i) += eps;
    mad::Vector meas = model.predictMeasurement(pert);
    jac.col(i) = (meas - base) / eps;
  }
  return jac;
}

} // namespace

TEST(FilterSupportTests, AllFiltersReturnTenStateOutput) {
  const auto measurement = loadFirstMeasurement();
  const auto model = makeModel(measurement);

  const std::vector<std::string> types = {
      "ekf", "ukf", "sir", "sis", "apf", "rpf", "adaptive", "robust", "gmf", "ekf-pf", "ukf-pf"};

  for (const auto& type : types) {
    const auto config = makeFilterConfig(type);
    auto filter = mad::createFilter(config, model);
    ASSERT_NE(filter, nullptr) << "Failed to create filter: " << type;

    filter->predict(0.0);
    mad::FilterInput_t input{measurement, 0.0};
    mad::FilterOutput_t output = filter->update(input);
    EXPECT_EQ(output.state.size(), 10) << "Filter type " << type << " did not return 10 states.";
    EXPECT_TRUE(output.state.allFinite()) << "Filter type " << type << " returned non-finite values.";
  }
}

TEST(FilterSupportTests, MadModelMeasurementJacobianIs1x10) {
  const auto measurement = loadFirstMeasurement();
  const auto model = makeModel(measurement);

  mad::Vector state = mad::Vector::Zero(10);
  state(0) = 10.0;
  state(2) = -5.0;
  state(4) = 1e-9;
  state(5) = 1e-9;
  state(6) = 1e-9;

  const mad::Matrix jac = numericJacobian(*model, state);
  EXPECT_EQ(jac.rows(), 1);
  EXPECT_EQ(jac.cols(), 10);
  EXPECT_TRUE(jac.allFinite());
}

TEST(FilterSupportTests, MadModelProcessJacobianMatchesConstantVelocity) {
  const auto measurement = loadFirstMeasurement();
  const auto model = makeModel(measurement);

  const double dt = 1.0 / 32.0;
  const mad::Matrix jac = model->processJacobian(dt);
  ASSERT_EQ(jac.rows(), 10);
  ASSERT_EQ(jac.cols(), 10);
  EXPECT_NEAR(jac(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(jac(0, 1), dt, 1e-12);
  EXPECT_NEAR(jac(2, 2), 1.0, 1e-12);
  EXPECT_NEAR(jac(2, 3), dt, 1e-12);
  for (int i = 4; i < 10; ++i) {
    EXPECT_NEAR(jac(i, i), 1.0, 1e-12);
  }
}
