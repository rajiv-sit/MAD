#include <gtest/gtest.h>

#include "mad/core/EKF.hpp"
#include "mad/core/MadModel.hpp"
#include "mad/core/UKF.hpp"

namespace {

mad::Measurement_t makeMeasurement() {
  mad::Measurement_t measurement;
  measurement.time = 0.0;
  measurement.sensorPosEcef = mad::Vector::Zero(3);
  measurement.sensorPosEcef(0) = 1.0;
  measurement.sensorPosEcef(1) = 2.0;
  measurement.sensorPosEcef(2) = 3.0;
  measurement.magneticTfc = 4.0;
  return measurement;
}

std::shared_ptr<mad::MadModel> makeModel(const mad::Measurement_t& measurement) {
  mad::Vector earthField(3);
  earthField << 2.5e-5, -3.4e-6, 3.6e-5;
  auto model = std::make_shared<mad::MadModel>(earthField, 0.05, 1e-18, mad::ObservationModel_e::kDipole);
  model->setMeasurementContext(measurement);
  return model;
}

} // namespace

TEST(EkfUkfTests, ProducesTenStateEstimate) {
  const mad::Measurement_t measurement = makeMeasurement();
  auto model = makeModel(measurement);
  mad::EKF ekf(model, 10);
  mad::UKF ukf(model, 10);

  mad::FilterInput_t input{measurement, 0.0};

  mad::FilterOutput_t ekfOutput = ekf.update(input);
  mad::FilterOutput_t ukfOutput = ukf.update(input);

  ASSERT_EQ(ekfOutput.state.size(), 10);
  EXPECT_TRUE(ekfOutput.state.allFinite());

  ASSERT_EQ(ukfOutput.state.size(), 10);
  EXPECT_TRUE(ukfOutput.state.allFinite());
}
