#include <gtest/gtest.h>

#include "mad/core/EKF.hpp"
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

} // namespace

TEST(EkfUkfTests, UpdatesStateFromMeasurement) {
  mad::EKF ekf(4);
  mad::UKF ukf(4);

  const mad::Measurement_t measurement = makeMeasurement();
  mad::FilterInput_t input{measurement, 0.0};

  mad::FilterOutput_t ekfOutput = ekf.update(input);
  mad::FilterOutput_t ukfOutput = ukf.update(input);

  ASSERT_EQ(ekfOutput.state.size(), 4);
  EXPECT_NEAR(ekfOutput.state(0), 1.0, 1e-9);
  EXPECT_NEAR(ekfOutput.state(1), 2.0, 1e-9);
  EXPECT_NEAR(ekfOutput.state(2), 3.0, 1e-9);
  EXPECT_NEAR(ekfOutput.state(3), 4.0, 1e-9);

  ASSERT_EQ(ukfOutput.state.size(), 4);
  EXPECT_NEAR(ukfOutput.state(0), 1.0, 1e-9);
  EXPECT_NEAR(ukfOutput.state(1), 2.0, 1e-9);
  EXPECT_NEAR(ukfOutput.state(2), 3.0, 1e-9);
  EXPECT_NEAR(ukfOutput.state(3), 4.0, 1e-9);
}
