#include <cmath>

#include <gtest/gtest.h>

#include "mad/core/MadModel.hpp"

namespace {

mad::Vector makeEarthField() {
  mad::Vector field(3);
  field << 2.5e-5, -3.4e-6, 3.6e-5;
  return field;
}

} // namespace

TEST(MadModelTests, DipoleAndBodyModelsReturnFiniteMeasurement) {
  const mad::Vector earthField = makeEarthField();
  mad::MadModel dipoleModel(earthField, 0.0, 1e-18, mad::ObservationModel_e::kDipole);
  mad::MadModel bodyModel(earthField, 0.0, 1e-18, mad::ObservationModel_e::kBodyFrame);

  mad::Measurement_t measurement;
  measurement.sensorPosEcef = mad::Vector::Zero(3);
  dipoleModel.setMeasurementContext(measurement);
  bodyModel.setMeasurementContext(measurement);

  mad::Vector state = mad::Vector::Zero(10);
  state(0) = 100.0;
  state(2) = 200.0;
  state(4) = 1.0;
  state(5) = 2.0;
  state(6) = 3.0;
  state(7) = 0.1;
  state(8) = 0.1;
  state(9) = 0.1;

  const mad::Vector dipole = dipoleModel.predictMeasurement(state);
  const mad::Vector body = bodyModel.predictMeasurement(state);

  ASSERT_EQ(dipole.size(), 1);
  ASSERT_EQ(body.size(), 1);
  EXPECT_TRUE(std::isfinite(dipole(0)));
  EXPECT_TRUE(std::isfinite(body(0)));
}
