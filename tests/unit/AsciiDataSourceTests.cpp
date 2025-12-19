#include <cmath>

#include <gtest/gtest.h>

#include "mad/data/AsciiDataSource.hpp"

namespace {

} // namespace

TEST(AsciiDataSourceTests, ReadsMeasurementAndConvertsUnits) {
  mad::AsciiDataSource source(MAD_REAL_DATA_PATH);
  ASSERT_TRUE(source.good());

  mad::Measurement_t measurement;
  ASSERT_TRUE(source.next(measurement));
  EXPECT_GT(measurement.time, -1.0);
  EXPECT_GT(std::abs(measurement.sensorPosEcef.x()), 0.0);
  EXPECT_GT(std::abs(measurement.sensorPosEcef.y()), 0.0);
  EXPECT_GT(std::abs(measurement.sensorPosEcef.z()), 0.0);
  EXPECT_GT(std::abs(measurement.magneticTfc), 0.0);
}
