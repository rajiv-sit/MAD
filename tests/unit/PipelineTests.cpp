#include <memory>

#include <gtest/gtest.h>

#include "mad/core/Filter.hpp"
#include "mad/data/DataSource.hpp"
#include "mad/pipeline/Pipeline.hpp"
#include "mad/tracking/Tracker.hpp"

namespace mad {

class TestSource final : public IDataSource {
public:
  bool next(Measurement_t& out) override {
    if (index >= 2) {
      return false;
    }
    out.time = static_cast<double>(index);
    out.magneticTfc = static_cast<double>(index);
    out.sensorPosEcef = Vector::Zero(3);
    ++index;
    return true;
  }

private:
  int index = 0;
};

class TestFilter final : public IFilter {
public:
  void predict(double dt) override { lastDt = dt; }

  FilterOutput_t update(const FilterInput_t& input) override {
    lastMeasurement = input.measurement;
    FilterOutput_t output;
    output.state = Vector::Constant(1, input.measurement.magneticTfc);
    output.covariance = Matrix::Identity(1, 1);
    return output;
  }

  double lastDt = 0.0;
  Measurement_t lastMeasurement;
};

class TestTracker final : public ITracker {
public:
  void step(const TrackState_t& input, double /*time*/) override { lastState = input; }
  const std::vector<TrackState_t>& tracks() const override { return trackStates; }

  TrackState_t lastState;
  std::vector<TrackState_t> trackStates;
};

} // namespace mad

TEST(PipelineTests, RunsPipelineAndUpdatesTracker) {
  auto dataSource = std::make_shared<mad::TestSource>();
  auto filter = std::make_shared<mad::TestFilter>();
  auto tracker = std::make_shared<mad::TestTracker>();

  mad::Pipeline pipeline(dataSource, filter, tracker);
  pipeline.run();

  EXPECT_NEAR(filter->lastDt, 1.0, 1e-9);
  EXPECT_NEAR(filter->lastMeasurement.magneticTfc, 1.0, 1e-9);
  EXPECT_EQ(tracker->lastState.id, 0);
  ASSERT_EQ(tracker->lastState.state.size(), 1);
  EXPECT_NEAR(tracker->lastState.state(0), 1.0, 1e-9);
}
