#include <cmath>
#include <memory>
#include <random>

#include <gtest/gtest.h>

#include "mad/core/ParticleFilter.hpp"

namespace mad {

class LinearModel final : public StateSpaceModel {
public:
  void setMeasurementContext(const Measurement_t& measurement) override { lastMeasurement = measurement; }

  Vector propagate(const Vector& state, double dt, std::mt19937& /*rng*/) const override {
    Vector next = state;
    if (next.size() > 0) {
      next(0) = state(0) + dt;
    }
    return next;
  }

  Vector predictMeasurement(const Vector& state) const override {
    Vector measurement(1);
    measurement(0) = state(0);
    return measurement;
  }

  Matrix processNoise(double /*dt*/) const override {
    return Matrix::Identity(1, 1) * 0.0;
  }

  Matrix measurementNoise() const override {
    return Matrix::Identity(1, 1) * 1e-2;
  }

  Measurement_t lastMeasurement;
};

class TestSIRFilter : public SIRParticleFilter {
public:
  TestSIRFilter(std::shared_ptr<StateSpaceModel> model, ParticleFilterOptions_t options)
      : SIRParticleFilter(std::move(model), std::move(options)) {}

  void initialize(double value) {
    for (auto& particle : particles) {
      particle.state = Vector::Constant(1, value);
      particle.weight = 1.0 / particles.size();
    }
  }
};

} // namespace mad

TEST(ParticleFilterTests, UpdatesMeanWithDeterministicParticles) {
  auto model = std::make_shared<mad::LinearModel>();
  mad::ParticleFilterOptions_t options;
  options.numParticles = 10;
  options.resamplePolicy = mad::ResamplePolicy_e::kNever;
  options.stateDimension = 1;

  mad::TestSIRFilter filter(model, options);
  filter.initialize(1.0);

  mad::Measurement_t measurement;
  measurement.magneticTfc = 1.0;
  mad::FilterInput_t input{measurement, 0.0};

  mad::FilterOutput_t output = filter.update(input);
  ASSERT_EQ(output.state.size(), 1);
  EXPECT_NEAR(output.state(0), 1.0, 1e-6);
}

TEST(ParticleFilterTests, RobustFilterReturnsFiniteEstimate) {
  auto model = std::make_shared<mad::LinearModel>();
  mad::ParticleFilterOptions_t options;
  options.numParticles = 5;
  options.resamplePolicy = mad::ResamplePolicy_e::kNever;
  options.stateDimension = 1;

  mad::RobustParticleFilter filter(model, options);

  mad::Measurement_t measurement;
  measurement.magneticTfc = 2.0;
  mad::FilterInput_t input{measurement, 0.0};

  mad::FilterOutput_t output = filter.update(input);
  ASSERT_EQ(output.state.size(), 1);
  EXPECT_TRUE(std::isfinite(output.state(0)));
}
