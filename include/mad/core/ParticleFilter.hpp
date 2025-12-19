#pragma once

#include <memory>
#include <random>
#include <vector>

#include "mad/core/Filter.hpp"

namespace mad {

struct Particle_t {
  Vector state;
  double weight = 1.0;
};

// Resampling trigger policy.
enum class ResamplePolicy_e {
  kNever,
  kAlways,
  kAdaptive
};

// Resampling algorithm selection.
enum class ResampleMethod_e {
  kMultinomial,
  kSystematic
};

// Single Gaussian component for mixture likelihoods.
struct GaussianMixtureComponent_t {
  Matrix covariance;
  double weight = 0.0;
};

// Runtime configuration for particle filters.
struct ParticleFilterOptions_t {
  int numParticles = 1000;
  ResamplePolicy_e resamplePolicy = ResamplePolicy_e::kAdaptive;
  ResampleMethod_e resampleMethod = ResampleMethod_e::kSystematic;
  double essThresholdRatio = 0.5;
  double regularizationBandwidth = 0.1;
  double studentTDof = 3.0;
  int stateDimension = 0;
  Vector initialState;
  std::vector<GaussianMixtureComponent_t> mixture;
  Matrix proposalCovariance;
};

class StateSpaceModel {
public:
  virtual ~StateSpaceModel() = default;
  virtual void setMeasurementContext(const Measurement_t& measurement) = 0;
  virtual Vector propagate(const Vector& state, double dt, std::mt19937& rng) const = 0;
  virtual Vector predictMeasurement(const Vector& state) const = 0;
  virtual Matrix processNoise(double dt) const = 0;
  virtual Matrix measurementNoise() const = 0;
};

// Base class for SMC filters with configurable resampling and proposals.
class ParticleFilterBase : public IFilter {
public:
  ParticleFilterBase(std::shared_ptr<StateSpaceModel> model, ParticleFilterOptions_t options);

  void predict(double dt) override;
  FilterOutput_t update(const FilterInput_t& input) override;

  double effectiveSampleSize() const { return effectiveSampleSizeInternal(); }
  const std::vector<Particle_t>& particlesRef() const { return particles; }

protected:
  virtual Vector propose(const Vector& state, double dt, const FilterInput_t& input);
  virtual void afterResample();

  double effectiveSampleSizeInternal() const;
  void normalizeWeights();
  void resample();

  virtual double logLikelihood(const Vector& predicted, const Vector& measured);

  std::shared_ptr<StateSpaceModel> model;
  ParticleFilterOptions_t options;
  std::vector<Particle_t> particles;
  std::mt19937 rng;
};

class SIRParticleFilter : public ParticleFilterBase {
public:
  using ParticleFilterBase::ParticleFilterBase;

protected:
  void afterResample() override;
};

class SISParticleFilter final : public ParticleFilterBase {
public:
  SISParticleFilter(std::shared_ptr<StateSpaceModel> model, ParticleFilterOptions_t options);

protected:
  void afterResample() override;
};

class AuxiliaryParticleFilter final : public ParticleFilterBase {
public:
  using ParticleFilterBase::ParticleFilterBase;

  FilterOutput_t update(const FilterInput_t& input) override;
};

class RegularizedParticleFilter final : public ParticleFilterBase {
public:
  using ParticleFilterBase::ParticleFilterBase;

protected:
  void afterResample() override;
};

class AdaptiveResamplingParticleFilter final : public ParticleFilterBase {
public:
  using ParticleFilterBase::ParticleFilterBase;

protected:
  void afterResample() override;
};

class RobustParticleFilter final : public ParticleFilterBase {
public:
  using ParticleFilterBase::ParticleFilterBase;

protected:
  double logLikelihood(const Vector& predicted, const Vector& measured) override;
};

class GaussianMixtureParticleFilter final : public ParticleFilterBase {
public:
  using ParticleFilterBase::ParticleFilterBase;

protected:
  double logLikelihood(const Vector& predicted, const Vector& measured) override;
};

class EkfParticleFilter final : public ParticleFilterBase {
public:
  using ParticleFilterBase::ParticleFilterBase;

protected:
  Vector propose(const Vector& state, double dt, const FilterInput_t& input) override;
};

class UkfParticleFilter final : public ParticleFilterBase {
public:
  using ParticleFilterBase::ParticleFilterBase;

protected:
  Vector propose(const Vector& state, double dt, const FilterInput_t& input) override;
};

} // namespace mad




