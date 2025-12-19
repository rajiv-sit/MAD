#pragma once

#include "mad/core/ParticleFilter.hpp"

namespace mad {

// Selects which observation model to use for magnetic anomaly prediction.
enum class ObservationModel_e {
  kDipole,
  kBodyFrame
};

// MAD-specific nonlinear magnetic anomaly model.
class MadModel : public StateSpaceModel {
public:
  MadModel(const Vector& earthField,
           double processNoiseVar,
           double measurementNoiseVar,
           ObservationModel_e observationModel,
           double measurementScale = 1.0);

  void setMeasurementContext(const Measurement_t& measurement) override;
  Vector propagate(const Vector& state, double dt, std::mt19937& rng) const override;
  Vector predictMeasurement(const Vector& state) const override;
  Matrix processNoise(double dt) const override;
  Matrix measurementNoise() const override;
  Matrix processJacobian(double dt) const;
  Matrix measurementJacobian(const Vector& state) const;
  void setMeasurementScale(double scale);
  double measurementScaleValue() const;

private:
  Vector predictDipoleMeasurement(const Vector& state) const;
  Vector predictBodyFrameMeasurement(const Vector& state) const;

  Vector earthField;
  double processNoiseVar = 0.0;
  double measurementNoiseVar = 0.0;
  ObservationModel_e observationModel = ObservationModel_e::kDipole;
  double measurementScale = 1.0;
  Vector sensorPosEcef = Vector::Zero(3);
};

} // namespace mad




