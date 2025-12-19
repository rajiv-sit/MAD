#include "mad/core/MadModel.hpp"

#include <cmath>
#include <random>

namespace mad {

namespace {

constexpr double kMu0Over4Pi = 1e-7;

// Computes the magnetic field of a dipole at a displacement in ECEF.
Vector computeDipoleField(const Vector& dipoleMoment, const Vector& displacement) {
  const double r2 = displacement.squaredNorm();
  const double r = std::sqrt(r2);
  const double r3 = r2 * r;
  const double r5 = r3 * r2;
  if (r5 <= 0.0) {
    return Vector::Zero(3);
  }

  const double mDotR = dipoleMoment.dot(displacement);
  Vector term1 = displacement * (3.0 * mDotR / r5);
  Vector term2 = dipoleMoment / r3;
  return kMu0Over4Pi * (term1 - term2);
}

// Combines permanent and induced moments into a single dipole vector.
Vector buildDipoleMoment(const Vector& state, const Vector& earthField) {
  Vector moment = Vector::Zero(3);
  if (state.size() >= 10) {
    moment(0) = state(4) + state(7) * earthField(0);
    moment(1) = state(5) + state(8) * earthField(1);
    moment(2) = state(6) + state(9) * earthField(2);
  }
  return moment;
}

} // namespace

MadModel::MadModel(const Vector& earthFieldInput,
                   double processNoiseVarInput,
                   double measurementNoiseVarInput,
                   ObservationModel_e observationModelInput)
    : earthField(earthFieldInput),
      processNoiseVar(processNoiseVarInput),
      measurementNoiseVar(measurementNoiseVarInput),
      observationModel(observationModelInput) {}

void MadModel::setMeasurementContext(const Measurement_t& measurement) {
  sensorPosEcef = measurement.sensorPosEcef;
}

Vector MadModel::propagate(const Vector& state, double dt, std::mt19937& rng) const {
  Vector next = state;
  if (next.size() >= 4) {
    next(0) = state(0) + state(1) * dt;
    next(2) = state(2) + state(3) * dt;
  }

  if (processNoiseVar > 0.0) {
    const double noiseScale = std::sqrt(processNoiseVar * std::max(dt, 0.0));
    std::normal_distribution<double> dist(0.0, noiseScale);
    for (int i = 0; i < next.size(); ++i) {
      next(i) += dist(rng);
    }
  }

  return next;
}

Vector MadModel::predictMeasurement(const Vector& state) const {
  if (observationModel == ObservationModel_e::kBodyFrame) {
    return predictBodyFrameMeasurement(state);
  }
  return predictDipoleMeasurement(state);
}

Matrix MadModel::processNoise(double /*dt*/) const {
  Matrix noise = Matrix::Identity(10, 10) * processNoiseVar;
  return noise;
}

Matrix MadModel::measurementNoise() const {
  Matrix noise = Matrix::Identity(1, 1) * measurementNoiseVar;
  return noise;
}

Vector MadModel::predictDipoleMeasurement(const Vector& state) const {
  Vector measurement = Vector::Zero(1);
  if (state.size() < 10 || earthField.size() < 3) {
    return measurement;
  }

  Vector targetPos(3);
  targetPos << state(0), state(2), 0.0;

  const Vector displacement = targetPos - sensorPosEcef;
  const Vector dipoleMoment = buildDipoleMoment(state, earthField);
  const Vector dipoleField = computeDipoleField(dipoleMoment, displacement);

  const double earthNorm = earthField.norm();
  if (earthNorm > 0.0) {
    measurement(0) = dipoleField.dot(earthField / earthNorm);
  }

  return measurement;
}

Vector MadModel::predictBodyFrameMeasurement(const Vector& state) const {
  Vector measurement = Vector::Zero(1);
  if (state.size() < 10 || earthField.size() < 3) {
    return measurement;
  }

  const double vx = state(1);
  const double vy = state(3);
  const double speed = std::sqrt(vx * vx + vy * vy);
  if (speed <= 0.0) {
    return measurement;
  }

  const double bN = earthField(0);
  const double bE = earthField(1);
  const double bV = earthField(2);

  const double bL = (bE * vy + bN * vx) / speed;
  const double bT = (bE * vx - bN * vy) / speed;

  const double mL = state(4) + state(7) * bL;
  const double mT = state(5) + state(8) * bT;
  const double mV = state(6) + state(9) * bV;

  Vector unitL(3);
  unitL << vx / speed, vy / speed, 0.0;
  Vector unitT(3);
  unitT << -vy / speed, vx / speed, 0.0;
  Vector unitV(3);
  unitV << 0.0, 0.0, 1.0;

  Vector dipoleMoment = mL * unitL + mT * unitT + mV * unitV;

  Vector targetPos(3);
  targetPos << state(0), state(2), 0.0;
  const Vector displacement = targetPos - sensorPosEcef;
  const Vector dipoleField = computeDipoleField(dipoleMoment, displacement);

  const double earthNorm = earthField.norm();
  if (earthNorm > 0.0) {
    measurement(0) = dipoleField.dot(earthField / earthNorm);
  }

  return measurement;
}

} // namespace mad
