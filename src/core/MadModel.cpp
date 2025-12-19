#include "mad/core/MadModel.hpp"

#include <cmath>
#include <random>

#include "mad/core/Logger.hpp"

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
                   ObservationModel_e observationModelInput,
                   double measurementScaleInput)
    : earthField(earthFieldInput),
      processNoiseVar(processNoiseVarInput),
      measurementNoiseVar(measurementNoiseVarInput),
      observationModel(observationModelInput),
      measurementScale(measurementScaleInput) {
  if (auto logger = Logger::Get()) {
    logger->info("MadModel: earthField [{:.3e}, {:.3e}, {:.3e}]",
                 earthField.size() > 0 ? earthField(0) : 0.0,
                 earthField.size() > 1 ? earthField(1) : 0.0,
                 earthField.size() > 2 ? earthField(2) : 0.0);
    logger->info("MadModel: processNoiseVar {:.3e} measurementNoiseVar {:.3e} measurementScale {:.3e}",
                 processNoiseVar,
                 measurementNoiseVar,
                 measurementScale);
  }
}

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
    Vector measurement = predictBodyFrameMeasurement(state);
    if (measurement.size() > 0) {
      measurement(0) *= measurementScale;
    }
    return measurement;
  }
  Vector measurement = predictDipoleMeasurement(state);
  if (measurement.size() > 0) {
    measurement(0) *= measurementScale;
  }
  return measurement;
}

Matrix MadModel::processNoise(double dt) const {
  Matrix noise = Matrix::Zero(10, 10);
  const double t = std::max(dt, 0.0);
  const double t2 = t * t;
  const double t3 = t2 * t;

  noise(0, 0) = t3 / 3.0;
  noise(0, 1) = t2 / 2.0;
  noise(1, 0) = t2 / 2.0;
  noise(1, 1) = t;
  noise(2, 2) = t3 / 3.0;
  noise(2, 3) = t2 / 2.0;
  noise(3, 2) = t2 / 2.0;
  noise(3, 3) = t;
  for (int i = 4; i < 10; ++i) {
    noise(i, i) = 1.0;
  }

  return noise * processNoiseVar;
}

Matrix MadModel::measurementNoise() const {
  Matrix noise = Matrix::Identity(1, 1) * measurementNoiseVar;
  return noise;
}

Matrix MadModel::processJacobian(double dt) const {
  Matrix jacobian = Matrix::Identity(10, 10);
  const double t = std::max(dt, 0.0);
  jacobian(0, 1) = t;
  jacobian(2, 3) = t;
  return jacobian;
}

Matrix MadModel::measurementJacobian(const Vector& state) const {
  const double eps = 1e-4;
  const Vector base = predictMeasurement(state);
  const int rows = static_cast<int>(base.size());
  const int cols = static_cast<int>(state.size());
  Matrix jacobian = Matrix::Zero(rows, cols);
  for (int i = 0; i < cols; ++i) {
    Vector pert = state;
    pert(i) += eps;
    Vector meas = predictMeasurement(pert);
    jacobian.col(i) = (meas - base) / eps;
  }
  return jacobian;
}

void MadModel::setMeasurementScale(double scale) {
  measurementScale = scale;
}

double MadModel::measurementScaleValue() const {
  return measurementScale;
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
