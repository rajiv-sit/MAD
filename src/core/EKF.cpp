#include "mad/core/EKF.hpp"

#include "mad/core/Logger.hpp"

namespace mad {

EKF::EKF(std::shared_ptr<MadModel> modelInput, int stateDim)
    : model(std::move(modelInput)),
      state(Vector::Zero(stateDim)),
      covariance(Matrix::Identity(stateDim, stateDim)),
      processNoise(Matrix::Identity(stateDim, stateDim) * 1e-3),
      measurementNoise(Matrix::Identity(1, 1) * 1e-2) {}

void EKF::predict(double dt) {
  if (!model) {
    return;
  }
  static std::size_t stepCount = 0;
  if (state.size() >= 4) {
    state(0) = state(0) + state(1) * dt;
    state(2) = state(2) + state(3) * dt;
  }
  const Matrix F = model->processJacobian(dt);
  processNoise = model->processNoise(dt);
  covariance = F * covariance * F.transpose() + processNoise;
  if ((stepCount % 50) == 0) {
    if (auto logger = Logger::GetClass("EKF")) {
      logger->debug("EKF predict step {} dt {:.6f} state x {:.3f} y {:.3f}", stepCount, dt, state(0), state(2));
    }
  }
  ++stepCount;
}

FilterOutput_t EKF::update(const FilterInput_t& input) {
  if (!model) {
    return {};
  }
  static std::size_t updateCount = 0;
  model->setMeasurementContext(input.measurement);

  const Vector y = model->predictMeasurement(state);
  const Matrix H = model->measurementJacobian(state);
  measurementNoise = model->measurementNoise();

  const Vector z = Vector::Constant(y.size(), input.measurement.magneticTfc);
  const Vector residual = z - y;
  const Matrix S = H * covariance * H.transpose() + measurementNoise;

  Matrix K = Matrix::Zero(covariance.rows(), S.rows());
  if (S.rows() > 0 && std::abs(S.determinant()) > 0.0) {
    K = covariance * H.transpose() * S.inverse();
  }
  state = state + K * residual;
  const Matrix I = Matrix::Identity(covariance.rows(), covariance.cols());
  covariance = (I - K * H) * covariance;

  if ((updateCount % 50) == 0) {
    if (auto logger = Logger::GetClass("EKF")) {
      const double hNorm = H.norm();
      const double kNorm = K.norm();
      logger->debug("EKF update step {} z {:.3e} y {:.3e} res {:.3e} |H| {:.3e} |K| {:.3e} x {:.3f} y {:.3f}",
                    updateCount,
                    z(0),
                    y.size() > 0 ? y(0) : 0.0,
                    residual.size() > 0 ? residual(0) : 0.0,
                    hNorm,
                    kNorm,
                    state(0),
                    state(2));
    }
  }
  ++updateCount;

  return FilterOutput_t{state, covariance};
}

void EKF::initialize(const Vector& initialState) {
  if (initialState.size() == state.size()) {
    state = initialState;
  }
}

} // namespace mad




