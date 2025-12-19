#include "mad/core/UKF.hpp"

#include "mad/core/Logger.hpp"

namespace mad {

UKF::UKF(std::shared_ptr<MadModel> modelInput, int stateDim)
    : model(std::move(modelInput)),
      state(Vector::Zero(stateDim)),
      covariance(Matrix::Identity(stateDim, stateDim)),
      processNoise(Matrix::Identity(stateDim, stateDim) * 1e-3),
      measurementNoise(Matrix::Identity(1, 1) * 1e-2) {}

void UKF::predict(double dt) {
  if (!model) {
    return;
  }
  static std::size_t stepCount = 0;
  processNoise = model->processNoise(dt);

  const int n = static_cast<int>(state.size());
  const double lambda = alpha * alpha * (n + kappa) - n;
  const double scale = std::sqrt(n + lambda);

  Matrix sigmaPoints(n, 2 * n + 1);
  sigmaPoints.col(0) = state;

  Eigen::LLT<Matrix> llt(covariance);
  Matrix L = llt.matrixL();
  for (int i = 0; i < n; ++i) {
    sigmaPoints.col(i + 1) = state + scale * L.col(i);
    sigmaPoints.col(i + 1 + n) = state - scale * L.col(i);
  }

  for (int i = 0; i < sigmaPoints.cols(); ++i) {
    if (sigmaPoints.col(i).size() >= 4) {
      sigmaPoints(0, i) = sigmaPoints(0, i) + sigmaPoints(1, i) * dt;
      sigmaPoints(2, i) = sigmaPoints(2, i) + sigmaPoints(3, i) * dt;
    }
  }

  Vector weightsMean = Vector::Constant(2 * n + 1, 1.0 / (2.0 * (n + lambda)));
  Vector weightsCov = weightsMean;
  weightsMean(0) = lambda / (n + lambda);
  weightsCov(0) = weightsMean(0) + (1.0 - alpha * alpha + beta);

  Vector predictedMean = Vector::Zero(n);
  for (int i = 0; i < sigmaPoints.cols(); ++i) {
    predictedMean += weightsMean(i) * sigmaPoints.col(i);
  }

  Matrix predictedCov = Matrix::Zero(n, n);
  for (int i = 0; i < sigmaPoints.cols(); ++i) {
    Vector diff = sigmaPoints.col(i) - predictedMean;
    predictedCov += weightsCov(i) * (diff * diff.transpose());
  }
  predictedCov += processNoise;

  state = predictedMean;
  covariance = predictedCov;
  if ((stepCount % 50) == 0) {
    if (auto logger = Logger::GetClass("UKF")) {
      logger->debug("UKF predict step {} dt {:.6f} state x {:.3f} y {:.3f}", stepCount, dt, state(0), state(2));
    }
  }
  ++stepCount;
}

FilterOutput_t UKF::update(const FilterInput_t& input) {
  if (!model) {
    return {};
  }
  static std::size_t updateCount = 0;
  model->setMeasurementContext(input.measurement);

  const int n = static_cast<int>(state.size());
  const double lambda = alpha * alpha * (n + kappa) - n;
  const double scale = std::sqrt(n + lambda);

  Matrix sigmaPoints(n, 2 * n + 1);
  sigmaPoints.col(0) = state;
  Eigen::LLT<Matrix> llt(covariance);
  Matrix L = llt.matrixL();
  for (int i = 0; i < n; ++i) {
    sigmaPoints.col(i + 1) = state + scale * L.col(i);
    sigmaPoints.col(i + 1 + n) = state - scale * L.col(i);
  }

  Vector weightsMean = Vector::Constant(2 * n + 1, 1.0 / (2.0 * (n + lambda)));
  Vector weightsCov = weightsMean;
  weightsMean(0) = lambda / (n + lambda);
  weightsCov(0) = weightsMean(0) + (1.0 - alpha * alpha + beta);

  Vector zPred = Vector::Zero(1);
  Matrix zSigma(1, 2 * n + 1);
  for (int i = 0; i < sigmaPoints.cols(); ++i) {
    const Vector z = model->predictMeasurement(sigmaPoints.col(i));
    zSigma(0, i) = z(0);
    zPred(0) += weightsMean(i) * z(0);
  }

  Matrix S = Matrix::Zero(1, 1);
  Matrix crossCov = Matrix::Zero(n, 1);
  for (int i = 0; i < sigmaPoints.cols(); ++i) {
    const double dz = zSigma(0, i) - zPred(0);
    Vector dx = sigmaPoints.col(i) - state;
    S(0, 0) += weightsCov(i) * dz * dz;
    crossCov += weightsCov(i) * dx * dz;
  }
  measurementNoise = model->measurementNoise();
  S += measurementNoise;

  const double zMeas = input.measurement.magneticTfc;
  const double residual = zMeas - zPred(0);
  const Matrix K = crossCov * S.inverse();
  state = state + K * Vector::Constant(1, residual);
  covariance = covariance - K * S * K.transpose();

  if ((updateCount % 50) == 0) {
    if (auto logger = Logger::GetClass("UKF")) {
      logger->debug("UKF update step {} z {:.3e} y {:.3e} res {:.3e} |K| {:.3e} x {:.3f} y {:.3f}",
                    updateCount,
                    zMeas,
                    zPred(0),
                    residual,
                    K.norm(),
                    state(0),
                    state(2));
    }
  }
  ++updateCount;

  return FilterOutput_t{state, covariance};
}

void UKF::initialize(const Vector& initialState) {
  if (initialState.size() == state.size()) {
    state = initialState;
  }
}

} // namespace mad




