#include "mad/core/ParticleFilter.hpp"

#include "mad/core/Logger.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace mad {

namespace {

constexpr double kPi = 3.14159265358979323846;

Vector sampleGaussian(const Vector& mean, const Matrix& cov, std::mt19937& rng) {
  Eigen::LLT<Matrix> llt(cov);
  Matrix L = llt.matrixL();
  std::normal_distribution<double> dist(0.0, 1.0);
  Vector z(mean.size());
  for (int i = 0; i < z.size(); ++i) {
    z(i) = dist(rng);
  }
  return mean + L * z;
}

Matrix sampleCovariance(const std::vector<Particle_t>& particles) {
  if (particles.empty()) {
    return Matrix();
  }

  const int dim = static_cast<int>(particles.front().state.size());
  Vector mean = Vector::Zero(dim);
  for (const auto& p : particles) {
    mean += p.weight * p.state;
  }

  Matrix cov = Matrix::Zero(dim, dim);
  for (const auto& p : particles) {
    Vector diff = p.state - mean;
    cov += p.weight * (diff * diff.transpose());
  }

  return cov;
}

double logGaussian(const Vector& residual, const Matrix& cov) {
  const int dim = static_cast<int>(residual.size());
  const double logDet = std::log(cov.determinant() + 1e-12);
  const double quad = residual.transpose() * cov.inverse() * residual;
  return -0.5 * (dim * std::log(2.0 * kPi) + logDet + quad);
}

Vector numericJacobian(const StateSpaceModel& model, const Vector& state, double eps = 1e-4) {
  const Vector base = model.predictMeasurement(state);
  const int m = static_cast<int>(base.size());
  const int n = static_cast<int>(state.size());
  Matrix J(m, n);
  for (int i = 0; i < n; ++i) {
    Vector pert = state;
    pert(i) += eps;
    Vector meas = model.predictMeasurement(pert);
    J.col(i) = (meas - base) / eps;
  }
  Vector flat = Vector::Zero(m * n);
  for (int r = 0; r < m; ++r) {
    for (int c = 0; c < n; ++c) {
      flat(r * n + c) = J(r, c);
    }
  }
  return flat;
}

Matrix unpackJacobian(const Vector& flat, int rows, int cols) {
  Matrix J(rows, cols);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      J(r, c) = flat(r * cols + c);
    }
  }
  return J;
}

} // namespace

ParticleFilterBase::ParticleFilterBase(std::shared_ptr<StateSpaceModel> model, ParticleFilterOptions_t options)
    : model(std::move(model)), options(std::move(options)), rng(std::random_device{}()) {
  const int n = options.numParticles;
  Vector initialState;
  if (options.initialState.size() > 0) {
    initialState = options.initialState;
  } else if (options.stateDimension > 0) {
    initialState = Vector::Zero(options.stateDimension);
  }
  particles.resize(n);
  for (auto& p : particles) {
    if (initialState.size() > 0) {
      p.state = initialState;
    }
    p.weight = 1.0 / n;
  }
}

void ParticleFilterBase::predict(double dt) {
  if (!model) {
    return;
  }
  for (auto& p : particles) {
    p.state = model->propagate(p.state, dt, rng);
  }
}

void ParticleFilterBase::initialize(const Vector& initialState) {
  if (particles.empty()) {
    return;
  }
  for (auto& p : particles) {
    p.state = initialState;
    p.weight = 1.0 / particles.size();
  }
}

FilterOutput_t ParticleFilterBase::update(const FilterInput_t& input) {
  if (!model) {
    return {};
  }

  static std::size_t updateCount = 0;
  model->setMeasurementContext(input.measurement);

  for (auto& p : particles) {
    p.state = propose(p.state, input.dt, input);
    const Vector yhat = model->predictMeasurement(p.state);
    const Vector y = Vector::Constant(yhat.size(), input.measurement.magneticTfc);
    p.weight = std::exp(logLikelihood(yhat, y));
  }

  normalizeWeights();

  const bool needsResample = (options.resamplePolicy == ResamplePolicy_e::kAlways) ||
                             (options.resamplePolicy == ResamplePolicy_e::kAdaptive &&
                              effectiveSampleSizeInternal() < options.essThresholdRatio * particles.size());
  if (needsResample) {
    resample();
    afterResample();
  }

  const int dim = static_cast<int>(particles.front().state.size());
  Vector mean = Vector::Zero(dim);
  for (const auto& p : particles) {
    mean += p.weight * p.state;
  }
  Matrix cov = sampleCovariance(particles);

  if ((updateCount % 100) == 0) {
    if (auto logger = Logger::Get()) {
      logger->debug("PF update step {} ess {:.1f} mean x {:.3f} y {:.3f}",
                    updateCount,
                    effectiveSampleSizeInternal(),
                    mean.size() > 0 ? mean(0) : 0.0,
                    mean.size() > 2 ? mean(2) : 0.0);
    }
  }
  ++updateCount;

  return FilterOutput_t{mean, cov};
}

Vector ParticleFilterBase::propose(const Vector& state, double dt, const FilterInput_t& /*input*/) {
  return model->propagate(state, dt, rng);
}

void ParticleFilterBase::afterResample() {}

double ParticleFilterBase::effectiveSampleSizeInternal() const {
  double sum = 0.0;
  for (const auto& p : particles) {
    sum += p.weight * p.weight;
  }
  return (sum > 0.0) ? 1.0 / sum : 0.0;
}

void ParticleFilterBase::normalizeWeights() {
  double sum = 0.0;
  for (const auto& p : particles) {
    sum += p.weight;
  }
  if (sum <= 0.0) {
    const double w = 1.0 / particles.size();
    for (auto& p : particles) {
      p.weight = w;
    }
    return;
  }
  for (auto& p : particles) {
    p.weight /= sum;
  }
}

void ParticleFilterBase::resample() {
  std::vector<Particle_t> resampled;
  resampled.resize(particles.size());

  if (options.resampleMethod == ResampleMethod_e::kSystematic) {
    std::uniform_real_distribution<double> dist(0.0, 1.0 / particles.size());
    double r = dist(rng);
    double c = particles.front().weight;
    size_t i = 0;
    for (size_t m = 0; m < particles.size(); ++m) {
      double u = r + m * (1.0 / particles.size());
      while (u > c && i + 1 < particles.size()) {
        ++i;
        c += particles[i].weight;
      }
      resampled[m].state = particles[i].state;
      resampled[m].weight = 1.0 / particles.size();
    }
  } else {
    std::vector<double> cdf(particles.size());
    double sum = 0.0;
    for (size_t i = 0; i < particles.size(); ++i) {
      sum += particles[i].weight;
      cdf[i] = sum;
    }
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    for (size_t m = 0; m < particles.size(); ++m) {
      double u = dist(rng);
      auto it = std::lower_bound(cdf.begin(), cdf.end(), u);
      size_t idx = static_cast<size_t>(std::distance(cdf.begin(), it));
      resampled[m].state = particles[idx].state;
      resampled[m].weight = 1.0 / particles.size();
    }
  }

  particles = std::move(resampled);
}

double ParticleFilterBase::logLikelihood(const Vector& predicted, const Vector& measured) {
  const Matrix R = model->measurementNoise();
  return logGaussian(measured - predicted, R);
}

void SIRParticleFilter::afterResample() {}

SISParticleFilter::SISParticleFilter(std::shared_ptr<StateSpaceModel> model, ParticleFilterOptions_t options)
    : ParticleFilterBase(std::move(model), std::move(options)) {
  this->options.resamplePolicy = ResamplePolicy_e::kNever;
}

void SISParticleFilter::afterResample() {
  options.resamplePolicy = ResamplePolicy_e::kNever;
}

FilterOutput_t AuxiliaryParticleFilter::update(const FilterInput_t& input) {
  if (!model) {
    return {};
  }
  model->setMeasurementContext(input.measurement);

  std::vector<double> auxWeights(particles.size());
  for (size_t i = 0; i < particles.size(); ++i) {
    const Vector pred = model->propagate(particles[i].state, input.dt, rng);
    const Vector yhat = model->predictMeasurement(pred);
    const Vector y = Vector::Constant(yhat.size(), input.measurement.magneticTfc);
    auxWeights[i] = std::exp(logLikelihood(yhat, y)) * particles[i].weight;
  }

  double sum = std::accumulate(auxWeights.begin(), auxWeights.end(), 0.0);
  if (sum <= 0.0) {
    return ParticleFilterBase::update(input);
  }
  for (double& w : auxWeights) {
    w /= sum;
  }

  std::vector<Particle_t> resampled;
  resampled.resize(particles.size());
  std::vector<double> cdf(auxWeights.size());
  double csum = 0.0;
  for (size_t i = 0; i < auxWeights.size(); ++i) {
    csum += auxWeights[i];
    cdf[i] = csum;
  }

  std::uniform_real_distribution<double> dist(0.0, 1.0);
  for (size_t m = 0; m < particles.size(); ++m) {
    double u = dist(rng);
    auto it = std::lower_bound(cdf.begin(), cdf.end(), u);
    size_t idx = static_cast<size_t>(std::distance(cdf.begin(), it));
    resampled[m] = particles[idx];
    resampled[m].weight = 1.0 / particles.size();
  }

  particles = std::move(resampled);

  for (auto& p : particles) {
    p.state = propose(p.state, input.dt, input);
    const Vector yhat = model->predictMeasurement(p.state);
    const Vector y = Vector::Constant(yhat.size(), input.measurement.magneticTfc);
    p.weight = std::exp(logLikelihood(yhat, y));
  }

  normalizeWeights();

  const int dim = static_cast<int>(particles.front().state.size());
  Vector mean = Vector::Zero(dim);
  for (const auto& p : particles) {
    mean += p.weight * p.state;
  }
  Matrix cov = sampleCovariance(particles);

  return FilterOutput_t{mean, cov};
}

void RegularizedParticleFilter::afterResample() {
  Matrix cov = sampleCovariance(particles);
  if (cov.size() == 0) {
    return;
  }
  cov *= options.regularizationBandwidth * options.regularizationBandwidth;
  for (auto& p : particles) {
    p.state = sampleGaussian(p.state, cov, rng);
  }
}

void AdaptiveResamplingParticleFilter::afterResample() {}

double RobustParticleFilter::logLikelihood(const Vector& predicted, const Vector& measured) {
  const Matrix R = model->measurementNoise();
  const Vector residual = measured - predicted;
  const double dof = std::max(1.0, options.studentTDof);
  const double quad = residual.transpose() * R.inverse() * residual;
  return -0.5 * (dof + residual.size()) * std::log1p(quad / dof);
}

double GaussianMixtureParticleFilter::logLikelihood(const Vector& predicted, const Vector& measured) {
  if (options.mixture.empty()) {
    return ParticleFilterBase::logLikelihood(predicted, measured);
  }
  const Vector residual = measured - predicted;
  double total = 0.0;
  for (const auto& comp : options.mixture) {
    total += comp.weight * std::exp(logGaussian(residual, comp.covariance));
  }
  return std::log(total + 1e-12);
}

Vector EkfParticleFilter::propose(const Vector& state, double dt, const FilterInput_t& input) {
  Vector xPred = model->propagate(state, dt, rng);
  Vector zPred = model->predictMeasurement(xPred);
  const int m = static_cast<int>(zPred.size());
  const int n = static_cast<int>(xPred.size());
  const Vector flat = numericJacobian(*model, xPred);
  Matrix H = unpackJacobian(flat, m, n);

  Matrix P = options.proposalCovariance;
  if (P.size() == 0) {
    P = Matrix::Identity(n, n) * 1e-2;
  }
  Matrix R = model->measurementNoise();
  Matrix S = H * P * H.transpose() + R;
  Matrix K = P * H.transpose() * S.inverse();
  Vector z = Vector::Constant(m, input.measurement.magneticTfc);
  Vector xUpdated = xPred + K * (z - zPred);
  Matrix pUpdated = (Matrix::Identity(n, n) - K * H) * P;

  return sampleGaussian(xUpdated, pUpdated, rng);
}

Vector UkfParticleFilter::propose(const Vector& state, double dt, const FilterInput_t& input) {
  Vector mean = model->propagate(state, dt, rng);
  const int n = static_cast<int>(mean.size());
  Matrix P = options.proposalCovariance;
  if (P.size() == 0) {
    P = Matrix::Identity(n, n) * 1e-2;
  }

  const double alpha = 1e-3;
  const double kappa = 0.0;
  const double beta = 2.0;
  const double lambda = alpha * alpha * (n + kappa) - n;

  Matrix A = ((n + lambda) * P).llt().matrixL();
  std::vector<Vector> sigmaPoints(2 * n + 1, mean);
  for (int i = 0; i < n; ++i) {
    sigmaPoints[i + 1] = mean + A.col(i);
    sigmaPoints[i + 1 + n] = mean - A.col(i);
  }

  std::vector<Vector> zSigma;
  zSigma.reserve(sigmaPoints.size());
  for (const auto& sp : sigmaPoints) {
    zSigma.push_back(model->predictMeasurement(sp));
  }

  const int m = static_cast<int>(zSigma.front().size());
  Vector zMean = Vector::Zero(m);
  double w0m = lambda / (n + lambda);
  double w0c = w0m + (1.0 - alpha * alpha + beta);
  double wi = 1.0 / (2.0 * (n + lambda));

  zMean += w0m * zSigma.front();
  for (size_t i = 1; i < zSigma.size(); ++i) {
    zMean += wi * zSigma[i];
  }

  Matrix S = Matrix::Zero(m, m);
  S += w0c * (zSigma.front() - zMean) * (zSigma.front() - zMean).transpose();
  for (size_t i = 1; i < zSigma.size(); ++i) {
    Vector dz = zSigma[i] - zMean;
    S += wi * (dz * dz.transpose());
  }
  S += model->measurementNoise();

  Matrix Pxz = Matrix::Zero(n, m);
  Vector dz0 = zSigma.front() - zMean;
  Pxz += w0c * (sigmaPoints.front() - mean) * dz0.transpose();
  for (size_t i = 1; i < zSigma.size(); ++i) {
    Vector dz = zSigma[i] - zMean;
    Pxz += wi * (sigmaPoints[i] - mean) * dz.transpose();
  }

  Matrix K = Pxz * S.inverse();
  Vector z = Vector::Constant(m, input.measurement.magneticTfc);
  Vector xUpdated = mean + K * (z - zMean);
  Matrix pUpdated = P - K * S * K.transpose();

  return sampleGaussian(xUpdated, pUpdated, rng);
}

} // namespace mad





