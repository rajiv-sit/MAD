#include "mad/data/AsciiDataSource.hpp"

#include <cmath>
#include <sstream>

#include <Eigen/Dense>

#include "mad/core/Logger.hpp"

namespace mad {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kFeetToMeters = 0.3048;
constexpr double kFlattening = 0.00335;
constexpr double kEquatorialRadius = 6378100.0;

Eigen::Vector3d llaToEcef(double latDeg, double lonDeg, double altM) {
  const double e2 = kFlattening * (2.0 - kFlattening);

  const double lat = latDeg * kDegToRad;
  const double lon = lonDeg * kDegToRad;

  const double sinLat = std::sin(lat);
  const double cosLat = std::cos(lat);
  const double sinLon = std::sin(lon);
  const double cosLon = std::cos(lon);

  const double N = kEquatorialRadius / std::sqrt(1.0 - e2 * sinLat * sinLat);
  const double x = (N + altM) * cosLat * cosLon;
  const double y = (N + altM) * cosLat * sinLon;
  const double z = (N * (1.0 - e2) + altM) * sinLat;

  return {x, y, z};
}

Eigen::Vector3d ecefToEnu(const Eigen::Vector3d& ecef,
                          const Eigen::Vector3d& refEcef,
                          double refLatRad,
                          double refLonRad) {
  const double sinLat = std::sin(refLatRad);
  const double cosLat = std::cos(refLatRad);
  const double sinLon = std::sin(refLonRad);
  const double cosLon = std::cos(refLonRad);

  Eigen::Vector3d diff = ecef - refEcef;
  const double east = -sinLon * diff.x() + cosLon * diff.y();
  const double north = -sinLat * cosLon * diff.x() - sinLat * sinLon * diff.y() + cosLat * diff.z();
  const double up = cosLat * cosLon * diff.x() + cosLat * sinLon * diff.y() + sinLat * diff.z();
  return {east, north, up};
}

} // namespace

AsciiDataSource::AsciiDataSource(const std::string& path) : fileStream(path) {
  if (auto logger = Logger::GetClass("AsciiDataSource")) {
    logger->info("AsciiDataSource opening {}", path);
  }
}

bool AsciiDataSource::good() const {
  return fileStream.good();
}

bool AsciiDataSource::next(Measurement_t& out) {
  std::string line;
  while (std::getline(fileStream, line)) {
    ++lineNumber;
    if (line.empty()) {
      continue;
    }

    std::istringstream iss(line);
    double time = 0.0;
    double tfc = 0.0;
    double acLat = 0.0;
    double acLon = 0.0;
    double acHeightFt = 0.0;
    double acHead = 0.0;
    double shipLat = 0.0;
    double shipLon = 0.0;
    double shipHeightFt = 0.0;
    double shipHead = 0.0;

    if (!(iss >> time >> tfc >> acLat >> acLon >> acHeightFt >> acHead >> shipLat >> shipLon >> shipHeightFt >> shipHead)) {
      ++invalidLineCount;
      if (invalidLineCount % 100 == 0) {
        if (auto logger = Logger::Get()) {
          logger->warn("Skipping invalid dataset line {} ({} errors so far).", lineNumber, invalidLineCount);
        }
      }
      continue;
    }

    const double acHeightM = acHeightFt * kFeetToMeters;
    const double shipHeightM = shipHeightFt * kFeetToMeters;
    out.time = time;
    // Convert nT to Tesla for consistent physical units.
    out.magneticTfc = tfc * 1e-9;
    const Eigen::Vector3d sensorEcef = llaToEcef(acLat, acLon, acHeightM);
    if (!hasReference) {
      refLatRad = acLat * kDegToRad;
      refLonRad = acLon * kDegToRad;
      refEcef = sensorEcef;
      hasReference = true;
      if (auto logger = Logger::Get()) {
        logger->info("Reference LLA set to lat {:.6f} lon {:.6f} alt {:.2f} m",
                     acLat,
                     acLon,
                     acHeightM);
      }
    }
    out.sensorPosEcef = ecefToEnu(sensorEcef, refEcef, refLatRad, refLonRad);

    const Eigen::Vector3d shipEcef = llaToEcef(shipLat, shipLon, shipHeightM);
    const Eigen::Vector3d shipPos = ecefToEnu(shipEcef, refEcef, refLatRad, refLonRad);
    Eigen::VectorXd truth = Eigen::VectorXd::Zero(10);
    truth(0) = shipPos.x();
    truth(2) = shipPos.y();
    truth(4) = 1e7;
    truth(5) = 1e6;
    truth(6) = 1e7;
    truth(7) = 800.0;
    truth(8) = 100.0;
    truth(9) = 1400.0;

    if (hasPrevTruth) {
      truth(1) = (truth(0) - lastTruthPos.x()) / samplePeriod;
      truth(3) = (truth(2) - lastTruthPos.y()) / samplePeriod;
    }

    lastTruthPos = shipPos;
    hasPrevTruth = true;

    out.truthState = truth;
    out.truthPosEcef = shipPos;
    out.hasTruth = true;
    return true;
  }

  return false;
}

} // namespace mad




