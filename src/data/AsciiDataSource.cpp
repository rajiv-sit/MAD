#include "mad/data/AsciiDataSource.hpp"

#include <cmath>
#include <sstream>

#include <Eigen/Dense>

namespace mad {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kFeetToMeters = 0.3048;

Eigen::Vector3d llaToEcef(double latDeg, double lonDeg, double altM) {
  const double a = 6378137.0;
  const double f = 1.0 / 298.257223563;
  const double e2 = f * (2.0 - f);

  const double lat = latDeg * kDegToRad;
  const double lon = lonDeg * kDegToRad;

  const double sinLat = std::sin(lat);
  const double cosLat = std::cos(lat);
  const double sinLon = std::sin(lon);
  const double cosLon = std::cos(lon);

  const double N = a / std::sqrt(1.0 - e2 * sinLat * sinLat);
  const double x = (N + altM) * cosLat * cosLon;
  const double y = (N + altM) * cosLat * sinLon;
  const double z = (N * (1.0 - e2) + altM) * sinLat;

  return {x, y, z};
}

} // namespace

AsciiDataSource::AsciiDataSource(const std::string& path) : fileStream(path) {}

bool AsciiDataSource::good() const {
  return fileStream.good();
}

bool AsciiDataSource::next(Measurement_t& out) {
  std::string line;
  while (std::getline(fileStream, line)) {
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
      continue;
    }

    const double acHeightM = acHeightFt * kFeetToMeters;
    out.time = time;
    // Convert nT to Tesla for consistent physical units.
    out.magneticTfc = tfc * 1e-9;
    out.sensorPosEcef = llaToEcef(acLat, acLon, acHeightM);
    return true;
  }

  return false;
}

} // namespace mad




