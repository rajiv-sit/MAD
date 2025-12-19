#pragma once

#include <fstream>
#include <string>

#include <Eigen/Dense>

#include "mad/data/DataSource.hpp"

namespace mad {

// Reads the raw MAD ASCII dataset and yields measurements sequentially.
class AsciiDataSource : public IDataSource {
public:
  explicit AsciiDataSource(const std::string& path);

  bool next(Measurement_t& out) override;
  bool good() const;

private:
  std::ifstream fileStream;
  Eigen::Vector3d lastTruthPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d refEcef = Eigen::Vector3d::Zero();
  double refLatRad = 0.0;
  double refLonRad = 0.0;
  bool hasReference = false;
  bool hasPrevTruth = false;
  double samplePeriod = 1.0 / 32.0;
  std::size_t lineNumber = 0;
  std::size_t invalidLineCount = 0;
};

} // namespace mad




