#pragma once

#include <fstream>
#include <string>

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
};

} // namespace mad




