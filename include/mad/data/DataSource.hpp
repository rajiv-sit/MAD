#pragma once

#include "mad/data/Measurement.hpp"

namespace mad {

// Streaming measurement source interface.
class IDataSource {
public:
  virtual ~IDataSource() = default;
  virtual bool next(Measurement_t& out) = 0;
};

} // namespace mad


