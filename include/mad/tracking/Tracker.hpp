#pragma once

#include <vector>

#include "mad/core/Types.hpp"

namespace mad {

struct TrackState_t {
  int id = -1;
  Vector state;
  Matrix covariance;
};

// Track management interface for downstream consumers.
class ITracker {
public:
  virtual ~ITracker() = default;
  virtual void step(const TrackState_t& input, double time) = 0;
  virtual const std::vector<TrackState_t>& tracks() const = 0;
};

} // namespace mad


