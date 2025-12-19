#pragma once

#include <vector>

#include "mad/tracking/Tracker.hpp"

namespace mad {

class NoopTracker : public ITracker {
public:
  void step(const TrackState_t& input, double /*time*/) override {
    trackStates.clear();
    trackStates.push_back(input);
  }

  const std::vector<TrackState_t>& tracks() const override { return trackStates; }

private:
  std::vector<TrackState_t> trackStates;
};

} // namespace mad



