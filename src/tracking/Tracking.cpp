#include "mad/tracking/Tracker.hpp"
#include "mad/core/Logger.hpp"

namespace mad {

class NullTracker : public ITracker {
public:
  void step(const TrackState_t& input, double /*time*/) override {
    trackStates.clear();
    trackStates.push_back(input);
    if (auto logger = Logger::GetClass("Tracker")) {
      logger->debug("NullTracker step id {} state_dim {}", input.id, static_cast<int>(input.state.size()));
    }
  }

  const std::vector<TrackState_t>& tracks() const override { return trackStates; }

private:
  std::vector<TrackState_t> trackStates;
};

} // namespace mad



