#include "mad/pipeline/Pipeline.hpp"

namespace mad {

Pipeline::Pipeline(std::shared_ptr<IDataSource> dataSource,
                   std::shared_ptr<IFilter> filter,
                   std::shared_ptr<ITracker> tracker)
    : dataSource(std::move(dataSource)),
      filter(std::move(filter)),
      tracker(std::move(tracker)) {}

void Pipeline::run() {
  Measurement_t measurement;
  double lastTime = 0.0;
  while (dataSource && dataSource->next(measurement)) {
    const double dt = measurement.time - lastTime;
    if (filter) {
      filter->predict(dt);
      FilterInput_t input{measurement, dt};
      FilterOutput_t output = filter->update(input);
      if (tracker) {
        TrackState_t trackState;
        trackState.id = 0;
        trackState.state = output.state;
        trackState.covariance = output.covariance;
        tracker->step(trackState, measurement.time);
      }
    }
    lastTime = measurement.time;
  }
}

} // namespace mad




