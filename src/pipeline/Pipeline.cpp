#include "mad/pipeline/Pipeline.hpp"

#include "mad/core/Logger.hpp"

namespace mad {

Pipeline::Pipeline(std::shared_ptr<IDataSource> dataSource,
                   std::shared_ptr<IFilter> filter,
                   std::shared_ptr<ITracker> tracker,
                   std::size_t maxScansInput)
    : dataSource(std::move(dataSource)),
      filter(std::move(filter)),
      tracker(std::move(tracker)),
      maxScans(maxScansInput) {
  if (auto logger = Logger::GetClass("Pipeline")) {
    logger->info("Pipeline created maxScans {}", maxScans == 0 ? -1 : static_cast<int>(maxScans));
  }
}

void Pipeline::run() {
  Measurement_t measurement;
  double lastTime = 0.0;
  bool hasLastTime = false;
  std::size_t stepCount = 0;
  while (dataSource && dataSource->next(measurement)) {
    if (maxScans > 0 && stepCount >= maxScans) {
    if (auto logger = Logger::GetClass("Pipeline")) {
      logger->info("Pipeline: reached max scans {}", maxScans);
    }
      break;
    }
    double dt = 0.0;
    if (hasLastTime) {
      dt = measurement.time - lastTime;
    } else {
      hasLastTime = true;
    }
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
    if (auto logger = Logger::GetClass("Pipeline")) {
      if ((stepCount % 200) == 0) {
        logger->debug("Pipeline step {} time {:.3f} dt {:.6f}", stepCount, measurement.time, dt);
      }
    }
    lastTime = measurement.time;
    ++stepCount;
  }
  if (auto logger = Logger::GetClass("Pipeline")) {
    logger->info("Pipeline completed after {} scans", stepCount);
  }
}

} // namespace mad




