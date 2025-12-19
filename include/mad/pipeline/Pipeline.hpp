#pragma once

#include <memory>

#include "mad/core/Filter.hpp"
#include "mad/data/DataSource.hpp"
#include "mad/tracking/Tracker.hpp"

namespace mad {

// Orchestrates data ingestion, filtering, and tracking.
class Pipeline {
public:
  Pipeline(std::shared_ptr<IDataSource> dataSource,
           std::shared_ptr<IFilter> filter,
           std::shared_ptr<ITracker> tracker,
           std::size_t maxScans = 0);

  void run();

private:
  std::shared_ptr<IDataSource> dataSource;
  std::shared_ptr<IFilter> filter;
  std::shared_ptr<ITracker> tracker;
  std::size_t maxScans = 0;
};

} // namespace mad




