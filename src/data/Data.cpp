#include "mad/data/DataSource.hpp"

namespace mad {

bool readNext(IDataSource& source, Measurement_t& out) {
  return source.next(out);
}

} // namespace mad




