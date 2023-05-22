#pragma once
#include <gtsam/nonlinear/Values.h>

namespace jrl {

typedef std::map<gtsam::Key, std::string> ValueTypes;

struct TypedValues {
  gtsam::Values values;
  ValueTypes types;

  TypedValues(gtsam::Values values, ValueTypes types) : values(values), types(types) {}
  TypedValues() {}
};

struct Entry {
  uint64_t stamp;
  std::vector<std::string> measurement_types;
  std::vector<bool> is_outlier;
  gtsam::NonlinearFactorGraph measurements;
  Entry(uint64_t stamp, std::vector<std::string> measurement_types, gtsam::NonlinearFactorGraph measurements,
        boost::optional<std::vector<bool>> outliers = boost::none)
      : stamp(stamp), measurement_types(measurement_types), measurements(measurements) {
    is_outlier = outliers.is_initialized() ? outliers.get() : std::vector<bool>(measurements.size(), false);
  }
};

}  // namespace jrl
