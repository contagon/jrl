#include "jrl/Types.h"

namespace jrl {
/**********************************************************************************************************************/
Entry::Entry(uint64_t stamp, std::vector<std::string> measurement_types, gtsam::NonlinearFactorGraph measurements)
    : stamp(stamp), measurement_types(measurement_types), measurements(measurements) {}

/**********************************************************************************************************************/
Entry Entry::remove(std::vector<std::string> remove_types) {
  std::vector<std::string> out_measurement_types;
  std::vector<bool> out_is_outlier;
  gtsam::NonlinearFactorGraph out_measurements;

  for (uint64_t i = 0; i < measurement_types.size(); ++i) {
    // If the type isn't in remove_types, keep it
    if (std::find(remove_types.begin(), remove_types.end(), measurement_types[i]) == remove_types.end()) {
      out_measurement_types.push_back(measurement_types[i]);
      out_measurements.push_back(measurements[i]);
    }
  }

  return Entry(stamp, out_measurement_types, out_measurements);
}

/**********************************************************************************************************************/
Entry Entry::filter(std::vector<std::string> filter_types) {
  std::vector<std::string> out_measurement_types;
  std::vector<bool> out_is_outlier;
  gtsam::NonlinearFactorGraph out_measurements;

  for (uint64_t i = 0; i < measurement_types.size(); ++i) {
    // If the type is in filter_types, keep it
    if (std::find(filter_types.begin(), filter_types.end(), measurement_types[i]) != filter_types.end()) {
      out_measurement_types.push_back(measurement_types[i]);
      out_measurements.push_back(measurements[i]);
    }
  }

  return Entry(stamp, out_measurement_types, out_measurements);
}

}  // namespace jrl