#include "jrl/Outliers.h"

namespace jrl {

Dataset addOutliers(Dataset dataset, double percOutliers, const boost::optional<std::vector<std::string>> outlierTypes,
                    const boost::optional<std::string> newName) {
  // TODO Insert warning if dataset already has outliers
  std::default_random_engine generator;
  std::bernoulli_distribution sampler(percOutliers);

  std::map<char, std::vector<Entry>> allNewMeasurements;
  // Iterate over all robots
  for (auto& robot : dataset.robots()) {
    std::vector<Entry> robotMeasurements = dataset.measurements(robot);
    // Iterate over each entry
    for (uint64_t i = 0; i < robotMeasurements.size(); ++i) {
      Entry& entry = robotMeasurements[i];
      // Finally over each factor
      for (uint64_t j = 0; j < entry.measurement_types.size(); ++j) {
        if (sampler(generator) &&
            (!outlierTypes.is_initialized() || std::find(outlierTypes->begin(), outlierTypes->end(),
                                                         entry.measurement_types[j]) != outlierTypes->end())) {
          entry.is_outlier[j] = true;
          gtsam::NonlinearFactor::shared_ptr outlierFactor =
              perturbFactor(entry.measurements.at(j), entry.measurement_types[j]);
          entry.measurements.replace(j, outlierFactor);
        }
      }
    }
    allNewMeasurements[robot] = robotMeasurements;
  }

  // Copy over ground truth/initialization
  boost::optional<std::map<char, TypedValues>> groundTruth = boost::none;
  if (dataset.containsGroundTruth()) {
    groundTruth = std::map<char, TypedValues>();
    for (auto& robot : dataset.robots()) {
      groundTruth.get()[robot] = dataset.groundTruthWithTypes(robot);
    }
  }

  boost::optional<std::map<char, TypedValues>> initialization = boost::none;
  if (dataset.containsInitialization()) {
    initialization = std::map<char, TypedValues>();
    for (auto& robot : dataset.robots()) {
      initialization.get()[robot] = dataset.initializationWithTypes(robot);
    }
  }

  std::string name = newName.is_initialized() ? newName.get() : dataset.name();

  return Dataset(name, dataset.robots(), allNewMeasurements, groundTruth, initialization);
}

gtsam::NonlinearFactor::shared_ptr perturbFactor(gtsam::NonlinearFactor::shared_ptr factor, std::string tag) {
  // Get helpers
  auto measurement_serializer = jrl::Writer().getDefaultMeasurementSerializer();
  auto value_serializer = jrl::Writer().getDefaultValueSerializer();
  auto measurement_parser = jrl::Parser().getDefaultMeasurementParsers();
  auto value_parser = jrl::Parser().getDefaultValueAccumulators();

  // Convert to json
  json measurementSerialized = measurement_serializer[tag](factor);

  // Get tag of saved measurement
  gtsam::Key key = 0;
  std::string measured_tag = measurementSerialized["measurement"]["type"].get<std::string>();

  // Get value of measured/covariance
  gtsam::Values valuesTemp;
  value_parser[measured_tag](measurementSerialized["measurement"], key, valuesTemp);
  gtsam::Matrix cov = io_measurements::parseCovariance(measurementSerialized["covariance"], valuesTemp.dim());

  // Get how far away for 99th percentile
  Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
  Eigen::VectorXd perturb =
      es.eigenvectors().real().colwise().sum() *
      std::sqrt(-2 * std::log(1 - 0.999999999) / es.eigenvalues().real().cwiseInverse().squaredNorm());

  // Perturb
  gtsam::VectorValues delta;
  delta.insert(key, perturb);
  valuesTemp = valuesTemp.retract(delta);

  // Put perturbed value back in
  measurementSerialized["measurement"] = value_serializer[measured_tag](key, valuesTemp);

  // Finally convert back to factor
  gtsam::NonlinearFactor::shared_ptr outlier = measurement_parser[tag](measurementSerialized);
  return outlier;
}

}  // namespace jrl