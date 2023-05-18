#include "jrl/Parser.h"

#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/VectorValues.h>

#include <fstream>

#include "jrl/IOMeasurements.h"
#include "jrl/IOValues.h"
#include "jrl/Writer.h"

using namespace jrl::io_measurements;
using namespace jrl::io_values;

namespace jrl {

Parser::Parser() {
  value_accumulators_ = loadDefaultValueAccumulators();
  measurement_parsers_ = loadDefaultMeasurementParsers();
}

/**********************************************************************************************************************/
std::map<std::string, ValueParser> Parser::loadDefaultValueAccumulators() {
  // clang-format off
  std::map<std::string, ValueParser> parser_functions = {
      {Pose2Tag,   [](json input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::Pose2>(&parse<gtsam::Pose2>, input, key, accum); }},
      {Pose3Tag,   [](json input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::Pose3>(&parse<gtsam::Pose3>, input, key, accum); }},
      {Point2Tag,  [](json input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::Point2>(&parse<gtsam::Point2>, input, key, accum); }},
      {Point3Tag,  [](json input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::Point3>(&parse<gtsam::Point3>, input, key, accum); }},
      {VectorTag,  [](json input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::Vector>(&parse<gtsam::Vector>, input, key, accum); }},
      {ScalarTag,  [](json input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<double>(&parse<double>, input, key, accum); }},
      {IMUBiasTag, [](json input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::imuBias::ConstantBias>(&parse<gtsam::imuBias::ConstantBias>, input, key, accum); }},
  };
  // clang-format on
  return parser_functions;
}

/**********************************************************************************************************************/
std::map<std::string, MeasurementParser> Parser::loadDefaultMeasurementParsers() {
  // clang-format off
  std::map<std::string, MeasurementParser> parser_functions = {
      {PriorFactorPose2Tag,         [](json input) { return parsePrior<gtsam::Pose2>(&parse<gtsam::Pose2>, input); }},
      {PriorFactorPose3Tag,         [](json input) { return parsePrior<gtsam::Pose3>(&parse<gtsam::Pose3>, input); }},
      {BetweenFactorPose2Tag,       [](json input) { return parseNoiseModel2<gtsam::Pose2, gtsam::BetweenFactor<gtsam::Pose2>>(&parse<gtsam::Pose2>, input); }},
      {BetweenFactorPose3Tag,       [](json input) { return parseNoiseModel2<gtsam::Pose3, gtsam::BetweenFactor<gtsam::Pose3>>(&parse<gtsam::Pose3>, input); }},
      {RangeFactorPose2Tag,         [](json input) { return parseNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose2>>(&parse<double>, input); }},
      {RangeFactorPose3Tag,         [](json input) { return parseNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose3>>(&parse<double>, input); }},
      {BearingRangeFactorPose2Tag,  [](json input) { return parseNoiseModel2<gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2>, gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2>>(&parseBearingRange<gtsam::Pose2, gtsam::Pose2>, input); }},
      {BearingRangeFactorPose3Tag,  [](json input) { return parseNoiseModel2<gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3>, gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3>>(&parseBearingRange<gtsam::Pose3, gtsam::Pose3>, input); }},
      {PriorFactorPoint2Tag,        [](json input) { return parsePrior<gtsam::Point2>(&parse<gtsam::Point2>, input); }},
      {PriorFactorPoint3Tag,        [](json input) { return parsePrior<gtsam::Point3>(&parse<gtsam::Point3>, input); }},
      {BetweenFactorPoint2Tag,      [](json input) { return parseNoiseModel2<gtsam::Point2, gtsam::BetweenFactor<gtsam::Point2>>(&parse<gtsam::Point2>, input); }},
      {BetweenFactorPoint3Tag,      [](json input) { return parseNoiseModel2<gtsam::Point3, gtsam::BetweenFactor<gtsam::Point3>>(&parse<gtsam::Point3>, input); }},
      {StereoFactorPose3Point3Tag,  [](json input) { return parseStereoFactor<gtsam::Pose3, gtsam::Point3>(&parse<gtsam::Pose3>, input); }},
      {PriorFactorIMUBiasTag,       [](json input) { return parsePrior<gtsam::imuBias::ConstantBias>(&parse<gtsam::imuBias::ConstantBias>, input); }},
      {CombinedIMUTag,              [](json input) { return parseCombinedIMUFactor(input); }},
  };
  // clang-format on
  return parser_functions;
}

/**********************************************************************************************************************/
TypedValues Parser::parseValues(json values_json) {
  gtsam::Values values;
  ValueTypes value_types;
  for (auto& value_element : values_json) {
    gtsam::Key key = value_element["key"].get<uint64_t>();
    std::string type_tag = value_element["type"].get<std::string>();
    value_types[key] = type_tag;
    value_accumulators_[type_tag](value_element, key, values);
  }
  return TypedValues(values, value_types);
}

/**********************************************************************************************************************/
std::vector<Entry> Parser::parseMeasurements(json measurements_json, double outlier_percent, const boost::optional<std::vector<std::string>> outlier_types) {
  std::vector<Entry> measurements;
  std::default_random_engine generator;
  std::bernoulli_distribution sampler(outlier_percent);
  std::map<std::string, ValueSerializer> value_serializer_ = jrl::Writer().getDefaultValueSerializer();

  // Iterate through each stamp
  for (auto& entry_element : measurements_json) {
    uint64_t stamp = entry_element["stamp"].get<uint64_t>();
    gtsam::NonlinearFactorGraph entry_measurements;
    std::vector<bool> is_outlier;
    std::vector<std::string> type_tags;

    // Iterate through each measurement in each stamp
    for (auto& measurement : entry_element["measurements"]) {
      std::string tag = measurement["type"].get<std::string>();

      // If we sample as an outlier and it's specified as a type to introduce outliers to, perturb significantly
      if(sampler(generator) && (!outlier_types.is_initialized() || std::find(outlier_types->begin(), outlier_types->end(), tag) != outlier_types->end())){
        gtsam::Key key = gtsam::symbol_shorthand::X(0);
        std::string measured_tag = measurement["measurement"]["type"].get<std::string>();
        // Get measurement
        gtsam::Values valuesTemp;
        value_accumulators_[measured_tag](measurement["measurement"], key, valuesTemp);
        gtsam::Matrix cov = parseCovariance(measurement["covariance"], valuesTemp.dim());
        // Get how far away for 99th percentile
        Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
        Eigen::VectorXd perturb = es.eigenvectors().real().colwise().sum() * std::sqrt(-2*std::log(1 - 0.99) / es.eigenvalues().real().cwiseInverse().squaredNorm());
        // Perturb
        gtsam::VectorValues delta;
        delta.insert(key, perturb);
        valuesTemp = valuesTemp.retract(delta);
        // Put it back
        measurement["measurement"] = value_serializer_[measured_tag](key, valuesTemp);
        is_outlier.push_back(true);
      } else {
        is_outlier.push_back(false);
      }

      type_tags.push_back(tag);
      entry_measurements.push_back(measurement_parsers_[tag](measurement));
    }
    measurements.push_back(Entry(stamp, type_tags, entry_measurements, is_outlier));
  }
  return measurements;
}

/**********************************************************************************************************************/
json Parser::parseJson(std::string input_file_name, bool decompress_from_cbor) {
  json parsed_json;
  if (decompress_from_cbor) {
    std::ifstream ifs(input_file_name, std::ios::binary);
    parsed_json = json::from_cbor(nlohmann::detail::input_adapter{ifs});
  } else {
    std::ifstream ifs(input_file_name);
    parsed_json = json::parse(ifs);
  }
  return parsed_json;
}

/**********************************************************************************************************************/
Dataset Parser::parseDataset(std::string dataset_file, bool decompress_from_cbor, double outlier_percent, const boost::optional<std::vector<std::string>> outlier_types) {
  json dataset_json = parseJson(dataset_file, decompress_from_cbor);

  // Parse Header information
  std::string name = dataset_json["name"];
  std::vector<char> robots = dataset_json["robots"].get<std::vector<char>>();

  // Parse Ground truth if it exists
  boost::optional<std::map<char, TypedValues>> groundtruth = boost::none;
  if (dataset_json.contains("groundtruth")) {
    groundtruth = std::map<char, TypedValues>();
    for (auto& el : dataset_json["groundtruth"].items()) {
      (*groundtruth)[el.key()[0]] = parseValues(el.value());
    }
  }

  // Parse Initialization if it exists
  boost::optional<std::map<char, TypedValues>> initialization = boost::none;
  if (dataset_json.contains("initialization")) {
    initialization = std::map<char, TypedValues>();
    for (auto& el : dataset_json["initialization"].items()) {
      (*initialization)[el.key()[0]] = parseValues(el.value());
    }
  }

  // Parse Measurements if it exists
  std::map<char, std::vector<Entry>> measurements;
  for (auto& el : dataset_json["measurements"].items()) {
    measurements[el.key()[0]] = parseMeasurements(el.value(), outlier_percent, outlier_types);
  }

  return Dataset(name, robots, measurements, groundtruth, initialization);
}

/**********************************************************************************************************************/
Results Parser::parseResults(std::string results_file, bool decompress_from_cbor) {
  json results_json = parseJson(results_file, decompress_from_cbor);

  // Parse Header information
  std::string dataset_name = results_json["dataset_name"];
  std::string method_name = results_json["method_name"];
  std::vector<char> robots = results_json["robots"].get<std::vector<char>>();

  // Parse Ground truth if it exists
  std::map<char, TypedValues> solutions;
  for (auto& el : results_json["solutions"].items()) {
    solutions[el.key()[0]] = parseValues(el.value());
  }
  return Results(dataset_name, method_name, robots, solutions);
}

/**********************************************************************************************************************/
MetricSummary Parser::parseMetricSummary(std::string metric_summary_file, bool decompress_from_cbor) {
  json results_json = parseJson(metric_summary_file, decompress_from_cbor);
  MetricSummary metric_summary;
  metric_summary.dataset_name = results_json["dataset_name"];
  metric_summary.method_name = results_json["method_name"];
  metric_summary.robots = results_json["robots"].get<std::vector<char>>();

  if (results_json.contains("robot_ate")) {
    metric_summary.robot_ate = results_json["robot_ate"].get<std::map<char, std::pair<double, double>>>();
  }
  if (results_json.contains("total_ate")) {
    metric_summary.total_ate = results_json["total_ate"].get<std::pair<double, double>>();
  }
  if (results_json.contains("sve")) {
    metric_summary.sve = results_json["sve"].get<std::pair<double, double>>();
  }
  if (results_json.contains("mean_residual")) {
    metric_summary.mean_residual = results_json["mean_residual"].get<double>();
  }

  return metric_summary;
}

}  // namespace jrl
