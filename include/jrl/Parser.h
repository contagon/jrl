#pragma once
#include <random>
#include <gtsam/nonlinear/Values.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "jrl/Dataset.h"
#include "jrl/Metrics.h"
#include "jrl/Results.h"

namespace jrl {

/// @brief Parses the value at key from given json and adds to the provided values
typedef std::function<void(json, gtsam::Key& key, gtsam::Values&)> ValueParser;
/// @brief Parses the given json into a factor
typedef std::function<gtsam::NonlinearFactor::shared_ptr(json)> MeasurementParser;

class Parser {
  /** Members **/
 private:
  /// @brief Mapping from value tag to corresponding value parsing function
  std::map<std::string, ValueParser> value_accumulators_;
  /// @brief Implicitly defines default value parsers
  /// @return The default value parsers, which are loaded into value_accumulators_ on initialization
  std::map<std::string, ValueParser> loadDefaultValueAccumulators();

  /// @brief Mapping from measurement tag to corresponding measurement parsing function
  std::map<std::string, MeasurementParser> measurement_parsers_;
  /// @brief Implicitly defines default measurement parsers
  /// @return The default measurement parsers, which are loaded into measurement_parsers_ on initialization
  std::map<std::string, MeasurementParser> loadDefaultMeasurementParsers();

  /** @brief Parses all values using the loaded value accumulators
   *  @param values_json Input JSON containing the serialized values
   *  @return Parsed Values as GTSAM types
   **/
  TypedValues parseValues(json values_json);

  /** @brief Parses all measurements using the loaded measurement parsers
   *  @param measurements_json Input JSON containing the serialized measurement entries
   *  @return Parsed measurement entries
   **/
  std::vector<Entry> parseMeasurements(json measurements_json, double percent_outliers=0.0, const boost::optional<std::vector<std::string>> types = boost::none);

  /** @brief Reads arbitrary JSON from file
   * @param input_file_name: The file from which to read the json
   * @param decompress_from_cbor: if true indicates that input file is compressed with cbor
   */
  json parseJson(std::string input_file_name, bool decompress_from_cbor);

  /** Interface **/
 public:
  /// @brief Constructors a parser object
  Parser();

  /// @brief Loads and parses a JRL file into a Dataset
  /// @param decompress_from_cbor if true indicates that input files are compressed with cbor and must be decompressed
  /// before parsing
  Dataset parseDataset(std::string dataset_file, bool decompress_from_cbor = false, double percent_outliers=0.0, const boost::optional<std::vector<std::string>> types = boost::none);

  /// @brief Loads and parses a JRR file into a Results
  /// @param decompress_from_cbor if true indicates that input files are compressed with cbor and must be decompressed
  /// before parsing
  Results parseResults(std::string results_file, bool decompress_from_cbor = false);

  /// @brief Loads and parses a JRM file into a MetricSummary
  /// @param decompress_from_cbor if true indicates that input files are compressed with cbor and must be decompressed
  /// before parsing
  MetricSummary parseMetricSummary(std::string metric_summary_file, bool decompress_from_cbor = false);

  // TODO
  // void registerValueParser(std::string tag, ValueParser parser_fn);
  // void registerMeasurementParser(std::string tag, MeasurementParser parser_fn);
};

}  // namespace jrl