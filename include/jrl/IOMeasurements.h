#pragma once
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <jrl/IOValues.h>

#include <nlohmann/json.hpp>

using json = nlohmann::json;
namespace jrl {

// Define statically the tags for measurements
static const std::string BetweenFactorPose2Tag = "BetweenFactorPose2";
static const std::string BetweenFactorPose3Tag = "BetweenFactorPose3";
static const std::string PriorFactorPose2Tag = "PriorFactorPose2";
static const std::string PriorFactorPose3Tag = "PriorFactorPose3";
static const std::string RangeFactorPose2Tag = "RangeFactorPose2";
static const std::string RangeFactorPose3Tag = "RangeFactorPose3";
static const std::string BearingRangeFactorPose2Tag = "BearingRangeFactorPose2";
static const std::string BearingRangeFactorPose3Tag = "BearingRangeFactorPose3";
static const std::string BetweenFactorPoint2Tag = "BetweenFactorPoint2";
static const std::string BetweenFactorPoint3Tag = "BetweenFactorPoint3";
static const std::string PriorFactorPoint2Tag = "PriorFactorPoint2";
static const std::string PriorFactorPoint3Tag = "PriorFactorPoint3";
static const std::string PriorFactorIMUBiasTag = "PriorFactorIMUBias";
static const std::string StereoFactorPose3Point3Tag = "StereoFactorPose3Point3";
static const std::string CombinedIMUTag = "CombinedIMU";

namespace io_measurements {

/**********************************************************************************************************************/
gtsam::Matrix parseMatrix(json input_json, int row, int col);
json serializeMatrix(gtsam::Matrix mat);

gtsam::Matrix parseCovariance(json input_json, int d);
json serializeCovariance(gtsam::Matrix covariance);

gtsam::NonlinearFactor::shared_ptr parseCombinedIMUFactor(json input_json);
json serializeCombinedIMUFactor(std::string type_tag, gtsam::NonlinearFactor::shared_ptr& factor);

/**********************************************************************************************************************/
template <typename T>
gtsam::NonlinearFactor::shared_ptr parsePrior(std::function<T(json)> val_parser_fn, json input_json) {
  // Get all required fields
  json key_json = input_json["k"];
  json measurement_json = input_json["mm"];
  json covariance_json = input_json["cov"];

  // Construct the factor
  T measured = val_parser_fn(measurement_json);
  int d = gtsam::traits<T>::GetDimension(measured);
  typename gtsam::Matrix covariance = parseCovariance(input_json["cov"], d);
  typename gtsam::PriorFactor<T>::shared_ptr factor = boost::make_shared<gtsam::PriorFactor<T>>(
      key_json.get<uint64_t>(), measured, gtsam::noiseModel::Gaussian::Covariance(parseCovariance(covariance_json, d)));
  return factor;
}

template <typename T>
json serializePrior(std::function<json(T)> val_serializer_fn, std::string type_tag,
                    gtsam::NonlinearFactor::shared_ptr& factor) {
  json output;
  typename gtsam::PriorFactor<T>::shared_ptr prior = boost::dynamic_pointer_cast<gtsam::PriorFactor<T>>(factor);
  gtsam::noiseModel::Gaussian::shared_ptr noise_model =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(prior->noiseModel());

  output["type"] = type_tag;
  output["k"] = prior->key();
  output["mm"] = val_serializer_fn(prior->prior());

  output["cov"] = serializeCovariance(noise_model->covariance());
  return output;
}

/**********************************************************************************************************************/
template <typename MEASURE, typename FACTOR>
gtsam::NonlinearFactor::shared_ptr parseNoiseModel2(std::function<MEASURE(json)> val_parser_fn, json input_json) {
  // Get all required fields
  json key1_json = input_json["k1"];
  json key2_json = input_json["k2"];
  json measurement_json = input_json["mm"];
  json covariance_json = input_json["cov"];

  // Construct the factor
  MEASURE measured = val_parser_fn(measurement_json);
  int d = gtsam::traits<MEASURE>::GetDimension(measured);
  typename FACTOR::shared_ptr factor =
      boost::make_shared<FACTOR>(key1_json.get<uint64_t>(), key2_json.get<uint64_t>(), measured,
                                 gtsam::noiseModel::Gaussian::Covariance(parseCovariance(covariance_json, d)));
  return factor;
}

template <typename MEASURE, typename FACTOR>
json serializeNoiseModel2(std::function<json(MEASURE)> val_serializer_fn, std::string type_tag,
                          gtsam::NonlinearFactor::shared_ptr& factor) {
  json output;
  typename FACTOR::shared_ptr between = boost::dynamic_pointer_cast<FACTOR>(factor);
  gtsam::noiseModel::Gaussian::shared_ptr noise_model =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(between->noiseModel());
  output["type"] = type_tag;
  output["k1"] = between->keys().front();
  output["k2"] = between->keys().back();
  output["mm"] = val_serializer_fn(between->measured());
  output["cov"] = serializeCovariance(noise_model->covariance());
  return output;
}

/**********************************************************************************************************************/
template <typename POSE, typename LANDMARK>
gtsam::NonlinearFactor::shared_ptr parseStereoFactor(std::function<POSE(json)> pose_parser_fn, json input_json) {
  // Get all required fields
  json key_pose_json = input_json["kp"];
  json key_landmark_json = input_json["klm"];
  json measurement_json = input_json["mm"];
  json calibration_json = input_json["cal"];
  json covariance_json = input_json["cov"];

  // Get optional field
  boost::optional<POSE> body_T_sensor = boost::none;
  if (input_json.contains("bTs")) {
    body_T_sensor = pose_parser_fn(input_json["bTs"]);
  }

  // Construct the factor
  gtsam::StereoPoint2 measured = io_values::parse<gtsam::StereoPoint2>(measurement_json);
  gtsam::Vector calibration_vector = parseMatrix(calibration_json, 6, 1);
  gtsam::Cal3_S2Stereo::shared_ptr calibration = boost::make_shared<gtsam::Cal3_S2Stereo>(calibration_vector);

  typename gtsam::GenericStereoFactor<POSE, LANDMARK>::shared_ptr factor =
      boost::make_shared<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(
          measured, gtsam::noiseModel::Gaussian::Covariance(parseCovariance(covariance_json, 3)),
          key_pose_json.get<uint64_t>(), key_landmark_json.get<uint64_t>(), calibration, body_T_sensor);

  return factor;
}

template <typename POSE, typename LANDMARK>
json serializeStereoFactor(std::function<json(POSE)> pose_serializer_fn, std::string type_tag,
                           gtsam::NonlinearFactor::shared_ptr& factor) {
  json output;
  typename gtsam::GenericStereoFactor<POSE, LANDMARK>::shared_ptr stereo_factor =
      boost::dynamic_pointer_cast<gtsam::GenericStereoFactor<POSE, LANDMARK>>(factor);
  gtsam::noiseModel::Gaussian::shared_ptr noise_model =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(stereo_factor->noiseModel());

  // Usual NoiseModel2 stuff
  output["type"] = type_tag;
  output["kp"] = stereo_factor->keys().front();
  output["klm"] = stereo_factor->keys().back();
  output["cov"] = serializeCovariance(noise_model->covariance());
  output["mm"] = io_values::serialize<gtsam::StereoPoint2>(stereo_factor->measured());

  // Extra stuff for this factor
  output["cal"] = serializeMatrix(stereo_factor->calibration()->vector());
  boost::optional<POSE> body_T_sensor = stereo_factor->body_P_sensor();
  if (body_T_sensor.is_initialized()) {
    output["bTs"] = pose_serializer_fn(body_T_sensor.get());
  }
  return output;
}

}  // namespace io_measurements

}  // namespace jrl