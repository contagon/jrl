#pragma once

#include <gtsam/linear/VectorValues.h>

#include <random>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "jrl/Dataset.h"
#include "jrl/IOMeasurements.h"
#include "jrl/IOValues.h"
#include "jrl/Parser.h"
#include "jrl/Writer.h"

namespace jrl {
Dataset addOutliers(Dataset dataset, double percOutliers,
                    const boost::optional<std::vector<std::string>> outlierTypes = boost::none,
                    const boost::optional<std::string> newName = boost::none,
                    const double std=10);

gtsam::NonlinearFactor::shared_ptr perturbFactor(gtsam::NonlinearFactor::shared_ptr factor, std::string tag, double std);
}  // namespace jrl