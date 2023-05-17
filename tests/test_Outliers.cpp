#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/slam/StereoFactor.h>
#include <jrl/Dataset.h>
#include <jrl/DatasetBuilder.h>
#include <jrl/Parser.h>
#include <jrl/Writer.h>
#include <jrl/IOMeasurements.h>

#include "gtest/gtest.h"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::L;

#define EXPECT_MATRICES_EQ(M_actual, M_expected) \
  EXPECT_TRUE(M_actual.isApprox(M_expected, 1e-6)) << "  Actual:\n" << M_actual << "\nExpected:\n" << M_expected


TEST(Outliers, Percentage){
    // Make a dummy factor
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
    gtsam::PriorFactor<gtsam::Pose2> factor(X(0), gtsam::Pose2::Identity(), cov);

    // Get everything ready
    int N = 10;
    gtsam::NonlinearFactorGraph graph;
    for(int i=0; i< N; i++){
        graph.push_back(factor);
    }
    std::vector<std::string> tags(N, jrl::PriorFactorPose2Tag);
    std::vector<bool> inliers(N, true);
    inliers.front() = false;

    std::map<char, std::vector<jrl::Entry>> entries = {{'a', {jrl::Entry(0, tags, graph, inliers)}}};
    jrl::Dataset dataset("outlier_test", {'a'}, entries, boost::none, boost::none);
    
    EXPECT_DOUBLE_EQ(0.1, dataset.percentOutliers('a'));
}