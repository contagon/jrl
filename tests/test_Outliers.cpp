#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/slam/StereoFactor.h>
#include <jrl/Dataset.h>
#include <jrl/DatasetBuilder.h>
#include <jrl/Parser.h>
#include <jrl/Writer.h>
#include <jrl/IOMeasurements.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "gtest/gtest.h"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::L;

#define EXPECT_MATRICES_NOT_EQ(M_actual, M_expected) \
  EXPECT_TRUE(!M_actual.isApprox(M_expected, 1e-6)) << "  Actual:\n" << M_actual << "\nExpected:\n" << M_expected


TEST(Outliers, CalcPercent){
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
    std::vector<bool> outliers(N, false);
    outliers.front() = true;

    std::map<char, std::vector<jrl::Entry>> entries = {{'a', {jrl::Entry(0, tags, graph, outliers)}}};
    jrl::Dataset dataset("outlier_test", {'a'}, entries, boost::none, boost::none);
    
    EXPECT_DOUBLE_EQ(0.1, dataset.percentOutliers('a'));
}

TEST(Outliers, AddRightPercent){
    // Fix seed
    srand(time(NULL));
    double outliers = 0.25;

    // Make a dummy factor
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
    gtsam::PriorFactor<gtsam::Pose2> factor(X(0), gtsam::Pose2::Identity(), cov);

    // Make dataset
    int N = 1000;
    gtsam::NonlinearFactorGraph graph;
    for(int i=0; i< N; i++){
        graph.push_back(factor);
    }
    std::vector<std::string> tags(N, jrl::PriorFactorPose2Tag);
    std::map<char, std::vector<jrl::Entry>> entries = {{'a', {jrl::Entry(0, tags, graph)}}};
    jrl::Dataset dataset("outlier_test", {'a'}, entries, boost::none, boost::none);

    // Write dataset
    jrl::Writer writer;
    writer.writeDataset(dataset, "outliers.jrl");

    // Load it back in w/ outliers
    jrl::Parser parser;
    jrl::Dataset read_dataset = parser.parseDataset("outliers.jrl", false, outliers);
    
    EXPECT_NEAR(outliers, read_dataset.percentOutliers('a'), 0.05);
}

TEST(Outliers, ChangesValue){
    // Make a dummy factor
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
    gtsam::PriorFactor<gtsam::Pose2> factor(X(0), gtsam::Pose2::Identity(), cov);

    // Make dataset
    int N = 1;
    gtsam::NonlinearFactorGraph graph;
    graph.push_back(factor);

    std::vector<std::string> tags(N, jrl::PriorFactorPose2Tag);
    std::map<char, std::vector<jrl::Entry>> entries = {{'a', {jrl::Entry(0, tags, graph)}}};
    jrl::Dataset dataset("outlier_test", {'a'}, entries, boost::none, boost::none);

    // Write dataset
    jrl::Writer writer;
    writer.writeDataset(dataset, "outliers.jrl");

    // Load it back in w/ outliers
    jrl::Parser parser;
    jrl::Dataset read_dataset = parser.parseDataset("outliers.jrl", false, 1.0);
    
    gtsam::PriorFactor<gtsam::Pose2>::shared_ptr outlier_factor = boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose2>>(read_dataset.factorGraph()[0]);
    EXPECT_MATRICES_NOT_EQ(factor.prior().matrix(), outlier_factor->prior().matrix());
}