#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/slam/StereoFactor.h>
#include <jrl/Dataset.h>
#include <jrl/DatasetBuilder.h>
#include <jrl/Parser.h>
#include <jrl/Writer.h>
#include <jrl/IOMeasurements.h>
#include <jrl/Outliers.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "gtest/gtest.h"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::L;

#define EXPECT_MATRICES_NOT_EQ(M_actual, M_expected) \
  EXPECT_TRUE(!M_actual.isApprox(M_expected, 1e-6)) << "  Actual:\n" << M_actual << "\nExpected:\n" << M_expected


TEST(Outliers, WriteParse){
    // Make a dummy factor
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
    gtsam::PriorFactor<gtsam::Pose2> factor(X(0), gtsam::Pose2::Identity(), cov);

    // Make dataset
    gtsam::NonlinearFactorGraph graph;
    graph.push_back(factor);
    graph.push_back(factor);
    std::vector<bool> outliers{true, false};

    std::vector<std::string> tags(graph.size(), jrl::PriorFactorPose2Tag);
    jrl::Entry entry(0, tags, graph, outliers);
    std::map<char, std::vector<jrl::Entry>> entries = {{'a', {entry}}};
    jrl::Dataset dataset("outlier_test", {'a'}, entries, boost::none, boost::none);

    // Write dataset
    jrl::Writer writer;
    writer.writeDataset(dataset, "outliers.jrl");

    // Load it back in 
    jrl::Parser parser;
    jrl::Dataset read_dataset = parser.parseDataset("outliers.jrl", false);
    jrl::Entry read_entry = read_dataset.measurements('a')[0];

    EXPECT_TRUE(read_entry.is_outlier[0]);
    EXPECT_FALSE(read_entry.is_outlier[1]);
}

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

    // Add outliers
    jrl::Dataset outlier_dataset = jrl::addOutliers(dataset, outliers);
    
    EXPECT_NEAR(outliers, outlier_dataset.percentOutliers('a'), 0.05);
}

TEST(Outliers, ChangesValue){
    // Make a dummy factor
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
    gtsam::PriorFactor<gtsam::Pose2>::shared_ptr factor = boost::make_shared<gtsam::PriorFactor<gtsam::Pose2>>(X(0), gtsam::Pose2::Identity(), cov);

    // Perturb factor
    gtsam::NonlinearFactor::shared_ptr outlierFactor = jrl::perturbFactor(factor, jrl::PriorFactorPose2Tag, 10);
    gtsam::PriorFactor<gtsam::Pose2>::shared_ptr outlierFactorPrior = boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose2>>(outlierFactor);
    EXPECT_MATRICES_NOT_EQ(factor->prior().matrix(), outlierFactorPrior->prior().matrix());
}

TEST(Outliers, SpecificType){
    // Make dummy factors
    gtsam::noiseModel::Gaussian::shared_ptr cov = gtsam::noiseModel::Gaussian::Covariance(Eigen::Matrix3d::Identity());
    gtsam::PriorFactor<gtsam::Pose2> factor_prior(X(0), gtsam::Pose2::Identity(), cov);
    gtsam::BetweenFactor<gtsam::Pose2> factor_between(X(0), X(1), gtsam::Pose2::Identity(), cov);

    // Make dataset
    gtsam::NonlinearFactorGraph graph;
    graph.push_back(factor_prior);
    graph.push_back(factor_between);

    std::vector<std::string> tags{jrl::PriorFactorPose2Tag, jrl::BetweenFactorPose2Tag};
    std::map<char, std::vector<jrl::Entry>> entries = {{'a', {jrl::Entry(0, tags, graph)}}};
    jrl::Dataset dataset("outlier_test", {'a'}, entries, boost::none, boost::none);

    // Add outliers
    std::vector<std::string> makeOutlier{jrl::PriorFactorPose2Tag};
    jrl::Dataset outlier_dataset = jrl::addOutliers(dataset, 1.0, makeOutlier);
    
    EXPECT_DOUBLE_EQ(0.5, outlier_dataset.percentOutliers('a'));
}

TEST(Outliers, PrecisionRecall){
    // Fix seed
    srand(time(NULL));

    // Make a dummy factor
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();

    // Make dataset
    int N = 1000;
    gtsam::NonlinearFactorGraph graph;
    for(int i=0; i< N-1; i++){
        graph.addPrior(X(0), gtsam::Pose2::Identity(), cov);
    }
    graph.addPrior(X(0), gtsam::Pose2(10, 10, 1), cov);
    
    std::vector<bool> isOutlier(N, false);
    isOutlier.back() = true;

    gtsam::Values gtValues;
    gtValues.insert(X(0), gtsam::Pose2::Identity());
    std::map<char, jrl::TypedValues> gtTypes = {{'a', jrl::TypedValues(gtValues, {{X(0), jrl::Pose2Tag}})}};

    std::vector<std::string> tags(N, jrl::PriorFactorPose2Tag);
    std::map<char, std::vector<jrl::Entry>> entries = {{'a', {jrl::Entry(0, tags, graph, isOutlier)}}};
    jrl::Dataset dataset("outlier_test", {'a'}, entries, gtTypes, gtTypes);

    jrl::Results results("test", "test", {'a'}, gtTypes);

    jrl::OutlierSummary summary = jrl::metrics::computeOutlierSummary('a', dataset, results);

    EXPECT_DOUBLE_EQ(1.0, summary.precision);
    EXPECT_DOUBLE_EQ(1.0, summary.recall);
}