/**
 * @file    GraphOptimizer.cpp
 * @brief   Class file for the SLAM backend using GTSAM solver library
 * @author  Alexander Wallén Kiessling
 */

#include "GraphOptimizer.h"

GraphOptimizer::GraphOptimizer():
    _nh("~")
    {_init();}

GraphOptimizer::~GraphOptimizer() = default;

void GraphOptimizer::_init()
{
    _priorNoise  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1.0, 1.0, 0.1));
    _odomNoise   = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.1));
    _rangeNoise  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.05, 0.2));
    gtsam::Symbol x0('P', 0);
    gtsam::Pose2 prior(0.0, 0.0, 0.0);
    _pose_symbols.push_back(x0);
    _initialEstimate.insert(x0, prior);
    _graph.add(gtsam::PriorFactor<gtsam::Pose2>(x0, prior, _priorNoise));
}

void GraphOptimizer::_add_landmark(const gtsam::Point2 landmarkMsg)
{
    
}

void GraphOptimizer::_add_pose(const gtsam::Pose2 poseMsg)
{
    gtsam::Symbol x('P', 0);
    _graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(
        _pose_symbols.back(), x, poseMsg, _odomNoise);
    _pose_symbols.push_back(x);
    _initialEstimate.insert(x, poseMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optimizer_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Started GraphOptimizer Node!");

    GraphOptimizer node;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
}
