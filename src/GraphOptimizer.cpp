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
    _data_association_threshold = 0.5;
    _loop_closure_threshold     = 0.1;
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
    _symbol_mtx.lock();
    gtsam::Symbol last_pose_symbol = _pose_symbols.back();
    _symbol_mtx.unlock();
}

void GraphOptimizer::_add_pose(const gtsam::Pose2 odomMsg)
{
    _symbol_mtx.lock();
    gtsam::Symbol x('P', 0);
    _graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(
        _pose_symbols.back(), x, odomMsg, _odomNoise);
    _pose_symbols.push_back(x);
    _symbol_mtx.unlock();
    _initialEstimate.insert(x, odomMsg);
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
