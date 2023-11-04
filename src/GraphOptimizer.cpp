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
    _current_pose[0] = 0.0;
    _current_pose[1] = 0.0;
    _current_pose[2] = 0.0;
    _data_association_threshold = 0.5;
    _loop_closure_threshold     = 0.1;
    _priorNoise  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1.0, 1.0, 0.1));
    _odomNoise   = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.1));
    _rangeNoise  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.3, 0.3));
    gtsam::Symbol x0('P', 0);
    gtsam::Symbol l0('L', 0);
    gtsam::Pose2 prior(0.0, 0.0, 0.0);
    _pose_symbols.push_back(x0);
    _landmark_symbols.push_back(l0);
    _initialEstimate.insert(x0, prior);
    _graph.add(gtsam::PriorFactor<gtsam::Pose2>(x0, prior, _priorNoise));


    _graph_odom_sub = _nh.subscribe<geometry_msgs::PointStamped>
        ("/estimator_node/odom", 10, &GraphOptimizer::_odom_cb,
        this, ros::TransportHints().tcpNoDelay(true));

    
    _graph_abs_pos_sub = _nh.subscribe<geometry_msgs::Pose2D>
        ("/estimator_node/absPose", 1, &GraphOptimizer::_absPose_cb,
        this);
    
    
    _graph_landmark_sub = _nh.subscribe<geometry_msgs::PoseArray>
        ("/feature_node/extracted_pos", 100, &GraphOptimizer::_landmark_cb,
        this, ros::TransportHints().tcpNoDelay(true));
        
}

void GraphOptimizer::_landmark_cb(const geometry_msgs::PoseArray::ConstPtr &msgIn)
{
    for(auto & pose: msgIn->poses)
    {
        if(pose.position.z > 0.9)
        {
            double min_distance = 100;
            double distance = 100;
            for (auto& it : _landmark_vector) 
            {
                distance = pow(it.first - pose.position.x, 2) + pow(it.second - pose.position.y, 2);
                if(distance < min_distance)
                {
                    min_distance = distance;
                }
            }
            if(min_distance > 2)
            {
                gtsam::Point2 landmark(pose.position.x, pose.position.y);
                _landmark_vector.push_back(std::make_pair(pose.position.x, pose.position.y));
                _add_landmark(landmark);
            }

        }
    }
    if(_opt_counter % 100)
    {
        _optimize_graph();
    }
    _opt_counter++;
}

void GraphOptimizer::_odom_cb(const geometry_msgs::PointStamped::ConstPtr &msgIn)
{
    gtsam::Pose2 odom(msgIn->point.x, msgIn->point.y, msgIn->point.z);
    _add_pose(odom);
}

void GraphOptimizer::_absPose_cb(const geometry_msgs::Pose2D::ConstPtr &msgIn)
{
    _pos_mtx.lock();
    _current_pose[0] = msgIn->x;
    _current_pose[1] = msgIn->y;
    _current_pose[2] = msgIn->theta;
    _pos_mtx.unlock();

}

void GraphOptimizer::_add_landmark(const gtsam::Point2 landmarkMsg)
{
    _symbol_mtx.lock();
    gtsam::Symbol x_last = _pose_symbols.back();
    _symbol_mtx.unlock();
    gtsam::Symbol l('L', _landmark_symbol_idx++);
    _landmark_symbols.push_back(l);


    _pos_mtx.lock();
    double dy = landmarkMsg[0] - _current_pose[1];
    double dx = landmarkMsg[1] - _current_pose[0];
    double range = sqrt(pow(dx, 2) + pow(dy, 2));
    double bearing = atan2(dy, dx) - _current_pose[2];
    _pos_mtx.unlock();
    gtsam::Point2 landmark(landmarkMsg[0], landmarkMsg[1]);
    _initialEstimate.insert(l, landmark);
    _graph.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
            x_last,
            l,
            bearing,
            range,
            _rangeNoise));
}

void GraphOptimizer::_add_pose(const gtsam::Pose2 odomMsg)
{
    _symbol_mtx.lock();
    gtsam::Symbol x('P', _odom_symbol_idx++);
    _graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
        _pose_symbols.back(), x, odomMsg, _odomNoise));
    _pose_symbols.push_back(x);
    _symbol_mtx.unlock();
    _initialEstimate.insert(x, odomMsg);
}

void GraphOptimizer::_optimize_graph()
{
    gtsam::LevenbergMarquardtOptimizer optimizer(_graph, _initialEstimate);
    gtsam::Values result = optimizer.optimize();
    result.print("Final Result:\n");
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
