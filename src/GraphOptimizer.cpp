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
    _nh.getParam("/forest_slam/optimization/data_association_threshold", _data_association_threshold);
    _nh.getParam("/forest_slam/optimization/loop_closure_threshold", _loop_closure_threshold);
    _nh.getParam("/forest_slam/optimization/prior_noise_x", _prior_noise_x);
    _nh.getParam("/forest_slam/optimization/prior_noise_y", _prior_noise_y);
    _nh.getParam("/forest_slam/optimization/prior_noise_theta", _prior_noise_theta);
    _nh.getParam("/forest_slam/optimization/odom_noise_x", _odom_noise_x);
    _nh.getParam("/forest_slam/optimization/odom_noise_y", _odom_noise_y);
    _nh.getParam("/forest_slam/optimization/odom_noise_theta", _odom_noise_theta);
    _nh.getParam("/forest_slam/optimization/range_noise_bearing", _range_noise_bearing);
    _nh.getParam("/forest_slam/optimization/range_noise_distance", _range_noise_distance);
    _nh.getParam("/forest_slam/optimization/init_pose", _init_pose);
    _nh.getParam("/forest_slam/optimization/optimization_interval", _optimization_interval);
    _nh.getParam("/forest_slam/optimization/landmark_probability_limit", _landmark_probability_limit);
    
    _current_pose[0] = _init_pose[0];
    _current_pose[1] = _init_pose[1];
    _current_pose[2] = _init_pose[2];

    _loop_counter = 0;

    _priorNoise  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(_prior_noise_x, _prior_noise_y, _prior_noise_theta));
    _odomNoise   = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(_odom_noise_x, _odom_noise_y, _odom_noise_theta));
    _rangeNoise  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(_range_noise_bearing, _range_noise_distance));
    _matchNoise  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1.0, 1.0, 3.0));

    gtsam::Symbol x0('P', 0);
    gtsam::Symbol l0('L', 0);
    gtsam::Pose2 prior(_init_pose[0], _init_pose[1], _init_pose[2]);

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

    _loop_detection_sub = _nh.subscribe<geometry_msgs::PoseArray>
        ("/feature_node/extracted_pos", 100, &GraphOptimizer::_loop_detection_cb,
        this, ros::TransportHints().tcpNoDelay(true));
        
}

void GraphOptimizer::_landmark_cb(const geometry_msgs::PoseArray::ConstPtr &msgIn)
{
    for(auto & pose: msgIn->poses)
    {
        if(pose.position.z > _landmark_probability_limit)
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
    if(_opt_counter % _optimization_interval == 0)
    {
        ROS_INFO("Optimizing");
        _optimize_graph();
    }
    _opt_counter++;
}

void GraphOptimizer::_loop_detection_cb(const geometry_msgs::PoseArray::ConstPtr &msgIn)
{
    int memory_interval = 10;
    if(_loop_counter % memory_interval == 0)
    {
        int bucket_count = 10;
        double max_distance = 64;
        double min_distance = 4;
        double dx = (max_distance - min_distance)/bucket_count;
        std::array<int, 10> histogram_new = { };

        for(auto & pose: msgIn->poses)
        {
            double distance = pow(_current_pose[0] - pose.position.x, 2) + pow(_current_pose[1]- pose.position.y, 2);
            distance = std::max(min_distance, distance - min_distance);
            int index = std::min(distance, max_distance) / dx;
            histogram_new[index]++;
        }

        int max_matches = 0;
        int max_ind = 0;
        int match_ind = 0;

        for(auto & histogram_old: _landmark_distances_memory)
        {
            int matches = std::inner_product(histogram_old.begin(), histogram_old.end(), histogram_new.begin(), 0,
                                    std::plus<>(), std::equal_to<>());

            if(matches > max_matches)
            {
                max_matches = matches;
                max_ind = match_ind * memory_interval;
            }    
            match_ind++;
        }

        if(max_matches > _loop_closure_threshold)
        {
            _symbol_mtx.lock();
            gtsam::Symbol x_matched = _pose_symbols[max_ind];
            gtsam::Pose2 match_odom(0, 0, 0);
            _graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            _pose_symbols.back(), x_matched, match_odom, _matchNoise));
            _symbol_mtx.unlock();
            ROS_INFO("Matched with %d matches!", max_matches);
        }

        _landmark_distances_memory.push_back(histogram_new);
    }
    _loop_counter++;
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
