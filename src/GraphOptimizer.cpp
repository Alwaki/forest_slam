/**
 * @file    GraphOptimizer.cpp
 * @brief   SLAM backend using GTSAM solver library
 * @author  Alexander Wallén Kiessling
 */

#include "GraphOptimizer.h"

// Constructor
GraphOptimizer::GraphOptimizer():
    nh_("~")
    {_init();}

// Destructor
GraphOptimizer::~GraphOptimizer() = default;

void GraphOptimizer::_init()
{
    nh_.getParam("/forest_slam/optimization/data_association_threshold", data_association_threshold_);
    nh_.getParam("/forest_slam/optimization/loop_closure_threshold", loop_closure_threshold_);
    nh_.getParam("/forest_slam/optimization/prior_noise_x", prior_noise_x_);
    nh_.getParam("/forest_slam/optimization/prior_noise_y", prior_noise_y_);
    nh_.getParam("/forest_slam/optimization/prior_noise_theta", prior_noise_theta_);
    nh_.getParam("/forest_slam/optimization/odom_noise_x", odom_noise_x_);
    nh_.getParam("/forest_slam/optimization/odom_noise_y", odom_noise_y_);
    nh_.getParam("/forest_slam/optimization/odom_noise_theta", odom_noise_theta_);
    nh_.getParam("/forest_slam/optimization/range_noise_bearing", range_noise_bearing_);
    nh_.getParam("/forest_slam/optimization/range_noise_distance", range_noise_distance_);
    nh_.getParam("/forest_slam/optimization/init_pose", init_pose_);
    nh_.getParam("/forest_slam/optimization/optimization_interval", optimization_interval_);
    nh_.getParam("/forest_slam/optimization/landmark_probability_limit", landmark_probability_limit_);
    
    _current_pose[0] = init_pose_[0];
    _current_pose[1] = init_pose_[1];
    _current_pose[2] = init_pose_[2];

    loop_counter_ = 0;

    _priorNoise  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(prior_noise_x_, prior_noise_y_, prior_noise_theta_));
    _odomNoise   = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(odom_noise_x_, odom_noise_y_, odom_noise_theta_));
    _rangeNoise  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(range_noise_bearing_, range_noise_distance_));
    _matchNoise  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1.0, 1.0, 3.0));

    gtsam::Symbol x0('P', 0);
    gtsam::Symbol l0('L', 0);
    gtsam::Pose2 prior(init_pose_[0], init_pose_[1], init_pose_[2]);

    _pose_symbols.push_back(x0);
    _landmark_symbols.push_back(l0);
    _initialEstimate.insert(x0, prior);
    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(x0, prior, _priorNoise));

    graph_odom_sub_ = nh_.subscribe<geometry_msgs::PointStamped>
        ("/estimator_node/odom", 10, &GraphOptimizer::_odom_cb,
        this, ros::TransportHints().tcpNoDelay(true));
   
    graph_abs_pos_sub_ = nh_.subscribe<geometry_msgs::Pose2D>
        ("/estimator_node/absPose", 1, &GraphOptimizer::_absPose_cb,
        this);
    
    graph_landmark_sub_ = nh_.subscribe<geometry_msgs::PoseArray>
        ("/feature_node/extracted_pos", 100, &GraphOptimizer::_landmark_cb,
        this, ros::TransportHints().tcpNoDelay(true));

    loop_detection_sub_ = nh_.subscribe<geometry_msgs::PoseArray>
        ("/feature_node/extracted_pos", 100, &GraphOptimizer::_loop_detection_cb,
        this, ros::TransportHints().tcpNoDelay(true));
        
}

void GraphOptimizer::_landmark_cb(const geometry_msgs::PoseArray::ConstPtr &msgIn)
{
    for(auto & pose: msgIn->poses)
    {
        if(pose.position.z >= landmark_probability_limit_)
        {
            double min_distance = 10000;
            double distance = 10000;
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
    if(_opt_counter % optimization_interval_ == 0)
    {
        ROS_INFO("Optimizing");
        _optimize_graph();
    }
    _opt_counter++;
}

void GraphOptimizer::_loop_detection_cb(const geometry_msgs::PoseArray::ConstPtr &msgIn)
{
    /*
    Idea:
    1. Convert the detected features into ranges and sample them

    2. two sample non-parametric hypothesis testing with kolmogorov

    3. If detection, use ICP or the above (with procrusted analysis) to get 
    constraint to add loop closure into graph
    */


   /*
   if(loop_counter_ % 10 == 0)
   {
    std::vector<double> new_memory;

    for(auto & pose: msgIn->poses)
    {
        // Sample features based on their probability
        double r = ((double) rand() / (RAND_MAX));
        if(r < pose.position.z)
        {
            // Convert to range for rotation invariance
            double distance = sqrt(pow(_current_pose[0] - pose.position.x, 2) + 
                                    pow(_current_pose[1]- pose.position.y, 2));
            new_memory.push_back(distance);
        }
    }

    double minDiff = 10000000000.0;
    bool loop_detected = 0;


    for(auto & old_memory: long_memory_)
    {
        auto [n1, n2, diff] = ks_test_(new_memory, old_memory);
        if(diff < minDiff) 
        {
            minDiff = diff;
            double threshold = 0.9 * sqrt((n1+n2)/(n1*n2));
            if(minDiff<threshold)
            {
                loop_detected = 1;
                ROS_INFO("Loop detected!");
                ROS_INFO("threshold: %lf", threshold);
                ROS_INFO("Min ks result: %lf", minDiff);
            }
        }
    }

    // Add location to memory
    short_memory_.push_back(new_memory);
    if(short_memory_.size() > 10)
    {
        long_memory_.push_back(short_memory_[0]);
        short_memory_.pop_front();
    }
   }
   loop_counter_++;
   */
}

std::tuple<double, double, double> GraphOptimizer::ks_test_(const std::vector<double>& sample1, const std::vector<double>& sample2) 
{
    // Combine and sort both samples
    std::vector<double> combined;
    combined.insert(combined.end(), sample1.begin(), sample1.end());
    combined.insert(combined.end(), sample2.begin(), sample2.end());
    std::sort(combined.begin(), combined.end());

    // Calculate ECDFs for both samples
    double n1 = static_cast<double>(sample1.size());
    double n2 = static_cast<double>(sample2.size());
    double maxDiff = 0.0;
    for (size_t i = 0; i < combined.size(); ++i) {
        double cdf1 = static_cast<double>(std::lower_bound(sample1.begin(), sample1.end(), combined[i]) - sample1.begin()) / n1;
        double cdf2 = static_cast<double>(std::lower_bound(sample2.begin(), sample2.end(), combined[i]) - sample2.begin()) / n2;
        double diff = std::abs(cdf1 - cdf2);
        if (diff > maxDiff) {
            maxDiff = diff;
        }
    }
    return std::make_tuple(n1, n2, maxDiff);
}


/*
void GraphOptimizer::_loop_detection_cb(const geometry_msgs::PoseArray::ConstPtr &msgIn)
{
    loop_counter_++;
    if(loop_counter_ % 30 == 0)
    {
        int bucket_count = 40;
        double max_distance = 64;
        double min_distance = 4;
        double dx = (max_distance - min_distance)/bucket_count;
        std::array<int, 40> histogram_new = { };
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

        if(loop_counter_ > 150)
        {
            std::vector<std::array<int, 40>>::iterator end_it = _landmark_distances_memory.end();
            std::prev(end_it, 10);
            for (auto it = _landmark_distances_memory.begin(); it != end_it;  ++it )
            {   
                auto & histogram_old = *it;
                int matches = std::inner_product(histogram_old.begin(), histogram_old.end(), 
                                                histogram_new.begin(), 0,
                                                std::plus<>(), std::equal_to<>());

                    if(matches > max_matches)
                    {
                        max_matches = matches;
                        max_ind = match_ind * 30;
                    }    
                    match_ind++;
            }
        }
        ROS_INFO("matches: %d", max_matches);
        // Successful loop detection, set constraint
        if(max_matches >= loop_closure_threshold_)
        {
            symbol_mtx_.lock();
            gtsam::Symbol x_matched = _pose_symbols[max_ind];
            gtsam::Pose2 match_odom(0, 0, 0);
            graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
            _pose_symbols.back(), x_matched, match_odom, _matchNoise));
            symbol_mtx_.unlock();
            //ROS_INFO("--------------------------------------");
            //ROS_INFO("Matched with %d matches!", max_matches);
            cooldown_counter_ = 0;
        }

        _landmark_distances_memory.push_back(histogram_new);
    }
}
*/

void GraphOptimizer::_odom_cb(const geometry_msgs::PointStamped::ConstPtr &msgIn)
{
    gtsam::Pose2 odom(msgIn->point.x, msgIn->point.y, msgIn->point.z);
    _add_pose(odom);
}

void GraphOptimizer::_absPose_cb(const geometry_msgs::Pose2D::ConstPtr &msgIn)
{
    pos_mtx_.lock();
    _current_pose[0] = msgIn->x;
    _current_pose[1] = msgIn->y;
    _current_pose[2] = msgIn->theta;
    pos_mtx_.unlock();

}

void GraphOptimizer::_add_landmark(const gtsam::Point2 landmarkMsg)
{
    symbol_mtx_.lock();
    gtsam::Symbol x_last = _pose_symbols.back();
    symbol_mtx_.unlock();
    gtsam::Symbol l('L', _landmark_symbol_idx++);
    _landmark_symbols.push_back(l);


    pos_mtx_.lock();
    double dy = landmarkMsg[0] - _current_pose[1];
    double dx = landmarkMsg[1] - _current_pose[0];
    double range = sqrt(pow(dx, 2) + pow(dy, 2));
    double bearing = atan2(dy, dx) - _current_pose[2];
    pos_mtx_.unlock();
    gtsam::Point2 landmark(landmarkMsg[0], landmarkMsg[1]);
    _initialEstimate.insert(l, landmark);
    graph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
            x_last,
            l,
            bearing,
            range,
            _rangeNoise));
}

void GraphOptimizer::_add_pose(const gtsam::Pose2 odomMsg)
{
    symbol_mtx_.lock();
    gtsam::Symbol x('P', _odom_symbol_idx++);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
        _pose_symbols.back(), x, odomMsg, _odomNoise));
    _pose_symbols.push_back(x);
    symbol_mtx_.unlock();
    _initialEstimate.insert(x, odomMsg);
}

void GraphOptimizer::_optimize_graph()
{
    gtsam::LevenbergMarquardtOptimizer optimizer(graph_, _initialEstimate);
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
