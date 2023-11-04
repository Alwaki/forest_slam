/**
 * @file GraphOptimizer.h
 * @brief Class file for the SLAM backend using GTSAM solver library
 * @author Alexander Wallén Kiessling
 */

#pragma once

// ROS library
#include <ros/ros.h>

// ROS Messages
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"

// GTSAM library
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/sam/BearingRangeFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/Values.h"
#include "iostream"
#include "cmath"
#include "Eigen/Dense"
#include "random"

// C++ libraries
#include <deque>
#include <thread>
#include <string>
#include <stdio.h>
#include <vector>

class GraphOptimizer
{
    public:
        GraphOptimizer();
        virtual ~GraphOptimizer();
        void get_map();

    private:
        void _init();
        void _landmark_cb(const geometry_msgs::PoseArray::ConstPtr &msgIn);
        void _odom_cb(const geometry_msgs::PointStamped::ConstPtr &msgIn);
        void _absPose_cb(const geometry_msgs::Pose2D::ConstPtr &msgIn);
        void _add_landmark(const gtsam::Point2 landmarkMsg);
        void _add_pose(const gtsam::Pose2 poseMsg);
        void _optimize_graph();
        
        double                                   _data_association_threshold;
        double                                   _loop_closure_threshold;
        std::mutex                               _symbol_mtx;
        std::mutex                               _pos_mtx;
        ros::NodeHandle                          _nh;
        gtsam::NonlinearFactorGraph              _graph; 
        std::vector<gtsam::Symbol>               _pose_symbols; 
        std::vector<gtsam::Symbol>               _landmark_symbols;
        std::map<gtsam::Symbol, gtsam::Pose2>    _abs_poses;
        std::map<gtsam::Symbol, gtsam::Point2>   _abs_landmarks;
        gtsam::noiseModel::Diagonal::shared_ptr  _priorNoise;
        gtsam::noiseModel::Diagonal::shared_ptr  _odomNoise;
        gtsam::noiseModel::Diagonal::shared_ptr  _rangeNoise;
        gtsam::Values                            _initialEstimate;
        gtsam::Vector3                           _current_pose;
        int                                      _landmark_symbol_idx = 1;
        int                                      _odom_symbol_idx = 1;
        int                                      _opt_counter = 1;
        std::vector<std::pair<float,float>>      _landmark_vector;


        ros::Subscriber                          _graph_odom_sub;
        ros::Subscriber                          _graph_landmark_sub;
        ros::Subscriber                          _graph_abs_pos_sub;
        
        
};