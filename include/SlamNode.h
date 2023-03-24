/**
 * @file SlamNode.h
 * @brief ROS interface node to handle incoming data and run all SLAM functionality
 * @author Alexander Wallén Kiessling
 */

#pragma once

#include "ros.h"

class SlamNode
{
    public:
       SlamNode();
       virtual ~SlamNode();
       void node_thread();

    private:
        ros::NodeHandle         _nh;
        ros::Subscriber         _lidar_ouster_sub;
        ros::Subscriber         _lidar_livox_sub;
        ros::Subscriber         _imu_vectornav_sub;
        ros::Publisher          _pos_landmark_pub;
        
};