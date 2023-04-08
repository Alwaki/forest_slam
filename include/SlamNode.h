/**
 * @file SlamNode.h
 * @brief ROS interface node to handle incoming data and run all SLAM functionality
 * @author Alexander Wallén Kiessling
 */

#pragma once

#include "ros.h"
#include <pcl/point_cloud.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "livox_ros_driver/CustomMsg.h"

#include <deque>
#include <thread>
#include <string>
#include <stdio.h>

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
        
        void _init_node();
};