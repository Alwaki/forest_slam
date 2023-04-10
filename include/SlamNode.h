/**
 * @file SlamNode.h
 * @brief ROS interface node to handle incoming data and run all SLAM functionality
 * @author Alexander Wallén Kiessling
 */

#pragma once

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <deque>
#include <thread>
#include <string>
#include <stdio.h>

struct EIGEN_ALIGN16 PointXYZIT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    std::uint32_t time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity) 
                                 (std::uint32_t, time, time))

typedef pcl::PointCloud<PointXYZIT>       CloudXYZIT;
typedef pcl::PointCloud<PointXYZIT>::Ptr  CloudXYZITPtr;


class SlamNode
{
    public:
       SlamNode();
       virtual ~SlamNode();

    private:
        ros::NodeHandle         _nh;
        ros::Subscriber         _lidar_ouster_sub;
        ros::Subscriber         _imu_vectornav_sub;
        ros::Subscriber         _imu_ouster_sub;

        std::mutex              _buffer_mtx;

        void _init_node();
        void _lidar_ouster_callback(const sensor_msgs::PointCloud2::ConstPtr &msgIn);
        void _imu_ouster_callback(const sensor_msgs::Imu::ConstPtr &msgIn);
        void _imu_vectornav_callback(const sensor_msgs::Imu::ConstPtr &msgIn);
};