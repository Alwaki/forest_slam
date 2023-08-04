/**
 * @file Odometry.h
 * @brief Class file for landmark based odometry
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

// TF transform libraries
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/buffer_core.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// Point cloud library
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

// C++ libraries
#include <deque>
#include <thread>
#include <string>
#include <stdio.h>

class OdometryEstimator
{
    public:
        OdometryEstimator();
        virtual ~OdometryEstimator();

    private:
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> _icp;
        ros::Publisher                          _icp_odom_pub;
        ros::NodeHandle                         _nh;
        tf2_ros::TransformBroadcaster           _dynamic_broadcaster;
        
};