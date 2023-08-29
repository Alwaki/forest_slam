/**
 * @file SlamNode.h
 * @brief ROS interface node to handle incoming data and run all SLAM functionality
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

struct PointXYZT
{
    PCL_ADD_POINT4D;
    std::uint32_t time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZT,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (std::uint32_t, time, time))

struct PointXYZIT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    float t;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity) (float, t, t))

class Preprocess
{
    public:
        Preprocess();
        virtual ~Preprocess();

    private:
        ros::NodeHandle                 _nh;
        ros::Subscriber                 _lidar_ouster_sub;
        ros::Publisher                  _lidar_ouster_filtered_pub;

        std::mutex                              _buffer_mtx;
        pcl::PointCloud<pcl::PointXYZ>::Ptr     _prev_cloud;
        bool                                    _prev_cloud_flag;

        geometry_msgs::Vector3                    _current_pos;
        geometry_msgs::Quaternion                 _current_rot;
        geometry_msgs::TransformStamped           _current_transform;
        ros::Time                                 _prev_time;
        uint32_t                                  _imu_counter;

        tf::TransformListener                   _tf_listener;

        void _init_node();
        void _lidar_ouster_callback(const sensor_msgs::PointCloud2::ConstPtr &msgIn);
        //void _lidar_odom_calculate(const sensor_msgs::PointCloud2::ConstPtr &msgIn);
        //void _dynamic_transformer(const geometry_msgs::PointStamped::ConstPtr &msgIn);
        void _imu_ouster_callback(const sensor_msgs::Imu::ConstPtr &msgIn);
        void _imu_vectornav_callback(const sensor_msgs::Imu::ConstPtr &msgIn);
};