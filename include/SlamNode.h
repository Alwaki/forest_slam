/**
 * @file SlamNode.h
 * @brief ROS interface node to handle incoming data and run all SLAM functionality
 * @author Alexander Wallén Kiessling
 */

#pragma once

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/buffer_core.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

/*
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
*/

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

class SlamNode
{
    public:
        SlamNode();
        virtual ~SlamNode();

    private:
        ros::NodeHandle                 _nh;
        ros::Subscriber                 _lidar_ouster_sub;
        ros::Subscriber                 _imu_vectornav_sub;
        ros::Subscriber                 _imu_ouster_sub;
        ros::Subscriber                 _icp_odom_sub;
        ros::Subscriber                 _dynamic_transformer_sub;
        ros::Publisher                  _lidar_ouster_filtered_pub;
        ros::Publisher                  _icp_odom_pub;


        std::mutex                              _buffer_mtx;
        pcl::PointCloud<pcl::PointXYZ>::Ptr     _prev_cloud;
        bool                                    _prev_cloud_flag;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> _icp;

        geometry_msgs::Point                    _total_odom;
        geometry_msgs::Quaternion               _current_rot;

        tf::TransformListener                   _tf_listener;
        tf2_ros::TransformBroadcaster           _dynamic_broadcaster;

        /*
        gtsam::NonlinearFactorGraph     _graph; 
        std::vector<gtsam::Symbol>      _pose_symbols; 
        std::vector<gtsam::Symbol>      _landmark_symbols;
        */

        void _init_node();
        void _lidar_ouster_callback(const sensor_msgs::PointCloud2::ConstPtr &msgIn);
        void _lidar_odom_calculate(const sensor_msgs::PointCloud2::ConstPtr &msgIn);
        void _dynamic_transformer(const geometry_msgs::PointStamped::ConstPtr &msgIn);
        void _lidar_feature_extraction(const pcl::PointCloud<PointXYZIT>::ConstPtr &msgIn);
        void _imu_ouster_callback(const sensor_msgs::Imu::ConstPtr &msgIn);
        void _imu_vectornav_callback(const sensor_msgs::Imu::ConstPtr &msgIn);

        /*
        void _add_landmark(const gtsam::Symbol::ConstPtr &landmarkMsg);
        void _add_pose(const gtsam::Symbol::ConstPtr &poseMsg);
        void _optimize();
        void _get_map();
        */
};