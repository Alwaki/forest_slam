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
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose2D.h"

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


class OdometryEstimator
{
    public:
        OdometryEstimator();
        virtual ~OdometryEstimator();

    private:

        // Parameters
        double                                  max_correspondence_distance_;
        double                                  maximum_iterations_;
        double                                  transformation_epsilon_;
        double                                  euclidean_fitness_epsilon_;

        // Values and counters
        double                                  yaw_diff_;
        double                                  total_distance_;
        ros::Time                               prev_time_;

        // Flags & Mutexes
        bool                                    prev_cloud_flag_;
        std::mutex                              pos_mtx_;

        // ROS handles, subsriber/publisher
        ros::NodeHandle                         nh_;
        ros::Publisher                          icp_odom_pub_;
        ros::Publisher                          current_pose2d_pub_;
        ros::Publisher                          lidar_processed_pub_;
        tf2_ros::TransformBroadcaster           dynamic_broadcaster_;
        tf::TransformListener                   tf_listener_;
        ros::Subscriber                         imu_vectornav_sub_;
        ros::Subscriber                         feature_pos_sub_;
        ros::Subscriber                         lidar_raw_sub_;

        // ROS objects
        geometry_msgs::Pose2D                   current_pos_;

        // Point cloud library objects
        pcl::PointCloud<pcl::PointXYZ>::Ptr     prev_cloud_;

        /**
         * @brief Initialize OdometryEstimator object
         */
        void init_node_();

        /**
         * @brief Scan registration of feature points
         */
        void feature_icp_(const geometry_msgs::PoseArray::ConstPtr &msgIn);

        /**
         * @brief Transform pose publisher
         */
        void dynamic_transform_broadcast_(const sensor_msgs::Imu::ConstPtr &msgIn);

        void lidar_deskew_(const sensor_msgs::PointCloud2::ConstPtr &msgIn);
};