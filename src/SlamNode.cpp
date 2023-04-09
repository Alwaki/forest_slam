/**
 * @file SlamNode.cpp
 * @brief ROS interface node to handle incoming data and run all SLAM functionality
 * @author Alexander Wallén Kiessling
 */

#include "SlamNode.h"
#include "GraphOptimizer.h"
//#include "FeatureEstimator.h"

SlamNode::SlamNode():
    _nh("~")
    {_init_node();}

SlamNode::~SlamNode() = default;

void SlamNode::_init_node()
{
    // Setup subscribers and publishers
    _lidar_livox_sub         = _nh.subscribe<livox_ros_driver::CustomMsg>
                                ("/livox/lidar", 10, &SlamNode::_lidar_livox_callback,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _lidar_ouster_sub        = _nh.subscribe<sensor_msgs::PointCloud2>
                                ("/os_cloud_node/points", 10, &SlamNode::_lidar_ouster_callback,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _imu_vectornav_sub       = _nh.subscribe<sensor_msgs::Imu>
                                ("/vectornav/IMU", 10, &SlamNode::_imu_vectornav_callback,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _imu_ouster_sub          = _nh.subscribe<sensor_msgs::Imu>
                                ("/os_cloud_node/imu", 10, &SlamNode::_imu_ouster_callback,
                                 this, ros::TransportHints().tcpNoDelay(true));
}

void SlamNode::_lidar_livox_callback(const livox_ros_driver::CustomMsg::ConstPtr &msgIn){}

void SlamNode::_lidar_ouster_callback(const sensor_msgs::PointCloud2::ConstPtr &msgIn){}

void SlamNode::_imu_ouster_callback(const sensor_msgs::Imu::ConstPtr &msgIn){}

void SlamNode::_imu_vectornav_callback(const sensor_msgs::Imu::ConstPtr &msgIn){}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Started SLAM Node!");

    SlamNode node;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    //node.node_thread();
    ros::waitForShutdown();
}




