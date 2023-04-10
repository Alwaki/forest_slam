/**
 * @file SlamNode.cpp
 * @brief ROS interface node to handle incoming data and run all SLAM functionality
 * @author Alexander Wallén Kiessling
 */

#include "SlamNode.h"
//#include "GraphOptimizer.h"
//#include "FeatureEstimator.h"

SlamNode::SlamNode():
    _nh("~")
    {_init_node();}

SlamNode::~SlamNode() = default;

void SlamNode::_init_node()
{
    // Setup subscribers and publishers
    _lidar_livox_sub   = _nh.subscribe<livox_ros_driver::CustomMsg>
                                ("/livox/lidar", 10, &SlamNode::_lidar_livox_callback,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _lidar_ouster_sub  = _nh.subscribe<sensor_msgs::PointCloud2>
                                ("/os_cloud_node/points", 10, &SlamNode::_lidar_ouster_callback,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _imu_vectornav_sub = _nh.subscribe<sensor_msgs::Imu>
                                ("/vectornav/IMU", 10, &SlamNode::_imu_vectornav_callback,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _imu_ouster_sub    = _nh.subscribe<sensor_msgs::Imu>
                                ("/os_cloud_node/imu", 10, &SlamNode::_imu_ouster_callback,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _lidar_livox_pub   = _nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_pc2", 10);
}

void SlamNode::_lidar_livox_callback(const livox_ros_driver::CustomMsg::ConstPtr &msgIn)
{
    // Not finished, currently need to change time conversion (fails as 
    // "Time is out of dual 32-bit range")
    // However, instead working with Ouster data for now

    /*
    // Lock mutex for multi-thread
    _lidar_livox_mtx.lock();

    // Change cloud to PCL structure and push to queue
    size_t cloudsize = msgIn->points.size();
    CloudXYZIT cloud;
    cloud.points.resize(cloudsize);

    for (size_t i = 0; i < cloudsize; i++)
        {
            auto &src = msgIn->points[i];
            auto &dst = cloud.points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.reflectivity;
            dst.time = src.offset_time;
        }

    // Publish corrected cloud
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(cloud, tempCloud);
    ROS_INFO("%ld", msgIn->timebase);
    tempCloud.header.stamp = ros::Time(msgIn->timebase);
    tempCloud.header.frame_id = "world";
    _lidar_livox_pub.publish(tempCloud);

    // Unlock mutex for multi-thread
    _lidar_livox_mtx.unlock();
    
    */
}

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




