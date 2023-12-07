/**
 * @file    OdometryEstimator.cpp
 * @brief   Creates odometry estimate with ICP on features, and orientation from IMU
 * @author  Alexander Wallén Kiessling
 */


#include "OdometryEstimator.h"

OdometryEstimator::OdometryEstimator():
    _nh("~")
    {_init_node();}

OdometryEstimator::~OdometryEstimator() = default;


void OdometryEstimator::_init_node()
{
    // Initialize variables
    _prev_cloud_flag = 0;
    _current_pos.x = 0;
    _current_pos.y = 0;
    _current_pos.theta = 0;
    _total_distance = 0;

    // Setup subscribers and publishers
    _feature_pos_sub = _nh.subscribe<geometry_msgs::PoseArray>
        ("/feature_node/extracted_pos", 100, &OdometryEstimator::_lidar_ICP,
        this, ros::TransportHints().tcpNoDelay(true));

    _imu_vectornav_sub = _nh.subscribe<sensor_msgs::Imu>
        ("/vectornav/IMU", 100, &OdometryEstimator::_dynamic_transform_broadcast,
        this, ros::TransportHints().tcpNoDelay(true));

    _icp_odom_pub  = _nh.advertise<geometry_msgs::PointStamped>(
        "/estimator_node/odom", 20);
    
    _current_pose2d_pub  = _nh.advertise<geometry_msgs::Pose2D>(
        "/estimator_node/absPose", 1);

}

void OdometryEstimator::_lidar_ICP(const geometry_msgs::PoseArray::ConstPtr &msgIn)
{
    // Create pointcloud from landmarks
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto & pose: msgIn->poses)
    {
        pcl::PointXYZ pt;
        pt.x = pose.position.x;
        pt.y = pose.position.y;
        pt.z = 0;
        icp_cloud->push_back(pt);
    }

    if(_prev_cloud_flag == 1)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaxCorrespondenceDistance (0.2);
        icp.setMaximumIterations (400);
        icp.setTransformationEpsilon (1e-8);
        icp.setEuclideanFitnessEpsilon (0.01);
        icp.setInputSource(icp_cloud);
        icp.setInputTarget(_prev_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*aligned_cloud);
        Eigen::Matrix4d transformation_matrix = icp.getFinalTransformation().cast<double>();
        geometry_msgs::PointStamped odom;
        odom.header.stamp = msgIn->header.stamp;
        odom.header.frame_id = msgIn->header.frame_id;
        odom.point.x = transformation_matrix(0,3);
        odom.point.y = transformation_matrix(1,3);
        _pos_mtx.lock();
        odom.point.z = _yaw_diff;
        _icp_odom_pub.publish(odom);
        _total_distance += sqrt(pow(odom.point.x, 2) + pow(odom.point.y, 2));
        ROS_INFO("Distance: %lf", _total_distance);
        _current_pos.x += odom.point.x;
        _current_pos.y += odom.point.y;
        _current_pose2d_pub.publish(_current_pos);
        //ROS_INFO("Pose, X: %lf, Y: %lf, Theta: %lf", _current_pos.x, _current_pos.y, _current_pos.theta);
        _pos_mtx.unlock();
    }
    
    _prev_cloud = icp_cloud;
    _prev_cloud_flag = 1;
}

void OdometryEstimator::_dynamic_transform_broadcast(const sensor_msgs::Imu::ConstPtr &msgIn)
{
    geometry_msgs::TransformStamped transform;
    transform.transform.rotation = msgIn->orientation;

    tf::Quaternion q(
        msgIn->orientation.x,
        msgIn->orientation.y,
        msgIn->orientation.z,
        msgIn->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    _pos_mtx.lock();
    _yaw_diff = _current_pos.theta - yaw;
    _current_pos.theta = yaw;
    _pos_mtx.unlock();
    
    //ROS_INFO("X: %lf, Y: %lf", _current_pos.x, _current_pos.y);
    transform.transform.translation.x = 0; //_current_pos.x;
    transform.transform.translation.y = 0; //_current_pos.y;
    transform.transform.translation.z = 0;
    transform.header.stamp = msgIn->header.stamp;
    transform.child_frame_id = "base_link";
    transform.header.frame_id = "map";
    _dynamic_broadcaster.sendTransform(transform);

    geometry_msgs::TransformStamped transform2;
    transform2.transform.rotation = msgIn->orientation;
    //ROS_INFO("X: %lf, Y: %lf", _current_pos.x, _current_pos.y);
    transform2.transform.translation.x = _current_pos.x;
    transform2.transform.translation.y = _current_pos.y;
    transform2.transform.translation.z = 0;
    transform2.header.stamp = msgIn->header.stamp;
    transform2.child_frame_id = "moving_base";
    transform2.header.frame_id = "map";
    _dynamic_broadcaster2.sendTransform(transform2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estimator_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Started Odometry Estimator Node!");

    OdometryEstimator node;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
}