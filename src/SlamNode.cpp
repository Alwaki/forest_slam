/**
 * @file SlamNode.cpp
 * @brief ROS interface node to handle incoming data and run all SLAM functionality
 * @author Alexander Wallén Kiessling
 */

#include "SlamNode.h"
//#include "FeatureEstimator.h"

SlamNode::SlamNode():
    _nh("~")
    {_init_node();}

SlamNode::~SlamNode() = default;

void SlamNode::_init_node()
{
    // Set paramaters
    _icp.setMaxCorrespondenceDistance (0.05);
    _icp.setMaximumIterations (50);
    _icp.setTransformationEpsilon (1e-8);
    _icp.setEuclideanFitnessEpsilon (1);

    // Initialize necessary variables
    _prev_cloud_flag = 0;
    _total_odom.x = 0;
    _total_odom.y = 0;
    _total_odom.z = 0;

    // Setup subscribers and publishers
    _lidar_ouster_sub  = _nh.subscribe<sensor_msgs::PointCloud2>
                                ("/os_cloud_node/points", 10, &SlamNode::_lidar_ouster_callback,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _imu_vectornav_sub = _nh.subscribe<sensor_msgs::Imu>
                                ("/vectornav/IMU", 10, &SlamNode::_imu_vectornav_callback,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _imu_ouster_sub    = _nh.subscribe<sensor_msgs::Imu>
                                ("/os_cloud_node/imu", 10, &SlamNode::_imu_ouster_callback,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _icp_odom_sub  = _nh.subscribe<sensor_msgs::PointCloud2>
                                ("/os_cloud_node/points", 10, &SlamNode::_lidar_odom_calculate,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _dynamic_transformer_sub  = _nh.subscribe<geometry_msgs::PointStamped>
                                ("/icp/transform", 10, &SlamNode::_dynamic_transformer,
                                 this, ros::TransportHints().tcpNoDelay(true));

    _lidar_ouster_filtered_pub = _nh.advertise<sensor_msgs::PointCloud2>("/os_cloud_node/points_filtered", 10);

    _icp_odom_pub = _nh.advertise<geometry_msgs::PointStamped>("/icp/transform", 10);

}

void SlamNode::_lidar_ouster_callback(const sensor_msgs::PointCloud2::ConstPtr &msgIn)
{
    /*
    1. Trim the point cloud height wise and distance wise (or somehow segment ground away)
    2. Publish trimmed point cloud for feature extraction and lidar odometry calculation
    */

    // Change to PCL format
    pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*msgIn, *pcl_cloud);

    // Downsample cloud
    pcl::PCLPointCloud2::Ptr pcl_voxel (new pcl::PCLPointCloud2 ());
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    voxel_grid.setInputCloud(pcl_cloud);
    voxel_grid.setLeafSize (0.15f, 0.15f, 0.15f);
    voxel_grid.filter (*pcl_voxel);

    // Passthrough filter
    pcl::PCLPointCloud2::Ptr pcl_pass (new pcl::PCLPointCloud2 ());
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud (pcl_voxel);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-2.0, 2.5);
    pass.filter (*pcl_pass);

    // Publish cloud
    _lidar_ouster_filtered_pub.publish(*pcl_pass);


}

void SlamNode::_lidar_odom_calculate(const sensor_msgs::PointCloud2::ConstPtr &msgIn)
{
    // Downsample cloud and convert to ICP compatible format
    pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr pcl_voxel (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*msgIn, *pcl_cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    voxel_grid.setInputCloud(pcl_cloud);
    voxel_grid.setLeafSize (0.2f, 0.2f, 0.2f);
    voxel_grid.filter (*pcl_voxel);
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pcl_voxel,*icp_cloud);

    if(_prev_cloud_flag == 1)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaxCorrespondenceDistance (0.05);
        icp.setMaximumIterations (50);
        icp.setTransformationEpsilon (1e-8);
        icp.setEuclideanFitnessEpsilon (1);
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
        odom.point.z = transformation_matrix(2,3);
        _icp_odom_pub.publish(odom);
    }
    
    _prev_cloud = icp_cloud;
    _prev_cloud_flag = 1;
}

void SlamNode::_dynamic_transformer(const geometry_msgs::PointStamped::ConstPtr &msgIn)
{
    geometry_msgs::TransformStamped transform;
    transform.transform.rotation = _current_rot;
    _total_odom.x += msgIn->point.x;
    _total_odom.y += msgIn->point.y;
    _total_odom.z += msgIn->point.z;

    // Set translation transform of drone from POS message
    transform.transform.translation.x = _total_odom.x;
    transform.transform.translation.y = _total_odom.y;
    transform.transform.translation.z = _total_odom.z;

    // Publish pose transform of drone
    transform.header.stamp = msgIn->header.stamp;
    transform.child_frame_id = "base_link";
    transform.header.frame_id = "map";
    _dynamic_broadcaster.sendTransform(transform);
}

void SlamNode::_lidar_feature_extraction(const pcl::PointCloud<PointXYZIT>::ConstPtr &msgIn)
{
    /*
    1. Extract landmark features from trimmed point cloud
    2. Pass these landmarks to optimizer
    */
}


void SlamNode::_imu_ouster_callback(const sensor_msgs::Imu::ConstPtr &msgIn)
{
    /*
    1. 
    
    */
}

void SlamNode::_imu_vectornav_callback(const sensor_msgs::Imu::ConstPtr &msgIn)
{
    _current_rot = msgIn->orientation;
}

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




