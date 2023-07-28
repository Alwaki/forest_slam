/**
 * @file    SlamNode.cpp
 * @brief   ROS interface node to handle incoming data and run all SLAM functionality
 *          for documentation, see header files.
 * @author  Alexander Wallén Kiessling
 */

#include "SlamNode.h"
//#include "FeatureEstimator.h"

SlamNode::SlamNode():
    _nh("~")
    {_init_node();}

SlamNode::~SlamNode() = default;

void SlamNode::_init_node()
{
    // Initialize necessary variables
    _imu_counter = 0;
    _current_pos.x = 0;
    _current_pos.y = 0;

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

    _lidar_ouster_filtered_pub  = _nh.advertise<sensor_msgs::PointCloud2>("/os_cloud_node/points_filtered", 10);

}

void SlamNode::_lidar_ouster_callback(const sensor_msgs::PointCloud2::ConstPtr &msgIn)
{
    /*
    1. Trim the point cloud height wise and distance wise (or somehow segment ground away)
    2. Publish trimmed point cloud for feature extraction and lidar odometry calculation
    */
    
    // Change to PCL format
    //pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2 ());
    //pcl_conversions::toPCL(*msgIn, *pcl_cloud);

    // Downsample cloud
    /*
    pcl::PCLPointCloud2::Ptr pcl_voxel (new pcl::PCLPointCloud2 ());
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    voxel_grid.setInputCloud(pcl_cloud);
    voxel_grid.setLeafSize(0.25f, 0.25f, 0.05f);
    voxel_grid.filter(*pcl_voxel);
    */

    // Passthrough filter
    //pcl::PCLPointCloud2::Ptr pcl_pass (new pcl::PCLPointCloud2 ());
    //pcl::PassThrough<pcl::PCLPointCloud2> pass_z;
    //pass_z.setInputCloud(pcl_cloud);
    //pass_z.setFilterFieldName("z");
    //pass_z.setFilterLimits(-1, 4);
    //pass_z.filter(*pcl_pass);

    /*
    pcl::PassThrough<pcl::PCLPointCloud2> pass_x;
    pass_x.setInputCloud (pcl_pass);
    pass_x.setFilterFieldName ("x");
    pass_x.setFilterLimits (_current_pos.x-10, _current_pos.x+10);
    pass_x.filter (*pcl_pass);
    pcl::PassThrough<pcl::PCLPointCloud2> pass_y;
    pass_y.setInputCloud (pcl_pass);
    pass_y.setFilterFieldName ("y");
    pass_y.setFilterLimits (_current_pos.y-10, _current_pos.y+10);
    pass_y.filter (*pcl_pass);
    */
    
    // Outlier removal
    //pcl::PCLPointCloud2::Ptr pcl_filtered (new pcl::PCLPointCloud2 ());
    //pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    //sor.setInputCloud(pcl_pass);
    //sor.setMeanK(2);
    //sor.setStddevMulThresh(5);
    //sor.filter(*pcl_filtered);
    

    // Convert back to ROS and transform to map frame
    std::string target = "map";
    sensor_msgs::PointCloud2 ros_cloud, map_cloud;
    //pcl_conversions::moveFromPCL(*pcl_pass, ros_cloud);
    pcl_ros::transformPointCloud(target, *msgIn, map_cloud, _tf_listener);
    map_cloud.header.stamp = msgIn->header.stamp;
    map_cloud.header.frame_id = "map";

    // Publish cloud
    _lidar_ouster_filtered_pub.publish(map_cloud);
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
    if(_imu_counter == 0)
    {
        _prev_time = msgIn->header.stamp;
    }
    else
    {
        ros::Duration dt_dur = msgIn->header.stamp - _prev_time;
        double dt = dt_dur.toSec();
        _prev_time = msgIn->header.stamp;
        tf::Quaternion q(msgIn->orientation.x, msgIn->orientation.y,
                         msgIn->orientation.z, msgIn->orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if(_imu_counter > 5200 && _imu_counter < 44200)
        {
            _current_pos.x += cos(yaw)*0.5*dt;
            _current_pos.y += sin(yaw)*0.5*dt;
        }
        

        geometry_msgs::TransformStamped transform;
        transform.transform.rotation = msgIn->orientation;
        transform.transform.translation.x = _current_pos.x;
        transform.transform.translation.y = _current_pos.y;
        transform.transform.translation.z = 0;
        transform.header.stamp = msgIn->header.stamp;
        transform.child_frame_id = "base_link";
        transform.header.frame_id = "map";
        _dynamic_broadcaster.sendTransform(transform);
    }
    _imu_counter++;
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



/*
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
*/


