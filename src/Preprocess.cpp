/**
 * @file    Preprocess.cpp
 * @brief   ROS interface node to handle incoming data and process it
 * @author  Alexander Wallén Kiessling
 */

#include "Preprocess.h"

Preprocess::Preprocess():
    _nh("~")
    {_init_node();}

Preprocess::~Preprocess() = default;

void Preprocess::_init_node()
{
    // Setup subscribers and publishers
    _lidar_ouster_sub  = _nh.subscribe<sensor_msgs::PointCloud2>
        ("/os_cloud_node/points", 1, &Preprocess::_lidar_ouster_callback,
        this, ros::TransportHints().tcpNoDelay(true));

    _lidar_ouster_filtered_pub  = _nh.advertise<sensor_msgs::PointCloud2>(
        "/os_cloud_node/points_filtered", 2);

    _lidar_ouster_sub_2  = _nh.subscribe<sensor_msgs::PointCloud2>
        ("/os_cloud_node/points", 1, &Preprocess::_pc_transformed_callback,
        this, ros::TransportHints().tcpNoDelay(true));

    _pc_transformed_pub  = _nh.advertise<sensor_msgs::PointCloud2>(
        "/os_cloud_node/points_transformed", 2);
}

void Preprocess::_lidar_ouster_callback(const sensor_msgs::PointCloud2::ConstPtr &msgIn)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->width = 0;
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;
    filtered_cloud->points.resize(filtered_cloud->width * filtered_cloud->height);
    filtered_cloud->header.frame_id = msgIn->header.frame_id;
    double dt = 0.1 / (msgIn->width*msgIn->height);
    ros::Duration time_offset(dt);
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msgIn, "x"); it != it.end(); ++it) {
        // Check 
        // 1. Point nonzero (zero points indicate invalid return value)
        // 2. Point above ground height Z (and not too high!)
        // 3. Point within rough 7m radius of XY
        if(it[0] != 0 && it[1] != 0)
        {
        if(4.0 > it[2] &&  it[2] > -1.0)
        {
        float distance = it[0] * it[0] + it[1] * it[1];
        if(distance < 64.0 && distance > 1)
        {
            auto time = msgIn->header.stamp + time_offset;
            geometry_msgs::PointStamped p_in, p_out;
            p_in.point.x = it[0];
            p_in.point.y = it[1];
            p_in.point.z = it[2];
            p_in.header.stamp = time;
            p_in.header.frame_id = msgIn->header.frame_id;
            _tf_listener.waitForTransform(msgIn->header.frame_id, "map", time, ros::Duration(0.01));
            _tf_listener.transformPoint("map", p_in, p_out);
            pcl::PointXYZ pt;
            pt.x = p_out.point.x;
            pt.y = p_out.point.y;
            pt.z = p_out.point.z;
            filtered_cloud->points.push_back(pt);
            filtered_cloud->width++;
        }
        }
        }
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*filtered_cloud, pcl_pc2);

    // Convert back to ROS and transform to map frame
    //std::string target = "map";
    sensor_msgs::PointCloud2 ros_cloud, map_frame_cloud;
    pcl_conversions::moveFromPCL(pcl_pc2, map_frame_cloud);
    //pcl_ros::transformPointCloud(target, *msgIn, map_cloud, _tf_listener);
    map_frame_cloud.header.stamp = msgIn->header.stamp;
    map_frame_cloud.header.frame_id = "map";

    // Publish cloud
    _lidar_ouster_filtered_pub.publish(map_frame_cloud);
}

void Preprocess::_pc_transformed_callback(const sensor_msgs::PointCloud2::ConstPtr &msgIn)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->width = 0;
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;
    filtered_cloud->points.resize(filtered_cloud->width * filtered_cloud->height);
    filtered_cloud->header.frame_id = msgIn->header.frame_id;
    double dt = 0.1 / (msgIn->width*msgIn->height);
    ros::Duration time_offset(dt);
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msgIn, "x"); it != it.end(); ++it) 
        {
            auto time = msgIn->header.stamp + time_offset;
            geometry_msgs::PointStamped p_in, p_out;
            p_in.point.x = it[0];
            p_in.point.y = it[1];
            p_in.point.z = it[2];
            p_in.header.stamp = time;
            p_in.header.frame_id = "sensor";
            _tf_listener.waitForTransform("sensor", "map", time, ros::Duration(0.01));
            _tf_listener.transformPoint("map", p_in, p_out);
            pcl::PointXYZ pt;
            pt.x = p_out.point.x;
            pt.y = p_out.point.y;
            pt.z = p_out.point.z;
            filtered_cloud->points.push_back(pt);
            filtered_cloud->width++;
        }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*filtered_cloud, pcl_pc2);

    // Convert back to ROS and transform to map frame
    //std::string target = "map";
    sensor_msgs::PointCloud2 ros_cloud, map_frame_cloud;
    pcl_conversions::moveFromPCL(pcl_pc2, map_frame_cloud);
    //pcl_ros::transformPointCloud(target, *msgIn, map_cloud, _tf_listener);
    map_frame_cloud.header.stamp = msgIn->header.stamp;
    map_frame_cloud.header.frame_id = "map";

    // Publish cloud
    _pc_transformed_pub.publish(map_frame_cloud);
        
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "preprocess_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Started Preprocess Node!");

    Preprocess node;
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

void Preprocess::_imu_vectornav_callback(const sensor_msgs::Imu::ConstPtr &msgIn)
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

*/


