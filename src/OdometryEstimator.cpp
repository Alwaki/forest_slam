/**
 * @file    OdometryEstimator.cpp
 * @brief   Creates odometry estimate with ICP on features, and orientation from IMU
 * @author  Alexander Wallén Kiessling
 */

#include "OdometryEstimator.h"

// Constructor
OdometryEstimator::OdometryEstimator():
    nh_("~")
    {init_node_();}

// Destructor
OdometryEstimator::~OdometryEstimator() = default;

/**
 * Initializes the OdometryEstimator object. 
 * Sets variables, instanciates publishers
 * and subscribers.
 */
void OdometryEstimator::init_node_()
{
    // Initialize variables
    prev_cloud_flag_ = 0;
    current_pos_.x = 0;
    current_pos_.y = 0;
    current_pos_.theta = 0;
    total_distance_ = 0;

    // Get parameters from server
    nh_.getParam("/forest_slam/icp/MaxCorrespondenceDistance", max_correspondence_distance_);
    nh_.getParam("/forest_slam/icp/MaximumIterations", maximum_iterations_);
    nh_.getParam("/forest_slam/icp/TransformationEpsilon", transformation_epsilon_);
    nh_.getParam("/forest_slam/icp/EuclideanFitnessEpsilon", euclidean_fitness_epsilon_);

    // Setup subscribers and publishers
    feature_pos_sub_ = nh_.subscribe<geometry_msgs::PoseArray>
        ("/feature_node/extracted_pos", 100, &OdometryEstimator::feature_icp_,
        this, ros::TransportHints().tcpNoDelay(true));

    imu_vectornav_sub_ = nh_.subscribe<sensor_msgs::Imu>
        ("/imu/data_raw", 100, &OdometryEstimator::dynamic_transform_broadcast_,
        this, ros::TransportHints().tcpNoDelay(true));

    lidar_raw_sub_  = nh_.subscribe<sensor_msgs::PointCloud2>
        ("/os_cloud_node/points", 1, &OdometryEstimator::lidar_deskew_,
        this, ros::TransportHints().tcpNoDelay(true));

    icp_odom_pub_  = nh_.advertise<geometry_msgs::PointStamped>(
        "/estimator_node/odom", 20);
    
    current_pose2d_pub_  = nh_.advertise<geometry_msgs::Pose2D>(
        "/estimator_node/absPose", 1);

    lidar_processed_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(
        "/os_cloud_node/points_transformed", 2);
}

/**
 * Scan registration of feature points with iterative closest point.
 * Publishes odometry in the horizontal plane.
 *
 * @param msgIn an array of feature locations, used to create pointcloud
 */
void OdometryEstimator::feature_icp_(const geometry_msgs::PoseArray::ConstPtr &msgIn)
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

    if(prev_cloud_flag_ == 1)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
        icp.setMaximumIterations(maximum_iterations_);
        icp.setTransformationEpsilon(transformation_epsilon_);
        icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
        icp.setInputSource(icp_cloud);
        icp.setInputTarget(prev_cloud_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*aligned_cloud);
        Eigen::Matrix4d transformation_matrix = icp.getFinalTransformation().cast<double>();
        geometry_msgs::PointStamped odom;
        odom.header.stamp = msgIn->header.stamp;
        odom.header.frame_id = msgIn->header.frame_id;
        odom.point.x = transformation_matrix(0,3);
        odom.point.y = transformation_matrix(1,3);
        pos_mtx_.lock();
        odom.point.z = yaw_diff_;
        icp_odom_pub_.publish(odom);
        total_distance_ += sqrt(pow(odom.point.x, 2) + pow(odom.point.y, 2));
        ROS_INFO("Distance: %lf", total_distance_);
        current_pos_.x += odom.point.x;
        current_pos_.y += odom.point.y;
        current_pose2d_pub_.publish( current_pos_);
        //ROS_INFO("Pose, X: %lf, Y: %lf, Theta: %lf", current_pos_.x,  current_pos_.y,  current_pos_.theta);
        pos_mtx_.unlock();
    }
    
    prev_cloud_ = icp_cloud;
    prev_cloud_flag_ = 1;
}

/**
 * Uses inertial and magnetic data combined with translation
 * to publish transform poses
 *
 * @param msgIn inertial data from IMU + MAG
 */
void OdometryEstimator::dynamic_transform_broadcast_(const sensor_msgs::Imu::ConstPtr &msgIn)
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

    pos_mtx_.lock();
    yaw_diff_ =  current_pos_.theta - yaw;
    current_pos_.theta = yaw;
    //ROS_INFO("theta: %lf", current_pos_.theta);
    pos_mtx_.unlock();
    
    transform.transform.translation.x = 0; // current_pos_.x;
    transform.transform.translation.y = 0; // current_pos_.y;
    transform.transform.translation.z = 0;
    transform.header.stamp = msgIn->header.stamp;
    transform.child_frame_id = "imu";
    transform.header.frame_id = "map";
    dynamic_broadcaster_.sendTransform(transform);
}


/**
 * Takes in a raw pointcloud from a 3D lidar sensor and attempts to filter
 * and deskew it. To deskew, the IMU sensor is used. For filtering, 
 * invalid points and points at certain range intervals are removed.
 *
 * @param msgIn a raw pointcloud to preprocess
 */
void OdometryEstimator::lidar_deskew_(const sensor_msgs::PointCloud2::ConstPtr &msgIn)
{
    // Create PCL structure 
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->width = 0;
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;
    filtered_cloud->points.resize(filtered_cloud->width * filtered_cloud->height);
    filtered_cloud->header.frame_id = msgIn->header.frame_id;


    // Get time for motion compensation
    double dt;
    if(prev_cloud_flag_ != 1)
    {
        dt = 0.1 / ( (double) msgIn->width*msgIn->height);
    }
    else
    {
        dt = (msgIn->header.stamp - prev_time_).toSec()/( (double) msgIn->width*msgIn->height);
    }
    ros::Duration time_offset(dt);
    prev_time_ = msgIn->header.stamp;

    // Remove points and compensate motion
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msgIn, "x"); it != it.end(); ++it) {
        if(it[0] != 0 && it[1] != 0)
        {
        if(4.0 > it[2] &&  it[2] > -1.0)
        {
        float distance = it[0] * it[0] + it[1] * it[1];
        if(distance < 81.0 && distance > 0.4)
        {
            auto time = msgIn->header.stamp + time_offset;
            geometry_msgs::PointStamped p_in, p_out;
            p_in.point.x = it[0];
            p_in.point.y = it[1];
            p_in.point.z = it[2];
            p_in.header.stamp = time;
            p_in.header.frame_id = "lidar";
            tf_listener_.waitForTransform("lidar", "map", time, ros::Duration(0.01));
            tf_listener_.transformPoint("map", p_in, p_out);
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
    sensor_msgs::PointCloud2 ros_cloud, map_frame_cloud;
    pcl_conversions::moveFromPCL(pcl_pc2, map_frame_cloud);
    map_frame_cloud.header.stamp = msgIn->header.stamp;
    map_frame_cloud.header.frame_id = "map";

    // Publish cloud
    lidar_processed_pub_.publish(map_frame_cloud);
}

/**
 * Creates OdometryEstimator object and 
 * begins node connection with ROS interface. 
 */
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