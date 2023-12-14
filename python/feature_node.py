#!/usr/bin/python3

# Custom modules
from LandmarkClass import *
from util import *
from clustering import *
from association import *

# ROS imports
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray

# Parameters
slice_count = 5
min_points = 8
epsilon = 0.2
max_uncertainty = 0.6
association_threshold = 0.8
observation_threshold = 0.1
downsample_size = 0.15
outlier_neighbors = 40
outlier_stdev = 0.1
distance_metric = 2    

def feature_extract(pcl_msg):

    # Convert from PointCloud2 to numpy
    pc_generator = pc2.read_points(pcl_msg)
    #print(pcl_msg.width)
    #print(pcl_msg.height)
    pcl = np.zeros((pcl_msg.width * pcl_msg.height, 3))
    for i, point in enumerate(pc_generator):
        # point[0]: x position; comes as float32
        # point[1]: y position; comes as float32
        # point[2]: z position; comes as float32
        # point[3]: intensity;  comes as float32
        # point[4]: time stamp; comes as uint32
        # point[5]: tag;        comes as uint32
        # point[6]: height;     comes as float32
        pcl[i, :] = point[0:3]

    pcd = arr2pcd(pcl[:, 0], pcl[:, 1], pcl[:, 2])
    pcd = remove_outliers(pcd, outlier_neighbors, outlier_stdev)
    pcd = remove_outliers(pcd, outlier_neighbors, outlier_stdev)
    pcl = pcd2arr(pcd)

    # Publish outlier filtered pointcloud for visualization
    cloud_points = []
    for xyz in pcl:
        cloud_points.append(xyz) 
    header = pcl_msg.header
    scaled_polygon_pcl = pc2.create_cloud_xyz32(header, cloud_points)
    filtered_cloud_pub.publish(scaled_polygon_pcl)

    # Cluster slices
    all_means, all_cov = cluster_all(pcl, slice_count, epsilon, min_points, max_uncertainty)

    # Create landmarks and associate them across slices
    landmarks = associate_slices(all_means, all_cov, 
                                    association_threshold, 
                                    max_uncertainty, distance_metric)
    
    # Mask (remove) landmarks which do not appear often
    landmarks = mask_landmarks(landmarks, slice_count, observation_threshold)

    # Convert to ROS pose array and publish positions of features
    marker_list = MarkerArray()
    pos_list = PoseArray()

    for i, landmark in enumerate(landmarks):
        landmark_pos = Pose()
        landmark_marker = Marker()
        x, y = landmark.getPos()
        certainty = landmark.getObsCount()/slice_count
        landmark_pos.position.x = x
        landmark_pos.position.y = y
        landmark_pos.position.z = certainty
        landmark_marker.header.frame_id = "map"
        landmark_marker.header.stamp = pcl_msg.header.stamp
        landmark_marker.type = 3
        landmark_marker.id = i
        landmark_marker.scale.x = 0.15
        landmark_marker.scale.y = 0.15
        landmark_marker.scale.z = 2.5
        landmark_marker.color.r = 1.1 - certainty
        landmark_marker.color.g = 0.0
        landmark_marker.color.b = certainty
        landmark_marker.color.a = 0.3 + 0.7*certainty
        landmark_marker.pose.position.x = x
        landmark_marker.pose.position.y = y
        landmark_marker.pose.position.z = 0.0
        landmark_marker.pose.orientation.x = 0
        landmark_marker.pose.orientation.y = 0
        landmark_marker.pose.orientation.z = 0
        landmark_marker.pose.orientation.w = 1
        landmark_marker.lifetime = rospy.Duration(0.1)
        marker_list.markers.append(landmark_marker)
        pos_list.poses.append(landmark_pos)
    feature_marker_pub.publish(marker_list)
    feature_pos_pub.publish(pos_list)





if __name__=="__main__":
    rospy.init_node('feature_node', anonymous=True)
    cloud_sub = rospy.Subscriber("/os_cloud_node/points_transformed", 
                                 data_class=PointCloud2, callback=feature_extract, 
                                 queue_size=10)
    feature_marker_pub = rospy.Publisher("feature_node/markers", 
                                      data_class=MarkerArray, queue_size=1)
    feature_pos_pub = rospy.Publisher("feature_node/extracted_pos", 
                                      data_class=PoseArray, queue_size=100)
    
    filtered_cloud_pub = rospy.Publisher("feature_node/points_outlier_filtered",
                                         data_class=PointCloud2, queue_size=10)
    while not rospy.is_shutdown():
        rospy.spin()



