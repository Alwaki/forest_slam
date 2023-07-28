#!/usr/bin/python3

from LandmarkClass import *
from util import *
from clustering import *
from association import *
from plotting import *

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

# Parameters
slice_count = 10
slice_max_points = 20000
sample_fraction = 0.01
min_radius = 0.1
cov_factor = 1
eps = 0.1
max_uncertainty = 0.8
association_threshold = 0.5
observation_threshold = 0.5
downsample_size = 0.1
outlier_neighbors = 100
outlier_stdev = 0.2
distance_metric = 2    

def feature_extract(pcl_msg):

    # Convert from PointCloud2 to numpy
    pc_generator = pc2.read_points(pcl_msg)
    print(pcl_msg.width)
    print(pcl_msg.height)
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

    # Simple ground filter
    #filter = pcl[2,:] > -1

    #pcl = pcl[filter]

    pcd = arr2pcd(pcl[:, 0], pcl[:, 1], pcl[:, 2])
    pcd = remove_outliers(pcd, outlier_neighbors, outlier_stdev)
    pcd = remove_outliers(pcd, outlier_neighbors, outlier_stdev)
    pcl = pcd2arr(pcd)

    # Cluster slices
    all_means, all_cov = cluster_all(pcl, slice_count, 
                                    slice_max_points, eps, 
                                    sample_fraction, min_radius, 
                                    cov_factor, max_uncertainty)

    # Create landmarks and associate them across slices
    landmarks = associate_slices(all_means, all_cov, 
                                    association_threshold, 
                                    max_uncertainty, distance_metric)
    
    # Mask (remove) landmarks which do not appear often
    landmarks = mask_landmarks(landmarks, slice_count, observation_threshold)

    # Convert to ROS pose array and publish positions of features
    feature_list = MarkerArray()

    for i, landmark in enumerate(landmarks):
        landmark_marker = Marker()
        x, y = landmark.getPos()
        landmark_marker.header.frame_id = "map"
        landmark_marker.header.stamp = pcl_msg.header.stamp
        landmark_marker.type = 3
        landmark_marker.id = i
        landmark_marker.scale.x = 0.2
        landmark_marker.scale.y = 0.2
        landmark_marker.scale.z = 3.0
        landmark_marker.color.r = 0.0
        landmark_marker.color.g = 0.0
        landmark_marker.color.b = 1.0
        landmark_marker.color.a = 0.8
        landmark_marker.pose.position.x = x
        landmark_marker.pose.position.y = y
        landmark_marker.pose.position.z = 0
        landmark_marker.pose.orientation.x = 0
        landmark_marker.pose.orientation.y = 0
        landmark_marker.pose.orientation.z = 0
        landmark_marker.pose.orientation.w = 1
        feature_list.markers.append(landmark_marker)
    feature_pos_pub.publish(feature_list)





if __name__=="__main__":
    rospy.init_node('feature_node', anonymous=True)
    cloud_sub = rospy.Subscriber("/os_cloud_node/points_filtered", 
                                 data_class=PointCloud2, callback=feature_extract, 
                                 queue_size=10)
    feature_pos_pub = rospy.Publisher("feature_node/extracted_pos", 
                                      data_class=MarkerArray, queue_size=10)
    #marker_pub = rospy.Publisher("marker", Marker, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()


#pcl = extract_3D("data/cloud_cut.las")

#landmarks = feature_extract(pcl)
#plot_landmarks(landmarks, slice_count, 1)



