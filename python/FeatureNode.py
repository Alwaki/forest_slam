from LandmarkClass import *
from util import *
from clustering import *
from association import *
from plotting import *

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3


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

def feature_extract(pcl):
    """
    """
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

    feature_list = []

    for landmark in landmarks:
        feature_list.append(landmark.getPos())
    return feature_list


def cloud_callback():
    # convert cloud

    # get features

    rviz_points = Marker()

    rviz_points.header.frame_id = "map"
    rviz_points.ns = "points"
    rviz_points.id = 1

    rviz_points.type = Marker.POINTS
    rviz_points.action = Marker.ADD

    rviz_points.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
    rviz_points.scale = Vector3(0.2, 0.2, 0.2)

    pass

if __name__=="__main__":
    marker_pub = rospy.Publisher("marker", Marker, queue_size=1)
    cloud_sub = rospy.Subscriber("/os_cloud_node/points_filtered", 
                                 data_class=PointCloud2, callback=cloud_callback, 
                                 queue_size=10)
    rospy.init_node('feature_node', anonymous=True)


#pcl = extract_3D("data/cloud_cut.las")

#landmarks = feature_extract(pcl)
#plot_landmarks(landmarks, slice_count, 1)



