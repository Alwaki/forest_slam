#!/usr/bin/python3

# Python tools
from matplotlib import pyplot as plt
import numpy as np
import matplotlib.animation as animation

# ROS imports
import rospy
from geometry_msgs.msg import PoseArray, PointStamped

def feature_plot_cb(feature_array_msg):
    for feature in feature_array_msg.poses:
        if feature.position.z >= 0.8:
            if len(feature_list_x) > 0:
                dist_x = 1000
                dist_y = 1000
                for i in range(len(feature_list_x)):
                    dist_x_i = abs(feature.position.x + curr_x[-1] - feature_list_x[i])
                    dist_y_i = abs(feature.position.y + curr_y[-1] - feature_list_y[i])
                    if dist_x > dist_x_i:
                        dist_x = dist_x_i
                    if dist_y > dist_y_i:
                        dist_y = dist_y_i
                if dist_y > 0.35:
                    if dist_x > 0.35:
                        feature_list_x.append(feature.position.x + curr_x[-1])
                        feature_list_y.append(feature.position.y + curr_y[-1])
            else:
                feature_list_x.append(feature.position.x + curr_x[-1])
                feature_list_y.append(feature.position.y + curr_y[-1])


def odom_plot_cb(odom_msg):
    odom_list_x.append(odom_msg.point.x)
    odom_list_y.append(odom_msg.point.y)
    curr_x.append(curr_x[-1] + odom_msg.point.x)
    curr_y.append(curr_y[-1] + odom_msg.point.y)

if __name__=="__main__":
    feature_list_x = []
    feature_list_y = []
    odom_list_x = []
    odom_list_y = []
    curr_x = [0]
    curr_y = [0]
    rospy.init_node('plotting_node', anonymous=True)
    feature_pos_sub = rospy.Subscriber("feature_node/extracted_pos", 
                                      data_class=PoseArray, callback=feature_plot_cb,
                                      queue_size=10)
    odom_sub = rospy.Subscriber("/estimator_node/odom", 
                                      data_class=PointStamped, callback=odom_plot_cb,
                                      queue_size=10)
    while not rospy.is_shutdown():
        rospy.spin()

    with open("myfile.txt", "w") as file1:
        print("hi")
        print(feature_list_x)
        print(feature_list_y)
        for i in range(len(feature_list_x)):
            file1.write(str(feature_list_x[i]) + " " + str(feature_list_y[i]) + "\n")

    plt.plot(curr_x, curr_y, color='black')
    plt.scatter(feature_list_x,feature_list_y, color='blue')
    plt.show()


