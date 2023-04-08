# forest_slam
## TODO
SlamNode is intended to be the main ROS interface. This is where backend and frontend are launched.

Backend needs to get landmarks and poses from the frontend. These are then optimized into a better map.

Frontend needs to provide landmark and pose information to backend, in terms of x-y for landmark and x-y-theta for pose. 

Current idea is to first segment ground points. This can be used to identify tree clusters, and to help estimate the tilt of the lidar between scans. 

Then identify tree clusters. Use curvature to get a center point, x-y which is returned to backend. We cam also do some form of registration of these landmarks between the latest and previous to get a new pose from the resultant transform, which is also passed to the backend.

## Overview

This package is used to perform optimization based SLAM with lidar and IMU, in dense forests. The backpack mounted rig used for testing is equipped with lidars of type (Ouster OS1-64-u & Livox Mid-70) as well as IMU (VectorNav). Cameras and additional sensors are also included in the test rig, but not used for this package.

**Keywords:** SLAM, forest, lidar, IMU

**Author: Alexander Wallén Kiessling<br />
Affiliation: [Robotics, Perception and Learning @KTH](https://www.kth.se/is/rpl/)<br />

The forest_slam package has been tested under [ROS] Melodic on Ubuntu 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies TOFIX

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)
- [PCL] (point cloud liBrary)
- [GTSAM] (SLAM library)
- [LivoxSDK] (Lidar)

#### Building TODO

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/ethz-asl/ros_best_practices.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage TODO

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch ros_package_template ros_package_template.launch
