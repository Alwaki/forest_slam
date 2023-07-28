#!/usr/bin/python3

import laspy
import open3d as o3d
import numpy as np

def extract_3D(filename):
    """ Reads a LAS file and returns the points in a numpy array

    Args:
        filename (string): file system path

    Returns:
        numpy array of xyz coordinates
    """
    with laspy.open(filename) as fh:
        las = fh.read()
        coords = np.vstack((las.x, las.y, las.z)).transpose()
    return coords

def arr2pcd(x, y, z):
    """ Transform numpy array into Open3D pointcloud object

    Args:
        x (np.array(float)): x coordinates of points
        y (np.array(float)): y coordinates of points
        z (np.array(float)): z coordinates of points

    Returns:
        o3d.geometry.Pointcloud: Open3D pointcloud object
    """
    xyz = np.stack((x, y, z)).astype(np.float64).transpose() 
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    return pcd

def pcd2arr(pcd):
    """ Transform Open3D pointcloud object into numpy array

    Args:
        pcd (o3d.geometry.Pointcloud): pointcloud object

    Returns:
        np.array(float): numpy array of xyz coordinates
    """
    return np.asarray(pcd.points)

def downsample_pcd(pcd, vxsz=0.05):
    """ Downsample pointcloud by voxelizing into specified size

    Args:
        pcd (o3d.geometry.Pointcloud): pointcloud object
        vxsz (float, optional): voxel size (in the units of pointcloud). Defaults to 0.05.

    Returns:
        pcd_new (o3d.geometry.Pointcloud): pointcloud object
    """
    pcd_new = pcd.voxel_down_sample(voxel_size=vxsz)
    return pcd_new

def remove_outliers(pcd, nb_neighbors = 50, std_ratio = 2):
    """ Removes outliers from pointcloud

    Args:
        pcd (o3d.geometry.Pointcloud): pointcloud object
        nb_neighbors (int, optional): neighbours included in calculating average distance for a point. Defaults to 50.
        std_ratio (int, optional): standard deviation threshold for filtering outliers. Defaults to 2.

    Returns:
        pcd_new (o3d.geometry.Pointcloud): pointcloud object (with removed outliers)
    """
    pcd_new, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return pcd_new
