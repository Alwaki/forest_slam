#!/usr/bin/python3

from sklearn.cluster import HDBSCAN
import numpy as np
from util import *

def cluster_all(coords, slice_count, epsilon, min_points, max_uncertainty):
    
    # Extract heights for slicing data vertically
    max_z = max(coords[:,2])
    min_z = min(coords[:,2])
    dz = (max_z - min_z)/slice_count
    
    all_means = []
    all_cov = []

    # Iterate across height slices
    for height_i in range(slice_count):
        # Extract slice
        idx = np.where((min_z + height_i*dz <= coords[:,2]) &  ( coords[:,2]<= min_z + (height_i+1)*dz))
        if len(idx[0]) > 1:
            slice = coords[idx]

            xy_slice = slice[:,[0,1]]

            # Cluster slice
            clusters_mean, clusters_cov = cluster_slice(xy_slice, min_points, epsilon, max_uncertainty)
            all_means.append(clusters_mean)
            all_cov.append(clusters_cov)
    return all_means, all_cov

def cluster_slice(xy_slice, min_points, epsilon, max_uncertainty):

    # Compute clustering
    clustering = HDBSCAN(min_cluster_size=min_points, cluster_selection_epsilon=epsilon).fit(xy_slice)
    labels = clustering.labels_
    unique_labels = set(labels)
    unique_labels.discard(-1)

    # Data structures to hold clusters
    clusters_mean = np.zeros(shape=(2,0))
    clusters_cov = np.zeros(shape=(2,0))

    for k in unique_labels:
    
        class_member_mask = labels == k
        xy = xy_slice[class_member_mask]
        mean_x = np.mean(xy[:,0])
        mean_y = np.mean(xy[:,1])
        cov_x =  np.var(xy[:,0]).item()
        cov_y = np.var(xy[:,1]).item()
        uncertainty_x = cov_x
        uncertainty_y = cov_y
        tot_uncertainty = np.sqrt(uncertainty_x**2 + uncertainty_y**2)

        if tot_uncertainty < max_uncertainty:
            clusters_mean = np.hstack((clusters_mean, np.array([[mean_x], [mean_y]])))
            clusters_cov = np.hstack((clusters_cov, np.array([[uncertainty_x], [uncertainty_y]])))

    return clusters_mean, clusters_cov