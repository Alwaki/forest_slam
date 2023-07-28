#!/usr/bin/python3

from sklearn.cluster import DBSCAN
from sklearn.cluster import HDBSCAN
import numpy as np
import matplotlib.pyplot as plt
from util import *

def cluster_all(coords, slice_count, slice_max_points, _eps, sample_fraction, min_radius, cov_factor, max_uncertainty):
    
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
            if np.size(xy_slice, 0) > slice_max_points:
                xy_slice = xy_slice[np.random.choice(xy_slice.shape[0], slice_max_points, replace=False), :]

            # Cluster slice
            samples_required = int(np.size(xy_slice, 0)*sample_fraction)
            samples_required = max(samples_required, 1)
            clusters_mean, clusters_cov = cluster_slice(xy_slice, samples_required, _eps, min_radius, cov_factor, max_uncertainty)
            all_means.append(clusters_mean)
            all_cov.append(clusters_cov)
    return all_means, all_cov

def cluster_slice(xy_slice, _min_samples, _eps, min_radius, cov_factor, max_uncertainty):

    # Compute clustering
    clustering = DBSCAN(eps=_eps, min_samples=_min_samples).fit(xy_slice)
    labels = clustering.labels_
    unique_labels = set(labels)
    unique_labels.discard(-1)
    core_samples_mask = np.zeros_like(labels, dtype=bool)
    core_samples_mask[clustering.core_sample_indices_] = True

    # Data structures to hold clusters
    clusters_mean = np.zeros(shape=(2,0))
    clusters_cov = np.zeros(shape=(2,0))
    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]

    for k in unique_labels:
    
        class_member_mask = labels == k
        xy = xy_slice[class_member_mask & core_samples_mask]
        mean_x = np.mean(xy[:,0])
        mean_y = np.mean(xy[:,1])
        cov_x =  np.std(xy[:,0]).item()
        cov_y = np.std(xy[:,1]).item()
        uncertainty_x = (min_radius+cov_factor*cov_x)
        uncertainty_y = (min_radius+cov_factor*cov_y)
        tot_uncertainty = np.sqrt(uncertainty_x**2 + uncertainty_y**2)

        if tot_uncertainty < max_uncertainty:
            clusters_mean = np.hstack((clusters_mean, np.array([[mean_x], [mean_y]])))
            clusters_cov = np.hstack((clusters_cov, np.array([[uncertainty_x], [uncertainty_y]])))

    return clusters_mean, clusters_cov