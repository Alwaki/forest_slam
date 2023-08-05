#!/usr/bin/python3

from LandmarkClass import *
from util import *
from itertools import compress

def associate_slices(all_means, all_cov, association_threshold, max_uncertainty, method):
    landmarks = []
    # First slice 
    means = all_means[0]
    cov = all_cov[0]
    for j in range(len(means[0])):
        landmarks.append(Landmark(mean=[means[0,j], means[1,j]], 
                                cov=[cov[0,j], cov[1,j]]))
    
    # For each slice after first
    for i in range(len(all_means)-1):
        slice_means = all_means[i+1]
        slice_cov = all_cov[i+1]
        # For measurement
        for j in range(len(all_means[i+1][1])):
            mean = [slice_means[0,j], slice_means[1,j]]
            cov = [slice_cov[0,j], slice_cov[1,j]]
            d_min = 100000
            ind_min = 0
            for ind, landmark in enumerate(landmarks):
                if ~landmark.getObserved():
                    if method == 0:
                        d = landmark.calculateEuclidean(mean, cov)
                    elif method == 1:
                        d = landmark.calculateMahalonobis(mean, cov)
                    elif method == 2:
                        d = landmark.calculateBhattacharyya(mean, cov)
                    else: 
                        d = landmark.calculateBhattacharyya(mean, cov)
                        d += landmark.calculateEuclidean(mean, cov)
                    if d < d_min:
                        ind_min = ind
                        d_min = d
            # Existing landmark
            if d_min < association_threshold:
                landmarks[ind_min].update(mean, cov)
                landmarks[ind_min].setObserved(True)
                # Check if uncertainty has grown too large
                test_cov = landmarks[ind_min].getCov()
                limit_check = np.sqrt(test_cov[0]**2 + test_cov[1]**2)
                if limit_check > max_uncertainty:
                    landmarks.pop(ind_min)

            # New landmark
            else:
                landmarks.append(Landmark(mean, cov))
        for l in landmarks:
            l.setObserved(False)
    return landmarks

def mask_landmarks(landmarks, slice_count, threshold):
    mask = []            
    for landmark in landmarks:
        if landmark.getObsCount() < threshold*slice_count:
            mask.append(False)
        else:
            if landmark.getObsCount() > slice_count:
                mask.append(False)
            else:
                mask.append(True)

    landmarks = list(compress(landmarks, mask))
    return landmarks