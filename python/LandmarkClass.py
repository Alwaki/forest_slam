#!/usr/bin/python3

import numpy as np
from scipy.spatial import distance

class Landmark:
    def __init__(self, mean, cov):
        self._mean = np.array(mean)
        self._cov = np.array([[cov[0], 0],[0, cov[1]]]) 
        self._n = 1.0
        self._observed = False

    def update(self, mean, cov):
        mean = np.array(mean)
        cov = np.array([[cov[0], 0],[0, cov[1]]]) 
        self._mean = (self._n*self._mean + mean)/(self._n+1.0)
        self._cov = self._n*self._cov/(self._n+1) + cov/(self._n+1.0)
        self._n += 1.0

    def calculateEuclidean(self, mean, cov):
        return np.sqrt((mean[0]-self._mean[0])**2+(mean[1]-self._mean[1])**2)

    def calculateMahalonobis(self, mean, cov):
        return distance.mahalanobis(mean, [self._mean[0], self._mean[1]], np.linalg.inv(self._cov + np.array([[cov[0], 0],[0, cov[1]]]) ))
    
    def calculateBhattacharyya(self, mean, cov):
        s1 = np.array([[cov[0], 0],[0, cov[1]]]) 
        s2 = self._cov
        s_tot = (s1 + s2) / 2.0
        s_tot_inv = np.linalg.inv(s_tot)
        mu1 = np.array(mean)
        mu2 = self._mean
        mu_d = mu1 - mu2
        d1 = np.linalg.det(s1)
        d2 = np.linalg.det(s2)
        dt = np.linalg.det(s_tot)

        temp = np.matmul(mu_d.T, s_tot_inv)
        temp2 = np.matmul(temp, mu_d)
        
        return 0.125 * temp2 + 0.5 * np.log(dt/(np.sqrt(max(1E-10, d1*d2))))

    
    def getPos(self):
        return [self._mean[0], self._mean[1]]
    
    def getCov(self):
        return [self._cov[0,0], self._cov[1,1]]
    
    def getObsCount(self):
        return self._n
    
    def getObserved(self):
        return self._observed

    def setObserved(self, state):
        self._observed = state
    