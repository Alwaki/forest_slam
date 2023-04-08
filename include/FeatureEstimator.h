/**
 * @file FeatureEstimator.h
 * @brief Class file for the SLAM frontend using PCL library
 * @author Alexander Wallén Kiessling
 */

#pragma once

class FeatureEstimator
{
    public:
        FeatureEstimator();
        virtual ~FeatureEstimator();
        void estimation_thread();       // Run callback based, estimate transform and landmarks

    private:
        void segment_landmarks();
        void segment_ground();
        void estimate_transform();      // Transform is based on ground and landmarks
        void estimate_ground();
        
};