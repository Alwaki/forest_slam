/**
 * @file SlamClass.h
 * @brief Class file for landmark based SLAM using GTSAM solver library
 * @author Alexander Wallén Kiessling
 */

#pragma once

#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/sam/BearingRangeFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/Values.h"
#include "iostream"
#include "cmath"
#include "Eigen/Dense"
#include "random"

class SLAM
{
    private:
        gtsam::NonlinearFactorGraph _graph; 
        std::vector<gtsam::Symbol> _pose_symbols; 
        std::vector<gtsam::Symbol> _landmark_symbols;
};