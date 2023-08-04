/**
 * @file    GraphOptimizer.cpp
 * @brief   Class file for the SLAM backend using GTSAM solver library
 * @author  Alexander Wallén Kiessling
 */

#include "GraphOptimizer.h"

GraphOptimizer::GraphOptimizer():
    _nh("~")
    {}

GraphOptimizer::~GraphOptimizer() = default;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optimizer_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Started GraphOptimizer Node!");

    GraphOptimizer node;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
}
