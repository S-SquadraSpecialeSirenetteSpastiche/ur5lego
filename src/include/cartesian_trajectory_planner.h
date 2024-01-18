#pragma once

#include "Eigen/Eigen"
#include <ros/ros.h>


// void compute_and_send_linear_trajectory(Eigen::Isometry3d ti, Eigen::Isometry3d tf, float time, float freq, ros::Publisher publisher);
void compute_and_send_arc_trajectory(
    pinocchio::Model model, Eigen::VectorXd q, Eigen::Isometry3d ti, 
    Eigen::Isometry3d tf, float time, float ds, ros::Publisher publisher);
