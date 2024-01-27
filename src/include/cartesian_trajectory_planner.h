#pragma once

#include "Eigen/Eigen"
#include <ros/ros.h>


// void compute_and_send_linear_trajectory(Eigen::Isometry3d ti, Eigen::Isometry3d tf, float time, float freq, ros::Publisher publisher);
std::pair<Eigen::VectorXd, bool> compute_and_send_arc_trajectory(
    pinocchio::Model model, Eigen::VectorXd q0, Eigen::Isometry3d ti, 
    Eigen::Isometry3d tf, float time, float ds, ros::Publisher publisher);
