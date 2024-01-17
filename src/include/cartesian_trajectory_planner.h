#pragma once

#include "Eigen/Eigen"
#include <ros/ros.h>


void compute_and_send_arc_trajectory(Eigen::Translation3d qi, Eigen::VectorXd qf, float tf, float freq, ros::Publisher publisher);
