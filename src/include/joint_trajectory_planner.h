#pragma once

#include "Eigen/Eigen"
#include "math_tools.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>


void send_arm_joint_angles(Eigen::VectorXd q, ros::Publisher publisher);
void send_gripper_joint_angles(_Float64 q, ros::Publisher publisher);

void compute_and_send_trajectory(Eigen::VectorXd qi, Eigen::VectorXd qf, float tf, float freq, ros::Publisher publisher);
void computeAndSendTrajectory(_Float64 qi, _Float64 qf, float tf, int steps, ros::Publisher publisher);