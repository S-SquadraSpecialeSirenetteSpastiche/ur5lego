#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H
#include "Eigen/Eigen"
#include "math_tools.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>


void send_arm_joint_angles(Eigen::VectorXd q, ros::Publisher publisher);
void send_gripper_joint_angles(_Float64 q, ros::Publisher publisher);

void computeAndSendTrajectory(Eigen::VectorXd q, Eigen::VectorXd qf, float t, int steps, ros::Publisher publisher);
void computeAndSendTrajectory(_Float64 qi, _Float64 qf, float tf, int steps, ros::Publisher publisher);
#endif