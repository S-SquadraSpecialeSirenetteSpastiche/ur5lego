#include "Eigen/Eigen"
#include "math_tools.h"
#include "inverse_kinematics.h"
#include "pinocchio/math/rpy.hpp"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>


void send_joint_positions(ros::Publisher publisher, Eigen::VectorXd q);

void computeAndSendTrajectory(Eigen::VectorXd q, Eigen::VectorXd qf, float t, int steps, ros::Publisher publisher);
