#include "Eigen/Eigen"
#include "math_tools.h"
#include <ros/ros.h>


void send_joint_positions(ros::Publisher publisher, Eigen::VectorXd q);

void computeAndSendTrajectory(Eigen::VectorXd q, Eigen::VectorXd qf, float t, int steps, ros::Publisher publisher);
