#include "Eigen/Eigen"
#include "math_tools.h"
#include <ros/ros.h>


void send_joint_positions(ros::Publisher publisher, Eigen::VectorXd q);

void compute_and_send_trajectory(Eigen::VectorXd qi, Eigen::VectorXd qf, float tf, float freq, ros::Publisher publisher);
