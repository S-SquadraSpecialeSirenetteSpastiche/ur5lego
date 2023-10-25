#include "include/trajectory_planner.h"

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);

    Eigen::VectorXd q0(6);
    q0 << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;

    computeAndSendTrajectory(Eigen::VectorXd::Zero(6), q0, 5.0, 100, pub);

    return 0;
}