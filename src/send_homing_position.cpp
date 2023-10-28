#include "include/trajectory_planner.h"

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <rosgraph_msgs/Clock.h>


ros::Publisher pub;
bool done = false;

void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
    if (msg->clock.toSec() > 0.0 && !done)
    {
        done = true;
        Eigen::VectorXd q0(6);
        q0 << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
        computeAndSendTrajectory(Eigen::VectorXd::Zero(6), q0, 5.0, 100, pub);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "send_homing_position");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
    ros::Subscriber sub = nh.subscribe("/clock", 10, clockCallback);

    while(!done){
        ros::spinOnce();
    }

    return 0;
}

