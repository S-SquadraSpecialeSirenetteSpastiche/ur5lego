#include "../include/inverse_kinematics.h"
#include "../include/joint_trajectory_planner.h"

#include "pinocchio/parsers/urdf.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Float64.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <ur5lego/GripperAction.h>
#include <string>


class GripperAction
{
protected:
    const std::string PUBLISHING_CHANNEL = "/gripper_joint_position";
    _Float64 q;
    // node that acts as action server
    ros::NodeHandle server_node;
    // node that sends commands to the robot
    ros::NodeHandle talker_node;
    ros::Publisher publisher;
    // NodeHandle instance must be created before this line
    actionlib::SimpleActionServer<ur5lego::GripperAction> action_server_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    ur5lego::GripperFeedback feedback_;
    ur5lego::GripperResult result_;
    
    pinocchio::Model model_;    // the model of the robot


public:
    GripperAction(std::string name) : action_server_(server_node, name, boost::bind(&GripperAction::executeCB, this, _1), false), action_name_(name)
    {
        publisher = talker_node.advertise<std_msgs::Float64>(PUBLISHING_CHANNEL, 10);
        q = 0;

        action_server_.start();
    }

    ~GripperAction(void)
    {
    }

    void executeCB(const ur5lego::GripperGoalConstPtr &goal){
        ROS_INFO_STREAM("Received goal: " << coordsToStr(goal->finger) << " to do in " << goal->time << "s");
        // computeAndSendTrajectory(q, goal->finger, goal->time, 200, publisher);
        // publish goal->finger to /gripper_joint_position
        std_msgs::Float64 command;
        command.data = goal->finger;
        publisher.publish(command);
        q = goal->finger;
        action_server_.setSucceeded(result_);
    }    

private: 
    /// @brief converts the coordinates to a string
    std::string coordsToStr(_Float64 X){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "(" <<  X << ")";
        return ss.str();
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_server");
    GripperAction gripperAction("gripper_server");

    ROS_DEBUG_STREAM("Gripper server ready.");

    ros::spin();

    return 0;
}
