#include "include/inverse_kinematics.h"
#include "include/trajectory_planner.h"

#include "pinocchio/parsers/urdf.hpp"
#include "std_msgs/Float64MultiArray.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <ur5lego/MoveAction.h>
#include <string>
#include <iostream>


class MoveAction
{
protected:
    // node that acts as action server
    ros::NodeHandle server_node;
    // node that sends commands to the robot
    ros::NodeHandle talker_node;
    ros::Publisher publisher;
    // NodeHandle instance must be created before this line
    actionlib::SimpleActionServer<ur5lego::MoveAction> action_server_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    ur5lego::MoveFeedback feedback_;
    ur5lego::MoveResult result_;
    
    pinocchio::Model model_;    // the model of the robot
    Eigen::VectorXd q;  // current configuration

public:
    MoveAction(std::string name) : action_server_(server_node, name, boost::bind(&MoveAction::executeCB, this, _1), false), action_name_(name)
    {
        publisher = talker_node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);

        const std::string urdf_file = ros::package::getPath("ur5lego") + std::string("/robot_description/ur5.urdf");
        pinocchio::urdf::buildModel(urdf_file, model_);
        q = Eigen::VectorXd(6);
        q << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;   // homing position

        action_server_.start();
    }

    ~MoveAction(void)
    {
    }

    void executeCB(const ur5lego::MoveGoalConstPtr &goal){
        // round the numbers published to the 2nd decimal place
        ROS_INFO_STREAM("Received goal: " << coordsToStr(goal->X, goal->Y, goal->Z, goal->r, goal->p, goal->y));

        std::pair<Eigen::VectorXd, bool> res = inverse_kinematics_wrapper(
            model_, Eigen::Vector3d(goal->X, goal->Y, goal->Z), Eigen::Vector3d(goal->r, goal->p, goal->y), q);

        if(res.second){
            computeAndSendTrajectory(q, res.first, 3.0, 1000, publisher);
            q = res.first;
        }

        result_.success = res.second;
        action_server_.setSucceeded(result_);
    }

private: 
    std::string coordsToStr(float X, float Y, float Z, float r, float p, float y){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "(" <<  X << ", " << Y << ", " << Z << ") (" << r << ", " << p << ", " << y << ")";
        return ss.str();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_server");

    MoveAction moveAction("move_server");
    ROS_INFO_STREAM("Server ready");

    ros::spin();

    return 0;
}
