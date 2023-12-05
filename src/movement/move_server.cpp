#include "../include/inverse_kinematics.h"
#include "../include/trajectory_planner.h"

#include "pinocchio/parsers/urdf.hpp"
#include "std_msgs/Float64MultiArray.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <ur5lego/MoveAction.h>
#include <ur5lego/GripperAction.h>
#include <string>


Eigen::VectorXd q;  // current configuration of arms 
Eigen::VectorXd q_gripper; //current configuration of gripper
Eigen::VectorXd q_curr; //current configuration of arms and gripper
bool gripper_open = true;

/// @brief action server that moves the robot
class MoveAction
{
protected:
    const std::string PACKAGE_NAME = "ur5lego";
    const std::string UR_DESCRIPTION = "/robot_description/ur5.urdf";
    const std::string PUBLISHING_CHANNEL = "/arm_joint_position";

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

public:
    MoveAction(std::string name) : action_server_(server_node, name, boost::bind(&MoveAction::executeCB, this, _1), false), action_name_(name)
    {
        publisher = talker_node.advertise<std_msgs::Float64MultiArray>(PUBLISHING_CHANNEL, 10);

        const std::string urdf_file = ros::package::getPath(PACKAGE_NAME) + std::string(UR_DESCRIPTION);
        pinocchio::urdf::buildModel(urdf_file, model_);
        q = Eigen::VectorXd(6);
        q_curr = Eigen::VectorXd(9);
        q_gripper = Eigen::VectorXd::Zero(3);
        q << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;   // homing position
        q_curr << q, q_gripper;

        action_server_.start();
    }

    ~MoveAction(void)
    {
    }

    /// @brief callback for the action server
    /// @param goal the goal sent by the client
    void executeCB(const ur5lego::MoveGoalConstPtr &goal){
        ROS_INFO_STREAM("Received goal: " << 
            coordsToStr(goal->X, goal->Y, goal->Z, goal->r, goal->p, goal->y) << " to do in " << goal->time << "s");

        std::pair<Eigen::VectorXd, bool> res = inverse_kinematics(
            model_, Eigen::Vector3d(goal->X, goal->Y, goal->Z), Eigen::Vector3d(goal->r, goal->p, goal->y), q);
        
        Eigen::VectorXd q_def = Eigen::VectorXd(9);
        if(res.second){
            if(gripper_open){
                q_gripper[0] = 1.0;
                q_gripper[1] = 1.0;
                q_gripper[2] = 1.0;
            }else{
                q_gripper[0] = 0.0;
                q_gripper[1] = 0.0;
                q_gripper[2] = 0.0;
            }
            q_def << res.first, q_gripper;
            computeAndSendTrajectory(q_curr, q_def, goal->time, 200, publisher);
            q = res.first;
            ROS_INFO("Inverse kinematics succeded");
        } else {
            ROS_WARN("Inverse kinematics failed");
        }

        result_.success = res.second;
        action_server_.setSucceeded(result_);
    }    

private: 
    /// @brief converts the coordinates to a string
    std::string coordsToStr(float X, float Y, float Z, float r, float p, float y){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "(" <<  X << ", " << Y << ", " << Z << ") (" << r << ", " << p << ", " << y << ")";
        return ss.str();
    } 
};

class GripperAction
{
protected:
    const std::string PUBLISHING_CHANNEL = "/ur5/joint_group_pos_controller/command";

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
        publisher = talker_node.advertise<std_msgs::Float64MultiArray>(PUBLISHING_CHANNEL, 10);
        // q = Eigen::VectorXd(9);

        action_server_.start();
    }

    ~GripperAction(void)
    {
    }
    //TODO: not clear how to implememtn it, for now it's only a copy of first server 
    /// @brief callback for the action server
    /// @param goal the goal sent by the client
    void executeCB(const ur5lego::GripperGoalConstPtr &goal){
        ROS_INFO_STREAM("Received goal: " << coordsToStr(goal->finger) << " to do in " << goal->time << "s");

        Eigen::VectorXd q_temp = Eigen::VectorXd(9);
        q_temp << q;
        q_temp << goal->finger;
        q_temp << goal->finger;
        q_temp << goal->finger;
        if(goal->finger != 0.0){
            gripper_open = false;
        }
        else{
            gripper_open = true;
        }
            
        computeAndSendTrajectory(q_curr , q_temp, goal->time, 200, publisher);
        for(int i = 6; i < 9; i ++){
            q_curr[i] = q_temp[i];
        }

        action_server_.setSucceeded(result_);
    }    

private: 
    /// @brief converts the coordinates to a string
    std::string coordsToStr(float X){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "(" <<  X << ")";
        return ss.str();
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_server");

    MoveAction moveAction("move_server");
    // GripperAction gripperAction("gripper_server");

    ROS_INFO_STREAM("Move server ready.");
    ROS_INFO_STREAM("Gripper server ready.");

    ros::spin();

    return 0;
}
