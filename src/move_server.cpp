#include "include/inverse_kinematics.h"
#include "pinocchio/parsers/urdf.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <ur5lego/MoveAction.h>
#include <string>
#include <iostream>


std::string coordsToStr(float a, float b, float c, float d, float e, float f){
    std::string s;
    s = "(" + std::to_string(a) + "," + std::to_string(b) + "," + std::to_string(c) + "), (" + 
        std::to_string(d) + "," + std::to_string(e) + "," + std::to_string(f) + ")";
    return s;
}


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
    
    pinocchio::Model model_;
    Eigen::VectorXd q;  // current configuration

public:
    MoveAction(std::string name) : action_server_(server_node, name, boost::bind(&MoveAction::executeCB, this, _1), false), action_name_(name)
    {
        publisher = talker_node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);

        const std::string urdf_file = ros::package::getPath("ur5lego") + std::string("/robot_description/ur5.urdf");
        pinocchio::urdf::buildModel(urdf_file, model_);
        // homing position
        q = Eigen::VectorXd(6);
        q << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
        // send_joint_positions();

        action_server_.start();
    }

    ~MoveAction(void)
    {
    }

    void send_joint_positions(){
        std_msgs::Float64MultiArray command;
        command.data.resize(6);
        for(int i=0; i<6; i++)
            command.data[i] = (float)q[i];
        publisher.publish(command);
    }

    void executeCB(const ur5lego::MoveGoalConstPtr &goal){
        ROS_INFO_STREAM("target: " << coordsToStr(goal->X, goal->Y, goal->Z, goal->r, goal->p, goal->y));
        std::pair<Eigen::VectorXd, bool> res = inverse_kinematics_bad(
            model_, Eigen::Vector3d(goal->X, goal->Y, goal->Z), Eigen::Vector3d(goal->r, goal->p, goal->y), q);

        if(res.second){
            ROS_INFO_STREAM("Convergence achieved!");
            q = res.first;
            send_joint_positions();
        }
        else{
            ROS_INFO_STREAM("Warning: the iterative algorithm has not reached convergence to the desired precision");
        }

        result_.success = res.second;
        action_server_.setSucceeded(result_);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_server");
    MoveAction moveAction("move_server");
    ROS_INFO_STREAM("Server ready");

    /*
    pinocchio::Model model;
    const std::string urdf_file = ros::package::getPath("ur5lego") + std::string("/robot_description/ur5.urdf");
    pinocchio::urdf::buildModel(urdf_file, model);

    Eigen::VectorXd q, q_ik, q_ik_bad;
    std::pair<Eigen::VectorXd, bool> ik_res, ik_bad_res;

    double k = 0.05;
    double X=-0.3, Y=-0.3, Z=0.6, r=0, p=0, y=0;
    std::vector<double> p0 = {X,    Y,      Z,      r, p, y};
    std::vector<double> p1 = {X+k,  Y,      Z,      r, p, y};
    std::vector<double> p2 = {X,    Y+k,    Z,      r, p, y};
    std::vector<double> p3 = {X,    Y,      Z+k,    r, p, y};
    std::vector<double> p4 = {X+k,  Y+k,    Z,      r, p, y};
    std::vector<double> p5 = {X+k,  Y,      Z+k,    r, p, y};
    std::vector<double> p6 = {X,    Y+k,    Z+k,    r, p, y};
    std::vector<double> p7 = {X+k,  Y+k, k, Z+k,    r, p, y};
    std::vector<std::vector<double>> destinations = {p0, p1, p2, p3, p4, p5, p6, p7};

    q = Eigen::VectorXd(6);
    q << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;

    int i=0;
    for(std::vector<double> p : destinations){
        ik_res = inverse_kinematics(model, Eigen::Vector3d(p[0], p[1], p[2]), Eigen::Vector3d(p[3], p[4], p[5]), q);
        ik_bad_res = inverse_kinematics_bad(model, Eigen::Vector3d(p[0], p[1], p[2]), Eigen::Vector3d(p[3], p[4], p[5]), q);

        if(ik_res.second && ik_bad_res.second){
            ROS_INFO_STREAM("Test p" << i << " solutions: ");
            ROS_INFO_STREAM(ik_res.first[0] << " " << ik_res.first[1] << " " << ik_res.first[2] << " " << ik_res.first[3] 
                << " " << ik_res.first[4] << " " << ik_res.first[5] << " ");
            ROS_INFO_STREAM(ik_bad_res.first[0] << " " << ik_bad_res.first[1] << " " << ik_bad_res.first[2] << " " << ik_bad_res.first[3] 
                << " " << ik_bad_res.first[4] << " " << ik_bad_res.first[5] << " ");
        } else {
            ROS_INFO_STREAM("Test p" << i << " failed: " << ik_res.second << " " << ik_bad_res.second);
        }
        i++;
    }
    */

    ros::spin();

    return 0;
}
