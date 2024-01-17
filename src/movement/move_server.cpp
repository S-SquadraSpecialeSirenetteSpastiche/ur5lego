#include "../include/inverse_kinematics.h"
#include "../include/joint_trajectory_planner.h"
#include "../include/cache_handler.h"

#include "pinocchio/parsers/urdf.hpp"
#include "std_msgs/Float64MultiArray.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <ur5lego/MoveAction.h>
#include <string>
#include <cstdlib>



/// @brief action server that moves the robot
class MoveAction
{
protected:
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
    Eigen::VectorXd q;  // current configuration of arms    
    
    pinocchio::Model model_;    // the model of the robot

    Cache cache;
    bool cache_enabled = true;

    const float DT = 100;

public:
    MoveAction(std::string name) : action_server_(server_node, name, boost::bind(&MoveAction::executeCB, this, _1), false), action_name_(name)
    {
        publisher = talker_node.advertise<std_msgs::Float64MultiArray>(PUBLISHING_CHANNEL, 10);

        // TODO: decide wether it makes more sense to use /robot_urdf/generated_urdf/ur5.urdf or /robot_urdf/ur5.urdf
        // they should be the same but the first one is generated from the xacro every time by ur5generic, the other one is static
        std::string urdf_file = std::string(std::getenv("LOCOSIM_DIR")) + "/robot_urdf/ur5.urdf";

        pinocchio::urdf::buildModel(urdf_file, model_);
        q = Eigen::VectorXd(6);
        q << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;   // homing position

        cache = parse_cache(ros::package::getPath("ur5lego") + "/data/ik_cache.txt");
        if (cache.empty()) {
            ROS_WARN("Error while parsing cache file, continuing without");
            cache_enabled = false;
        }

        action_server_.start();
    }

    ~MoveAction(void)
    {
    }

    /// @brief callback for the action server
    /// @param g the goal sent by the client
    void executeCB(const ur5lego::MoveGoalConstPtr &g){
        ROS_INFO_STREAM("Received goal: " << coordsToStr(g->X, g->Y, g->Z, g->r, g->p, g->y) << " to do in " << g->time << "s");

        std::pair<Eigen::VectorXd, bool> res;
        if (cache_enabled) {
            res = inverse_kinematics(model_, Eigen::Vector3d(g->X, g->Y, g->Z), Eigen::Vector3d(g->r, g->p, g->y), cache);
        } else {
            res = inverse_kinematics_without_cache(model_, Eigen::Vector3d(g->X, g->Y, g->Z), Eigen::Vector3d(g->r, g->p, g->y), q);
        }

        if(res.second){
            ROS_DEBUG_STREAM("Inverse kinematics succeded, q: " << res.first.transpose());
            compute_and_send_trajectory(q, res.first, g->time, DT, publisher);
            q = res.first;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_server");

    MoveAction moveAction("move_server");

    ROS_DEBUG_STREAM("Move server ready.");

    ros::spin();

    return 0;
}
