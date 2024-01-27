#include "../include/inverse_kinematics.h"
#include "../include/joint_trajectory_planner.h"
#include "../include/cartesian_trajectory_planner.h"
#include "../include/cache_handler.h"

#include "pinocchio/parsers/urdf.hpp"
#include "std_msgs/Float64MultiArray.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <ur5lego/MoveAction.h>
#include <string>
#include <cstdlib>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>


enum ControlType{
    JOINT_SPACE,
    CARTESIAN_SPACE,
};


/// @brief action server that moves the robot
class MoveAction
{
protected:
    // const ControlType CONTROL_TYPE = CARTESIAN_SPACE;
    const std::string PUBLISHING_CHANNEL = "/arm_joint_position";
    const float FREQ = 200;     // frequency of the points of the trajectory

    Eigen::VectorXd q;          // current configuration of the arm
    Eigen::Vector3d ee_pos;     // current position of the end effector
    Eigen::Vector3d ee_rot;     // current rotation of the end effector

    pinocchio::Model model_;    // the model of the robot

    Cache cache;                // cache object used to speed up the inverse kinematics
    bool cache_enabled = true;  // if false the cache is not used
    
    ros::NodeHandle server_node;    // node that acts as action server
    ros::NodeHandle talker_node;    // node used to send the joints
    ros::Publisher publisher;       // publisher used to send the joints
    
    // action server stuff
    actionlib::SimpleActionServer<ur5lego::MoveAction> action_server_;
    std::string action_name_;
    ur5lego::MoveFeedback feedback_;
    ur5lego::MoveResult result_;


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

        // TODO: put here the actual position and orientation of the end effector
        ee_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
        ee_rot = Eigen::Vector3d(0.0, 0.0, 0.0);

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

        // if we have to move from one side of the table to the other, use the arc trajectory, 
        // otherwise use the one over the joint space
        ControlType control_type = this->getControlType(ee_pos[0], ee_pos[1], g->X, g->Y);
        ROS_INFO_STREAM("Control type: " << control_type);

        std::pair<Eigen::VectorXd, bool> res;
        if (control_type == JOINT_SPACE) {
            if (cache_enabled) {
                res = inverse_kinematics_cache(model_, Eigen::Vector3d(g->X, g->Y, g->Z), Eigen::Vector3d(g->r, g->p, g->y), cache);
            } else {
                res = inverse_kinematics_interpolate(model_, Eigen::Vector3d(g->X, g->Y, g->Z), Eigen::Vector3d(g->r, g->p, g->y), q);
            }
        } else if (control_type == CARTESIAN_SPACE) {
            // build ti from ee_pos and ee_rot
            Eigen::Isometry3d ti = Eigen::Isometry3d::Identity();
            ti.translation() = ee_pos;
            ti.linear() = (Eigen::AngleAxisd(ee_rot[2], Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(ee_rot[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(ee_rot[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
            Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
            tf.translation() = Eigen::Vector3d(g->X, g->Y, g->Z);
            tf.linear() = (Eigen::AngleAxisd(g->y, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(g->p, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(g->r, Eigen::Vector3d::UnitX())).toRotationMatrix();
            res = compute_and_send_arc_trajectory(model_, q, ti, tf, g->time, 0.2, publisher);
        } else {
            ROS_ERROR("Unimplemented");
            return;
        }
        
        if(res.second){
            ROS_INFO_STREAM("Inverse kinematics succeded, q: " << res.first.transpose());
            compute_and_send_trajectory(q, res.first, g->time, FREQ, publisher);
            // update the current configuration
            this->q = res.first;
            this->ee_pos = Eigen::Vector3d(g->X, g->Y, g->Z);
            this->ee_rot = Eigen::Vector3d(g->r, g->p, g->y);
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

    /// @brief returns the control type to use to avoid the singularity in the center of the workspace
    ControlType getControlType(float currX, float currY, float desX, float desY){
        // we have to avoid the singularity if we move to the other side of the table: currX*desX<0
        // and the y coordinate of the middle point is close to the robot: (currY+desY)/2.0 < 0.4
        return (currX*desX < 0 && (currY+desY)/2.0 < 0.4) ? CARTESIAN_SPACE : JOINT_SPACE;
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
