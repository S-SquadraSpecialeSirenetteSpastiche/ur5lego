#include "../include/inverse_kinematics.h"
#include "../include/trajectory_planner.h"
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
    const float DT = 500;
    // if Y start or Y end are less than this and we want to move to the other side of the table
    // we need to take measures to avoid the singularity in the center
    const float MIN_Y_FOR_NORMAL_TRAJECTORY = 0.2;

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
    Eigen::Vector3d curr_pos;
    Eigen::Vector3d curr_rot;

    Cache cache;
    bool cache_enabled = true;


public:
    MoveAction(std::string name) : action_server_(server_node, name, boost::bind(&MoveAction::executeCB, this, _1), false), action_name_(name)
    {
        publisher = talker_node.advertise<std_msgs::Float64MultiArray>(PUBLISHING_CHANNEL, 10);

        std::string urdf_file = std::string(std::getenv("LOCOSIM_DIR")) + "/robot_urdf/ur5.urdf";

        pinocchio::urdf::buildModel(urdf_file, model_);
        q = Eigen::VectorXd(6);
        q << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;   // homing position
        // position and orientation of the homing position, used only to avoid the center singularity
        curr_pos = Eigen::Vector3d(-0.150661, 0.164984, 0.366207);
        curr_rot = Eigen::Vector3d(0.35143, -1.31318, 2.81847);

        generate_cache();

        cache = parse_cache(ros::package::getPath("ur5lego") + "/data/ik_cache.txt");
        if (cache.empty()) {
            ROS_WARN("Error while parsing cache file, continuing without");
            cache_enabled = false;
        }

        pinocchio::Data data(model_);

        action_server_.start();
    }

    ~MoveAction(void)
    {
    }

    /// @brief callback for the action server
    /// @param g the goal sent by the client
    void executeCB(const ur5lego::MoveGoalConstPtr &g){
        ROS_INFO_STREAM("Received goal: " << coordsToStr(g->X, g->Y, g->Z, g->r, g->p, g->y) << " to do in " << g->time << "s");
        float time = g->time;   // must do this because g is read only and I might want to change it later

        // compute the inverse kinematics
        std::pair<Eigen::VectorXd, bool> res;
        if (cache_enabled) {
            res = inverse_kinematics(model_, Eigen::Vector3d(g->X, g->Y, g->Z), Eigen::Vector3d(g->r, g->p, g->y), cache);
        } else {
            res = inverse_kinematics_without_cache(model_, Eigen::Vector3d(g->X, g->Y, g->Z), Eigen::Vector3d(g->r, g->p, g->y), q);
        }

        if(res.second){
            Eigen::VectorXd q1 = res.first;
            ROS_INFO_STREAM("Inverse kinematics succeded, q: " << q1.transpose());
            if(center_singularity(curr_pos(0), curr_pos(1), g->X, g->Y)){
                ROS_INFO_STREAM("Motion through the center singularity detected");
                Eigen::VectorXd q01 = q;    // joints for the intermediat motion to avoid center singularity
                // quick hack to make sure the rotation is always forwards
                // if the rotation is really big we just move halfway and figure it out later
                // TODO: we can rotate a max of PI - std::numeric_limits<float>::epsilon() radiants at a time without problems
                if(abs(q1[0] - q[0]) < M_PI){
                    q01[0] = q1[0];
                } else {
                    q01[0] = (q1[0]+q01[0])/2.0;
                }
                compute_and_send_trajectory(q, q01, time/2.0, DT, publisher);
                q = q01;
                time /= 2.0; // we have used the first half of the time to rotate the shoulder pan, so we halve the time to complete the motion
                sleep(0.5);  // sleep for a short time so that the real robot can compensate the inertia
            }
            compute_and_send_trajectory(q, q1, time, DT, publisher);
            q = q1;
            curr_pos = Eigen::Vector3d(g->X, g->Y, g->Z);
            curr_rot = Eigen::Vector3d(g->r, g->p, g->y);
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

    /// @brief checks if the trajectory goes through the center singularity
    bool center_singularity(float currX, float currY, float desX, float desY){
        // we go trough the singularity if we move to the other side of the table: currX*desX<0
        // and the y coordinate of the middle point is close to the robot: (currY+desY)/2.0 < 0.4
        return (currX*desX < 0 && currY < MIN_Y_FOR_NORMAL_TRAJECTORY && desY < MIN_Y_FOR_NORMAL_TRAJECTORY);
    }

    /// @brief generates the cache file, used to speed up the inverse kinematics
    void generate_cache(){
        std::string cache_file = ros::package::getPath("ur5lego") + "/data/ik_cache.txt";
        std::ofstream cache_stream(cache_file, std::ios::out | std::ios::trunc);
        if (!cache_stream.is_open()) {
            ROS_WARN_STREAM("Error while opening cache file " << cache_file);
        }
        for(float z = 0.4; z <= 0.6; z += 0.1){
            for(float y = 0.2; y <= 0.5; y += 0.1){
                for(float x = -0.5; x <= 0.5; x += 0.1){
                    if(x*x+y*x < 0.3*0.3) continue;    // skip the points too close to the center of the robot
                    std::pair<Eigen::VectorXd, bool> res = inverse_kinematics_without_cache(model_, Eigen::Vector3d(x, y, z), Eigen::Vector3d(0, -M_PI/2, M_PI/2), q);
                    if(res.second){
                        cache_stream << x << " " << y << " " << z << " " << res.first.transpose() << std::endl;
                        q = res.first;
                    } else {
                        ROS_WARN_STREAM("Inverse kinematics failed for " << x << " " << y << " " << z);
                        cache_stream << x << " " << y << " " << z << " " << "NaN NaN NaN NaN NaN NaN" << std::endl;
                    }
                }
            }
        }
        cache_stream.close();
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
