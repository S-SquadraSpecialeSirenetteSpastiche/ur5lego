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
    const float FREQ = 500;
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

        // load cache object
        if(cache_enabled){
            std::vector<std::string> cache_paths;
            cache_paths.push_back(ros::package::getPath("ur5lego") + "/data/ik_cache.txt");
            cache_paths.push_back(ros::package::getPath("ur5lego") + "/data/ik_cache_extra.txt");
            cache = parse_cache(cache_paths);
            if (cache.empty()) {
                ROS_WARN("Error while parsing cache file, continuing without");
                cache_enabled = false;
            }
        }

        pinocchio::Data data(model_);

        action_server_.start();
    }

    ~MoveAction(void)
    {
    }

    Eigen::Vector3f check_params(const ur5lego::MoveGoalConstPtr &g){
        Eigen::Vector3f res;
        res << g->X, g->Y, g->Z;
        if(res[1] < -0.1){
            res[1] = -0.1;
        }
        if(res[2]>0.62){
            res[2] = 0.62;
        }
        else if(res[2]<0.2){
            res[2] = 0.2;
        }
        return res;
    }
    /// @brief callback for the action server, this is the starting point of each motion
    /// @param g the goal sent by the client
    void executeCB(const ur5lego::MoveGoalConstPtr &g){
        ROS_INFO_STREAM("Received goal: " << coordsToStr(g->X, g->Y, g->Z, g->r, g->p, g->y) << " to do in " << g->time << "s");
        float time = g->time;   // must do this because g is read only and I might want to change it later

        // compute the inverse kinematics
        std::pair<Eigen::VectorXd, bool> res;
        Eigen::Vector3f checked_params = check_params(g);
        if (cache_enabled) {
            res = inverse_kinematics(model_, Eigen::Vector3d(checked_params[0], checked_params[1], checked_params[2]), Eigen::Vector3d(g->r, g->p, g->y), cache);
        } else {
            res = inverse_kinematics_without_cache(model_, Eigen::Vector3d(checked_params[0], checked_params[1], checked_params[2]), Eigen::Vector3d(g->r, g->p, g->y), q);
        }

        if(res.second){
            Eigen::VectorXd q1 = res.first;
            ROS_INFO_STREAM("Inverse kinematics succeded, q: " << q1.transpose());
            if(center_singularity(curr_pos(0), curr_pos(1), checked_params[0], checked_params[1])){
                ROS_INFO_STREAM("Motion through the center singularity detected");
                Eigen::VectorXd q01 = q;
                q01[0] = get_intermediate_shoulder_pan(q[0], q1[0]);
                compute_and_send_trajectory(q, q01, time/2.0, FREQ, publisher);
                q = q01;
                time /= 2.0; // we have used the first half of the time to rotate the shoulder pan, so we halve the time to complete the motion
                sleep(0.5);  // sleep for a short time so that the real robot can compensate the inertia
            }
            compute_and_send_trajectory(q, q1, time, FREQ, publisher);
            q = q1;
            curr_pos = Eigen::Vector3d(checked_params[0], checked_params[1], checked_params[2]);
            curr_rot = Eigen::Vector3d(g->r, g->p, g->y);
            // save_position();
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

    /// @brief computes the value of the shoulder pan to rotate in order to avoid the singularity in the center
    /// @param start the current value of the shoulder pan
    /// @param end the value of the shoulder pan we want to reach
    /// @return the value of the shoulder pan to rotate in order to avoid the singularity in the center
    /// and to get as close as possible to the end value
    double get_intermediate_shoulder_pan(double start, double end){
        double diff = end - start;
        if (diff > M_PI) diff -= 2 * M_PI;
        else if (diff < -M_PI) diff += 2 * M_PI;

        // if the difference is less than PI we can go there directly, otherwise we do a motion as big as possible
        if (abs(diff) < M_PI) {
            return end;
        } else {
            return start + M_PI + std::numeric_limits<double>::epsilon() * (diff > 0) ? -1 : 1;
        }
    }

    /// @brief saves the current position and orientation in the cache file and the current cache object
    void save_position(){
        Point3D pos = {curr_pos(0), curr_pos(1), curr_pos(2)};
        cache.push_back(std::make_pair(pos, q));

        std::string cache_file = ros::package::getPath("ur5lego") + "/data/ik_cache_extra.txt";
        std::ofstream cache_stream(cache_file, std::ios::out | std::ios::app);
        if (!cache_stream.is_open()) {
            ROS_WARN_STREAM("Error while opening cache file " << cache_file);
        }
        cache_stream << curr_pos(0) << " " << curr_pos(1) << " " << curr_pos(2) << " " << q.transpose() << std::endl;
        cache_stream.close();
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
