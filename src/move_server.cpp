#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ur5lego/MoveAction.h>
#include <string>
#include <iostream>

std::string getCoords(int a, int b, int c, int d, int e, int f){
    std::string s;
    s = "(" + std::to_string(a) + "," + std::to_string(b) + "," + std::to_string(c) + "), (" + 
        std::to_string(d) + "," + std::to_string(e) + "," + std::to_string(f) + ")";
    return s;
}

class MoveAction
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    actionlib::SimpleActionServer<ur5lego::MoveAction> action_server_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    ur5lego::MoveFeedback feedback_;
    ur5lego::MoveResult result_;

public:
    MoveAction(std::string name) : action_server_(nh_, name, boost::bind(&MoveAction::executeCB, this, _1), false), action_name_(name)
    {
        action_server_.start();
    }

    ~MoveAction(void)
    {
    }

    void executeCB(const ur5lego::MoveGoalConstPtr &goal)
    {
        ROS_INFO_STREAM("Executing... target: " << getCoords(goal->X, goal->Y, goal->Z, goal->r, goal->p, goal->y));
        result_.success = true;
        action_server_.setSucceeded(result_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_server");

    MoveAction moveAction("move_server");
    ros::spin();

    return 0;
}
