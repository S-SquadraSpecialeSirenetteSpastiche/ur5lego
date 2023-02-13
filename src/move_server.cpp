#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ur5lego/MoveAction.h>

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
        ROS_INFO("Executing... target:(%i,%i,%i), (%i,%i,%i)", goal->X, goal->Y, goal->Z, goal->r, goal->p, goal->y);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fibonacci");

    MoveAction fibonacci("fibonacci");
    ros::spin();

    return 0;
}
