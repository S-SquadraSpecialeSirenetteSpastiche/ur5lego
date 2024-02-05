#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Empty.h>
#include <ur5lego/MoveAction.h>
#include <iomanip>
#include <cstdlib>


int main(int argc, char** argv) {
    ros::init(argc, argv, "generate_cache_client");

    // Create the action client
    actionlib::SimpleActionClient<ur5lego::MoveAction> ac("move_server", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    ur5lego::MoveGoal goal;
    goal.r = 0;
    goal.p = -M_PI/2;
    goal.y = M_PI/2;
    goal.time = 1.5;
    
    // send random goals to the server z[0.3, 0.6], x[-0.5, 0.5], y[-0.1, 0.5]
    while(true){
        goal.X = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 0.5;
        goal.Y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 0.6 - 0.1;
        goal.Z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 0.3 + 0.3;

        // Round the random numbers to 3 decimal places
        goal.X = std::round(goal.X * 1000) / 1000;
        goal.Y = std::round(goal.Y * 1000) / 1000;
        goal.Z = std::round(goal.Z * 1000) / 1000;


        ac.sendGoal(goal);
    }
    

    return 0;
}