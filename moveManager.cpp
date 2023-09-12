#include <ros/ros.h> 
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ur5lego/MoveAction.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Pose.h"
#include "ur5lego/Pose.h"
#include "queue"
#include "move_manager.h"

using namespace ros;
using namespace std;

MoveManager::MoveManager(){
    ac = new  actionlib::SimpleActionClient<ur5lego::MoveAction>("test_action", true);
    ROS_INFO("Waiting for action server to start.");
    ac->waitForServer();
    ROS_INFO("Action server started.");

    //NodeHandle nh;
    height = 50; //da sistemare;
    fixed_pos.position.x = 0.3; //da sistemare
    fixed_pos.position.y = height; //da sistemare
    fixed_pos.position.z = 0.3; //da sistemare
    fixed_pos.orientation.x = 3.14; //da sistemare
    fixed_pos.orientation.y = 3.14; //da sistemare
    fixed_pos.orientation.z = 3.14; //da sistemare*/
}   

void MoveManager::goalSetter(_Float32 X, _Float32 Y, _Float32 Z, _Float32 r, _Float32 p, _Float32 y, ur5lego::MoveGoal & goal){
    //save position
    goal.X = X;
    goal.Y = Y;
    goal.Z = Z;
    std::cout << X << Y << Z;
    //save orientation
    goal.r = r;
    goal.p = p;
    goal.y = y;
    ROS_INFO_STREAM("X  nellla  funzione: "<<goal.X);

    ROS_INFO_STREAM("Goal setted, ready to be sent\n");
}

void MoveManager::goalSetter(ur5lego::Pose msg, ur5lego::MoveGoal & goal){
    //save position
    goal.X = msg.position.x;
    goal.Y = msg.position.y;
    goal.Z = msg.position.z;
    //save orientation
    goal.r = msg.orientation.x;
    goal.p = msg.orientation.y;
    goal.y = msg.orientation.z;
    ROS_INFO_STREAM("X  nellla  funzione: "<<goal.X);

    ROS_INFO_STREAM("Goal setted, ready to be sent\n");
}


void MoveManager::goalSender(ur5lego::MoveGoal & goal){
    ac->sendGoal(goal);
    ROS_INFO("Goal sent");
    bool finished_before_timeout = ac->waitForResult(ros::Duration(30.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        exit(1);
    }
}


void MoveManager::actionPlanner(queue<ur5lego::Pose::ConstPtr> &pos_msgs){

    //start action
    if(!pos_msgs.empty()){
        ur5lego::Pose::ConstPtr msg = pos_msgs.front();
        ur5lego::MoveGoal goal;
        
        //save position
        _Float32 X = msg->position.x;
        _Float32 Y = msg->position.y;
        _Float32 Z = msg->position.z;
        std::cout << X << Y << Z;
        //save orientation
        _Float64 r = msg->orientation.x;
        _Float64 p = msg->orientation.y;
        _Float64 y = msg->orientation.z;

        ROS_INFO("dati ricevuti: X:%f ,Y:%f , Z:%f",X, Y, Z );
        
        //move above the object
        goalSetter(X,Y-d,Z,r,p,y, goal);
        goalSender(goal);
        
        //descend and grab the object
        goalSetter(X,Y,Z,r,p,y, goal);
        goalSender(goal);
        //grab(true) -> is an action! not implemented yet
        
        //lift the brick
        goalSetter(X,Y-d,Z,r,p,y, goal);
        goalSender(goal);
        //TODO: GRAB

        //move the brick to fixed_pos
        goalSetter(fixed_pos, goal);
        goalSender(goal);

        //lower the brick -> da sistemare
        goalSetter(fixed_pos, goal);
        goalSender(goal);
        fixed_pos.position.y = fixed_pos.position.y-0.1;
        //grab(false)

        pos_msgs.pop();
    }
}  
