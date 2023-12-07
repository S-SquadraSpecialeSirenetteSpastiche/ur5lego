#include <ros/ros.h> 
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ur5lego/MoveAction.h>
#include <ur5lego/GripperAction.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Pose.h"
#include "ur5lego/Pose.h"
#include "queue"
#include "include/move_manager.h"

using namespace ros;
using namespace std;



MoveManager::MoveManager(){
    move_client = new actionlib::SimpleActionClient<ur5lego::MoveAction>("move_server", true);
    gripper_client = new actionlib::SimpleActionClient<ur5lego::GripperAction>("gripper_server", true);
    ROS_INFO("Waiting for move server to start.");
    move_client->waitForServer();
    ROS_INFO("Move server started.");
    ROS_INFO("Waiting for gripper server to start.");
    gripper_client->waitForServer();
    ROS_INFO("Gripper server started.");

    //NodeHandle nh;
    height = 1; //da sistemare;

    fixed_pos.position.x = (_Float32)(0.3); //da sistemare
    fixed_pos.position.y = (_Float32)(0.3); //da sistemare
    fixed_pos.position.z = (_Float32)(0.3); //da sistemare
    fixed_pos.orientation.x = (_Float64)(1.5707); //da sistemare
    fixed_pos.orientation.y = (_Float64)(-1.5707); //da sistemare
    fixed_pos.orientation.z = (_Float64)(0); //da sistemare*/

    homing.position.x = (_Float32)(-0.32); //da sistemare
    homing.position.y = (_Float32)(0.38); //da sistemare
    homing.position.z = (_Float32)(0.2); //da sistemare
    homing.orientation.x = (_Float64)(1.5707); //da sistemare
    homing.orientation.y = (_Float64)(-1.5707); //da sistemare
    homing.orientation.z = (_Float64)(0); //da sistemare*/
}   

void MoveManager::goalSetter(_Float32 X, _Float32 Y, _Float32 Z, _Float64 r, _Float64 p, _Float64 y, ur5lego::MoveGoal & goal){
    //save position
    goal.X = X;
    goal.Y = Y;
    goal.Z = Z;
    std::cout << X << Y << Z;
    //save orientation
    goal.r = r;
    goal.p = p;
    goal.y = y;
    goal.time = 3.0;
    ROS_INFO("Action:   Coordinates: %f, %f, %f", goal.X, goal.Y, goal.Z);

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
    goal.time = 3.0;
    ROS_INFO_STREAM("Y  nellla  funzione: "<<goal.Y);

    ROS_INFO_STREAM("Goal setted, ready to be sent\n");
}


void MoveManager::goalSender(ur5lego::MoveGoal & goal){
    move_client->sendGoal(goal);
    ROS_INFO("Goal sent");
    bool finished_before_timeout = move_client->waitForResult(ros::Duration(30.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = move_client->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        //exit(1);
    }
}

void MoveManager::grab(ur5lego::GripperGoal goal, bool grab){
    if(grab){
        goal.finger = 2.0;
    }else{
        goal.finger = 0.0;
    }
        goal.time = 3.0;
    ROS_INFO_STREAM("Gripper goal setted, ready to be sent.");
    gripper_client->sendGoal(goal);
}


void MoveManager::actionPlanner(queue<ur5lego::Pose::ConstPtr> &pos_msgs){

    //start action
    if(!pos_msgs.empty()){
        ur5lego::Pose::ConstPtr msg = pos_msgs.front();
        ur5lego::MoveGoal goal;
        ur5lego::GripperGoal hand;
        
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
        goalSetter(X,Y,Z-d,r,p,y, goal);
        goalSender(goal);
        
        //descend and grab the object
        goalSetter(X,Y,Z,r,p,y, goal);
        goalSender(goal);
        //grab(true) -> is an action! not implemented yet
        grab(hand, true);

        
        //lift the brick
        goalSetter(X,Y,Z-d,r,p,y, goal);
        goalSender(goal);
        //TODO: GRAB
        
        //move the brick to fixed_pos
        goalSetter(fixed_pos, goal);
        goalSender(goal);
        
        //lower the brick -> da sistemare
        goalSetter(fixed_pos.position.x, fixed_pos.position.y, fixed_pos.position.z+d, fixed_pos.orientation.x, fixed_pos.orientation.y, fixed_pos.orientation.z, goal);
        goalSender(goal);

        //return to homing position
        goalSetter(homing, goal);
        goalSender(goal);
        //
        //fixed_pos.position.y = fixed_pos.position.y-0.1;
        grab(hand, false);

        
        pos_msgs.pop();
    }
}  
