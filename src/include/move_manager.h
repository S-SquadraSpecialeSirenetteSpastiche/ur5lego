#ifndef MOVE_MANAGER_H
#define MOVE_MANAGER_H

#include <ros/ros.h> 
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ur5lego/MoveAction.h>
#include "ur5lego/Pose.h"
#include "queue"

class MoveManager {
    public:
    actionlib::SimpleActionClient<ur5lego::MoveAction> * ac;
    ros::Subscriber test_position;
   
    int height;
    _Float32 d = 0.3;
    ur5lego::Pose fixed_pos;
    ur5lego::Pose homing;
    
    MoveManager();
    void goalSetter(_Float32 X, _Float32 Y, _Float32 Z, _Float64 r, _Float64 p, _Float64 y, ur5lego::MoveGoal & goal);
    void goalSetter(ur5lego::Pose msg, ur5lego::MoveGoal & goal);
    void goalSender(ur5lego::MoveGoal & goal);
    void actionPlanner(std::queue<ur5lego::Pose::ConstPtr> &pos_msgs);
};

#endif