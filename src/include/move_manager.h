#ifndef MOVE_MANAGER_H
#define MOVE_MANAGER_H

#include <ros/ros.h> 
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ur5lego/MoveAction.h>
#include <ur5lego/GripperAction.h>
#include "ur5lego/Pose.h"
#include "Eigen/Dense"
#include "queue"

class MoveManager {
    public:
    actionlib::SimpleActionClient<ur5lego::MoveAction> * move_client;
    actionlib::SimpleActionClient<ur5lego::GripperAction> * gripper_client;
    ros::Subscriber test_position;
   
    int height;
    _Float32 d = 0.15;
    ur5lego::Pose fixed_pos;
    ur5lego::Pose homing;

    enum Lego{
        X1_Y1_Z2,
        X1_Y2_Z1,
        X1_Y2_Z2,
        X1_Y2_Z2_CHAMFER,
        X1_Y2_Z2_TWINFILLET,
        X1_Y3_Z2,
        X1_Y3_Z2_FILLET,
        X1_Y4_Z1,
        X1_Y4_Z2,
        X2_Y2_Z2,
        X2_Y2_Z2_FILLET,
    };

    ur5lego::Pose * position_list;

    MoveManager();
    ur5lego::Pose positionConverter(ur5lego::Pose::ConstPtr msg);
    void goalSetter(_Float32 X, _Float32 Y, _Float32 Z, _Float64 r, _Float64 p, _Float64 y, ur5lego::MoveGoal & goal);
    void goalSetter(ur5lego::Pose msg, ur5lego::MoveGoal & goal);
    void goalSender(ur5lego::MoveGoal & goal);
    void grab(ur5lego::GripperGoal goal, bool grab);
    void actionPlanner(std::queue<ur5lego::Pose::ConstPtr> &pos_msgs);
    
};

#endif