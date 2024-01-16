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

    fixed_pos.position.x = (_Float32)(0.3); //da sistemare
    fixed_pos.position.y = (_Float32)(0.3); //da sistemare
    fixed_pos.position.z = (_Float32)(0.40); //da sistemare
    fixed_pos.orientation.x = (_Float64)(0);
    fixed_pos.orientation.y = (_Float64)(-1.57);
    fixed_pos.orientation.z = (_Float64)(1.57);

    homing.position.x = (_Float32)(-0.32); //da sistemare
    homing.position.y = (_Float32)(0.38); //da sistemare
    homing.position.z = (_Float32)(0.2); //da sistemare
    homing.orientation.x = (_Float64)(1.5707); //da sistemare
    homing.orientation.y = (_Float64)(-1.5707); //da sistemare
    homing.orientation.z = (_Float64)(0); //da sistemare*/

    ///positions where each type of lego should be moved to
    position_list = new ur5lego::Pose[11];
    position_list[X1_Y1_Z2] = fixed_pos;
    position_list[X1_Y2_Z1] = fixed_pos;
    position_list[X1_Y2_Z2] = fixed_pos;
    position_list[X1_Y2_Z2_CHAMFER] = fixed_pos;
    position_list[X1_Y2_Z2_TWINFILLET] = fixed_pos;
    position_list[X1_Y3_Z2] = fixed_pos;
    position_list[X1_Y3_Z2_FILLET] = fixed_pos;
    position_list[X1_Y4_Z1] = fixed_pos;
    position_list[X1_Y4_Z2] = fixed_pos;
    position_list[X2_Y2_Z2] = fixed_pos;
    position_list[X2_Y2_Z2_FILLET] = fixed_pos;

}   

/// @brief this function converts the coordinates of the brick from the camera frame to the robot frame
/// @param msg the message containing the coordinates of the brick in the camera frame
/// @return converted_msg the message containing the coordinates of the brick in the robot
ur5lego::Pose MoveManager::positionConverter(ur5lego::Pose::ConstPtr msg){
    ur5lego::Pose converted_msg;
    ROS_INFO("Converting Camera coordinates to Robot coordinates");
    Eigen::Vector3d T; //translation vector
    T << 0.5, 0.3, 0.6;
    Eigen::Matrix4d M; //roto-translation matrix
    M << 0, -1, 0, T[0],
            -1, 0, 0, T[1],
            0, 0, -1, T[2],
            0, 0, 0, 1;

    Eigen::Matrix3d R; //rotation matrix rpy
    R << 0, -1, 0,
            -1, 0, 0,
            0, 0, -1;
    Eigen::Vector4d camera_pov_coordinates;
    Eigen::Vector3d camera_pov_orientation; 
    Eigen::Vector4d robot_pov_coordinates;

    camera_pov_coordinates << msg->position.x, msg->position.y, msg->position.z, 1;
    camera_pov_orientation << msg->orientation.x, msg->orientation.y, msg->orientation.z;

    Eigen::Matrix3d roll_matrix;
    roll_matrix << 1, 0, 0,
                    0, cos(msg->orientation.x), -sin(msg->orientation.x),
                    0, sin(msg->orientation.x), cos(msg->orientation.x);

    Eigen::Matrix3d pitch_matrix;
    pitch_matrix << cos(msg->orientation.y), 0, sin(msg->orientation.y),
                    0, 1, 0,
                    -sin(msg->orientation.y), 0, cos(msg->orientation.y);
    Eigen::Matrix3d yaw_matrix;
    yaw_matrix << cos(msg->orientation.z), -sin(msg->orientation.z), 0,
                    sin(msg->orientation.z), cos(msg->orientation.z), 0,
                    0, 0, 1;
    Eigen::Matrix3d RPY = roll_matrix * pitch_matrix * yaw_matrix;

    robot_pov_coordinates = M * camera_pov_coordinates;
    Eigen::Matrix3d rotated_rpy = R * RPY;

    double roll = 0; //atan2(rotated_rpy(2,1), rotated_rpy(2,2));
    double pitch = -1.57; //atan2(-rotated_rpy(2,0), sqrt(pow(rotated_rpy(2,1),2) + pow(rotated_rpy(2,2),2)));
    double yaw = 1.57; //atan2(rotated_rpy(1,0), rotated_rpy(0,0));

    converted_msg.legoType = msg->legoType;
    converted_msg.position.x = (_Float32)(robot_pov_coordinates[0]);
    converted_msg.position.y = (_Float32)(robot_pov_coordinates[1]);
    converted_msg.position.z = (_Float32)(robot_pov_coordinates[2] - height);
    converted_msg.orientation.x = (_Float64)(roll);
    converted_msg.orientation.y = (_Float64)(pitch);
    converted_msg.orientation.z = (_Float64)(yaw);

    return converted_msg;
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
}
 
void MoveManager::goalSender(ur5lego::MoveGoal & goal){
    move_client->sendGoal(goal);
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
        goal.finger = 35.0;
    }else{
        goal.finger = 70.0;
    }
    goal.time = 1.0;
    ROS_INFO_STREAM("Gripper goal setted, ready to be sent.");
    gripper_client->sendGoal(goal);
}


void MoveManager::actionPlanner(queue<ur5lego::Pose::ConstPtr> &pos_msgs){

    //start action
    if(!pos_msgs.empty()){
        ur5lego::Pose msg = positionConverter(pos_msgs.front());
        ur5lego::MoveGoal goal;
        ur5lego::GripperGoal hand;
        
        Lego lego_type = static_cast<Lego>(msg.legoType);


        //save position
        _Float32 X = msg.position.x;
        _Float32 Y = msg.position.y;
        _Float32 Z = msg.position.z;
        std::cout << X << Y << Z;
        //save orientation
        _Float64 r = msg.orientation.x;
        _Float64 p = msg.orientation.y;
        _Float64 y = msg.orientation.z;

        ROS_INFO("Posizione blocco: X:%f ,Y:%f , Z:%f", X, Y, Z );
        
        /*
        the following block of code is used to move the robot to the position
        where the brick is located. The robot will move above the brick, then
        it will descend and grab the brick, then it will lift the brick and
        move it to the fixed position, then it will lower the brick and release
        it. Finally, it will return to the homing position.
        */
        //move above the object
        goalSetter(X,Y,Z-d,r,p,y, goal);
        goalSender(goal);
        
        //descend and grab the object
        goalSetter(X,Y,Z,r,p,y, goal);
        goalSender(goal);
        grab(hand, true);

        
        //lift the brick
        goalSetter(X,Y,Z-d,r,p,y, goal);
        goalSender(goal);
        
        //move the brick to fixed_pos
        goalSetter(position_list[lego_type], goal);
        goalSender(goal);
        
        //lower the brick
        goalSetter(position_list[lego_type].position.x,
                     position_list[lego_type].position.y, 
                     position_list[lego_type].position.z + d, //lower brick
                     position_list[lego_type].orientation.x, 
                     position_list[lego_type].orientation.y, 
                     position_list[lego_type].orientation.z, 
                     goal);
        goalSender(goal);
        grab(hand, false); //release the brick

        goalSetter(position_list[lego_type], goal);
        goalSender(goal);

        //return to homing position
        goalSetter(homing, goal);
        goalSender(goal);
        //end action

        
        pos_msgs.pop();
    }
}  
