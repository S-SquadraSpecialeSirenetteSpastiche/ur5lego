#include "ros/ros.h"
#include "Lego.h"
#include "include/move_manager.h"
#include "queue"
#include "ur5lego/Pose.h"

using namespace ros;

std::queue<ur5lego::Pose::ConstPtr> pos_msgs;

void positionCallback(const ur5lego::Pose::ConstPtr & msg){
    ROS_INFO("I received the message!");
    pos_msgs.push(msg);
}

int main(int argc, char **argv){

    init(argc, argv, "test_move_sub");
    MoveManager mg;
    NodeHandle nh;

    Subscriber test_position = nh.subscribe("lego_position", 1000, positionCallback);

    while(ok()){
    mg.actionPlanner(pos_msgs);

    ros::spinOnce();
    }

    //exit
    return 0;
}