#include "ros/ros.h"
#include "../include/move_manager.h"
#include "queue"
#include "ur5lego/Pose.h"
#include "ur5lego/BlockPosition.h"

using namespace ros;

std::queue<ur5lego::Pose> pos_msgs;

int main(int argc, char **argv){

    init(argc, argv, "listener");
    MoveManager mg;
    NodeHandle nh;

    ServiceClient client = nh.serviceClient<ur5lego::BlockPosition>("get_position");

    ROS_DEBUG_STREAM("controller ready");
    ur5lego::BlockPosition req;
    req.request.call = true;

    while(ok()){
        // call the service to get the position of the lego
        if(client.call(req)){
            ur5lego::Pose pose = req.response.pose;
            if (pose.position.x > 0.72){
                continue;
            }
            pos_msgs.push(pose);
        }
        mg.actionPlanner(pos_msgs);
    }

    return 0;
}