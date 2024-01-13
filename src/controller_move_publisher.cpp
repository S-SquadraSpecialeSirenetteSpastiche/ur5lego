#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ur5lego/Pose.h"
#include <cmath>
#include <queue>
#include <sstream>
#include <iostream>

using namespace ros;
using namespace std;

queue<ur5lego::Pose::ConstPtr> camera_pos_msgs;

void cameraCallback(const ur5lego::Pose::ConstPtr & msg){
    std::cout << "I received the message from camera!" << std::endl;
    ROS_INFO("I received the message from camera!");
    camera_pos_msgs.push(msg);
}

int main(int argc, char **argv){

  //srand(time(NULL));

  init(argc, argv, "talker");

  ROS_INFO("Ready to transmit positions");

  NodeHandle node_chatter;
  NodeHandle node_camera;

  Subscriber camera_sub = node_camera.subscribe("camera_position_channel", 1000, cameraCallback);
  Publisher chatter_pub = node_chatter.advertise<ur5lego::Pose>("lego_position", 1000);

  while (ok())
  {
    
    if(!camera_pos_msgs.empty()){
      ur5lego::Pose msg;
      msg = *camera_pos_msgs.front();
      ROS_INFO("X:%f, Y:%f, Z:%f", msg.position.x, msg.position.y, msg.position.z);
      chatter_pub.publish(msg);
      camera_pos_msgs.pop();
    }
    
    /*
    ur5lego::Pose msg;
    msg.position.x = (_Float32)(0.45); //(rand()%10)/20;
    msg.position.y = (_Float32)(0.43); //(rand()%10)/30;
    msg.position.z = (_Float32)(0.58);
    msg.orientation.x = (_Float64)(0);
    msg.orientation.y = (_Float64)(-1.57);
    msg.orientation.z = (_Float64)(1.57);

    ROS_INFO("X:%f, Y:%f, Z:%f", msg.position.x, msg.position.y, msg.position.z);
    chatter_pub.publish(msg);
    */
    spinOnce();
  }


  return 0;
}