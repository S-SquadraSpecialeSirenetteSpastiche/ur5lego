#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ur5lego/Pose.h"
#include <cmath>

#include <sstream>

using namespace ros;
using namespace std;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv){

  srand(time(NULL));

  init(argc, argv, "talker");

  NodeHandle nh;

  Publisher chatter_pub = nh.advertise<ur5lego::Pose>("lego_position", 1000);

  Rate loop_rate(0.05);

  int count = 0;
  while (ok())
  {

    ur5lego::Pose msg;
    msg.position.x = (_Float32)(0.45); //(rand()%10)/20;
    msg.position.y = (_Float32)(0.43); //(rand()%10)/30;
    msg.position.z = (_Float32)(0.5);
    msg.orientation.x = (_Float64)(1.5707);
    msg.orientation.y = (_Float64)(-1.5707);
    msg.orientation.z = (_Float64)(0);

    ROS_INFO("X:%f, Y:%f, Z:%f", msg.position.x, msg.position.y, msg.position.z);

    chatter_pub.publish(msg);

    loop_rate.sleep();
    ++count;
  }


  return 0;
}