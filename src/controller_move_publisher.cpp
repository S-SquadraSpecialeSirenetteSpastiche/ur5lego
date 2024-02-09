#include "ros/ros.h"
#include "ur5lego/Pose.h"
#include "ur5lego/BlockPosition.h"
#include <cmath>
#include <queue>
#include <sstream>
#include <iostream>

using namespace ros;
using namespace std;

bool positionSetup(ur5lego::BlockPosition::Request  &req, ur5lego::BlockPosition::Response &res)
{
  ur5lego::Pose result;
  result.position.x = 0.0;
  result.position.y = 0.0;
  result.position.z = 0.0;
  result.orientation.x = 0.0;
  result.orientation.y = 0.0;
  result.orientation.z = 0.0;
  res.position = result;

  return true;
}

int main(int argc, char **argv){

  init(argc, argv, "camera_publisher");

  ROS_DEBUG("Ready to transmit positions");

  NodeHandle nh;

  ServiceServer camera_service = nh.advertiseService("get_position", positionSetup);

  spin();
}