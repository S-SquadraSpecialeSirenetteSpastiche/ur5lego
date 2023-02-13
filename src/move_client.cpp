#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ur5lego/MoveAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ur5lego::MoveAction> ac("idk", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  ur5lego::MoveGoal goal;
  goal.X = 20;
  goal.Y = 30;
  goal.Z = 40;
  goal.r = 10;
  goal.p = 13;
  goal.y = 27;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    ROS_INFO("Action finished");
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}