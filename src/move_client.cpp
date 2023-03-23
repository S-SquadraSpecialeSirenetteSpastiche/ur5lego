#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ur5lego/MoveAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ur5lego::MoveAction> ac("move_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  ur5lego::MoveGoal goal;
  goal.X = (_Float32)0.1;
  goal.Y = (_Float32)0.0;
  goal.Z = (_Float32)-0.2;
  goal.r = (_Float32)0;
  goal.p = (_Float32)0;
  goal.y = (_Float32)0;
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