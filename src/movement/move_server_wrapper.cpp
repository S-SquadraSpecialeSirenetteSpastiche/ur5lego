#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ur5lego/MoveAction.h>
#include <std_msgs/Float32MultiArray.h>

typedef actionlib::SimpleActionClient<ur5lego::MoveAction> MoveActionClient;


/// @brief callback for the move_channel topic
/// @param msg the message containing the goal to send to the move server
void moveCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() != 7) {
        ROS_WARN("Received message with %d floats instead of 7", (int)msg->data.size());
        return;
    }

    ur5lego::MoveGoal goal;
    goal.X = msg->data[0];
    goal.Y = msg->data[1];
    goal.Z = msg->data[2];
    goal.r = msg->data[3];
    goal.p = msg->data[4];
    goal.y = msg->data[5];
    goal.time = msg->data[6];

    MoveActionClient ac("move_server", true);
    ac.waitForServer();

    ROS_INFO("Sending goal to move server");
    ac.sendGoal(goal);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_server_wrapper");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("move_channel", 10, moveCallback);

    ros::spin();

    return 0;
}
