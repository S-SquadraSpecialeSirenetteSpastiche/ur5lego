#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ur5lego/MoveAction.h>
#include <fstream>
#include <unistd.h>

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

    std::ifstream file("/home/utente/my_ws/src/ur5lego/src/dest.txt");
    int n;
    file >> n;

    ur5lego::MoveGoal *goals = new ur5lego::MoveGoal[n];


    for(int i=0; i<n; i++){
        // su Gazebo asse rosso x, verde y, blu z
        float X, Y, Z, r, p, y;
        // se li metto uno dopo l'altro non funziona
        file >> X;
        file >> Y;
        file >> Z;
        file >> r;
        file >> p;
        file >> y;
        goals[i].X = (_Float32)X;
        goals[i].Y = (_Float32)Y;
        goals[i].Z = (_Float32)Z;
        goals[i].r = (_Float32)r;
        goals[i].p = (_Float32)p;
        goals[i].y = (_Float32)y;
    }
    file.close();

    int i=0;
    while(ros::ok()){
        ROS_INFO("Sending goal %d", i);

        ac.sendGoal(goals[i]);

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(20.0));
        if (finished_before_timeout){
            ROS_INFO("Action finished");
        }
        else{
            ROS_INFO("Action did not finish before the time out.");
            break;
        }

        i++;
        if(i>=n) i=0;
        sleep(1);
    }

    delete[] goals;

    return 0;
}