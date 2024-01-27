#include "../include/joint_trajectory_planner.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>


/// @brief sends the joint angles with a given publisher
/// @param q         the joint positions to send
/// @param publisher the publisher to send the positions with
void send_arm_joint_angles(Eigen::VectorXd q, ros::Publisher publisher){
    std_msgs::Float64MultiArray command;
    command.data.resize(6);
    for(int i=0; i<6; i++)
        command.data[i] = (_Float64)q[i];
    publisher.publish(command);
}

void send_gripper_joint_angles(_Float64 q, ros::Publisher publisher){
    std_msgs::Float64 command;
    command.data = q;
    publisher.publish(command);
}

/// @brief sends the joint angles with a given publisher
/// @param qi        the startiong joint positions
/// @param qf        the target joint positions
/// @param tf        the time to go from the starting position to the target
/// @param dt        number of poisitions to send per second
/// @param publisher the publisher to send the positions with
void compute_and_send_trajectory(Eigen::VectorXd qi, Eigen::VectorXd qf, float tf, float freq, ros::Publisher publisher){
    float time = 0.0;
    float dt = 1.0/freq;

    Eigen::VectorXd q = qi; // position sent so far
    Eigen::VectorXd c;    // polynomial coefficients

    ros::Rate rate = ros::Rate(freq);

    while(time < tf){
        for(int jointi=0; jointi<q.size(); jointi++){
            c = thirdOrderPolynomialTrajectory(tf, qi[jointi], qf[jointi]); // TODO: do we really need to compute this every time?
            q[jointi] = c[0] + c[1]*time + c[2]*pow(time,2) + c[3]*pow(time,3);
        }
        
        send_arm_joint_angles(q, publisher);
        time += dt;
        rate.sleep();
    }
}


// TODO: usare una frequenza anche qui o generalizzare la funzione con un vettore
// per prendere un qualunque numero di elementi
void computeAndSendTrajectory(_Float64 qi, _Float64 qf, float tf, int steps, ros::Publisher publisher){
    float dt = tf/steps;
    float time = 0;

    _Float64 q = qi; // position sent so far
    Eigen::VectorXd c = Eigen::VectorXd(4);    // polynomial coefficients

    ros::Rate rate = ros::Rate(steps/tf);

    while(time < tf){
        
        c = thirdOrderPolynomialTrajectory(tf, qi, qf); //from math_tools
        q = c[0] + c[1]*time + c[2]*pow(time,2) + c[3]*pow(time,3);
        
        send_gripper_joint_angles(q, publisher);
        
        time += dt;

        rate.sleep();
    }
}
