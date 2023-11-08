#include "include/trajectory_planner.h"
#include <std_msgs/Float64MultiArray.h>


/// @brief sends the joint angles with a given publisher
/// @param publisher the publisher instance that will send the vector
/// @param q         the vector to send
void send_joint_positions(ros::Publisher publisher, Eigen::VectorXd q){
    std_msgs::Float64MultiArray command;
    command.data.resize(6);
    for(int i=0; i<6; i++)
        command.data[i] = (float)q[i];
    publisher.publish(command);
    ros::spinOnce();    // spin once to make sure the callback is processed
}


/// @brief sends the joint angles with a given publisher
/// @param qi       the startiong joint positions
/// @param qf       the target joint positions
/// @param t        the time to go from the starting position to the target
/// @param steps    the number of steps to make while sending the positions
void computeAndSendTrajectory(Eigen::VectorXd qi, Eigen::VectorXd qf, float tf, int steps, ros::Publisher publisher){
    float dt = tf/steps;
    float time = 0;

    Eigen::VectorXd q = qi; // position sent so far
    Eigen::VectorXd c;    // polynomial coefficients

    ros::Rate rate = ros::Rate(steps/tf);

    Eigen::VectorXd q_diff = (q-qf).cwiseAbs();
    while(time < tf){
        for(int jointi=0; jointi<6; jointi++){
            c = thirdOrderPolynomialTrajectory(tf, qi[jointi], qf[jointi]);
            q[jointi] = c[0] + c[1]*time + c[2]*pow(time,2) + c[3]*pow(time,3);
        }

        send_joint_positions(publisher, q);
        
        time += dt;
        q_diff = (q-qf).cwiseAbs();

        rate.sleep();
    }
}

