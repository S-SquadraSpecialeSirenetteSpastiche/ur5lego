#include "include/trajectory_planner.h"


/// @brief sends the joint angles with a given publisher
/// @param publisher the publisher instance that will send the vector
/// @param q         the vector to send
void send_joint_positions(ros::Publisher publisher, Eigen::VectorXd q){
    std_msgs::Float64MultiArray command;
    command.data.resize(6);
    for(int i=0; i<6; i++)
        command.data[i] = (float)q[i];
    publisher.publish(command);
}


/// @brief sends the commands to bring the joints from a position to another
/// @param qi       the startiong joint positions
/// @param qf       the target joint positions
/// @param t        the time to go from the starting position to the target
/// @param steps    the number of commands to send
void computeAndSendTrajectory(Eigen::VectorXd qi, Eigen::VectorXd qf, float t, int steps, ros::Publisher publisher){
    double eps = 1e-6;
    float dt = t/steps;
    float time = 0;
    // ros::Rate send_position_rate(dt);    // TODO: capire perchè questa non va

    Eigen::VectorXd q = qi; // position sent so far
    // Eigen::VectorXd v(6), a(6);
    Eigen::VectorXd c;    // polynomial coefficients

    // iterate until all values of |q-qf| are less than eps
    // the second expression takes care of the negative differences)
    while((q-qf).maxCoeff() > eps || -1*(q-qf).minCoeff() > eps ){
        for(int i=0; i<6; i++){
            c = thirdOrderPolynomialTrajectory(t, q[i], qf[i]);

            q[i] = c[0] + c[1]*time + c[2]*pow(time,2) + c[3]*pow(time,3);
            // v[i] = c[1] + 2*c[2]*time + 3*c[3]*pow(time,2);
            // a[i] = 2*c[2] + 6*c[3]*time;
        }

        send_joint_positions(publisher, q);
        
        time += dt;
        // send_position_rate.sleep();
        ros::Duration(dt).sleep();
    }
}

