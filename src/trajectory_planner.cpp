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
/// @param q        the startiong joint positions
/// @param qf       the target joint positions
/// @param t        the time to go from the starting position to the target
/// @param steps    the number of commands to send
void computeAndSendTrajectory(Eigen::VectorXd q, Eigen::VectorXd qf, float t, int steps, ros::Publisher publisher){
    double eps = 1e-6;
    float dt = t/steps;
    float time = 0;
    ros::Rate rate(dt);

    Eigen::VectorXd v(6), a(6);
    Eigen::VectorXd c;    /* coefficienti del polinomio */

    // itero fino a che tutti i valori di q e qf sono distanti meno di eps
    // la seconda espressione si occupa delle differenze negative
    while((q-qf).maxCoeff() > eps || -1*(q-qf).minCoeff() > eps ){
        for(int i=0; i<6; i++){
            c = fifthOrderPolynomialTrajectory(t, q[i], qf[i]);

            q[i] = c[0] + c[1]*t + c[2]*pow(t,2) + c[3]*pow(t,3) + c[4]*pow(t,4) + c[5]*pow(t,5);
            v[i] = c[1] + 2*c[2]*t + 3*c[3]*pow(t,2) + 4*c[4]*pow(t,3) + 5*c[5]*pow(t,4);
            a[i] = 2*c[2] + 6*c[3]*t + 12*c[4]*pow(t,2) + 20*c[5]*pow(t,3);
        }

        send_joint_positions(publisher, q);

        time += dt;
        rate.sleep();
    }

    send_joint_positions(publisher, qf);
}
