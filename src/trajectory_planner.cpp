#include "include/trajectory_planner.h"


/// @brief sends the commands to bring the joints from a position to another
/// @param q the startiong joint positions
/// @param qf the target joint positions
/// @param t the time to go from the starting position to the target
/// @param steps the number of commands to send
void computeAndSendTrajectory(Eigen::VectorXd q, Eigen::VectorXd qf, float t, int steps){
    double eps = 1e-6;
    float dt = t/steps;
    float time = 0;
    Ros::Rate send_position_rate(dt);

    // itero fino a che tutti i valori di q e qf sono distanti meno di eps
    // la seconda espressione si occupa delle differenze negative
    while((q1-qf).maxCoeff() > eps || -1*(q1-qf).minCoeff() > eps ){
        for(i=0; i<6; i++){
            Eigen::VectorXd a = fifthOrderPolynomialTrajectory(t, q[i], qf[i]);
            q[i] = a[0] + a[1]*t + a[2]*pow(t,2) + a[3]*pow(t,3) + a[4]*pow(t,4) + a[5]*pow(t,5);
            v[i] = a[1] + 2*a[2]*t + 3*a[3]*pow(t,2) + 4*a[4]*pow(t,3) + 5*a[5]*pow(t,4);
            a[i] = 2*a[2] + 6*a[3]*t + 12*a[4]*pow(t,2) + 20*a[5]*pow(t,3);
        }

        // publish q, v, and a here

        time += dt;
        rate.sleep();
    }
}
