#include "include/trajectory_planner.h"


Eigen::VectorXd fifthOrderPolynomialTrajectory(float tf, double start_q, double start_v, double start_a, double end_q, double end_v, double end_a){
    Eigen::MatrixXd poly_matrix(6, 6);
    poly_matrix <<  1,    0,    0,          0,            0,             0,
                    0,    1,    0,          0,            0,             0,
                    0,    0,    2,          0,            0,             0,
                    1,    tf,   pow(tf,2),  pow(tf, 3),   pow(tf, 4),    pow(tf, 5),
                    0,    1,    2*tf,       3*pow(tf,2),  4*pow(tf,3),   5*pow(tf,4),
                    0,    0,    2,          6*tf,         12*pow(tf,2),  20*pow(tf,3);

    Eigen::VectorXd poly_vector = Eigen::VectorXd(6);
    poly_vector << start_q, start_v, start_a, end_q, end_v, end_a;

    Eigen::VectorXd coefficients = poly_matrix.inverse()*poly_vector;

    return coefficients;
}

Eigen::VectorXd fifthOrderPolynomialTrajectory(float tf, double start_q, double end_q){
    return fifthOrderPolynomialTrajectory(tf, start_q, 0, 0, end_q, 0,0);
}


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
void computeAndSendTrajectory(Eigen::VectorXd qi, Eigen::VectorXd qf, float t, int steps, ros::Publisher publisher){
    double eps = 1e-6;
    float dt = t/steps;
    float time = 0;
    // ros::Rate send_position_rate(dt);    // TODO: capire perchè questa non va

    Eigen::VectorXd q = qi; // posizione che sono arrivato ad inviare
    Eigen::VectorXd v(6), a(6);
    Eigen::VectorXd c;    // coefficienti del polinomio

    ROS_INFO_STREAM("   dt " << dt);
    ROS_INFO_STREAM("   inizio " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5] << " ");
    ROS_INFO_STREAM("   fine " << qf[0] << " " << qf[1] << " " << qf[2] << " " << qf[3] << " " << qf[4] << " "<< qf[5] << " ");

    // itero fino a che tutti i valori di q e qf sono distanti meno di eps
    // la seconda espressione si occupa delle differenze negative
    while((q-qf).maxCoeff() > eps || -1*(q-qf).minCoeff() > eps ){
        for(int i=0; i<6; i++){
            c = fifthOrderPolynomialTrajectory(t, q[i], qf[i]);     // TODO: questi si possono calcolare una volta sola per ogni i, non serve ricalcolarli tutti ogni ripetizione del while

            q[i] = c[0] + c[1]*time + c[2]*pow(time,2) + c[3]*pow(time,3) + c[4]*pow(time,4) + c[5]*pow(time,5);
            v[i] = c[1] + 2*c[2]*time + 3*c[3]*pow(time,2) + 4*c[4]*pow(time,3) + 5*c[5]*pow(time,4);
            a[i] = 2*c[2] + 6*c[3]*time + 12*c[4]*pow(time,2) + 20*c[5]*pow(time,3);
        }

        send_joint_positions(publisher, q);
        // ROS_INFO_STREAM("   invio " << q[0] << " " << q[1] << " " << q[2] << " "<< q[3] << " " << q[4] << " "<< q[5] << " ");
        
        time += dt;
        // send_position_rate.sleep();
        ros::Duration(dt).sleep();
    }

    // send_joint_positions(publisher, qf);
}
