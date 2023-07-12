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
void computeAndSendTrajectory(Eigen::VectorXd qi, Eigen::VectorXd qf, float t, int steps, ros::Publisher publisher){
    double eps = 1e-6;
    float dt = t/steps;
    float time = 0;
    // ros::Rate send_position_rate(dt);    // TODO: capire perchè questa non va

    Eigen::VectorXd q = qi; // posizione che sono arrivato ad inviare
    Eigen::VectorXd v(6), a(6);
    Eigen::VectorXd c;    // coefficienti del polinomio

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


/// @brief computes and sends the joint angles to go to a desired position in a specific time frame
/// the trajectory is a line (TODO: it is not a line yet) that starts from the current position (obtained from q0) and ends to the desired one
/// @param model                the model of the robot
/// @param target_position      desired end effector position
/// @param target_rotation   desired end effector rotation expressed in euler angles
/// @param q0                   current joint configuration
/// @param time_to_move         seconds to complete the movement
/// @return true if it succeeds, false otherwise
bool compute_and_send_trajectory_2(pinocchio::Model model, Eigen::Vector3d target_position, 
    Eigen::Vector3d target_rotation, Eigen::VectorXd q0, float time_to_move, ros::Publisher publisher){

    double max_step_size_position = 0.05;   // max meters per step
    double max_step_size_rotation = 0.05;   // max radiants per step

    pinocchio::Data data(model);
    pinocchio::FrameIndex frame_id = model.getFrameId("ee_link", (pinocchio::FrameType)pinocchio::BODY);

    //TODO: non tutti questi updateFramePlacements servono
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeAllTerms(model, data, q0, Eigen::VectorXd::Zero(model.nv));
    pinocchio::updateFramePlacements(model, data);
    pinocchio::SE3 start_pos = pinocchio::updateFramePlacement(model, data, frame_id);
    pinocchio::updateFramePlacements(model, data);

    Eigen::VectorXd difference_pos(3);  // difference between current XYZ value and desired one
    Eigen::VectorXd difference_rot(3);  // difference between current rpy value and desired one

    ROS_INFO_STREAM("posizione iniziale: " << start_pos.translation());

    for(int i=0; i<3; i++){
        difference_pos[i] = std::abs(target_position[i] - start_pos.translation()(i));
        difference_rot[i] = std::abs(target_rotation[i] - start_pos.rotation()(i));
    }

    // calcolo il numero di passi nel percorso
    double max_diff_pos=0, max_diff_rot=0;
    for(int i=0; i<3; i++){
        max_diff_pos = std::max(difference_pos[i], max_diff_pos);
        max_diff_rot = std::max(difference_rot[i], max_diff_rot);
    }
    int steps_for_max_pos = std::ceil(max_diff_pos/max_step_size_position);
    int steps_for_max_rot = std::ceil(max_diff_rot/max_step_size_rotation);
    int n_steps = std::max(steps_for_max_pos, steps_for_max_rot);

    ROS_INFO_STREAM("n step: " << n_steps);

    // calcolo la dimensione dei passi
    Eigen::VectorXd step_size(6);
    for(int i=0; i<3; i++)
        step_size[i] = difference_pos[i]/(double)n_steps;
    for(int i=0; i<3; i++)
        step_size[i+3] = difference_rot[i]/(double)n_steps;

    ROS_INFO_STREAM("step sizes: " << step_size);

    // per ogni passo calcolo la cinematica inversa
    // l'algoritmo che calcola la cinematica inversa è veloce perchè la differenza dall'ultima posizione è piccola
    Eigen::VectorXd q(6);   // angoli joint attuali
    Eigen::Vector3d position_sofar = start_pos.translation();
    Eigen::Vector3d orientation_sofar = pinocchio::rpy::matrixToRpy(start_pos.rotation());
    q = q0;
    for(int i=0; i<n_steps; i++){
        std::pair<Eigen::VectorXd, bool> pos_sofar = inverse_kinematics(model, position_sofar, orientation_sofar, q);
        if(!pos_sofar.second)
            return false;
        q = pos_sofar.first;

        for(int j=0; j<3; j++){
            position_sofar[j] += step_size[j];
            orientation_sofar[j] += step_size[j+3];
        }
        // ROS_INFO_STREAM("pos so far: " << position_sofar);
    }

    ROS_INFO_STREAM("posizione finale: " << position_sofar);
    ROS_INFO_STREAM("q finale: " << q);

    // send_joint_positions(publisher, q);
    computeAndSendTrajectory(q0, q, time_to_move, 1000, publisher);

    return true;

    // calcolo la dimensione dei passi e dei resti
    /*
    double step_size_position[3], step_size_rotation[3], remainder_position[3], remainder_rotation[3];
    for(int i=0; i<3; i++){
        step_size_position[i] = difference[i]/n_steps;
        step_size_rotation[i] = difference[i+3]/n_steps;
        remainder_position[i] = target_pos[i] - step_size_position[i]*n_steps;
        remainder_position[i] = target_rot[i] - step_size_rotation[i]*n_steps;
    }
    */

    /*
    Eigen::Vector3d step_size_position(difference[0]/n_steps, difference[1]/n_steps, difference[2]/n_steps);
    Eigen::Vector3d step_size_rotation(difference[3]/n_steps, difference[4]/n_steps, difference[5]/n_steps);
    Eigen::Vector3d remainder_position(target_pos[0]-step_size_position[0]*n_steps, target_pos[1]-step_size_position[1]*n_steps, target_pos[2]-step_size_position[2]*n_steps);
    Eigen::Vector3d remainder_rotation(target_rot[0]-step_size_rotation[0]*n_steps, target_rot[1]-step_size_rotation[1]*n_steps, target_rot[2]-step_size_rotation[2]*n_steps);
    */

}