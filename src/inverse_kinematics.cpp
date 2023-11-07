#include "include/inverse_kinematics.h"
#include <chrono>
#include <ros/ros.h>


/// @brief computes the inverse kinematics of a 6DOF robot
/// @param model                the robot model
/// @param target_position      the desired position of the end effector
/// @param target_orientation_rpy   the desired orientation of the end effector, in roll pitch yaw representation
/// @param q0                   the initial guess for the inverse kinematics
/// @return a pair containing the solution of the inverse kinematics and a boolean value indicating if the algorithm was successful
std::pair<Eigen::VectorXd, bool> inverse_kinematics_wrapper(
    pinocchio::Model model, Eigen::Vector3d target_position, Eigen::Vector3d target_orientation_rpy, Eigen::VectorXd q0){
    
    pinocchio::Data data(model);
    pinocchio::FrameIndex frame_id = model.getFrameId("ee_link", (pinocchio::FrameType)pinocchio::BODY);

    pinocchio::computeAllTerms(model, data, q0, Eigen::VectorXd::Zero(model.nv));
    pinocchio::SE3 start_pos = pinocchio::updateFramePlacement(model, data, frame_id);

    int n_steps = 10;

    Eigen::VectorXd q(6);
    Eigen::Vector3d position_sofar = start_pos.translation();
    Eigen::Vector3d orientation_sofar = pinocchio::rpy::matrixToRpy(start_pos.rotation());
    q = q0;

    double step_size[6];    // non è proprio una dimensione perchè potrebbe essere negativa ma così è più comodo sommarla
    for(int i=0; i<3; i++){
        step_size[i] = (target_position[i] - position_sofar[i])/(double)n_steps;
        step_size[i+3] = (target_orientation_rpy[i] - orientation_sofar[i])/(double)n_steps;
    }

    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for(int i=0; i<n_steps; i++){
        for(int j=0; j<3; j++){
            position_sofar[j] += step_size[j];
            orientation_sofar[j] += step_size[j+3];
        }

        std::pair<Eigen::VectorXd, bool> ikresult = inverse_kinematics(model, position_sofar, orientation_sofar, q);
        if(!ikresult.second)
            return std::make_pair(q, false);    // il valore di q qui è irrilevante perchè l'algoritmo non ha avuto successo
        q = ikresult.first;
    }
    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // ROS_INFO_STREAM("Time for ik algorithm = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]");

    return std::make_pair(q, true);
}

/// @brief computes the inverse kinematics of a 6DOF robot, this function only works for small movements and therefore should not be called directly
/// @param model                the robot model
/// @param target_position      the desired position of the end effector
/// @param target_orientation_rpy   the desired orientation of the end effector, in roll pitch yaw representation
/// @param q0                   the initial guess for the inverse kinematics
/// @return a pair containing the solution of the inverse kinematics and a boolean value indicating if the algorithm was successful
std::pair<Eigen::VectorXd, bool> inverse_kinematics(
    pinocchio::Model model, Eigen::Vector3d target_position, Eigen::Vector3d target_orientation_rpy, Eigen::VectorXd q0){

    Eigen::Matrix3d target_orientation = euler_to_rotation_matrix(Eigen::Vector3d(target_orientation_rpy));
    pinocchio::FrameIndex frame_id = model.getFrameId("ee_link", (pinocchio::FrameType)pinocchio::BODY);

    pinocchio::Data data(model);

    double eps = 1e-6;
    double alpha = 1, beta = 0.5, gamma = 0.5;

    int niter = 0;  // iterations so far
    int max_iter = 200;

    double lambda = 1e-8;
    bool out_of_workspace = false;
    bool success = false;


    while(niter < max_iter){ 
        // compute position and orientation of the ee with the current guess
        pinocchio::computeAllTerms(model, data, q0, Eigen::VectorXd::Zero(model.nv));
        pinocchio::SE3 pos_q0 = pinocchio::updateFramePlacement(model, data, frame_id);

        // compute the jacobian in q0
        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q0, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);

        // compute the error between the current position and the desired one
        Eigen::VectorXd e_bar_q0(model.nv);
        e_bar_q0 << target_position - pos_q0.translation(), errorInSO3(pos_q0.rotation(), target_orientation);
        Eigen::VectorXd grad(model.nv);
        grad = jacobian.transpose()*e_bar_q0;

        // if the error is small enough, we're done
        if(grad.norm() < eps){
            success = true;
            if(e_bar_q0.norm() > 0.1){
                out_of_workspace = true;
            }
            break;
        }

        // dq is the difference between the last guess (q0) and the new one (q1)
        Eigen::VectorXd dq(model.nv);
        dq = (jacobian + lambda*Eigen::MatrixXd::Identity(6, model.nv)).inverse()*e_bar_q0;
        Eigen::VectorXd q1(model.nv);

        while(true){
            // compute new guess and the position of the ee with the new guess
            q1 = q0 + dq * alpha;
            pinocchio::SE3 pos_q1;
            pinocchio::computeAllTerms(model, data, q1, Eigen::VectorXd::Zero(model.nv));
            pos_q1 = pinocchio::updateFramePlacement(model, data, frame_id);

            // compute error of the new guess
            Eigen::VectorXd e_bar_q1(model.nv);
            e_bar_q1 << target_position - pos_q1.translation(), errorInSO3(pos_q1.rotation(), target_orientation);

            // if the new guess is worse than the previous one, we adjust alpha and try again
            // otherwise the iteration is complete
            double improvement = e_bar_q0.norm() - e_bar_q1.norm();
            if(improvement < 0.0){
                alpha *= beta;
            }
            else{
                q0 = q1;
                alpha = 1;
                break;
            }
        }
        niter++;
    }

    // prevent angles more than 2pi or less than -2pi
    for(int i=0; i<model.nv; i++){
        while(q0[i] >= 2*M_PI)
            q0[i] -= 2*M_PI;
        while(q0[i] <= -2*M_PI)
            q0[i] += 2*M_PI;
    }
    
    return std::make_pair(q0, success && !out_of_workspace);
}
