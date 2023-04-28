#include "include/inverse_kinematics.h"
#include <ros/ros.h>


std::pair<Eigen::VectorXd, bool> inverse_kinematics(
    pinocchio::Model model, Eigen::Vector3d target_position, Eigen::Vector3d target_orientation_rpy, Eigen::VectorXd q){

    Eigen::Matrix3d target_orientation = euler_to_rotation_matrix(Eigen::Vector3d(target_orientation_rpy));
    std::string frame_name = "ee_link";
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(model.nv);

    double e_bar = 1;
    double eps = 1e-6;

    double alpha = 1, beta = 0.5, gamma = 0.5;

    int niter = 0;
    int max_iter = 200;

    double lambda = 1e-6;
    bool out_of_workspace = false;

    pinocchio::FrameIndex frame_id = pinocchio::getFrameId(frame_name, (pinocchio::FrameType)pinocchio::BODY)

    // while(true)
        myComputeAllTerms(model, data, q0);

        pinocchio::SE3 pos_q0 = pinocchio::updateFramePlacement(model, data, frame_id);
        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q0, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);

        ROS_INFO_STREAM(jacobian);
}


void myComputeAllTerms(pinocchio::Model model, pinocchio::Data data, Eigen::VectorXd q){
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::crba(model, data, q);
    pinocchio::nonLinearEffects(model, data, q, Eigen::VectorXd::Zero(model.nv));
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, Eigen::VectorXd::Zero(model.nv));
    pinocchio::updateFramePlacements(model, data);
}


Eigen::Matrix3d euler_to_rotation_matrix(Eigen::Vector3d rpy){
    Eigen::Matrix3d Rx, Ry, Rz;
    double r=rpy[0], p=rpy[1], y=rpy[2];

    Rx <<   1, 0,      0,
            0, cos(r), -1*sin(r),
            0, sin(r),  cos(r);
            
    Ry <<   cos(p),    0, sin(p),
            0,         1, 0,
            -1*sin(p), 0, cos(p);

    Rz <<   cos(y), -1*sin(y), 0,
            sin(y), cos(y),    0,
            0,      0,         1;

    return Rz*(Ry*Rx);
}


// implements the inverse kinematics function
std::pair<Eigen::VectorXd, bool> inverse_kinematics_bad(
    pinocchio::Model model, Eigen::Vector3d target_position, Eigen::Vector3d target_orientation, Eigen::VectorXd q){

    const pinocchio::SE3 oMdes(euler_to_rotation_matrix(target_orientation), target_position);

    const int JOINT_ID = 6;     // id of the last joint
    const double eps  = 1e-3;   // exit successfully if norm of the error is less than this
    const int IT_MAX  = 1000;   // max iterations before failure
    const double DT   = 1e-2;   // delta time
    const double damp = 1e-6;   // dampling factor

    pinocchio::Data data(model);
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();

    bool success = false;
    Eigen::Matrix<double, 6, 1> err;
    Eigen::VectorXd v(model.nv);

    int i;
    for (i=0; i<IT_MAX; i++) {
        pinocchio::forwardKinematics(model, data, q);
        // data.oMi[JOINT_ID] corresponds to the placement of the sixth joint
        // previously computed by forwardKinematics
        const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);

        // dMi corresponds to the transformation between the desired pose and the current one
        err = pinocchio::log6(dMi).toVector();
        if(err.norm() < eps){
            success = true;
            break;
        }

        // compute the evolution of the configuration by solving the inverse kinematics
        // in order to avoid problems at singularities, we employ the damped pseudo-inverse
        // implementing the equation as v. This way to compute the damped pseudo-inverse
        // was chosen because of its simplicity of implementation.
        // It is not necessarily the best nor the fastest way, 
        // and using a fixed damping factor is not necessarily the best course of action.
        pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
        // add the obtained tangent vector to the current configuration q
        // integrate amounts to a simple sum. The resulting error will be verified in the next iteration.
        q = pinocchio::integrate(model, q, v*DT);
    }
    
    ROS_INFO_STREAM("iterations: " << i);

    return std::make_pair(q, success);
}