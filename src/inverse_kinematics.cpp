#include "include/inverse_kinematics.h"

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

Eigen::Matrix3d euler_to_rotation_matrix_2(Eigen::Vector3d rpy){
    Eigen::Matrix3d Rx, Ry, Rz;
    double r=rpy[0], p=rpy[1], y=rpy[2];

    Rx <<   1, 0,         0,
            0, cos(r),    sin(r),
            0, -1*sin(r), cos(r);

    Ry <<   cos(p), 0, -1*sin(p),
            0,      1, 0,
            sin(p), 0, cos(p);

    Rz <<   cos(y),    sin(y), 0,
            -1*sin(y), cos(y), 0,
            0,         0,      1;

    return Rx*(Ry*Rz);
}


// implements the inverse kinematics function
// parameters:
// model: the model of the robot
// q: the starting position
// oMdes: the desired final position
// returns:
// a pair, containing the resulting joint angles as the first element
// and a boolean that indicates wether or not the operation was successful
std::pair<Eigen::VectorXd, bool> inverse_kinematics(
    pinocchio::Model model, Eigen::Vector3d target_position, Eigen::Vector3d target_orientation, Eigen::VectorXd q){

    const pinocchio::SE3 oMdes(euler_to_rotation_matrix(target_orientation), target_position);

    const int JOINT_ID = 6;     // id of the last joint
    const double eps  = 1e-2;   // exit successfully if norm of the error is less than this
    const int IT_MAX  = 1000;   // max iterations before failure
    const double DT   = 1e-2;   // delta time
    const double damp = 1e-4;   // dampling factor (???)

    pinocchio::Data data(model);
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();

    bool success = false;
    Eigen::Matrix<double, 6, 1> err;
    Eigen::VectorXd v(model.nv);

    for (int i=0; i<IT_MAX; i++) {
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
        // and using a fixed dampinocchiog factor is not necessarily the best course of action.
        pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
        // add the obtained tangent vector to the current configuration q
        // integrate amounts to a simple sum. The resulting error will be verified in the next iteration.
        q = pinocchio::integrate(model, q, v*DT);
    }

    return std::make_pair(q, success);
}