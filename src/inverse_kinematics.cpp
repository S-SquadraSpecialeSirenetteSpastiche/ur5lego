#include "include/inverse_kinematics.h"
#include <ros/ros.h>

/*
The damping constant λ depends on the details of the multibody and the target
positions and must be chosen carefully to make DLS numerically stable.
It should large enough so that the solutions for ∆θ are well-behaved near
singularities but not so large that the convergence rate is too slow
*/


pinocchio::SE3 compute_position(pinocchio::Model &model, pinocchio::Data &data, Eigen::VectorXd q, pinocchio::FrameIndex frame_id){
    pinocchio::computeAllTerms(model, data, q, Eigen::VectorXd::Zero(model.nv));
    return pinocchio::updateFramePlacement(model, data, frame_id);
}


std::pair<Eigen::VectorXd, bool> inverse_kinematics(
    pinocchio::Model model, Eigen::Vector3d target_position, Eigen::Vector3d target_orientation_rpy, Eigen::VectorXd q0){

    Eigen::Matrix3d target_orientation = euler_to_rotation_matrix(Eigen::Vector3d(target_orientation_rpy));
    pinocchio::FrameIndex frame_id = model.getFrameId("ee_link", (pinocchio::FrameType)pinocchio::BODY);

    pinocchio::Data data(model);

    double eps = 1e-6;
    double alpha = 1, beta = 0.5, gamma = 0.5;

    int niter = 0;
    int max_iter = 200;

    double lambda = 1e-6;
    bool out_of_workspace = false;
    bool success = false;


    while(true){
        //TODO: non tutti questi updateFramePlacements servono
        pinocchio::updateFramePlacements(model, data);
        pinocchio::computeAllTerms(model, data, q0, Eigen::VectorXd::Zero(model.nv));
        pinocchio::updateFramePlacements(model, data);
        pinocchio::SE3 pos_q0 = pinocchio::updateFramePlacement(model, data, frame_id);
        pinocchio::updateFramePlacements(model, data);

        // ROS_INFO_STREAM(pos_q0.translation());
    
        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q0, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);

        Eigen::VectorXd e_bar_q0(model.nv);
        e_bar_q0 << (target_position - pos_q0.translation()), (pos_q0.rotation()*pinocchio::log3(pos_q0.rotation().transpose()*target_orientation));
        Eigen::VectorXd grad(model.nv);
        grad = jacobian.transpose()*e_bar_q0;

        Eigen::VectorXd dq(model.nv);
        dq = (jacobian + 1e-8*Eigen::MatrixXd::Identity(6, model.nv)).inverse()*e_bar_q0;

        if(grad.norm() < eps){
            success = true;
            // print("IK Convergence achieved!, norm(grad) :", np.linalg.norm(grad))
            // print("Inverse kinematics solved in {} iterations".format(niter))
            if(e_bar_q0.norm() > 0.1){
                // print("THE END EFFECTOR POSITION IS OUT OF THE WORKSPACE, norm(error) :", np.linalg.norm(e_bar))
                out_of_workspace = true;
            }
            break;
        }
        if(niter >= max_iter){
            success = false;    // per bellezza
            break;
        }

        Eigen::VectorXd q1(model.nv);

        while(true){
            q1 = q0 + dq * alpha;

            pinocchio::SE3 pos_q1;
            pinocchio::computeAllTerms(model, data, q1, Eigen::VectorXd::Zero(model.nv));
            pos_q1 = pinocchio::updateFramePlacement(model, data, frame_id);

            Eigen::VectorXd e_bar_q1(model.nv);
            e_bar_q1 << (target_position - pos_q1.translation()), (pos_q1.rotation()*pinocchio::log3(pos_q1.rotation().transpose()*target_orientation));

            double improvement = e_bar_q0.norm() - e_bar_q1.norm();
            float threshold = 0.0;  // even more strict: gamma*alpha*np.linalg.norm(e_bar)

            if(improvement < threshold){
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

    for(int i=0; i<model.nv; i++){
        while(q0[i] >= 2*3.1415927)
            q0[i] -= 2*3.1415927;
        while(q0[i] <= -2*3.1415927)
            q0[i] += 2*3.1415927;
    }

    // ROS_INFO_STREAM("q: " << q0);
    
    return std::make_pair(q0, success);
}


// FIXME: dovrebbe fare la stessa cosa di computeAllTerms ma non la fa
void myComputeAllTerms(pinocchio::Model model, pinocchio::Data data, Eigen::VectorXd q){
    pinocchio::forwardKinematics(model, data, q);   // dovrebbe essere inutile, computeJointJacobians lo fa già
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
    const double eps  = 1e-2;   // exit successfully if norm of the error is less than this
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
    
    // ROS_INFO_STREAM("iterations: " << i);
    // ROS_INFO_STREAM("q: " << q);

    for(int i=0; i<model.nv; i++){
        while(q[i] >= 2*3.1415927)
            q[i] -= 2*3.1415927;
        while(q[i] <= -2*3.1415927)
            q[i] += 2*3.1415927;
    }

    return std::make_pair(q, success);
}