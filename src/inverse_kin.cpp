#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "ros/ros.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "inverse_kin_simple");

    const std::string urdf_filename = std::string("/opt/openrobots/share/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf");

    pinocchio::Model model;
    // pinocchio::urdf::buildModel(urdf_filename, model);   // riga che serve ma distrugge tutto
    pinocchio::buildModels::manipulator(model);
    pinocchio::Data data(model);

    const int JOINT_ID = 6;     // id of the last join
    const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 0., 1.));   // destination

    Eigen::VectorXd q = pinocchio::neutral(model);  // initial configuration
    const double eps  = 1e-3;   // exit successfully if norm of the errors is less than this
    const int IT_MAX  = 10000;  // max iterations before failure
    const double DT   = 1e-3;   // delta time
    const double damp = 1e-6;   // dampling factor (???)

    pinocchio::Data::Matrix6x J(6,model.nv);
    J.setZero();

    bool success = false;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(model.nv);

    
    for (int i=0; i<IT_MAX; i++) {
        pinocchio::forwardKinematics(model,data,q);
        // data.oMi[JOINT_ID] corresponds to the placement of the sixth joint (previously computed by forwardKinematics)
        const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);

        // dMi corresponds to the transformation between the desired pose and the current one
        err = pinocchio::log6(dMi).toVector();

          
        // If the error norm is below the previously-defined threshold
        // we have found the solution and we break out of the loop
        // the norm of a spatial velocity does not make physical sense, since it mixes linear and angular quantities. 
        // A more rigorous implementation should treat the linar part and the angular part separately. 
        // In this example, however, we choose to slightly abuse the notation in order to keep it simple.
        if(err.norm() < eps)
        {
            success = true;
            break;
        }

        // compute the evolution of the configuration by solving the inverse kinematics
        // in order to avoid problems at singularities, we employ the damped pseudo-inverse
        // implementing the equation as v. This way to compute the damped pseudo-inverse was chosen
        // because of its simplicity of implementation. It is not necessarily the best nor the fastest way, 
        // and using a fixed damping factor is not necessarily the best course of action.
        pinocchio::computeJointJacobian(model,data,q,JOINT_ID,J);
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
        // add the obtained tangent vector to the current configuration q
        // integrate amounts to a simple sum. The resulting error will be verified in the next iteration.
        q = pinocchio::integrate(model,q,v*DT);
        if(!(i%100))
            std::cout << i << ": error = " << err.transpose() << std::endl;
    }

    if(success) 
        std::cout << "Convergence achieved!" << std::endl;
    else 
        std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;

    std::cout << "\nresult: " << q.transpose() << std::endl;
    std::cout << "\nfinal error: " << err.transpose() << std::endl;
}