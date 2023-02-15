#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ur5lego/MoveAction.h>
#include <string>
#include <iostream>

std::string coordsToStr(float a, float b, float c, float d, float e, float f){
    std::string s;
    s = "(" + std::to_string(a) + "," + std::to_string(b) + "," + std::to_string(c) + "), (" + 
        std::to_string(d) + "," + std::to_string(e) + "," + std::to_string(f) + ")";
    return s;
}

std::string qToStr(Eigen::VectorXd q){
    std::string s;
    // i cicli for sono molto difficili
    s = "[" + std::to_string(q(0)) + "," + std::to_string(q(1)) + "," + std::to_string(q(2)) + ", " + 
        std::to_string(q(3)) + "," + std::to_string(q(4)) + "," + std::to_string(q(5)) + "]";
    return s;
}

class MoveAction
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    actionlib::SimpleActionServer<ur5lego::MoveAction> action_server_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    ur5lego::MoveFeedback feedback_;
    ur5lego::MoveResult result_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    const int JOINT_ID = 6;

public:
    MoveAction(std::string name) : action_server_(nh_, name, boost::bind(&MoveAction::executeCB, this, _1), false), action_name_(name)
    {
        const std::string urdf_file = std::string("/opt/openrobots/share/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf");
        pinocchio::urdf::buildModel(urdf_file, model_);         // ur5
        // pinocchio::buildModels::manipulator(model_);         // manipolatore generico
        action_server_.start();
    }

    ~MoveAction(void)
    {
    }

    void executeCB(const ur5lego::MoveGoalConstPtr &goal)
    {
        ROS_INFO_STREAM("Executing... target: " << coordsToStr(goal->X, goal->Y, goal->Z, goal->r, goal->p, goal->y));
        result_.success = move(goal->X, goal->Y, goal->Z);
        action_server_.setSucceeded(result_);
    }

    bool move(double X, double Y, double Z){
        pinocchio::Data data(model_);
        const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(X, Y, Z));   // destination

        Eigen::VectorXd q = pinocchio::neutral(model_);  // initial configuration
        const double eps  = 1e-3;   // exit successfully if norm of the errors is less than this
        const int IT_MAX  = 20000;  // max iterations before failure
        const double DT   = 1e-3;   // delta time
        const double damp = 1e-6;   // dampling factor (???)

        pinocchio::Data::Matrix6x J(6,model_.nv);
        J.setZero();

        bool success = false;
        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        Vector6d err;
        Eigen::VectorXd v(model_.nv);

        
        for (int i=0; i<IT_MAX; i++) {
            pinocchio::forwardKinematics(model_,data,q);
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
            pinocchio::computeJointJacobian(model_,data,q,JOINT_ID,J);
            pinocchio::Data::Matrix6 JJt;
            JJt.noalias() = J * J.transpose();
            JJt.diagonal().array() += damp;
            v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
            // add the obtained tangent vector to the current configuration q
            // integrate amounts to a simple sum. The resulting error will be verified in the next iteration.
            q = pinocchio::integrate(model_,q,v*DT);
            if(!(i%100)){
                // std::cout << i << ": error = " << err.transpose() << std::endl;
                ROS_INFO_STREAM(std::to_string(i) + ": " + std::to_string(err.norm()) + "   " + qToStr(q));
            }
        }

        if(success) 
            ROS_INFO_STREAM("Convergence achieved!");
        else 
            ROS_INFO_STREAM("Warning: the iterative algorithm has not reached convergence to the desired precision");

        // std::cout << "\nresult: " << q.transpose() << std::endl;
        // std::cout << "\nfinal error: " << err.transpose() << std::endl;

        return success;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_server");

    MoveAction moveAction("move_server");
    ros::spin();

    return 0;
}
