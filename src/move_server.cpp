#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <ros/ros.h>
#include <ros/package.h>
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
    pinocchio::Model model, Eigen::VectorXd q, pinocchio::SE3 oMdes){

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

class MoveAction
{
protected:
    // node that acts as action server
    ros::NodeHandle server_node;
    // node that sends commands to the robot
    ros::NodeHandle talker_node;
    ros::Publisher publisher;
    // NodeHandle instance must be created before this line
    actionlib::SimpleActionServer<ur5lego::MoveAction> action_server_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    ur5lego::MoveFeedback feedback_;
    ur5lego::MoveResult result_;
    
    pinocchio::Model model_;
    Eigen::VectorXd q;  // current configuration, assumed neutral at start

public:
    MoveAction(std::string name) : action_server_(server_node, name, boost::bind(&MoveAction::executeCB, this, _1), false), action_name_(name)
    {
        publisher = talker_node.advertise<std_msgs::Float64MultiArray>(
            "/ur5/joint_group_pos_controller/command", 10);

        const std::string urdf_file = 
            ros::package::getPath("ur5lego") + std::string("/robot_description/ur5.urdf");
        pinocchio::urdf::buildModel(urdf_file, model_);
        q = pinocchio::neutral(model_);

        action_server_.start();
    }

    ~MoveAction(void)
    {
    }

    void send_joint_positions(){
        std_msgs::Float64MultiArray command;
        command.data.resize(6);
        for(int i=0; i<6; i++)
            command.data[i] = (float)q[i];
        publisher.publish(command);
    }

    void executeCB(const ur5lego::MoveGoalConstPtr &goal){
        ROS_INFO_STREAM("target: " << coordsToStr(
            goal->X, goal->Y, goal->Z, goal->r, goal->p, goal->y));
        result_.success = move(goal->X, goal->Y, goal->Z, goal->r, goal->p, goal->y);
        action_server_.setSucceeded(result_);
    }

    // tries to compute the inverse kinematics
    // to bring the joint number JOINT_ID to the desired position
    // if successful, sends the desired position to the robot
    bool move(double X, double Y, double Z, double r, double p, double y){
        const pinocchio::SE3 oMdes(euler_to_rotation_matrix(Eigen::Vector3d(r, p, y)), Eigen::Vector3d(X, Y, Z));
        std::pair<Eigen::VectorXd, bool> res = inverse_kinematics(model_, q, oMdes);

        if(res.second){
            ROS_INFO_STREAM("Convergence achieved!");
            q = res.first;
            send_joint_positions();
        }
        else{
            ROS_INFO_STREAM("Warning: the iterative algorithm has not reached convergence to the desired precision");
        }

        return res.second;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_server");
    MoveAction moveAction("move_server");
    ROS_INFO_STREAM("Server ready");
    ros::spin();

    return 0;
}
