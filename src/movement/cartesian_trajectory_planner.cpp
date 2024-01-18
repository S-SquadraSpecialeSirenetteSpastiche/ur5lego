#include "../include/inverse_kinematics.h"
#include "../include/joint_trajectory_planner.h"
#include "../include/cartesian_trajectory_planner.h"

#include <Eigen/Geometry>


void compute_and_send_arc_trajectory(
    pinocchio::Model model, Eigen::VectorXd q, Eigen::Isometry3d ti, 
    Eigen::Isometry3d tf, float time, float ds, ros::Publisher publisher)
    {

    std::vector<Eigen::VectorXd> joint_positions = std::vector<Eigen::VectorXd>();

    Eigen::Isometry3d tmiddle;
    // center point of the arc is an easily reachable point reagdless of the position we're in
    tmiddle.translation() = Eigen::Vector3d(0.0, 0.4, 0.4);
    // rotation of tmiddle is half way between ti and tf
    Eigen::Quaterniond qcenter = ti.rotation().slerp(0.5, tf.rotation());
    tmiddle.linear() = qcenter.toRotationMatrix();

    // compute the normal of the plane containing the arc
    // Eigen::Vector3d v1 = tmiddle.translation() - ti.translation();
    // Eigen::Vector3d v2 = tf.translation() - ti.translation();
    // Eigen::Vector3d normal = v1.cross(v2);


    double total_distance = (ti.translation() - tmiddle.translation()).norm() + (tmiddle.translation() - tf.translation()).norm();
    // generate the points on the arc
    for (double s = 0; s <= total_distance; s += ds) {
        double t = s / total_distance;
        Eigen::Isometry3d point;
        if (s <= (ti.translation() - tmiddle.translation()).norm()) {
            // interpolate between ti and tmiddle
            point.translation() = ti.translation() + t * (tmiddle.translation() - ti.translation());
            point.linear() = ti.rotation().slerp(t, tmiddle.rotation()).toRotationMatrix();
        } else {
            // interpolate between tmiddle and tf
            point.translation() = tmiddle.translation() + (t - 0.5) * 2 * (tf.translation() - tmiddle.translation());
            point.linear() = tmiddle.rotation().slerp((t - 0.5) * 2, tf.rotation()).toRotationMatrix();
        }

        // LEFT HERE
        joint_positions.push_back(
            inverse_kinematics(model, point.position, pinocchio_from_rot_matrix_to_rpy_or_something(point.linear()), q).second);
        q = joint_positions.back();
    }
    
    ros::Rate rate = ros::Rate(tf/joint_positions.size());

    for(Eigen::VectorXd position : joint_positions){
        send_arm_joint_angles(position, publisher);
        rate.sleep();
    }
}