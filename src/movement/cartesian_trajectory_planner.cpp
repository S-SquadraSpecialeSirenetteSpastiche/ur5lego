#include "../include/inverse_kinematics.h"
#include "../include/joint_trajectory_planner.h"
#include "../include/cartesian_trajectory_planner.h"

#include "../include/trajectory.h"


/// @brief computes the points on the arc, assuming that the arc is on the xy plane
/// @param p1 first point of the arc
/// @param p2 second point of the arc
/// @param p3 third point of the arc
/// @param ds distance between two consecutive points on the arc
/// @return the points on the arc
std::vector<Eigen::Vector3d> compute_arc_points(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, double ds) {
    // calculate the circle's center and radius
    Eigen::Vector3d a = p2 - p1;
    Eigen::Vector3d b = p3 - p2;
    Eigen::Vector3d c = p1 - p3;

    double d = 2 * (a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) + c.x() * (a.y() - b.y()));

    double ux = ((a.norm() * b.y() - a.norm() * c.y()) + (b.norm() * c.y() - b.norm() * a.y()) + (c.norm() * a.y() - c.norm() * b.y())) / d;
    double uy = ((a.norm() * c.x() - a.norm() * b.x()) + (b.norm() * a.x() - b.norm() * c.x()) + (c.norm() * b.x() - c.norm() * a.x())) / d;

    Eigen::Vector3d circle_center(ux, uy, p1.z());
    double circle_radius = (p1 - circle_center).norm();

    ROS_INFO_STREAM("circle_center: " << circle_center.transpose());
    ROS_INFO_STREAM("circle_radius: " << circle_radius);

    // calculate the start and end angles of the arc
    Eigen::Vector3d start_vector = (p1 - circle_center).normalized();
    Eigen::Vector3d end_vector = (p3 - circle_center).normalized();
    double start_angle = atan2(start_vector.y(), start_vector.x());
    double end_angle = atan2(end_vector.y(), end_vector.x());

    if(start_angle > end_angle){
        end_angle += 2 * M_PI;
    }

    ROS_INFO_STREAM("start_angle: " << start_angle);
    ROS_INFO_STREAM("end_angle: " << end_angle);

    // generate the points on the arc
    std::vector<Eigen::Vector3d> arc_points;
    for (double angle = start_angle; angle <= end_angle; angle += ds / circle_radius) {
        Eigen::Vector3d point = circle_center + circle_radius * Eigen::Vector3d(cos(angle), sin(angle), 0);
        arc_points.push_back(point);
    }

    return arc_points;
}


std::pair<Eigen::VectorXd, bool> compute_and_send_arc_trajectory(
    pinocchio::Model model, Eigen::VectorXd q0, Eigen::Isometry3d ti, 
    Eigen::Isometry3d tf, float time, float ds, ros::Publisher publisher)
{
    ROS_INFO_STREAM("ti: " << ti.translation().transpose() << " tf: " << tf.translation().transpose());

    // compute intermediate point of the arc
    Eigen::Isometry3d tmiddle;
    tmiddle.translation() = Eigen::Vector3d(0.0, 0.4, (ti.translation()(2)+tf.translation()(2))/2.0);
    // rotation of tmiddle is half way between ti and tf
    Eigen::Quaterniond q1(ti.rotation());
    Eigen::Quaterniond q2(tf.rotation());
    tmiddle.linear() = q1.slerp(0.5, q2).toRotationMatrix();
    ROS_INFO_STREAM("tmiddle: " << tmiddle.translation().transpose());
 
    // compute the normal of the plane containing the arc
    Eigen::Vector3d v1 = tmiddle.translation() - ti.translation();
    Eigen::Vector3d v2 = tf.translation() - ti.translation();
    Eigen::Vector3d normal = v1.cross(v2);

    // compute the rotation matrices to rotate the normal to the z axis and vice versa
    Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), normal);
    Eigen::Matrix3d xy_to_plane = rotation.toRotationMatrix();
    // rotation matrix to rotate a point on the plane with the points of the trajectory to the xy plane
    Eigen::Matrix3d plane_to_xy = xy_to_plane.inverse();
    ROS_INFO_STREAM("xy_to_plane: " << xy_to_plane);
    ROS_INFO_STREAM("plane_to_xy: " << plane_to_xy);

    // generate the points on the arc
    std::vector<Eigen::Vector3d> arc_points = compute_arc_points(
        plane_to_xy*ti.translation(), plane_to_xy*tmiddle.translation(), plane_to_xy*tf.translation(), ds);
    ROS_INFO_STREAM("computed " << arc_points.size() << " points on the arc");
    
    // compute joint angles to send
    std::vector<Eigen::VectorXd> joint_positions = std::vector<Eigen::VectorXd>();
    Eigen::VectorXd prevq = q0;
    int i=0;
    for (Eigen::Vector3d point : arc_points) {
        ROS_INFO_STREAM("point on xy plane: " << point.transpose());
        // rotate the point to the normal plane
        Eigen::Vector3d rotated_point = xy_to_plane * point;
        ROS_INFO_STREAM("point on normal plane: " << rotated_point.transpose());
        // compute the inverse kinematics, use a low precision for all the steps but the last one
        float eps = i==arc_points.size()-1 ? 1e-6 : 1e-4;
        std::pair<Eigen::VectorXd, bool> ik = inverse_kinematics(model, rotated_point, Eigen::Vector3d(0, -1.57, 1.57), prevq, eps, false);
        if (!ik.second) {
            ROS_ERROR("Inverse kinematics failed at point (%f, %f, %f)", rotated_point.x(), rotated_point.y(), rotated_point.z());
            return std::make_pair(Eigen::VectorXd(), false);
        }
        joint_positions.push_back(ik.first);
        prevq = ik.first;
        i++;
    }

    ros::Rate rate = ros::Rate(time/joint_positions.size());

    for(Eigen::VectorXd position : joint_positions){
        send_arm_joint_angles(position, publisher);
        rate.sleep();
    }

    return std::make_pair(joint_positions.back(), true);
}
