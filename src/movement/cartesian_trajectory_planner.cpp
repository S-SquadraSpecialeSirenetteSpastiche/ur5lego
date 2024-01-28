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
    double eps = 1e-6;
    double deltaZ12 = fabs(p1(2) - p2(2));
    double deltaZ23 = fabs(p2(2) - p3(2));
    assert(deltaZ12<eps && deltaZ23<eps);

    Eigen::Vector2d p1_2d = Eigen::Vector2d(p1(0), p1(1));
    Eigen::Vector2d p2_2d = Eigen::Vector2d(p2(0), p2(1));
    Eigen::Vector2d p3_2d = Eigen::Vector2d(p3(0), p3(1));

    double d = 2 * (p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()));

    double ux = ((p1.x() * p1.x() + p1.y() * p1.y()) * (p2.y() - p3.y()) + 
                 (p2.x() * p2.x() + p2.y() * p2.y()) * (p3.y() - p1.y()) + 
                 (p3.x() * p3.x() + p3.y() * p3.y()) * (p1.y() - p2.y())) / d;

    double uy = ((p1.x() * p1.x() + p1.y() * p1.y()) * (p3.x() - p2.x()) + 
                 (p2.x() * p2.x() + p2.y() * p2.y()) * (p1.x() - p3.x()) + 
                 (p3.x() * p3.x() + p3.y() * p3.y()) * (p2.x() - p1.x())) / d;

    Eigen::Vector2d center = Eigen::Vector2d(ux, uy);
    double radius = (center - p1_2d).norm();
    

    // Calculate start and end angles
    double start_angle = atan2(p1_2d(1) - center(1), p1_2d(0) - center(0));
    double end_angle = atan2(p3_2d(1) - center(1), p3_2d(0) - center(0));

    // Adjust angles for counterclockwise direction
    if (start_angle < 0) start_angle += 2 * M_PI;
    if (end_angle < 0) end_angle += 2 * M_PI;
    if (start_angle > end_angle) end_angle += 2 * M_PI;

    std::vector<Eigen::Vector3d> points = std::vector<Eigen::Vector3d>();
    for (double theta = start_angle; theta <= end_angle; theta += ds / radius) {
        Eigen::Vector2d point_2d = center + radius * Eigen::Vector2d(cos(theta), sin(theta));
        Eigen::Vector3d point = Eigen::Vector3d(point_2d(0), point_2d(1), p1(2));
        points.push_back(point);
    }

    return points;
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
    Eigen::Quaterniond start_rot(ti.rotation());
    Eigen::Quaterniond end_rot(tf.rotation());
    tmiddle.linear() = start_rot.slerp(0.5, end_rot).toRotationMatrix();
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

    // generate the points on the arc
    std::vector<Eigen::Vector3d> arc_points = compute_arc_points(
        plane_to_xy*ti.translation(), plane_to_xy*tmiddle.translation(), plane_to_xy*tf.translation(), ds);
    ROS_INFO_STREAM("computed " << arc_points.size() << " points on the arc");
    
    // compute joint angles to send
    std::vector<Eigen::VectorXd> joint_positions = std::vector<Eigen::VectorXd>();
    Eigen::VectorXd prevq = q0;
    for (int i=0; i<arc_points.size(); i++){
        // rotate the point to the normal plane
        Eigen::Vector3d rotated_point = xy_to_plane * arc_points[i];
        // compute the inverse kinematics
        // compute ee rotation by interpolating between ti and tf
        // Eigen::Quaterniond rot = start_rot.slerp((double)i/(double)arc_points.size(), end_rot);
        // Eigen::Vector3d rot_euler = rot.toRotationMatrix().eulerAngles(2, 1, 0);
        // std::pair<Eigen::VectorXd, bool> ik = inverse_kinematics(model, rotated_point, rot_euler, prevq);
        std::pair<Eigen::VectorXd, bool> ik = inverse_kinematics(model, rotated_point, Eigen::Vector3d(0, -1.57, 1.57), prevq);
        if (!ik.second) {
            // ROS_ERROR("Inverse kinematics failed at point (%f, %f, %f, %f, %f, %f)", 
            //     rotated_point(0), rotated_point(1), rotated_point(2), 
            //     rot_euler(0), rot_euler(1), rot_euler(2));
            return std::make_pair(Eigen::VectorXd(), false);
        }
        joint_positions.push_back(ik.first);
        prevq = ik.first;
        ROS_INFO_STREAM(i);
    }

    ros::Rate rate = ros::Rate(time/joint_positions.size());

    for(Eigen::VectorXd position : joint_positions){
        send_arm_joint_angles(position, publisher);
        rate.sleep();
    }

    return std::make_pair(joint_positions.back(), true);
}
