#include "Eigen/Eigen"
#include "pinocchio/spatial/explog.hpp"
#include <cmath>
#include <iostream>


Eigen::VectorXd fifthOrderPolynomialTrajectory(float tf, double start_q, double end_q);
Eigen::VectorXd fifthOrderPolynomialTrajectory(float tf, double start_q, double start_v, double start_a, double end_q, double end_v, double end_a);
Eigen::VectorXd thirdOrderPolynomialTrajectory(float tf, double start_q, double end_q);
Eigen::VectorXd fifthOrderPolynomialTrajectory(float tf, double start_q, double start_v, double end_q, double end_v);
Eigen::Matrix3d euler_to_rotation_matrix(Eigen::Vector3d rpy);
Eigen::Vector3d errorInSO3(Eigen::Matrix3d r1, Eigen::Matrix3d r2);