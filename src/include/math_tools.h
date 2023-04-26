#include "Eigen/Eigen"
#include <cmath>
#include <iostream>


Eigen::VectorXd fifthOrderPolynomialTrajectory(float tf, double start_q, double end_q);
Eigen::VectorXd fifthOrderPolynomialTrajectory(float tf, double start_q, double start_v, double start_a, double end_q, double end_v, double end_a);
Eigen::Matrix3d euler_to_rotation_matrix(Eigen::Vector3d rpy);