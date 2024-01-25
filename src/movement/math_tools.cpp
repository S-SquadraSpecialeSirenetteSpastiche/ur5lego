#include "../include/math_tools.h"


/// @brief computes the coefficients of a fifth order polynomial, to move a joint from a position to another
/// @param tf       time to move the joint (s)
/// @param start_q  starting position
/// @param start_v  starting velocity
/// @param start_a  starting acceleration
/// @param end_q    final position
/// @param end_v    final velocity
/// @param end_a    final acceleration
/// @return a vector containing the 6 coefficients of the polynomial that describes the joint's position
Eigen::VectorXd fifthOrderPolynomialTrajectory(float tf, double start_q, double start_v, double start_a, double end_q, double end_v, double end_a){
    Eigen::MatrixXd poly_matrix(6, 6);
    poly_matrix <<  1,    0,    0,          0,            0,             0,
                    0,    1,    0,          0,            0,             0,
                    0,    0,    2,          0,            0,             0,
                    1,    tf,   pow(tf,2),  pow(tf, 3),   pow(tf, 4),    pow(tf, 5),
                    0,    1,    2*tf,       3*pow(tf,2),  4*pow(tf,3),   5*pow(tf,4),
                    0,    0,    2,          6*tf,         12*pow(tf,2),  20*pow(tf,3);

    Eigen::VectorXd poly_vector = Eigen::VectorXd(6);
    poly_vector << start_q, start_v, start_a, end_q, end_v, end_a;

    Eigen::VectorXd coefficients = poly_matrix.inverse()*poly_vector;

    return coefficients;
}


/// @brief computes the coefficients of a fifth order polynomial, to move a joint from a position to another
/// @param tf       time to move the joint (s)
/// @param start_q  starting position
/// @param end_q    final position
/// @return a vector containing the 6 coefficients of the polynomial that describes the joint's position
Eigen::VectorXd fifthOrderPolynomialTrajectory(float tf, double start_q, double end_q){
    return fifthOrderPolynomialTrajectory(tf, start_q, 0, 0, end_q, 0, 0);
}


/// @brief computes the coefficients of a third order polynomial, to move a joint from a position to another
/// @param tf       time to move the joint (s)
/// @param start_q  starting position
/// @param start_v  starting velocity
/// @param end_q    final position
/// @param end_v    final velocity
/// @return a vector containing the 4 coefficients of the polynomial that describes the joint's position
Eigen::VectorXd thirdOrderPolynomialTrajectory(float tf, double start_q, double start_v, double end_q, double end_v){
    Eigen::MatrixXd poly_matrix(4, 4);
    poly_matrix <<  1,    0,    0,          0,
                    0,    1,    0,          0,
                    1,    tf,   pow(tf,2),  pow(tf, 3),
                    0,    1,    2*tf,       3*pow(tf,2);

    Eigen::VectorXd poly_vector = Eigen::VectorXd(4);
    poly_vector << start_q, start_v, end_q, end_v;

    Eigen::VectorXd coefficients = poly_matrix.inverse()*poly_vector;

    return coefficients;
}


/// @brief computes the coefficients of a third order polynomial, to move a joint from a position to another, 
/// assuming starting and final velocity equal to 0
/// @param tf       time to move the joint (s)
/// @param start_q  starting position
/// @param end_q    final position
/// @return a vector containing the 4 coefficients of the polynomial that describes the joint's position
Eigen::VectorXd thirdOrderPolynomialTrajectory(float tf, double start_q, double end_q){
    return thirdOrderPolynomialTrajectory(tf, start_q, 0, end_q, 0);
}


/// @brief converts an orientation from a roll pitch yaw representation to a rotation matrix
/// @param rpy a vector containing the roll, pitch, and yaw angles
/// @return a matrix representig the transformation obtained by applying the given rotations on the absolute axis
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


/// @brief calculates the error between two rotation matrices
/// @param r1 the first rotation matrix
/// @param r2 the second rotation matrix
/// @return a vector containing the error between the two rotation matrices in rpy notation
Eigen::Vector3d errorInSO3(Eigen::Matrix3d r1, Eigen::Matrix3d r2){
    return r1*pinocchio::log3(r1.transpose()*r2);
}