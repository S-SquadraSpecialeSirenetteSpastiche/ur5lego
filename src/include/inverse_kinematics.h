#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/math/rpy.hpp"
#include "math_tools.h"


std::pair<Eigen::VectorXd, bool> inverse_kinematics_wrapper(pinocchio::Model model, Eigen::Vector3d target_position, 
    Eigen::Vector3d target_rotation, Eigen::VectorXd q0);

std::pair<Eigen::VectorXd, bool> inverse_kinematics(
    pinocchio::Model model, Eigen::Vector3d target_position, Eigen::Vector3d target_orientation, Eigen::VectorXd q);

std::pair<Eigen::VectorXd, bool> inverse_kinematics_bad(
    pinocchio::Model model, Eigen::Vector3d target_position, Eigen::Vector3d target_orientation, Eigen::VectorXd q);

void myComputeAllTerms(pinocchio::Model model, pinocchio::Data data, Eigen::VectorXd q);