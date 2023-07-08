#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "math_tools.h"


void myComputeAllTerms(pinocchio::Model model, pinocchio::Data data, Eigen::VectorXd q);

Eigen::Matrix3d euler_to_rotation_matrix(Eigen::Vector3d rpy);

std::pair<Eigen::VectorXd, bool> inverse_kinematics(
    pinocchio::Model model, Eigen::Vector3d target_position, Eigen::Vector3d target_orientation, Eigen::VectorXd q);

std::pair<Eigen::VectorXd, bool> inverse_kinematics_bad(
    pinocchio::Model model, Eigen::Vector3d target_position, Eigen::Vector3d target_orientation, Eigen::VectorXd q);