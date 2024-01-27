#include <Eigen/Eigen>

struct ArcTrajectory{
    Eigen::Vector3d start_pos;
    Eigen::Vector3d start_rot;
    Eigen::Vector3d end_pos;
    Eigen::Vector3d end_rot;
    float time;
    float ds;
};