#pragma once

#include <vector>
#include <Eigen/Core>

struct Point3D {
    double x;
    double y;
    double z;
};

typedef std::vector<std::pair<Point3D, Eigen::VectorXd>> Cache;

Cache parse_cache(const std::vector<std::string>& filepaths);

Eigen::VectorXd find_closest(const Cache& data, double X, double Y, double Z);