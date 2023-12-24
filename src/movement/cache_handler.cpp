#include "../include/cache_handler.h"

#include <fstream>
#include <sstream>


Cache parse_cache(const std::string& filepath) {
    Cache cache;

    std::ifstream file(filepath);
    if (!file.is_open()) {
        return cache;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double x, y, z;
        double q0, q1, q2, q3, q4, q5;

        // If a line is broken skip it
        if (!(iss >> x >> y >> z >> q0 >> q1 >> q2 >> q3 >> q4 >> q5)) {
            continue;
        }

        Point3D point;
        point.x = x;
        point.y = y;
        point.z = z;
        Eigen::VectorXd q(6);
        q << q0, q1, q2, q3, q4, q5;

        cache.emplace_back(point, q);
    }

    file.close();

    return cache;
}


Eigen::VectorXd find_closest(const Cache& cache, double X, double Y, double Z) {
    double minDistance = std::numeric_limits<double>::max();
    Eigen::VectorXd closestQ;

    for (const auto& entry : cache) {
        double distance = std::abs(entry.first.x - X) + std::abs(entry.first.y - Y) + std::abs(entry.first.z - Z);
        if (distance < minDistance) {
            minDistance = distance;
            closestQ = entry.second;
        }
    }

    return closestQ;
}
