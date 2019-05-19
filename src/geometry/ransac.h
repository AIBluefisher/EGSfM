// BSD 3-Clause License

#ifndef GEOMETRY_RANSAC_H
#define GEOMETRY_RANSAC_H

#include <vector>
#include <cmath>

#include "Eigen/Core"
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

namespace GraphSfM {
namespace geometry {

inline int GetSampleCount(int s, double ol_ratio, double p = 0.99)
{
    return std::log(1 - p) / std::log(1 - pow(1 - ol_ratio, (double)s));
}

}   // namespace geometry
}   // namespace GraphSfM

#endif
