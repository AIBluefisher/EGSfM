#include "rotation.h"

#include <ceres/rotation.h>

#include "eigen3/Eigen/Core"
// #include "eigen3/Eigen/Geometry"
#include <glog/logging.h>
#include <limits>

namespace GraphSfM {

// Use Ceres to perform a stable composition of rotations. This is not as
// efficient as directly composing angle axis vectors (see the old
// implementation commented above) but is more stable.
Eigen::Vector3d MultiplyRotations(const Eigen::Vector3d& rotation1,
                                  const Eigen::Vector3d& rotation2) 
{
    Eigen::Matrix3d rotation1_mat, rotation2_mat;
    ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation1_mat.data());
    ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation2_mat.data());

    const Eigen::Matrix3d rotation = rotation1_mat * rotation2_mat;
    Eigen::Vector3d rotation_aa;
    ceres::RotationMatrixToAngleAxis(rotation.data(), rotation_aa.data());
    return rotation_aa;
}

}  // namespace GraphSfM
