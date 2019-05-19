#ifndef GEOMETRY_ALIGN_ROTATIONS_H
#define GEOMETRY_ALIGN_ROTATIONS_H

#include <eigen3/Eigen/Core>
#include <vector>

namespace GraphSfM {

// Rotates the "rotation" set of orientations such that the orientations are
// most closely aligned in an L2 sense. That is, "rotation" is transformed such
// that R_rotation * R_gt_rotation^t is minimized.
void AlignRotations(const std::vector<Eigen::Vector3d>& gt_rotation,
                    std::vector<Eigen::Vector3d>* rotation);

}  // namespace GraphSfM

#endif 
