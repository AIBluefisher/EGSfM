#ifndef GEOMETRY_SIMILARITY_GRAPH_OPTIMIZATION_H
#define GEOMETRY_SIMILARITY_GRAPH_OPTIMIZATION_H

#include <vector>
#include <unordered_map>

#include "Eigen/Core"
#include "Eigen/Dense"

#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/local_parameterization.h>
#include <ceres/loss_function.h>
#include <ceres/autodiff_cost_function.h>

#include "sfm_aligner.h"
#include "bundle_adjustment.h"

namespace GraphSfM {
namespace geometry {

struct Reprojection3DCostFunctor
{
Eigen::Vector3d _point1;
Eigen::Vector3d _point2;

Reprojection3DCostFunctor(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2)
{
    _point1 = Eigen::Vector3d::Zero();
    _point2 = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; i++) {
        _point1[i] = point1[i];
        _point2[i] = point2[i];
    }
}

template <typename T>
bool operator()(const T* const sim3, T* residual) const
{
    T transformed_point[3];
    //T rotation[3];
    //rotation[0] = sim3[1]; rotation[1] = sim3[2]; rotataion[2] = sim3[3];
    T point1[3];
    point1[0] = (T)_point1[0];
    point1[1] = (T)_point1[1];
    point1[2] = (T)_point1[2];
    ceres::AngleAxisRotatePoint(sim3, point1, transformed_point);

    transformed_point[0] = sim3[6] * transformed_point[0] + sim3[3];
    transformed_point[1] = sim3[6] * transformed_point[1] + sim3[4];
    transformed_point[2] = sim3[6] * transformed_point[2] + sim3[5];

    residual[0] = transformed_point[0] - (T)_point2[0];
    residual[1] = transformed_point[1] - (T)_point2[1];
    residual[2] = transformed_point[2] - (T)_point2[2];
    
    return true;
}

static ceres::CostFunction* Create(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2)
{
    return new ceres::AutoDiffCostFunction<Reprojection3DCostFunctor, 3, 7>(
                new Reprojection3DCostFunctor(point1, point2));
}

};


class SimilarityGraphOptimization
{
using MapSim3 = std::unordered_map<size_t, std::unordered_map<size_t, std::vector<double>>>;
private:
    SimilarityGraph _sim_graph;
    // (cluster_id, boundary_cameras)
    // std::unordered_map<size_t, std::vector<Pose3>> _cluster_boundary_cameras; 

public:
    SimilarityGraphOptimization();
    ~SimilarityGraphOptimization();

    // void SetOptimizedOption(const BAOption& optimized_option);

    void SetSimilarityGraph(const SimilarityGraph& sim_graph);
    SimilarityGraph GetSimilarityGraph() const;

    void SimilarityAveraging(std::unordered_map<size_t, std::vector<Pose3>>&
                             cluster_boundary_cameras);
    
private:
    void UpdateSimilarityGraph(const MapSim3& map_sim3);

};

}   // namespace geometry
}   // namespace GraphSfM

#endif