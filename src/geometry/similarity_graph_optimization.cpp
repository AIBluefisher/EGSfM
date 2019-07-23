#include "similarity_graph_optimization.h"

#include <unordered_map>
#include <vector>

using namespace std;

namespace GraphSfM {
namespace geometry {

SimilarityGraphOptimization::SimilarityGraphOptimization()
{

}

SimilarityGraphOptimization::~SimilarityGraphOptimization()
{

}

// void SimilarityGraphOptimization::SetOptimizedOption(const BAOption& optimized_option)
// {
//     _optimized_option = optimized_option;
// }

void SimilarityGraphOptimization::SetSimilarityGraph(const SimilarityGraph& sim_graph)
{
    _sim_graph = sim_graph;
}

SimilarityGraph SimilarityGraphOptimization::GetSimilarityGraph() const
{
    return _sim_graph;
}

void SimilarityGraphOptimization::SimilarityAveraging(
            std::unordered_map<size_t, std::vector<Pose3>>& cluster_boundary_cameras)
{
    ceres::Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(openMVG::Square(4.0));

    MapSim3 map_sim3;
    size_t edge_num = 0;
    
    for (auto outer_it = _sim_graph.begin(); outer_it != _sim_graph.end(); ++outer_it) {
        size_t i = outer_it->first;
        for (auto inner_it = outer_it->second.begin(); inner_it != outer_it->second.end(); ++inner_it) {
            edge_num++;
            
            size_t j = inner_it->first;
            Sim3 sim3 = inner_it->second;
            // LOG(INFO) << sim3.s << "\n" 
            //           << sim3.R << "\n"
            //           << sim3.t;
            double angle_axis[3];

            ceres::RotationMatrixToAngleAxis(sim3.R.data(), angle_axis);

            std::vector<double> parameter_block(7);
            parameter_block[0] = angle_axis[0]; 
            parameter_block[1] = angle_axis[1]; 
            parameter_block[2] = angle_axis[2];
            parameter_block[3] = sim3.t[0];
            parameter_block[4] = sim3.t[1];
            parameter_block[5] = sim3.t[2];
            parameter_block[6] = sim3.s;

            map_sim3[i].insert(make_pair(j, parameter_block));

            std::vector<Pose3> vec_boundary_cameras1 = cluster_boundary_cameras[i];
            std::vector<Pose3> vec_boundary_cameras2 = cluster_boundary_cameras[j];

            int size = vec_boundary_cameras1.size();
            for (int k = 0; k < size; k++) {
                const Eigen::Vector3d c_ik = vec_boundary_cameras1[k].center();
                const Eigen::Vector3d c_jk = vec_boundary_cameras1[k].center();

                ceres::CostFunction* cost_function = Reprojection3DCostFunctor::Create(c_ik, c_jk);
                problem.AddResidualBlock(cost_function, loss_function, &map_sim3[i][j][0]);
            }
        }
    }

    BAOption ba_option;
    if (edge_num > 100 &&
        (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE) ||
         ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE) ||
         ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))) {
        ba_option.preconditioner_type = ceres::JACOBI;
        ba_option.linear_solver_type = ceres::SPARSE_SCHUR;
        
        if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE)) {
            ba_option.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        } else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE)) {
            ba_option.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
        } else {
            ba_option.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
        }

    } else {
        ba_option.linear_solver_type = ceres::DENSE_SCHUR;
        ba_option.sparse_linear_algebra_library_type = ceres::NO_SPARSE;
    }

    ba_option.num_threads = omp_get_max_threads();
    ba_option.num_linear_solver_threads = omp_get_max_threads();
    ba_option.loss_function_type = LossFunctionType::HUBER;
    ba_option.logging_type = ceres::SILENT;
    ba_option.minimizer_progress_to_stdout = true;

    ceres::Solver::Options options;
    options.num_threads = ba_option.num_threads;
    // options.num_linear_solver_threads = ba_option.num_linear_solver_threads;
    options.linear_solver_type = ba_option.linear_solver_type;
    options.sparse_linear_algebra_library_type = ba_option.sparse_linear_algebra_library_type;
    options.preconditioner_type = ba_option.preconditioner_type;
    options.logging_type = ba_option.logging_type;
    options.minimizer_progress_to_stdout = ba_option.minimizer_progress_to_stdout;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    LOG(INFO) << summary.BriefReport();

    if (!summary.IsSolutionUsable()) {
        LOG(ERROR) << "Similarity Averaging failed!";
    } else {
        LOG(INFO) << "Initial RMSE: " 
                  << std::sqrt(summary.initial_cost / summary.num_residuals);
        LOG(INFO) << "Final RMSE: " 
                  << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
                  << "Time: " << summary.total_time_in_seconds << "\n";
    }

    this->UpdateSimilarityGraph(map_sim3);
}

void SimilarityGraphOptimization::UpdateSimilarityGraph(const MapSim3& map_sim3)
{
    for (auto outer_it = _sim_graph.begin(); outer_it != _sim_graph.end(); ++outer_it) {
        size_t i = outer_it->first;
        for (auto inner_it = outer_it->second.begin(); inner_it != outer_it->second.end(); ++inner_it) {
            size_t j = inner_it->first;
            Sim3& sim3 = inner_it->second;

            std::vector<double> vec_sim3 = map_sim3.at(i).at(j);
            
            ceres::AngleAxisToRotationMatrix(&vec_sim3[0], sim3.R.data());

            sim3.t[0] = vec_sim3[3];
            sim3.t[1] = vec_sim3[4];
            sim3.t[2] = vec_sim3[5];
            sim3.s = vec_sim3[6];
        }
    }
}

}   // namespace geometry
}   // namespace GraphSfM
