#ifndef GEOMETRY_BUNDLE_ADJUSTMENT_H
#define GEOMETRY_BUNDLE_ADJUSTMENT_H

#include <vector>
#include <unordered_map>
#include <memory>

#include "glog/logging.h"

#include "Eigen/Core"
#include "Eigen/Dense"

#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/local_parameterization.h>
#include <ceres/loss_function.h>
#include <ceres/autodiff_cost_function.h>

#include "openMVG/sfm/sfm.hpp"
#include "openMVG/sfm/sfm_data.hpp"

using namespace openMVG::sfm;

namespace GraphSfM {
namespace geometry {

struct PinholeCameraCostFunctor
{
    double observed_x;
    double observed_y;

    PinholeCameraCostFunctor(double x, double y) : observed_x(x), observed_y(y) {}

    template <typename T>
    bool operator()(const T* const intrinsic,
                    const T* const pose,
                    const T* const X,
                    T* residual) const
    {
        // pose[0,1,2] are the angle-axis rotation
        T p[3];
        ceres::AngleAxisRotatePoint(pose, X, p);

        // pose[3,4,5] are the translation
        p[0] += pose[3];
        p[1] += pose[4];
        p[2] += pose[5];

        // transform point to image plane
        const T xp = p[0] / p[2];
        const T yp = p[1] / p[2];

        // apply intrinsic parameters
        const T focal = intrinsic[0];
        const T px = intrinsic[1];
        const T py = intrinsic[2];

        // apply focal length and principal point to get the final image coordiantes
        residual[0] = focal * xp + px - observed_x;
        residual[1] = focal * yp + py - observed_y;
        
        return true;
    }

    static ceres::CostFunction* Create(double x, double y)
    {
        // <CostFunctor, num_resiudals, num_params0, num_params1, num_params2>
        return (new ceres::AutoDiffCostFunction<PinholeCameraCostFunctor, 2, 3, 6, 3>(
                    new PinholeCameraCostFunctor(x, y)));
    }
};


struct PinholeCameraRadialK1CostFunctor
{
    double observed_x;
    double observed_y;

    PinholeCameraRadialK1CostFunctor(double x, double y) : observed_x(x), observed_y(y) {}

    template <typename T>
    bool operator()(const T* const intrinsic, 
                    const T* const pose,
                    const T* const X,
                    T* residual) const
    {
        // pose[0,1,2] are the angle-axis rotation
        T p[3];
        ceres::AngleAxisRotatePoint(pose, X, p);

        // pose[3,4,5] are the translation
        p[0] += pose[3];
        p[1] += pose[4];
        p[2] += pose[5];

        // transform point to image plane
        const T xp = p[0] / p[2];
        const T yp = p[1] / p[2];

        // apply intrinsic parameters
        const T& focal = intrinsic[0];
        const T& px = intrinsic[1];
        const T& py = intrinsic[2];
        const T& k1 = intrinsic[3];

        const T r = xp * xp + yp * yp;
        const T distortion = 1.0 + k1 * r;

        residual[0] = focal * xp * distortion + px - observed_x;
        residual[1] = focal * yp * distortion + py - observed_y;

        return true;
    }

    static ceres::CostFunction* Create(double x, double y)
    {
        return (new ceres::AutoDiffCostFunction<PinholeCameraRadialK1CostFunctor, 2, 4, 6, 3>(
                    new PinholeCameraRadialK1CostFunctor(x, y)));
    }
};

struct PinholeCameraRadialK3CostFunctor
{
    double observed_x;
    double observed_y;

    PinholeCameraRadialK3CostFunctor(double x, double y) : observed_x(x), observed_y(y) { }

    template <typename T>
    bool operator()(const T* const intrinsic,
                    const T* const pose,
                    const T* const X,
                    T* residual) const
    {
        // pose[0,1,2] are the angle-axis rotation
        T p[3];
        ceres::AngleAxisRotatePoint(pose, X, p);

        // pose[3,4,5] are the translation
        p[0] += pose[3];
        p[1] += pose[4];
        p[2] += pose[5];

        // transform point to image plane
        const T xp = p[0] / p[2];
        const T yp = p[1] / p[2];

        // apply intrinsic parameters
        const T& focal = intrinsic[0];
        const T& px = intrinsic[1];
        const T& py = intrinsic[2];
        const T& k1 = intrinsic[3];
        const T& k2 = intrinsic[4];
        const T& k3 = intrinsic[5];

        const T r = xp * xp + yp * yp;
        const T distortion = 1.0 + r * (k1 + k2 * r + k3 * r * r);

        Eigen::Map<Eigen::Matrix<T, 2, 1>> error(residual);
        error << focal * xp * distortion + px - observed_x,
                 focal * yp * distortion + py - observed_y;

        return true;
    }

    static ceres::CostFunction* Create(double x, double y) 
    {
        return (new ceres::AutoDiffCostFunction<PinholeCameraRadialK3CostFunctor, 2, 6, 6, 3>(
                    new PinholeCameraRadialK3CostFunctor(x, y)));
    }
};

struct PinholeCameraBrownT2CostFunctor
{
    double observed_x;
    double observed_y;

    PinholeCameraBrownT2CostFunctor(double x, double y) : observed_x(x), observed_y(y) { }

    template <typename T>
    bool operator()(const T* const intrinsic,
                    const T* const pose,
                    const T* const X,
                    T* residual) const
    {
        // pose[0,1,2] are the angle-axis rotation
        T p[3];
        ceres::AngleAxisRotatePoint(pose, X, p);

        // pose[3,4,5] are the translation
        p[0] += pose[3];
        p[1] += pose[4];
        p[2] += pose[5];

        // transform point to image plane
        const T xp = p[0] / p[2];
        const T yp = p[1] / p[2];

        // apply intrinsic parameters
        const T& focal = intrinsic[0];
        const T& px = intrinsic[1];
        const T& py = intrinsic[2];
        const T& k1 = intrinsic[3];
        const T& k2 = intrinsic[4];
        const T& k3 = intrinsic[5];
        const T& t1 = intrinsic[6];
        const T& t2 = intrinsic[7];

        const T r = xp * xp + yp * yp;
        const T distortion = 1.0 + r * (k1 + k2 * r + k3 * r * r);

        const T t_x = 2.0 * t1 * xp * yp + t2 * (r + 2.0 * xp * xp);
        const T t_y = 2.0 * t2 * xp * yp + t1 * (r + 2.0 * yp * yp);

        residual[0] = focal * (xp * distortion + t_x) + px - observed_x;
        residual[1] = focal * (yp * distortion + t_y) + py - observed_y;

        return true;
    }

    static ceres::CostFunction* Create(double x, double y)
    {
        return (new ceres::AutoDiffCostFunction<PinholeCameraBrownT2CostFunctor, 2, 8, 6, 3>(
                    new PinholeCameraBrownT2CostFunctor(x, y)));
    }
};

struct RefineOption
{
    bool refine_intrinsics;
    bool refine_rotations;
    bool refine_translations;
    bool refine_structures;

    RefineOption() { }

    RefineOption(bool ri = false, bool rr = false, bool rt = false, bool rs = false)
    {
        refine_intrinsics = ri;
        refine_rotations = rr;
        refine_translations = rt;
        refine_structures = rs;
    }

    RefineOption(const RefineOption& ba_option)
    {
        refine_intrinsics = ba_option.refine_intrinsics;
        refine_rotations = ba_option.refine_rotations;
        refine_translations = ba_option.refine_translations;
        refine_structures = ba_option.refine_structures;
    }
};

enum LossFunctionType 
{ HUBER, SOFTLONE, CAUCHY, ARCTAN, TUKEY};

struct BAOption
{
    // As bundle adjustment uses Levenberg-Marquardt algorithm to sovle the 
    // least square problem, which is a default value in 
    // ceres-solver(trust_region_strategy_type), thus we don't need to set 
    // this value.

    // Number of threads used by Ceres for evaluating the cost and
    // jacobians.
    int num_threads;

    // Number of threads used by Ceres to solve the Newton
    // step. Currently only the SPARSE_SCHUR solver is capable of
    // using this setting.
    int num_linear_solver_threads;

    // the type of loss function binding with ceres-solver
    //  - Huber(Loss)
    //  - SoftLOne(Loss)
    //  - Cauchy(Loss)
    //  - Arctan(Loss)
    //  - Tolerant(Loss)
    //  - Tukey(Loss)
    int loss_function_type;

    ceres::LinearSolverType linear_solver_type;

    // Ceres supports using multiple sparse linear algebra libraries
    // for sparse matrix ordering and factorizations. Currently,
    // SUITE_SPARSE and CX_SPARSE are the valid choices, depending on
    // whether they are linked into Ceres at build time.
    ceres::SparseLinearAlgebraLibraryType sparse_linear_algebra_library_type;

    // Type of preconditioner to use with the iterative linear solvers.
    ceres::PreconditionerType preconditioner_type;

    ceres::LoggingType logging_type;

    bool minimizer_progress_to_stdout;

    BAOption()
    {
        num_threads = 1;
        num_linear_solver_threads = 1;
        loss_function_type = 0;
        preconditioner_type = ceres::JACOBI;
        logging_type = ceres::LoggingType::SILENT;
        minimizer_progress_to_stdout = true;
    }
};

class BundleAdjuster
{
private:
    BAOption _ba_option;
    std::unordered_map<size_t, std::vector<double>> _poses_parameters;
    std::unordered_map<size_t, std::vector<double>> _intrinsics_parameters;

public:
    // BundleAdjuster();
    BundleAdjuster(const BAOption& ba_option);
    ~BundleAdjuster();

    void Refine(SfM_Data& sfm_data, const RefineOption& refine_option);

    void SetPoseParameterBlock(const SfM_Data& sfm_data, 
                               ceres::Problem* problem, 
                               bool refine_rotations = true,
                               bool refine_translations = true);
    void SetIntrinsicParameterBlock(const SfM_Data& sfm_data, 
                                    ceres::Problem* problem,
                                    bool refine_intrisics = false);
    void SetStructureParameterBlock(const SfM_Data& sfm_data, 
                                    ceres::Problem* problem,
                                    bool refine_structures = true);

private:
    ceres::LossFunction* CreateLossFunction(double a);
    ceres::CostFunction* CreateCostFunction(const openMVG::cameras::EINTRINSIC& cam_type,
                                            const Eigen::Vector2d& x);
};




}   // namespace geometry
}   // namespace GraphSfM

#endif