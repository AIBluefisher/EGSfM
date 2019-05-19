#ifndef GEOMETRY_SFM_OPTIMIZER_H
#define GEOMETRY_SFM_OPTIMIZER_H

#include <iostream>
#include <memory>
#include <unordered_map>
#include <utility>
#include <string>

#include "glog/logging.h"

#include "Eigen/Core"
#include "Eigen/Dense"

#include <ceres/types.h>
#include <ceres/rotation.h>

#include "solvers/prosac.h"
#include "solvers/estimator.h"
#include "bundle_adjustment.h"

#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/multiview/triangulation.hpp"
#include "openMVG/multiview/triangulation_nview.hpp"
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_BA.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_filters.hpp"
#include "openMVG/sfm/pipelines/sfm_engine.hpp"


using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::geometry;

namespace GraphSfM {
namespace geometry {

struct TriData
{
    Eigen::Vector3d x;
    Eigen::Vector3d x_ud;   // undistorted coordinate
    Eigen::Vector3d K_x;
    openMVG::geometry::Pose3 pose;

    TriData()
    {

    }

    ~TriData()
    {

    }

    TriData(const Eigen::Vector3d& tx,
            const Eigen::Vector3d& tx_ud,
            const Eigen::Vector3d& tK_x)
    {
        x = Eigen::Vector3d::Zero();
        x_ud = Eigen::Vector3d::Zero();
        K_x = Eigen::Vector3d::Identity();
        for (int i = 0; i < 3; i++) {
            x[i] = tx[i];
            x_ud[i] = tx_ud[i];
            K_x[i] = tK_x[i];
        }
    }

    TriData& operator=(const TriData& tri_data)
    {
        this->x = Eigen::Vector3d::Zero();
        this->x_ud = Eigen::Vector3d::Zero();
        this->K_x = Eigen::Vector3d::Identity();
        for (int i = 0; i < 3; i++) {
            this->x[i] = tri_data.x[i];
            this->x_ud[i] = tri_data.x_ud[i];
            this->K_x[i] = tri_data.K_x[i];
        }
        this->pose = tri_data.pose;
        return *this;
    }
};

class TriangulationEstimator : public Estimator<TriData, Eigen::Vector4d>
{
public:
    TriangulationEstimator() {}
    ~TriangulationEstimator() {}

    double SampleSize() const { return 2; }

    bool EstimateModel(const std::vector<TriData>& data, 
                       std::vector<Eigen::Vector4d>* models) const
    {
        Eigen::Vector4d model = Eigen::Vector4d::Zero();
        
        auto pose0 = data[0].pose, pose1 = data[1].pose;
        Eigen::Vector3d x0_ud = data[0].x_ud, x1_ud = data[1].x_ud;
        Eigen::Vector3d x0 = data[0].x, x1 = data[1].x;
        Eigen::Matrix<double, 3, 4> P0 = pose0.asMatrix(), P1 = pose1.asMatrix();
        
        openMVG::TriangulateDLT(P0, x0_ud, P1, x1_ud, &model);
        
        const Eigen::Vector3d ray0 = (pose0.rotation().transpose() * data[0].K_x).normalized(),
                              ray1 = (pose1.rotation().transpose() * data[1].K_x).normalized();
        const double mag = ray0.norm() * ray1.norm();
        const double dot_angle = ray0.dot(ray1);
        double angle = openMVG::R2D(std::acos(openMVG::clamp(dot_angle / mag, -1.0 + 1.e-8, 1.0 - 1.e-8)));
        
        // Check triangulation result
        using namespace openMVG::cameras;
        if (angle > 2.0 && // check angle (small angle leads to imprecise triangulation)
            CheiralityTest(x0_ud, pose0, x1_ud, pose1, model.hnormalized())) // && // check positive depth
            //residual0.norm() < 4.0 && residual1.norm() < 4.0) 
        {
            models->push_back(model);
            return true;
        } else { return false; }
    }

    double Error(const TriData& tri_data, const Eigen::Vector4d& X) const
    {
        Eigen::Matrix<double, 3, 4> P = tri_data.pose.asMatrix();
        Eigen::Vector3d x = tri_data.x;

        Eigen::Vector3d reprojected_x = P * X;
        double error = (reprojected_x.hnormalized() - x.hnormalized()).norm();
        return error;
    }
}; 

struct SfMOptimizerOption
{
    std::string features_dir;
    std::string tracks_filename;
    size_t minimum_visible_views;  // 3(default)

    SfMOptimizerOption() { }

    SfMOptimizerOption(const SfMOptimizerOption& option)
    {
        features_dir = option.features_dir;
        tracks_filename = option.tracks_filename;
        minimum_visible_views = option.minimum_visible_views;
    }
};

class SfMOptimizer
{
private:
    std::shared_ptr<openMVG::sfm::Features_Provider> _features_provider;
    SfMOptimizerOption _optimizer_option;

public:
    SfMOptimizer();
    ~SfMOptimizer();

    void SetOptimizerOption(const SfMOptimizerOption& option);

    bool LoadFeatures(const openMVG::sfm::SfM_Data& sfm_data);

    bool TriangulateRansac(openMVG::sfm::SfM_Data& sfm_data);

    bool TriangulateTrack(const openMVG::sfm::SfM_Data& sfm_data,
                          const openMVG::tracks::submapTrack& track,
                          Eigen::Vector3d& X,
                          openMVG::sfm::Observations& obs);
    
    size_t RemoveTracksWithLargeResidual(openMVG::sfm::SfM_Data& sfm_data,
                                        const double thresh,
                                        const size_t min_track_length);
    size_t RemoveTracksWithSmallAngle(openMVG::sfm::SfM_Data& sfm_data, 
                                      const double min_accepted_angle);

    bool BadTrackRejector(openMVG::sfm::SfM_Data& sfm_data, double precision, size_t count);

    bool BundleAdjustment(openMVG::sfm::SfM_Data& sfm_data, const RefineOption& refine_option);

    bool RotationAveraging();

    bool TranslationAveraging();
};

}   // namespace geometry
}   // namespace GraphSfM

#endif