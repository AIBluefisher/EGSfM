#include "sfm_optimizer.h"
#include "bundle_adjustment.h"
#include "omp.h"

namespace GraphSfM {
namespace geometry {

SfMOptimizer::SfMOptimizer()
{
    _features_provider = std::make_shared<openMVG::sfm::Features_Provider>();
}

SfMOptimizer::~SfMOptimizer()
{

}

void SfMOptimizer::SetOptimizerOption(const SfMOptimizerOption& option)
{
    _optimizer_option = option;
}

bool SfMOptimizer::LoadFeatures(const openMVG::sfm::SfM_Data& sfm_data)
{
    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    const std::string image_describer = 
        stlplus::create_filespec(_optimizer_option.features_dir, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(image_describer);
    if (!regions_type) {
        LOG(ERROR) << "Invalid: " << image_describer << " regions type file.";
        return false;
    }

    // features reading
    if (!_features_provider->load(sfm_data, _optimizer_option.features_dir, regions_type)) {
        LOG(ERROR) << "Invalid features.";
        return false;
    } 
}

bool SfMOptimizer::TriangulateRansac(openMVG::sfm::SfM_Data& sfm_data)
{
    const openMVG::sfm::Poses poses = sfm_data.poses;

    // Recover tracks
    LOG(INFO) << "Loading tracks...";
    openMVG::tracks::STLMAPTracks tracks;
    openMVG::tracks::TracksBuilder tracks_builder;
    if (!tracks_builder.LoadFromFile(_optimizer_option.tracks_filename, tracks)) {
        LOG(ERROR) << "Can't load tracks! Please specify the correct filename!";
        return false;
    }
    LOG(INFO) << "tracks size: " << tracks.size();

    // data structure transform
    // set visible tracks for all calibrated images
    // { image_id, feature_id, track_id }
    std::unordered_map<size_t, std::unordered_map<size_t, size_t>> images_visible_tracks;
    // { track_id, occurence_nums }
    // std::unordered_map<size_t, size_t> tracks_effv;
    std::vector<size_t> tracks_effv(tracks.size(), 0);
    // #pragma omp parallel for
    for (auto track_it = tracks.begin(); track_it != tracks.end(); ++track_it) {
        size_t track_id = track_it->first;
        openMVG::tracks::submapTrack submap_track = track_it->second;
        for (auto strack_it = submap_track.begin(); strack_it != submap_track.end(); ++strack_it) {
            size_t image_id = strack_it->first, feature_id = strack_it->second;
            if (poses.find(image_id) != poses.end()) {
                // #pragma omp critical
                // {
                    images_visible_tracks[image_id].insert(std::make_pair(feature_id, track_id));
                    // Find effective tracks could be triangulated
                    tracks_effv[track_id]++;
                // }
            }
        }
    }

    // std::unordered_map<size_t, double> track_error;
    size_t extended_track = 0;
    std::unordered_map<size_t, bool> track_valid;
    std::unordered_map<size_t, Eigen::Vector3d> scene_X; 

    #pragma omp parallel for
    for (size_t i = 0; i < tracks_effv.size(); i++) {
    // for (auto it = tracks_effv.begin(); it != tracks_effv.end(); ++it) {
        size_t track_id = i;
        size_t visible_views_num = tracks_effv[i];
        
        if (visible_views_num >= _optimizer_option.minimum_visible_views &&
            sfm_data.structure.count(track_id) == 0) {
            #pragma omp critical
            {                
                Eigen::Vector3d X = Eigen::Vector3d::Zero();
                openMVG::sfm::Observations obs;
                openMVG::tracks::submapTrack track = tracks[track_id];
                if (TriangulateTrack(sfm_data, track, X, obs)) {
                        track_valid[track_id] = true;
                        scene_X[track_id] = X;
                        // track_error[track_id] = error_max;
                        sfm_data.structure[track_id].obs = std::move(obs);
                        sfm_data.structure[track_id].X = X;
                        extended_track++;
                }
            }
        }
    }
    LOG(INFO) << "Track extended: " << extended_track;
    return true;
}

bool SfMOptimizer::TriangulateTrack(const openMVG::sfm::SfM_Data& sfm_data,
                                    const openMVG::tracks::submapTrack& track,
                                    Eigen::Vector3d& X,
                                    openMVG::sfm::Observations& obs)
{
    // LOG(INFO) << "TriangulateTrack";
    const openMVG::sfm::Poses poses = sfm_data.GetPoses();
    const openMVG::sfm::Intrinsics intrinsics = sfm_data.GetIntrinsics();

    // Compute the effective number of calibrated cameras
    size_t num_calibrated_cameras = 0;
    for (auto track_it = track.begin(); track_it != track.end(); ++track_it) {
        size_t image_id = track_it->first;
        if (poses.find(image_id) != poses.end()) {
            num_calibrated_cameras++;
        }
    }
    if (num_calibrated_cameras < _optimizer_option.minimum_visible_views) return false;
    
    // std::vector<Eigen::Matrix<double, 3, 4>> Ps;
    // Eigen::Matrix3Xd xs;
    std::vector<TriData> data;
    std::vector<size_t> feats_id, views_id;
    std::vector<Eigen::Vector2d> xs;
    // int idx = 0;
    for (auto track_it = track.begin(); track_it != track.end(); ++track_it) {
        // Collect projection matrices and points together
        size_t view_id = track_it->first;
        // LOG(INFO) << view_id;
        if (sfm_data.views.find(view_id) == sfm_data.views.end()) continue;
        const openMVG::sfm::View* view = sfm_data.views.at(view_id).get();
        const openMVG::geometry::Pose3 pose = sfm_data.GetPoseOrDie(view);
        const openMVG::cameras::IntrinsicBase* intrinsic = 
                                sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
        // Eigen::MatrixXd P = intrinsic->get_projective_equivalent(pose);
        Eigen::Matrix<double, 3, 4> P = pose.asMatrix();
        // Ps.push_back(P);

        size_t feature_id = track_it->second;
        const Eigen::Vector2d x = 
            _features_provider->feats_per_view[view_id][feature_id].coords().cast<double>();

        // xs.col(Ps.size() - 1) = (intrinsic->get_ud_pixel(x)).homogeneous();
        Eigen::Vector2d x_ud = intrinsic->get_ud_pixel(x);
        Eigen::Vector3d hx = x.homogeneous(), hx_ud = x_ud.homogeneous();
        Eigen::Vector3d K_x = intrinsic->operator()(x);
        // hx[0] = ux[0]; hx[1] = ux[1]; hx[2] = 1.0;
        
        TriData tri_data(hx, hx_ud, K_x);
        tri_data.pose = pose;
        views_id.push_back(view_id);
        feats_id.push_back(feature_id);
        xs.push_back(x);
        data.push_back(tri_data);
    }

    Eigen::Vector4d hX = Eigen::Vector4d::Zero(); // estimated 3d points in homogeneous coordinate
    // openMVG::TriangulateNView(xs, Ps, &hX);
    TriangulationEstimator tri_estimator;
    RansacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>((unsigned int)time(NULL));
    params.error_thresh = 2.0;
    params.max_iterations = 1000; 

    Prosac<TriangulationEstimator> prosac_estimator(params, tri_estimator);
    prosac_estimator.Initialize();
    RansacSummary summary;
    prosac_estimator.Estimate(data, &hX, &summary);
    std::vector<int> inlier_indeces = summary.inliers;
    
    if (inlier_indeces.size() < 2) return false;

    // fill datas
    for (auto index : inlier_indeces) {
        openMVG::sfm::Observation ob;
        ob.x = xs[index];
        ob.id_feat = feats_id[index];
        obs[views_id[index]] = ob;
    }
    X = hX.hnormalized();

    LOG(INFO) << "Triangulate structures with " << inlier_indeces.size() << "inliers";
    LOG(INFO) << "Total iterations: " << summary.num_iterations;

    return true;
}

size_t SfMOptimizer::RemoveTracksWithLargeResidual(openMVG::sfm::SfM_Data& sfm_data,
                                        const double thresh,
                                        const size_t min_track_length)
{
    size_t outlier_count = 0;
    Landmarks::iterator it_tracks = sfm_data.structure.begin();
    while (it_tracks != sfm_data.structure.end()) {
        Observations& obs = it_tracks->second.obs;
        Observations::iterator it_obs = obs.begin();
        while (it_obs != obs.end()) {
            const View* view = sfm_data.views.at(it_obs->first).get();
            const openMVG::geometry::Pose3 pose = sfm_data.GetPoseOrDie(view);
            const openMVG::cameras::IntrinsicBase* intrinsic = 
                    sfm_data.intrinsics.at(view->id_intrinsic).get();
            const openMVG::Vec2 residual = 
                    intrinsic->residual(pose(it_tracks->second.X), it_obs->second.x);
            if (residual.norm() > thresh) {
                ++outlier_count;
                it_obs = obs.erase(it_obs);
            } else { ++it_obs; }
        }
        if (obs.empty() || obs.size() < min_track_length) {
            it_tracks = sfm_data.structure.erase(it_tracks);
        } else { ++it_tracks; }
    }
    return outlier_count;
}
size_t SfMOptimizer::RemoveTracksWithSmallAngle(openMVG::sfm::SfM_Data& sfm_data, 
                                      const double min_accepted_angle)
{
    size_t removed_track_num = 0;
    Landmarks::iterator iter_tracks = sfm_data.structure.begin();

    while (iter_tracks != sfm_data.structure.end()) {
        Observations& obs = iter_tracks->second.obs;
        double max_angle = 0.0;
        for (Observations::const_iterator ite_obs1 = obs.begin(); ite_obs1 != obs.end(); ++ite_obs1) {
            const View* view1 = sfm_data.views.at(ite_obs1->first).get();
            const openMVG::geometry::Pose3 pose1 = sfm_data.GetPoseOrDie(view1);
            const openMVG::cameras::IntrinsicBase* intrinsic1 = 
                    sfm_data.intrinsics.at(view1->id_intrinsic).get();
            
            Observations::const_iterator ite_obs2 = ite_obs1;
            ++ite_obs2;
            for (; ite_obs2 != obs.end(); ++ite_obs2) {
                const View* view2 = sfm_data.views.at(ite_obs2->first).get();
                const openMVG::geometry::Pose3 pose2 = sfm_data.GetPoseOrDie(view2);
                const openMVG::cameras::IntrinsicBase* intrinsic2 = 
                                    sfm_data.intrinsics.at(view2->id_intrinsic).get();
                const double angle = openMVG::cameras::AngleBetweenRay(pose1, intrinsic1, 
                                                                       pose2, intrinsic2,
                                                                       ite_obs1->second.x, ite_obs2->second.x);
                max_angle = std::max(angle, max_angle);
            }
        }
        if (max_angle < min_accepted_angle) {
            iter_tracks = sfm_data.structure.erase(iter_tracks);
            removed_track_num++;
        } else { ++iter_tracks; }
    }
    return removed_track_num;
}

bool SfMOptimizer::BundleAdjustment(openMVG::sfm::SfM_Data& sfm_data, const RefineOption& refine_option)
{
    BAOption ba_option;
    if (sfm_data.GetPoses().size() > 100 &&
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

    BundleAdjuster bundle_adjuster(ba_option);
    bundle_adjuster.Refine(sfm_data, refine_option);
    return true;
}

bool SfMOptimizer::BadTrackRejector(openMVG::sfm::SfM_Data& sfm_data, double precision, size_t count) {
    // Ensure there is no remaining outliers
    const size_t large_residual_count = this->RemoveTracksWithLargeResidual(sfm_data, precision, 2);
    const size_t small_angle_count = this->RemoveTracksWithSmallAngle(sfm_data, 2.0);
    size_t total_bad_tracks = large_residual_count + small_angle_count;

    LOG(INFO) << total_bad_tracks << " tracks are removed";

    return total_bad_tracks > count;
}

bool SfMOptimizer::RotationAveraging()
{
    // TODO: (chenyu)
    return true;
}

bool SfMOptimizer::TranslationAveraging()
{
    // TODO: (chenyu)
    return true;
}

}   // namespace geometry
}   // namespace GraphSfM