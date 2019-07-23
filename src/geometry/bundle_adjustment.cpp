#include "bundle_adjustment.h"

namespace GraphSfM {
namespace geometry {

// BundleAdjuster::BundleAdjuster() { }

BundleAdjuster::BundleAdjuster(const BAOption& ba_option)
{
    _ba_option = ba_option;
}

BundleAdjuster::~BundleAdjuster() { }

void BundleAdjuster::Refine(SfM_Data& sfm_data, const RefineOption& refine_option)
{
    ceres::Problem problem;
  
    this->SetPoseParameterBlock(sfm_data, &problem, 
                                refine_option.refine_rotations, 
                                refine_option.refine_translations);
    this->SetIntrinsicParameterBlock(sfm_data, &problem, refine_option.refine_intrinsics);
    // this->SetStructureParameterBlock(sfm_data, &problem, refine_option.refine_structures);

    // for (auto it = _poses_parameters.begin(); it != _poses_parameters.end(); ++it) {
    //     std::vector<double> pose = it->second;
    //     LOG(INFO) << pose[0] << " "
    //               << pose[1] << " "
    //               << pose[2] << " "
    //               << pose[3] << " "
    //               << pose[4] << " "
    //               << pose[5];
    // }
    ceres::LossFunction* loss_function = this->CreateLossFunction(openMVG::Square(4.0));
    for (auto it = sfm_data.structure.begin(); it != sfm_data.structure.end(); ++it) {
        openMVG::sfm::Landmark landmark = it->second;

        problem.AddParameterBlock(it->second.X.data(), 3);

        const openMVG::sfm::Observations& obs = it->second.obs;
        
        for (auto it_obs = obs.begin(); it_obs != obs.end(); ++it_obs) {
            size_t view_id = it_obs->first;
            
            openMVG::sfm::View* view = sfm_data.views.at(view_id).get();
            openMVG::cameras::IntrinsicBase* intrinsic = 
                        sfm_data.intrinsics.at(view->id_intrinsic).get();

            ceres::CostFunction* cost_function = 
                this->CreateCostFunction(intrinsic->getType(), it_obs->second.x);
            
            problem.AddResidualBlock(cost_function, 
                                     loss_function, 
                                     &_intrinsics_parameters[view->id_intrinsic][0],
                                     &_poses_parameters[view->id_pose][0],
                                     it->second.X.data());
        }
        if (!refine_option.refine_structures) {
            problem.SetParameterBlockConstant(it->second.X.data());
        }
    }

    ceres::Solver::Options options;
    options.num_threads = _ba_option.num_threads;
    // options.num_linear_solver_threads = _ba_option.num_linear_solver_threads;
    options.linear_solver_type = _ba_option.linear_solver_type;
    options.sparse_linear_algebra_library_type = _ba_option.sparse_linear_algebra_library_type;
    options.preconditioner_type = _ba_option.preconditioner_type;
    options.logging_type = _ba_option.logging_type;
    options.minimizer_progress_to_stdout = _ba_option.minimizer_progress_to_stdout;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    LOG(INFO) << summary.BriefReport();

    if (!summary.IsSolutionUsable()) {
        LOG(INFO) << "Bundle Adjustment failed!";
        return;
    } else {
        LOG(INFO) << "Bundle Adjustment Statistics"
                  << "Views: " << sfm_data.views.size() << "\n"
                  << "Poses: " << sfm_data.poses.size() << "\n"
                  << "Intri: " << sfm_data.intrinsics.size() << "\n"
                  << "tracks: " << sfm_data.structure.size() << "\n"
                  << "Initial RMSE: " 
                  << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
                  << "Final RMSE: " 
                  << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
                  << "Time: " << summary.total_time_in_seconds << "\n";
    }

    // update camera poses with refined data
    if (refine_option.refine_rotations || refine_option.refine_translations) {
        for (auto it = sfm_data.poses.begin(); it != sfm_data.poses.end(); ++it) {
            const size_t pose_id = it->first;
            Eigen::Matrix3d R_refined = Eigen::Matrix3d::Identity();
            ceres::AngleAxisToRotationMatrix(&_poses_parameters[pose_id][0], R_refined.data());
            Eigen::Vector3d t_refined(_poses_parameters[pose_id][3], 
                                      _poses_parameters[pose_id][4],
                                      _poses_parameters[pose_id][5]);
            openMVG::geometry::Pose3& pose = it->second;
            pose = openMVG::geometry::Pose3(R_refined, -R_refined.transpose() * t_refined);
        }
    }

    // update camera intrinsics with refined data
    // FIXME: (chenyu) segmentation here
    if (refine_option.refine_intrinsics) {
        for (auto it = sfm_data.intrinsics.begin(); it != sfm_data.intrinsics.end(); ++it) {
            const size_t cam_id = it->first;
            std::vector<double> vec_cams;
            for (int i = 0; i < sfm_data.intrinsics[cam_id]->getParams().size(); i++) {
                vec_cams[i] = _intrinsics_parameters[cam_id][i];
            }
            it->second.get()->updateFromParams(vec_cams);
        }
    }

    return;
}

void BundleAdjuster::SetPoseParameterBlock(const SfM_Data& sfm_data,
                                             ceres::Problem* problem,
                                             bool refine_rotations,
                                             bool refine_translations)
{
    auto poses = sfm_data.GetPoses();

    for (auto it = poses.begin(); it != poses.end(); ++it) {
        Eigen::Matrix3d R = it->second.rotation();
        Eigen::Vector3d t = it->second.translation();
        double angle_axis[3];
        ceres::RotationMatrixToAngleAxis(R.data(), angle_axis);

        // double pose[6];
        std::vector<double> pose(6);
        for (int i = 0; i < 3; i++) {
            pose[i] = angle_axis[i];
            pose[i + 3] = t[i];
        }

        // LOG(INFO) << pose[0] << " "
        //   << pose[1] << " "
        //   << pose[2] << " "
        //   << pose[3] << " "
        //   << pose[4] << " "
        //   << pose[5];

        _poses_parameters.insert(std::make_pair(it->first, pose));

        double* parameter_block = &pose[0];
        problem->AddParameterBlock(parameter_block, 6);

        // settings if parameters are required not refined
        if (!refine_rotations && !refine_translations) {
            problem->SetParameterBlockConstant(parameter_block);
        } else {
            std::vector<int> constant_parameters_indeces;
            if (!refine_rotations) {
                for (int i = 0; i < 3; i++) constant_parameters_indeces.push_back(i);
            }
            if (!refine_translations) {
                for (int i = 3; i < 6; i++) constant_parameters_indeces.push_back(i);
            }
            if (!constant_parameters_indeces.empty()) {
                ceres::SubsetParameterization* subset_parameter = 
                    new ceres::SubsetParameterization(6, constant_parameters_indeces);
                problem->SetParameterization(parameter_block, subset_parameter);
            }
        }
    }
}

void BundleAdjuster::SetIntrinsicParameterBlock(const SfM_Data& sfm_data,
                                                  ceres::Problem* problem,
                                                  bool refine_intrinsics)
{
    auto intrinsics = sfm_data.GetIntrinsics();
    
    for (auto it = intrinsics.begin(); it != intrinsics.end(); ++it) {
        openMVG::cameras::IntrinsicBase* intrinsic = it->second.get();

        if (openMVG::cameras::isValid(intrinsic->getType())) {
            std::vector<double> vec_intrinsic_param = intrinsic->getParams();
            const int size = vec_intrinsic_param.size();

            // double intrinsic_param[size];
            // for (int i = 0; i < size; i++) {
            //     intrinsic_param[i] = vec_intrinsic_param[i];
            // }
            double* intrinsic_param = &vec_intrinsic_param[0];

            _intrinsics_parameters[(size_t)it->first] = vec_intrinsic_param;
            problem->AddParameterBlock(intrinsic_param, size);

            if (!refine_intrinsics) {
                problem->SetParameterBlockConstant(intrinsic_param);
            }
        } else {
            LOG(ERROR) << "Unsupported camera type";
        }
    }
}

void BundleAdjuster::SetStructureParameterBlock(const SfM_Data& sfm_data,
                                                  ceres::Problem* problem,
                                                  bool refine_structures)
{
    openMVG::sfm::Landmarks structure = sfm_data.structure;
    ceres::LossFunction* loss_function = this->CreateLossFunction(openMVG::Square(4.0));
    
    for (auto it = structure.begin(); it != structure.end(); ++it) {
        openMVG::sfm::Landmark landmark = it->second;
        Eigen::Vector3d X = landmark.X;
        std::vector<double> vec_X(3);
        vec_X[0] = X[0]; vec_X[1] = X[1]; vec_X[2] = X[2];

        problem->AddParameterBlock(&vec_X[0]/*it->second.X.data()*/, 3);

        openMVG::sfm::Observations obs = it->second.obs;
        
        for (auto it_obs = obs.begin(); it_obs != obs.end(); ++it_obs) {
            size_t view_id = it_obs->first;
            Eigen::Vector2d x = it_obs->second.x;
            
            openMVG::sfm::View* view = sfm_data.views.at(view_id).get();
            openMVG::cameras::IntrinsicBase* intrinsic = 
                        sfm_data.intrinsics.at(view->id_intrinsic).get();
            openMVG::sfm::Pose3 pose = sfm_data.poses.at(view->id_pose);

            ceres::CostFunction* cost_function = 
                this->CreateCostFunction(intrinsic->getType(), it_obs->second.x);
            
            problem->AddResidualBlock(cost_function, 
                                      loss_function, 
                                      &_intrinsics_parameters[view->id_intrinsic][0],
                                      &_poses_parameters[view->id_pose][0],
                                      // it->second.X.data()
                                      &vec_X[0]
                                      );
        }
        if (!refine_structures) {
            problem->SetParameterBlockConstant(&vec_X[0]/*it->second.X.data()*/);
        }
    }
}

ceres::LossFunction* BundleAdjuster::CreateLossFunction(double a)
{
    ceres::LossFunction* loss_function;

    switch (_ba_option.loss_function_type) {
        case HUBER:
            loss_function = new ceres::HuberLoss(a);
            break;
        case SOFTLONE:
            loss_function = new ceres::SoftLOneLoss(a);
            break;
        case CAUCHY:
            loss_function = new ceres::CauchyLoss(a);
            break;
        case ARCTAN:
            loss_function = new ceres::ArctanLoss(a);
            break;
        case TUKEY:
            loss_function = new ceres::TukeyLoss(a);
            break;
        default:
            loss_function = new ceres::HuberLoss(a);
            break;
    }

    return loss_function;
}

ceres::CostFunction* BundleAdjuster::CreateCostFunction(const openMVG::cameras::EINTRINSIC& cam_type,
                                                        const Eigen::Vector2d& x)
{
    ceres::CostFunction* cost_function;

    switch (cam_type) {
        using namespace openMVG::cameras;
        case EINTRINSIC::PINHOLE_CAMERA :
            cost_function = PinholeCameraCostFunctor::Create(x[0], x[1]);
            break;
        case EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
            cost_function = PinholeCameraRadialK1CostFunctor::Create(x[0], x[1]);
            break;
        case EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
            cost_function = PinholeCameraRadialK3CostFunctor::Create(x[0], x[1]);
            break;
        case EINTRINSIC::PINHOLE_CAMERA_BROWN:
            cost_function = PinholeCameraBrownT2CostFunctor::Create(x[0], x[1]);
            break;
        default:
            cost_function = PinholeCameraCostFunctor::Create(x[0], x[1]);
            break;
    }

    return cost_function;
}

}   // namespace geometry
}   // namespace GraphSfM
