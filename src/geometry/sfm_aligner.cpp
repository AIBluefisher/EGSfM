#include <fstream>
#include <utility>
#include <limits>
#include <algorithm>

#include <glog/logging.h>

#include "openMVG/sfm/sfm.hpp"
#include "openMVG/geometry/rigid_transformation3D_srt.hpp"
#include "openMVG/sfm/sfm_data_BA.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_filters.hpp"
#include "openMVG/sfm/pipelines/sfm_engine.hpp"
#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/multiview/rotation_averaging.hpp"
// #include "software/SfM/SfMPlyHelper.hpp"
#include "omp.h"

#include "util/ply_helper.h"
#include "sfm_aligner.h"
#include "ransac_similarity.h"
#include "math/l1_solver.h"
#include "bundle_adjustment.h"
#include "similarity_graph_optimization.h"

using namespace std;


namespace GraphSfM {
namespace geometry {

SfMAligner::SfMAligner()
{

}

SfMAligner::~SfMAligner()
{

}

SfM_Data SfMAligner::GetGlobalSfMData() const
{
    return _sfm_data;
}

void SfMAligner::SetAlignOption(const AlignOption& ao)
{
    _align_option = ao;
}

bool SfMAligner::InitializeGraph(const std::vector<std::string>& sfm_datas_path)
{
    for (size_t i = 0; i < sfm_datas_path.size(); i++) {
        std::string path = sfm_datas_path[i];
        if (stlplus::file_exists(path)) {
            ClusterNode cluster_node(i, path);
            _cluster_graph.AddNode(cluster_node);
        } else {
            LOG(WARNING) << "file " << path << "doesn't exists";
        }
    }

    // Add edges
    vector<ClusterNode> nodes = _cluster_graph.SerializeNodes();
    // #pragma omp parallel
    // {
        // #pragma omp for
        for(int i = 0; i < nodes.size(); i++) {
            string sd_path1 = nodes[i].sfm_data_path;
            for(int j = i + 1; j < nodes.size(); j++) {
                string sd_path2 = nodes[j].sfm_data_path;
                ClusterEdge edge(i, j, 0);
                
                SfM_Data sfm_data1, sfm_data2;
                if(!Load(sfm_data1, sd_path1, ESfM_Data(ALL))) {
                    LOG(ERROR) << "The input SfM_Data file \"" << sd_path1 << "\" cannot be read.";
                    return false;
                }
                if(!Load(sfm_data2, sd_path2, ESfM_Data(ALL))) {
                    LOG(ERROR) << "The input SfM_Data file \"" << sd_path2 << "\" cannot be read.";
                    return false;
                }

                this->ConstructEdge(sfm_data1, sfm_data2, edge);
                std::cout << std::endl;

                if(edge.weight != numeric_limits<float>::max()) {
                    _cluster_graph.AddEdge(edge);

                    int pose_size = edge.src_poses.size();
                    std::vector<Pose3> vec_src_poses(pose_size), vec_dst_poses(pose_size);
                    for (int k = 0; k < pose_size; k++) {
                        vec_src_poses[k] = edge.src_poses[k].second;
                        vec_dst_poses[k] = edge.dst_poses[k].second;
                    }
                    _cluster_boundary_cameras[i] = vec_src_poses;
                    _cluster_boundary_cameras[j] = vec_dst_poses;
                }
            }
        }
    // }
    return true;
}

bool SfMAligner::InitializeGraph(const std::vector<std::string>& sfm_datas_path,
                                 const std::vector<SfM_Data>& sfm_datas)
{
    for (size_t i = 0; i < sfm_datas_path.size(); i++) {
        std::string path = sfm_datas_path[i];
        if (stlplus::file_exists(path)) {
            ClusterNode cluster_node(i, path);
            cluster_node.sfm_data = sfm_datas[i];
            _cluster_graph.AddNode(cluster_node);
        } else {
            LOG(WARNING) << "file " << path << "doesn't exists";
        }
    }

    // Add edges
    vector<ClusterNode> nodes = _cluster_graph.SerializeNodes();
    // #pragma omp parallel
    // {
        // #pragma omp for
        for(int i = 0; i < nodes.size(); i++) {
            SfM_Data sfm_data1 = nodes[i].sfm_data;
            for(int j = i + 1; j < nodes.size(); j++) {
                SfM_Data sfm_data2 = nodes[j].sfm_data;
                ClusterEdge edge(i, j, 0);
                
                this->ConstructEdge(sfm_data1, sfm_data2, edge);
                
                if(edge.weight != numeric_limits<float>::max()) {
                    _cluster_graph.AddEdge(edge);

                    int pose_size = edge.src_poses.size();
                    std::vector<Pose3> vec_src_poses(pose_size), vec_dst_poses(pose_size);
                    for (int k = 0; k < pose_size; k++) {
                        vec_src_poses[k] = edge.src_poses[k].second;
                        vec_dst_poses[k] = edge.dst_poses[k].second;
                    }
                    _cluster_boundary_cameras[i] = vec_src_poses;
                    _cluster_boundary_cameras[j] = vec_dst_poses;
                }
            }
        }
    // }
    return true;
}

void SfMAligner::ConstructEdge(const SfM_Data& sfm_data1, const SfM_Data& sfm_data2, ClusterEdge& edge)
{
    Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity(3, 3), 
                    R2 = Eigen::Matrix3d::Identity(3, 3);
    Eigen::Vector3d t1 = Eigen::Vector3d::Zero(),
                    t2 = Eigen::Vector3d::Zero();
    double s1 = 1.0, s2 = 1.0;

    if (_align_option.use_common_cameras) {
        FindCommonCameras(sfm_data1, sfm_data2, edge.src_poses, edge.dst_poses);
        if(edge.src_poses.size() < 2) edge.weight = numeric_limits<float>::max();
        else if (edge.src_poses.size() < 5) {
            ComputeSimilarityByCameraMotions(edge.src_poses, edge.dst_poses, R1, t1, s1);
            ComputeSimilarityByCameraMotions(edge.dst_poses, edge.src_poses, R2, t2, s2);
            
            std::vector<Eigen::Vector3d> centers1 = GetCameraCenters(edge.src_poses),
                                         centers2 = GetCameraCenters(edge.dst_poses);
            double msd1 = CheckReprojError(centers1, centers2, s1, R1, t1),
                   msd2 = CheckReprojError(centers2, centers1, s2, R2, t2);

            edge.weight = max(msd1, msd2);

            if (edge.weight > 0.002) edge.weight = numeric_limits<float>::max();

            if (edge.weight != numeric_limits<float>::max()) {
                _sim3_graph[edge.src][edge.dst] = Sim3(R1, t1, s1);
                _sim3_graph[edge.dst][edge.src] = Sim3(R2, t2, s2);
            }
        } else {
            vector<Eigen::Vector3d> src_cameras = edge.GetSrcCameraCenter();
            vector<Eigen::Vector3d> dst_cameras = edge.GetDstCameraCenter();
            double msd1 = 0.0, msd2 = 0.0;
            FindSimilarityTransform(src_cameras, dst_cameras, R1, t1, s1, msd1);
            FindSimilarityTransform(dst_cameras, src_cameras, R2, t2, s2, msd2);
            
            edge.weight = max(msd1, msd2);

            if (edge.weight > 0.002) edge.weight = numeric_limits<float>::max();

            if (edge.weight != numeric_limits<float>::max()) {
                _sim3_graph[edge.src][edge.dst] = Sim3(R1, t1, s1);
                _sim3_graph[edge.dst][edge.src] = Sim3(R2, t2, s2);
            }
        }
    }
    if (_align_option.use_common_structures) {
        FindCommonObservations(sfm_data1, sfm_data2, edge.src_observations, edge.dst_observations);
        if (!edge.src_observations.size()) edge.weight = numeric_limits<float>::max();
        else {
            vector<Vector3d> src_observations = edge.src_observations;
            vector<Vector3d> dst_observations = edge.dst_observations;
            double msd1 = 0.0, msd2 = 0.0;
            FindSimilarityTransform(src_observations, dst_observations, R1, t1, s1, msd1);
            FindSimilarityTransform(dst_observations, src_observations, R2, t2, s2, msd2);
            
            edge.weight = max(msd1, msd2);

            if (edge.weight > 0.002) edge.weight = numeric_limits<float>::max();

            if (edge.weight != numeric_limits<float>::max()) {
                _sim3_graph[edge.src][edge.dst] = Sim3(R1, t1, s1);
                _sim3_graph[edge.dst][edge.src] = Sim3(R2, t2, s2);
            }
        }
    }
}


void SfMAligner::UpdateGraph()
{
    // update the graph weight after similarity averaging
    auto edges = _cluster_graph.GetEdges();
    for (auto it = edges.begin(); it != edges.end(); ++it) {
        auto em = it->second;
        for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
            ClusterEdge edge = em_it->second;
            size_t src = edge.src, dst = edge.dst;
            Sim3 sim31 = _sim3_graph.at(src).at(dst);
            Sim3 sim32 = _sim3_graph.at(dst).at(src);
            
            std::vector<Eigen::Vector3d> centers1 = GetCameraCenters(edge.src_poses),
                                         centers2 = GetCameraCenters(edge.dst_poses);
            double msd1 = CheckReprojError(centers1, centers2, sim31.s, sim31.R, sim31.t);
            double msd2 = CheckReprojError(centers2, centers1, sim32.s, sim32.R, sim32.t);
            edge.weight = std::max(msd1, msd2);

            _cluster_graph.AlterEdge(edge);
        }
    }

    LOG(INFO) << "Constructing Minimum Spanning Tree";
    vector<ClusterEdge> mst_edges = _cluster_graph.Kruskal();
    LOG(INFO) << "Construct Minimum Spanning Tree complete\n";

    // Remove edges that are not exists in MST
    auto adj_maps = _cluster_graph.SerializeEdges();
    // auto adj_maps = _cluster_graph.GetEdges();
    #pragma omp parallel
    {
        #pragma omp for
        for(int i = 0; i < adj_maps.size(); i++) {
            for(auto it = adj_maps[i].begin(); it != adj_maps[i].end(); ++it) {
                #pragma omp critical
                {
                    ClusterEdge le1(it->second.src, it->second.dst);
                    ClusterEdge le2(it->second.dst, it->second.src);
                    if(find(mst_edges.begin(), mst_edges.end(), le1) == mst_edges.end() &&
                       find(mst_edges.begin(), mst_edges.end(), le2) == mst_edges.end()) {
                        LOG(INFO) << "Remove edge: " << it->second.src << ", " << it->second.dst << endl;
                        _cluster_graph.DeleteEdge(it->second.src, it->second.dst);
                        _cluster_graph.DeleteEdge(it->second.dst, it->second.src);
                    }
                }
            }
        }
    }
    _cluster_graph.ShowInfo();
}

void SfMAligner::MergeClusters(string img_path)
{
    _paths.resize(_cluster_graph.GetNodesNum());
    _sim3s_to_base.resize(_cluster_graph.GetNodesNum());
    this->StoreOriginalSfMDatas();
    int layer = 1;

    // Merge nodes until only one node left
    while(_cluster_graph.GetNodesNum() > 1) {
        _cluster_graph.CountOutDegrees();
        _cluster_graph.CountInDegrees();
        _cluster_graph.CountDegrees();
        unordered_map<size_t, size_t> degree = _cluster_graph.GetDegrees();

        LOG(INFO) << "Merge the " << layer++ << "-th layer leaf nodes";
        vector<int> indeces;
        if (_cluster_graph.GetNodesNum() == 2) {
            indeces.push_back(degree.begin()->first);
        } else {
            for (auto it = degree.begin(); it != degree.end(); ++it) {
                LOG(INFO) << "node: " << it->first << ", "
                          << "degree: " << it->second;
                if (it->second == 1) indeces.push_back(it->first);
            }
        }

        if (indeces.empty()) break;

        for (auto idx : indeces) {
            if (idx == -1) break;
            ClusterEdge edge = _cluster_graph.FindConnectedEdge(idx);

            LOG(INFO) << "find node [degree = 1]: " << idx;
            LOG(INFO) << edge.src << "->" << edge.dst << ": " << edge.weight;
            LOG(INFO) << "common 3D size: " << edge.src_poses.size();

            // src is the node with degree=1
            int src = (idx == edge.src) ? edge.src : edge.dst;
            int dst = (idx == edge.src) ? edge.dst : edge.src;

            LOG(INFO) << "Merge Clusters: " << src << "->" << dst
                      << ": " << edge.weight;

            _base_cluster_index = dst;
            Sim3 sim = _sim3_graph[src][dst];
            _paths[src].insert(make_pair(dst, sim));
            _cluster_graph.DeleteNode(src);
            _cluster_graph.DeleteEdge(src, dst);
            _cluster_graph.DeleteEdge(dst, src);
            _cluster_graph.ShowInfo();
        }
    }

    for (int i = 0; i < _ori_sfmdatas.size(); i++) {
        if (i != _base_cluster_index) ComputePath(i, _base_cluster_index);
    }
    _sim3s_to_base[_base_cluster_index].s = 1;
    _sim3s_to_base[_base_cluster_index].R = Eigen::Matrix3d::Identity();
    _sim3s_to_base[_base_cluster_index].t = Eigen::Vector3d::Zero();

    this->MergeSfMDatas(img_path);
    LOG(INFO) << "generate colors";
    std::vector<Eigen::Vector3d> colors = this->GenerateColors(_ori_sfmdatas.size());
    LOG(INFO) << "Render clusters";
    this->RenderClusters(colors);
    LOG(INFO) << "Render Cluster";
    this->RenderCluster(_base_cluster_index, colors);
}

double SfMAligner::CheckReprojError(const vector<Eigen::Vector3d>& src_observations,
                                    const vector<Eigen::Vector3d>& dst_observations,
                                    double scale,
                                    Eigen::Matrix3d R,
                                    Eigen::Vector3d t)
{
    double reproj_err = 0.0;
    int size = src_observations.size();
    for (int i = 0; i < size; i++) {
        Eigen::Vector3d reproj_obv = scale * R * src_observations[i] + t;
        reproj_err += (reproj_obv - dst_observations[i]).norm();
    }
    LOG(INFO) << "Sum Reprojection Error: " << reproj_err;
    LOG(INFO) << "Points Size: " << src_observations.size();
    LOG(INFO) << "Mean Reprojection Error: "
              << (size ? (reproj_err / size) : numeric_limits<float>::max());
    return size ? (reproj_err / size) : numeric_limits<float>::max();
}

void SfMAligner::FindCommonCameras(const SfM_Data& sfm_data1,
                                   const SfM_Data& sfm_data2,
                                   std::vector<pair<int, openMVG::geometry::Pose3>>& poses1,
                                   std::vector<pair<int, openMVG::geometry::Pose3>>& poses2)
{
    Views views1 = sfm_data1.GetViews(), views2 = sfm_data2.GetViews();
    Poses pose_list1 = sfm_data1.GetPoses(), pose_list2 = sfm_data2.GetPoses();

    for (auto it1 = pose_list1.begin(); it1 != pose_list1.end(); ++it1) {
        auto it2 = pose_list2.find(it1->first);
        if (it2 != pose_list2.end()) {
            poses1.push_back(make_pair(it1->first, it1->second));
            poses2.push_back(make_pair(it2->first, it2->second));
            _boundary_camera_indexes.insert(it1->first);
            LOG(INFO) << "[" << poses1.size() << "]: " << it1->first << " - " << it2->first;
        }
    }
    LOG(INFO) << "common camera size: " << poses1.size();
}

void SfMAligner::FindCommonObservations(const SfM_Data& sfm_data1,
                                        const SfM_Data& sfm_data2,
                                        std::vector<Eigen::Vector3d>& observations1,
                                        std::vector<Eigen::Vector3d>& observations2)
{
    // Landmarks inview_landmarks1, inview_landmarks2;
    Landmarks landmarks1 = sfm_data1.GetLandmarks(), landmarks2 = sfm_data2.GetLandmarks();

    for (auto it1 = landmarks1.begin(); it1 != landmarks1.end(); ++it1) {
        int trackid = it1->first;
        auto it2 = landmarks2.find(trackid);

        if (it2 != landmarks2.end()) {
            auto landmark1 = it1->second, landmark2 = it2->second;
            observations1.push_back(it1->second.X);
            observations2.push_back(it2->second.X);
        }
    }
    LOG(INFO) << "common observations size: " << observations1.size();
}

void SfMAligner::FindSimilarityTransform(const std::vector<Eigen::Vector3d>& observations1,
                                         const std::vector<Eigen::Vector3d>& observations2,
                                         Eigen::Matrix3d& R,
                                         Eigen::Vector3d& t,
                                         double& scale,
                                         double& msd)
{
    std::vector<Eigen::Vector3d> inliers1, inliers2;
    double threshold = 0.001;
    double p = 0.99;

    if (observations1.size() > 5) {
        LOG(INFO) << "Finding Similarity by RANSAC";
        RansacSimilarity(observations1, observations2, inliers1, inliers2, R, t, scale, threshold, p);
        LOG(INFO) << "inliers size: " << inliers1.size();
        // Re-compute similarity by inliers
        Eigen::MatrixXd x1 = Eigen::MatrixXd::Zero(3, inliers1.size()),
                        x2 = Eigen::MatrixXd::Zero(3, inliers2.size());
        for(int i = 0; i < inliers1.size(); i++) {
            x1.col(i) = inliers1[i];
            x2.col(i) = inliers2[i];
        }
        openMVG::geometry::FindRTS(x1, x2, &scale, &t, &R);
        // Optional non-linear refinement of the found parameters
        openMVG::geometry::Refine_RTS(x1, x2, &scale, &t, &R);
    }

    if (observations1.size() <= 5 || inliers1.size() <= 5) {
        Eigen::MatrixXd x1 = Eigen::MatrixXd::Zero(3, observations1.size()),
                        x2 = Eigen::MatrixXd::Zero(3, observations2.size());
        for (int i = 0; i < observations1.size(); i++) {
            x1.col(i) = observations1[i];
            x2.col(i) = observations2[i];
        }
        openMVG::geometry::FindRTS(x1, x2, &scale, &t, &R);
        openMVG::geometry::Refine_RTS(x1, x2, &scale, &t, &R);
    }

    LOG(INFO) << "scale: " << scale;
    LOG(INFO) << "rotation: \n" << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << "\n"
                            << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << "\n"
                            << R(2, 0) << " " << R(2, 1) << " " << R(2, 2);
    LOG(INFO) << "translation: " << t[0] << ", " << t[1] << ", " << t[2];

    if (inliers1.size() < 4) msd = numeric_limits<float>::max();
    else msd = CheckReprojError(inliers1, inliers2, scale, R, t);
}

bool SfMAligner::ComputeSimilarityByCameraMotions(
    std::vector<std::pair<int, openMVG::geometry::Pose3>>& poses1,
    std::vector<std::pair<int, openMVG::geometry::Pose3>>& poses2,
    Eigen::Matrix3d& relative_r,
    Eigen::Vector3d& relative_t,
    double& scale)
{
    // my hybrid approach by combining "Divide and Conquer: Efficient Large-Scale
    // Structure from Motion Using Graph Partitioning" and RANSAC

    std::vector<Eigen::Vector3d> camera_centers1 = GetCameraCenters(poses1);
    std::vector<Eigen::Vector3d> camera_centers2 = GetCameraCenters(poses2);
    std::vector<Eigen::Matrix3d> camera_rotations1 = GetCameraRotations(poses1);
    std::vector<Eigen::Matrix3d> camera_rotations2 = GetCameraRotations(poses2);
    std::vector<Eigen::Vector3d> ts1 = GetCameraTranslations(poses1);
    std::vector<Eigen::Vector3d> ts2 = GetCameraTranslations(poses2);
    const int n = camera_centers1.size();

    // compute relative scale from a->b
    std::vector<double> scales;
    for (int i = 0; i < n; i++) {
        Eigen::Vector3d center_a1 = camera_centers1[i];
        Eigen::Vector3d center_b1 = camera_centers2[i];
        for (int j = i + 1; j < n; j++) {
            Eigen::Vector3d center_a2 = camera_centers1[j];
            Eigen::Vector3d center_b2 = camera_centers2[j];
            double scale_ab = (center_b1 - center_b2).norm() / 
                             (center_a1 - center_a2).norm();
            scales.push_back(scale_ab);
        }
    }
    // retrieve the median of scales, according to 
    // the equation (5) of the paper "Divide and Conquer: Efficient Large-Scale
    // Structure from Motion Using Graph Partitioning" 
    std::sort(scales.begin(), scales.end());
    scale = scales[scales.size() / 2];

    // compute relative rotation & relative translation from a->b
    std::vector<Correspondence3D> corres3d;
    std::vector<CorrespondenceEuc> input_datas;
    for (int i = 0; i < camera_centers1.size() ;i++) {
        corres3d.emplace_back(camera_centers1[i], camera_centers2[i]);
        input_datas.push_back(make_pair(Euclidean3D(camera_rotations1[i], ts1[i]),
                                        Euclidean3D(camera_rotations2[i], ts2[i])));
    }
    EuclideanEstimator euc_estimator(scale, corres3d);
    
    Euclidean3D euc3d;
    RansacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>((unsigned int)time(NULL));
    params.error_thresh = 0.002;
    params.max_iterations = 1000;
    
    Prosac<EuclideanEstimator> prosac_euc3(params, euc_estimator);
    prosac_euc3.Initialize();
    RansacSummary summary;
    prosac_euc3.Estimate(input_datas, &euc3d, &summary);

    relative_r = euc3d.R;
    relative_t = euc3d.t;
    
    LOG(INFO) << "scale: " << scale;
    LOG(INFO) << "rotation: \n" 
              << relative_r(0, 0) << " " << relative_r(0, 1) << " " << relative_r(0, 2) << "\n"
              << relative_r(1, 0) << " " << relative_r(1, 1) << " " << relative_r(1, 2) << "\n"
              << relative_r(2, 0) << " " << relative_r(2, 1) << " " << relative_r(2, 2);
    LOG(INFO) << "translation: " 
              << relative_t[0] << ", " << relative_t[1] << ", " << relative_t[2];

    return true;
}

bool SfMAligner::Refine()
{
    RefineOption refine_option(false, false, false, true);
    LOG(INFO) << "bundle adjustment";
    _sfm_optimizer.BundleAdjustment(_sfm_data, refine_option);

    refine_option.refine_rotations = true;
    refine_option.refine_translations = true;
    _sfm_optimizer.BundleAdjustment(_sfm_data, refine_option);

    return true;
}

ClusterGraph SfMAligner::GetClusterGraph() const
{
    return _cluster_graph;
}

void SfMAligner::StoreOriginalSfMDatas()
{
    std::vector<ClusterNode> nodes = _cluster_graph.SerializeNodes();
    for (int i = 0; i < _cluster_graph.GetNodesNum(); i++) {
        _ori_sfmdatas.push_back(nodes[i].sfm_data_path);
    }
}

void SfMAligner::ComputePath(int src, int dst)
{
    LOG(INFO) << "Computing Path: " << src << "->" << dst;
    std::queue<int> qu;
    qu.push(src);
    Eigen::Matrix4d sim3 = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    double s = 1.0;

    Sim3 sim(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Identity(), 1.0);
    LOG(INFO) << "v: " << src;
    while (!qu.empty()) {
        int u = qu.front(); qu.pop();
        auto it = _paths[u].begin();
        int v = it->first;
        LOG(INFO) << "v: " << v;
        s = it->second.s * s;
        r = it->second.R * r.eval();
        t = it->second.s * it->second.R * t.eval() + it->second.t;
        if (v == dst) {
            sim.s = s; sim.R = r; sim.t = t;
            _sim3s_to_base[src] = sim;
            return;
        }
        else qu.push(v);
    }
    LOG(INFO) << "\n";
}

void SfMAligner::TransformPly(const string& path1,
                              const string& path2,
                              const Eigen::Matrix3d& R,
                              const Eigen::Vector3d& t,
                              const double& scale) const
{
    vector<Eigen::Vector3d> vec_points, points_colors;
    plyHelper::ReadPly(vec_points, path1, points_colors);

    for(int i = 0; i < vec_points.size(); i++) {
        vec_points[i] = scale * R * vec_points[i].eval() + t;
    }
    plyHelper::ExportToPly(vec_points, &points_colors, path2.c_str());
}

void SfMAligner::TransformPly(const string& path1,
                              const string& path2,
                              const Eigen::Matrix3d& R,
                              const Eigen::Vector3d& t,
                              const double& scale,
                              const Eigen::Vector3d& color) const
{
    vector<Eigen::Vector3d> vec_points, points_colors;
    plyHelper::ReadPly(vec_points, path1, points_colors);
    std::vector<Eigen::Vector3d>().swap(points_colors);

    for(int i = 0; i < vec_points.size(); i++) {
        vec_points[i] = scale * R * vec_points[i].eval() + t;
        points_colors.push_back(color);
    }
    plyHelper::ExportToPly(vec_points, &points_colors, path2.c_str());
}

std::vector<Eigen::Vector3d> SfMAligner::GenerateColors(int size) const
{
    std::vector<Eigen::Vector3d> colors;
    srand((unsigned int)time(NULL));
    for (int i = 0; i < size; i++) {
        double r = (double)(rand() % 256);
        double g = (double)(rand() % 256);
        double b = (double)(rand() % 256);
        colors.push_back(Eigen::Vector3d(r, g, b));
    }
    return colors;
}

void SfMAligner::RenderClusters(const std::vector<Eigen::Vector3d>& colors) const
{
    for (int i = 0; i < _ori_sfmdatas.size(); i++) {
        if (i != _base_cluster_index) { this->RenderCluster(i, colors); }
    }
}

void SfMAligner::RenderCluster(const int idx,
                               const std::vector<Eigen::Vector3d>& colors) const
{
    std::string dir = stlplus::folder_part(_ori_sfmdatas[idx]);
    LOG(INFO) << dir;
    Eigen::Matrix3d R = _sim3s_to_base[idx].R;
    Eigen::Vector3d t = _sim3s_to_base[idx].t;
    double s = _sim3s_to_base[idx].s;
    std::string ori_ply_path = dir + "/robust.ply";
    std::string trans_ply_path = dir + "/robust_transformed.ply";
    this->TransformPly(ori_ply_path, trans_ply_path, R, t, s, colors[idx]);
}

void SfMAligner::MergeSfMDatas(string img_path)
{
    if(!Load(_sfm_data, _ori_sfmdatas[_base_cluster_index], ESfM_Data(ALL))) {
        LOG(ERROR) << "The input SfM_Data file \""
                   << _ori_sfmdatas[_base_cluster_index]
                   << "\" cannot be read." << endl;
        return;
    }

    _sfm_data.s_root_path = img_path;
    Views& views = _sfm_data.views;
    Poses& poses = _sfm_data.poses;
    Intrinsics& intrinsics = _sfm_data.intrinsics;
    Landmarks& structure = _sfm_data.structure;
    Landmarks& control_points = _sfm_data.control_points;

    for (int i = 0; i < _ori_sfmdatas.size(); i++) {
        if (i == _base_cluster_index) continue;
        SfM_Data sfm_data;
        SfM_Data transformed_sfm_data;
        Eigen::Matrix3d R = _sim3s_to_base[i].R;
        Eigen::Vector3d t = _sim3s_to_base[i].t;
        double scale = _sim3s_to_base[i].s;

        if (Load(sfm_data, _ori_sfmdatas[i], ESfM_Data(ALL))) {
            transformed_sfm_data.s_root_path = img_path;
            transformed_sfm_data.views = sfm_data.views;
            transformed_sfm_data.intrinsics = sfm_data.intrinsics;
            transformed_sfm_data.control_points = sfm_data.control_points;

            // Copy views
            for(auto vi = sfm_data.views.begin(); vi != sfm_data.views.end(); ++vi) {
                views.insert(make_pair(vi->first, vi->second));
                auto intrinsic_it = sfm_data.intrinsics.find(vi->second->id_intrinsic);
                intrinsics[intrinsic_it->first] = intrinsic_it->second;
            }

            // Copy control_points
            uint32_t cp_s_id = control_points.size();
            for(auto ci = sfm_data.control_points.begin(); ci != sfm_data.control_points.end(); ++ci) {
                control_points.insert(make_pair(cp_s_id++, ci->second));
            }

            // copy and transform unconnected camera poses
            for(auto pi = sfm_data.poses.begin(); pi != sfm_data.poses.end(); ++pi) {
                // TODO: Do not transform the pose that has same pose_id with sfm_data2
                Eigen::Vector3d center = scale * R * pi->second.center() + t;
                Eigen::Matrix3d rotation = pi->second.rotation() * R.transpose();
                transformed_sfm_data.poses[pi->first] = openMVG::geometry::Pose3(rotation, center);

                poses.insert(make_pair(pi->first, openMVG::geometry::Pose3(rotation, center)));
                if (_boundary_camera_indexes.find(pi->first) == _boundary_camera_indexes.end()) {
                    poses.insert(make_pair(pi->first, openMVG::geometry::Pose3(rotation, center)));
                } else {
                    _boundary_camera_poses[pi->first].push_back(openMVG::geometry::Pose3(rotation, center));
                }
            }

            // TODO: How to optimize the poses of boundary cameras
            // averaging connected cameras
            // for (auto pi = _boundary_camera_poses.begin(); pi != _boundary_camera_poses.end(); ++pi) {
                // openMVG::geometry::Pose3 pose = MotionAveraging(pi->second);
                // poses.insert(make_pair(pi->first, pose));
            // }

            // Transform structure
            uint32_t ss_id = structure.size();
            for(auto si= sfm_data.structure.begin(); si != sfm_data.structure.end(); ++si) {
                size_t track_id = si->first;
                // if (structure.find(trackId) != structure.end()) continue;
                Landmark landmark = si->second;
                landmark.X = scale * R * landmark.X.eval() + t;
                transformed_sfm_data.structure[track_id] = landmark;

                structure.insert(make_pair(ss_id++, landmark));
                // structure.insert(make_pair(trackId, landmark));
            }

            // if (Save(transformed_sfm_data, _ori_sfmdatas[i], ESfM_Data::ALL)) {
            //     LOG(INFO) << "Original sfm data " << _ori_sfmdatas[i] << "is covered!";
            // } 
        } else {
            LOG(WARNING) << "sfm data " << _ori_sfmdatas[i] << "load failed!";
        }
    }

    if (_align_option.perform_global_ba) {
        Refine();
    }
}

void SfMAligner::ExportReport(const std::string& output_dir) const 
{
    LOG(INFO) << "Generating SfM Reconstruction Report...";
    Generate_SfM_Report(_sfm_data,
                        stlplus::create_filespec(output_dir, "SfMReconstruction_Report.html"));

    string path = output_dir + "/final_sfm_data.json";
    Save(_sfm_data,
         stlplus::create_filespec(stlplus::folder_part(path),
                                  stlplus::basename_part(path), "ply"),
         ESfM_Data(ALL));
    LOG(INFO) << "Saving final sfm data " << path;
    if (Save(_sfm_data, path, ESfM_Data(ALL))) {
        LOG(INFO) << "sfm data successfully saved in " << path;
    } else {
        LOG(INFO) << "Save sfm data " << path << " failed!";
    }
}

void SfMAligner::Retriangulate()
{
    LOG(INFO) << "Retriangulating";

    SfMOptimizerOption option;
    option.minimum_visible_views = 3;
    option.features_dir = _align_option.features_dir;
    option.tracks_filename = _align_option.tracks_filename;

    _sfm_optimizer.SetOptimizerOption(option);
    _sfm_optimizer.LoadFeatures(_sfm_data);
    _sfm_optimizer.TriangulateRansac(_sfm_data);

    LOG(INFO) << "Performing bundle adjustment...";
    RefineOption refine_option(false, false, false, true);
    _sfm_optimizer.BundleAdjustment(_sfm_data, refine_option);

    LOG(INFO) << "\n\n-------------------------------" << "\n"
              << "-- Structure from Motion (statistics):\n"
              << " - Camera calibrated: " << _sfm_data.GetPoses().size()
              << " from " << _sfm_data.GetViews().size() << " input images.\n"
              << " - Tracks, #3D points: " << _sfm_data.GetLandmarks().size() << "\n"
              << "-------------------------------" << "\n";
}

void SfMAligner::SimilarityAveraging()
{
    LOG(INFO) << "Similarity Averaging...";
    SimilarityGraphOptimization sim3_graph_optimizer;
    sim3_graph_optimizer.SetSimilarityGraph(_sim3_graph);

    sim3_graph_optimizer.SimilarityAveraging(_cluster_boundary_cameras);

    _sim3_graph = sim3_graph_optimizer.GetSimilarityGraph();
}


}   // namespace geometry
}   // namespace GraphSfM