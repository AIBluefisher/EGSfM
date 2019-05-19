#ifndef i23dSFM_SFM_ALIGNER_HPP
#define i23dSFM_SFM_ALIGNER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <memory>
#include <glog/logging.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <ceres/types.h>
#include <ceres/rotation.h>

#include "graph/graph.h"
#include "sfm_optimizer.h"
#include "sim3.h"

#include "openMVG/sfm/sfm.hpp"
#include "openMVG/geometry/rigid_transformation3D_srt.hpp"

using namespace openMVG::sfm;
using namespace openMVG::geometry;

namespace GraphSfM {
namespace geometry {
struct Intrinsic
{
    double focal_length;
    double px, py;
    double k1;

    Intrinsic(double f, double p1, double p2, double k)
    {
        focal_length = f;
        px = p1;
        py = p2;
        k1 = k;
    }
};

static std::vector<Eigen::Vector3d> GetCameraCenters(
    const std::vector<std::pair<int, openMVG::geometry::Pose3>>& poses)
{	
    LOG(INFO) << "poses size: " << poses.size();
	std::vector<Eigen::Vector3d> centers;
	for (auto p : poses) {
		centers.push_back(p.second.center());
	}
	return centers;
}

static std::vector<Eigen::Matrix3d> GetCameraRotations(
    const std::vector<std::pair<int, openMVG::geometry::Pose3>>& poses)
{
    std::vector<Eigen::Matrix3d> rotations;
    for (auto p : poses) {
        rotations.push_back(p.second.rotation());
    }
    return rotations;
}

static std::vector<Eigen::Vector3d> GetCameraTranslations(
    const std::vector<std::pair<int, openMVG::geometry::Pose3>>& poses)
{
    std::vector<Eigen::Vector3d> translations;
    for (auto p : poses) {
        translations.push_back(p.second.translation());
    }
    return translations;
}

static Pose3 MotionAveraging(const std::vector<Pose3>& poses) 
{
    // averaging rotations
    double angle_axis[3] = {0, 0, 0};
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (int i = 0; i < poses.size(); i++) {
        Eigen::Matrix3d R = poses[i].rotation();
        Eigen::Vector3d c = poses[i].center();
        double lie_algebra[3];
        ceres::RotationMatrixToAngleAxis((const double*)R.data(), lie_algebra);
        for (int idx = 0; idx < 3; idx++) {
            angle_axis[idx] += lie_algebra[idx];
            center[idx] += c[idx];
        }
    }

    for (int i = 0; i < 3; i++) {
        angle_axis[i] /= poses.size();
        center[i] /= poses.size();
    }

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    ceres::AngleAxisToRotationMatrix(angle_axis, rotation.data());
    return Pose3(rotation, center);
}

struct AlignOption
{
    bool use_common_cameras;
    bool use_common_structures;
    bool perform_global_ba;
    std::string features_dir;
    std::string tracks_filename;

    AlignOption()
    {
        use_common_cameras = true;
        use_common_structures = false;
        perform_global_ba = false;
    }
    
    AlignOption(const bool uc, const bool us, const bool pgb)
    {
        use_common_cameras = uc;
        use_common_structures = us;
        perform_global_ba = pgb; 
    }

    AlignOption(const AlignOption& ao)
    {
        use_common_cameras = ao.use_common_cameras;
        use_common_structures = ao.use_common_structures;
        perform_global_ba = ao.perform_global_ba;
        features_dir = ao.features_dir;
        tracks_filename = ao.tracks_filename;
    }
};

struct Euclidean3D
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    Euclidean3D()
    {
        R = Eigen::Matrix3d::Identity();
        t = Eigen::Vector3d::Zero();
    }

    Euclidean3D(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
    {
        R = rotation;
        t = translation;
    }

    Euclidean3D(const Euclidean3D& eu)
    {
        R = eu.R;
        t = eu.t;
    }
};

struct ClusterNode : graph::Node
{
    std::string sfm_data_path;
    SfM_Data sfm_data;

    ClusterNode() : graph::Node(-1) { sfm_data_path = ""; }

    ClusterNode(const int& idx, const std::string& path = "") : graph::Node(idx)
    {
        // id = idx;
        sfm_data_path = path;
    }

    ClusterNode(const ClusterNode& node) : graph::Node(node.id)
    {
        // id = node.id;
        sfm_data_path = node.sfm_data_path;
        sfm_data = node.sfm_data;
    }
};

struct ClusterEdge : graph::Edge
{
    // Eigen::Matrix3d R;
    // Eigen::Vector3d t;
    // double scale;

    std::vector<std::pair<int, openMVG::geometry::Pose3>> src_poses;
    std::vector<std::pair<int, openMVG::geometry::Pose3>> dst_poses;
    std::vector<Eigen::Vector3d> src_observations;
    std::vector<Eigen::Vector3d> dst_observations;

    ClusterEdge(int s = -1, int d = -1, float w = 0.0) : graph::Edge(s, d, w) {}
    
    ClusterEdge(const ClusterEdge& e)
    : graph::Edge(e.src, e.dst, e.weight)
    {
        src_poses = e.src_poses;
        dst_poses = e.dst_poses;
        src_observations = e.src_observations;
        dst_observations = e.dst_observations;
    }

    bool operator == (const ClusterEdge& e)
    {
        return e.src == src && e.dst == dst;
    }

    std::vector<Eigen::Vector3d> GetSrcCameraCenter()
	{	
        LOG(INFO) << "src poses size: " << src_poses.size();
		std::vector<Eigen::Vector3d> centers;
		for(auto p : src_poses) {
			centers.push_back(p.second.center());
		}
		return centers;
	}

	std::vector<Eigen::Vector3d> GetDstCameraCenter()
	{	
        LOG(INFO) << "dst poses size: " << dst_poses.size();
		std::vector<Eigen::Vector3d> centers;
		for(auto p : dst_poses) {
			centers.push_back(p.second.center());
		}
		return centers;
	}
};

using ClusterGraph = graph::Graph<ClusterNode, ClusterEdge>;
using SimilarityGraph = std::unordered_map<size_t, std::unordered_map<size_t, Sim3>>;
// (i, j, F) or (i, j, E)
// using ViewGraph = std::unordered_map<size_t, std::unordered_map<size_t, Eigen::Matrix3d>>;

class SfMAligner 
{
private:
    //  sfm data that stores landmarks and poses
    SfM_Data _sfm_data; 

    // merged sfm_data path
    std::string _merged_sfm_data_path;

    AlignOption _align_option;

    
    //  Graph that stores the connection information between clusters.
    //  Each node represents a cluster, node.idx is the id of cluster, node.image_name
    //  is the absolute path of sfm_data.json in cluster.
    //  Weight of edge between clusters represents the number of common 3D observations
    ClusterGraph _cluster_graph;

    SfMOptimizer _sfm_optimizer;

    // similarity graph, each cluster is represented as a node,
    // and each edge is the relative similarity transformation
    SimilarityGraph _sim3_graph;
    
    std::vector<std::unordered_map<size_t, Sim3>> _paths;

    std::vector<std::string> _ori_sfmdatas;
    
    // the simalarity transformation to the base cluster, which is used for 
    // final transformation
    std::vector<Sim3> _sim3s_to_base;   

    // the index of the base cluster
    int _base_cluster_index;

    // the indexes of boundary cameras
    std::unordered_set<size_t> _boundary_camera_indexes;

    // (image_id, camera_poses_in_different_clusters)
    std::unordered_map<size_t, std::vector<Pose3>> _boundary_camera_poses;

    // (cluster_id, boundary_cameras)
    std::unordered_map<size_t, std::vector<Pose3>> _cluster_boundary_cameras;
    
public:
    // Constructor
    SfMAligner();
    // Destructor
    ~SfMAligner();  

    SfM_Data GetGlobalSfMData() const;

    void SetAlignOption(const AlignOption& ao);

    /** 
     * @brief  Initialize cluster graph after local incremental SfM
     * @note   
     * @param  sfm_datas_path: absolute path of file that stores all the paths of clusters
     * @retval True if initilalize succeed, else false
     */
    bool InitializeGraph(const std::vector<std::string>& sfm_datas_path);
    bool InitializeGraph(const std::vector<std::string>& sfm_datas_path, 
                         const std::vector<SfM_Data>& sfm_datas);

    void ConstructEdge(const SfM_Data& sfm_data1, const SfM_Data& sfm_data2, ClusterEdge& edge);

    /** 
     * @brief  Update cluster graph by the path of MST
     * @note   
     * @retval None
     */
    void UpdateGraph(); 

    /** 
     * @brief Merge two clusters, sfm_data should be merged, and the original two nodes
     * should be replaced by the new merged node. By default, the cluster with less 3D points
     * is merged into the other cluster with more 3D point 
     * @note   
     * @param  dir: directory that stores the merged sfm_data.json
     * @retval None
     */
    void MergeClusters(std::string img_path = "");
    /** 
     * @brief  Find common cameras between two clusters
     * @note   
     * @param  sfm_data1: sfm_data belongs to cluster1
     * @param  sfm_data2: sfm_data belongs to cluster2
     * @param  poses1: camera poses belong to cluster1
     * @param  poses2: camera poses belong to cluster2
     * @retval None
     */
    void FindCommonCameras(const SfM_Data& sfm_data1, 
                            const SfM_Data& sfm_data2,
                            std::vector<std::pair<int, openMVG::geometry::Pose3>>& poses1,
                            std::vector<std::pair<int, openMVG::geometry::Pose3>>& poses2);
    /** 
     * @brief  Find common 3D observations between two clusters
     * @note   
     * @param  sfm_data1: sfm_data belongs to cluster1
     * @param  sfm_data2: sfm_data belongs to cluster2
     * @param  observations1: observations belongs to cluster1
     * @param  observations2: observations belongs to cluster2
     * @retval None
     */
    void FindCommonObservations(const SfM_Data& sfm_data1, 
                                const SfM_Data& sfm_data2,
                                std::vector<Eigen::Vector3d>& observations1,
                                std::vector<Eigen::Vector3d>& observations2);
    /** 
     * @brief  Find similarity transform between two clusters by using common observations
     * @note   
     * @param  observations1: observations belongs to cluster1
     * @param  observations2: observations belongs to cluster2
     * @param  R: rotation matrix of similarity transform
     * @param  t: translation vector of similarity transform
     * @param  scale: scale of similarity transform
     * @retval None
     */
    void FindSimilarityTransform(const std::vector<Eigen::Vector3d>& observations1,
                                 const std::vector<Eigen::Vector3d>& observations2,
                                 Eigen::Matrix3d& R,
                                 Eigen::Vector3d& t,
                                 double& scale,
                                 double& msd);
    
    bool ComputeSimilarityByCameraMotions(
        std::vector<std::pair<int, openMVG::geometry::Pose3>>& poses1,
        std::vector<std::pair<int, openMVG::geometry::Pose3>>& poses2,
        Eigen::Matrix3d& relative_rotation,
        Eigen::Vector3d& relative_t,
        double& scale);

    /** 
     * @brief  Refine the structure by bundle adjustment after all clusters are merged into one
     * @note
     * @retval True if refine complete, else false
     */
    bool Refine(); 

    void Retriangulate();

    void SimilarityAveraging();

    /** 
     * @brief  Get cluster graph
     * @note   
     * @retval ClusterGraph
     */
    ClusterGraph GetClusterGraph() const; 

    /** 
     * @brief  Generate .ply file from json file in path1 and stored in path2
     * @note   
     * @param  path1: path that store the sfm_data.json file
     * @param  path2: path that store the .ply file
     * @param  R: rotation matrix of similarity transform
     * @param  t: translation vector of similarity transform
     * @param  scale: scale of similarity transform
     * @retval None
     */
    void TransformPly(const std::string& path1, 
                      const std::string& path2, 
                      const Eigen::Matrix3d& R, 
                      const Eigen::Vector3d& t, 
                      const double& scale) const;

    void TransformPly(const std::string& path1, 
                      const std::string& path2, 
                      const Eigen::Matrix3d& R, 
                      const Eigen::Vector3d& t, 
                      const double& scale,
                      const Eigen::Vector3d& color) const;
    void ComputePath(int src, int dst);

    std::vector<Eigen::Vector3d> GenerateColors(int size) const;
    void RenderCluster(const int idx, const std::vector<Eigen::Vector3d>& colors) const;
    void RenderClusters(const std::vector<Eigen::Vector3d>& colors) const;
    void ExportReport(const std::string& output_dir) const;

    
    
private:
    /** 
     * @brief  Check the reprojection error between 3D points
     * @note   
     * @param  src_observations: 
     * @param  dst_observations: 
     * @param  scale: scale
     * @param  R:  rotation matrix
     * @param  t: translation
     * @retval None
     */
    double CheckReprojError(const std::vector<Eigen::Vector3d>& src_observations, 
                            const std::vector<Eigen::Vector3d>& dst_observations, 
                            double scale,
                            Eigen::Matrix3d R,
                            Eigen::Vector3d t );

    void StoreOriginalSfMDatas();

    void MergeSfMDatas(std::string img_path);
};

}   // namespace geometry
}   // namespace GraphSfM

#endif