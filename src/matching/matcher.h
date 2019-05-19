#ifndef MATCHING_MATCHER_H
#define MATCHING_MATCHER_H

#include <iostream>
#include <string>

// #ifdef USE_OPENMVG
// #include "openMVG/graph/graph_graphviz_export.hpp"
// #include "openMVG/graph/graph.hpp"
#include "openMVG/graph/graph_builder.hpp"
#include "openMVG/graph/graph_graphviz_export.hpp"
#include <lemon/list_graph.h>
#include "openMVG/image/image_io.hpp"
#include "openMVG/image/image_concat.hpp"
#include "openMVG/image/image_warping.hpp"
#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
#include "openMVG/features/svg_features.hpp"
#include "openMVG/matching/svg_matches.hpp"
#include "openMVG/geometry/pose3.hpp"
#include "openMVG/matching/indMatchDecoratorXY.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "openMVG/multiview/triangulation.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/multiview/solver_fundamental_kernel.hpp"
#include "openMVG/multiview/solver_homography_kernel.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"
#include "openMVG/stl/stl.hpp"
#include "openMVG/types.hpp"

using namespace openMVG;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::image;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::robust;
// #endif

namespace GraphSfM {
namespace matching {

class Matcher
{
private:
    Image<unsigned char> _image_l;
    Image<unsigned char> _image_r;
    
    PointFeatures _feats_l;
    PointFeatures _feats_r;

    // std::unique_ptr<Image_describer> _img_describer;
    std::shared_ptr<sfm::Regions_Provider> _regions_provider;
    std::map<IndexT, std::shared_ptr<features::Regions>> _regions_per_img;

    std::vector<uint32_t> _inliers;
    IndMatches _vec_putative_matches;
    PairWiseMatches _matches;
    sfm::SfM_Data _sfm_data;

public:
    // constructor
    Matcher();
    // Matcher(const std::string& filename_l, const std::string& filename_r);

    // extract features
    void Extract();
    bool LoadRegionsFromFile(const std::string& matches_dir, 
                             const std::string& filename);
    void LoadPairwiseRegions(const size_t& i, const size_t& j,
                             const std::string& filename_l, 
                             const std::string& filename_r);

    // matching & geometric filtering
    void InitialMatching();
    // void GeometricFiltering(Eigen::Matrix3d& M, GeometricRelationType& type);
    void HomographyFiltering(Eigen::Matrix3d& H, std::vector<uint32_t>& vec_inliers);
    void FundamentalFiltering(Eigen::Matrix3d& F, std::vector<uint32_t>& vec_inliers);
    void EssentialFiltering(const size_t& i, 
                            const size_t& j,
                            Eigen::Matrix3d& R, 
                            std::vector<uint32_t>& vec_inliers);

    void CacheValidMatches(size_t i, size_t j);
    PairWiseMatches GetMatches() const;
    size_t TotalMatchesNumber() const;

    void ShowInfo() const;
    bool ExportMatches(const std::string& dir, const std::string& filename) const;

    void Clear();

// private:
    bool ReadIntrinsic(const size_t& i, Mat3& K);

};


} // namespace matching
} // namespace GraphSfM

#endif