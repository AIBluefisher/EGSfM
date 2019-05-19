#include "matcher.h"
#include "stlplus3/stlplus.h"

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"

#include <fstream>
#include <glog/logging.h>

namespace GraphSfM {
namespace matching {

Matcher::Matcher() 
{

}

// Matcher::Matcher(const std::string& filename_l, const std::string& filename_r)
// {
    // ReadImage(filename_l.c_str(), &_image_l);
    // ReadImage(filename_r.c_str(), &_image_r);
    // assert(_image_l.data() && _image_r.data());
// }

void Matcher::Extract()
{
    // std::unique_ptr<Image_describer> img_describer;
    // img_describer.reset(new SIFT_Anatomy_Image_describer(SIFT_Anatomy_Image_describer::Params()));

    // if (img_describer == nullptr) {
        // std::cerr << "Invalid Image Describer type" << std::endl;
        // return ;
    // }

    // // Detect regions by img_describer
    // img_describer->Describe(_image_l, _regions_per_img[0]);
    // img_describer->Describe(_image_r, _regions_per_img[1]);

    // _feats_l = _regions_per_img.at(0)->GetRegionsPositions();
    // _feats_r = _regions_per_img.at(1)->GetRegionsPositions();
}

bool Matcher::LoadRegionsFromFile(const std::string& matches_dir, const std::string& filename)
{
    using namespace openMVG::sfm;
    if (!Load(_sfm_data, filename, sfm::ESfM_Data(ESfM_Data::VIEWS | ESfM_Data::INTRINSICS))) {
        LOG(ERROR) << "\nThe Input SfM_Data file \"" << filename << "\" Cannot be read.\n";
        return false;
    }

    // Load SfM Scene regions
    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    const std::string img_describer = stlplus::create_filespec(matches_dir, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(img_describer);
    if (!regions_type) {
        LOG(ERROR) << "Invalid: "
                  << img_describer << " regions type file." << std::endl;
        return false;
    }

    unsigned int ui_max_cache_size = 0;
    // Load the corresponding view regions
    if (ui_max_cache_size == 0) {
        // Default regions provider (load & store all regions in memory)
        _regions_provider = std::make_shared<Regions_Provider>();
    } else {
        // Cached regions provider (load & store regions on demand)
        _regions_provider = std::make_shared<Regions_Provider_Cache>(ui_max_cache_size);
    }   
    
    // C_Progress_display progress;
    if (!_regions_provider->load(_sfm_data, matches_dir, regions_type)) {
        LOG(ERROR) << std::endl << "Invalid regions." << std::endl;
        return false;
    }
}

void Matcher::LoadPairwiseRegions(const size_t& i, const size_t& j,
                                  const std::string& filename_l, 
                                  const std::string& filename_r) 
{
    ReadImage(filename_l.c_str(), &_image_l);
    ReadImage(filename_r.c_str(), &_image_r);
    assert(_image_l.data() && _image_r.data());

    std::shared_ptr<features::Regions> regionsi = _regions_provider->get(i);
    std::shared_ptr<features::Regions> regionsj = _regions_provider->get(j);

    _regions_per_img[0] = regionsi;
    _regions_per_img[1] = regionsj;

    _feats_l = _regions_per_img.at(0)->GetRegionsPositions();
    _feats_r = _regions_per_img.at(1)->GetRegionsPositions();
}

void Matcher::InitialMatching()
{
    // Compute corresponding points
    // matching -> find nearest neighbor filtered with distance ratio
    DistanceRatioMatch(0.8, 
                       CASCADE_HASHING_L2,
                       *_regions_per_img.at(0).get(),
                       *_regions_per_img.at(1).get(),
                       _vec_putative_matches);

    IndMatchDecorator<float> matchDeduplicator(_vec_putative_matches, _feats_l, _feats_r);
    matchDeduplicator.getDeduplicated(_vec_putative_matches);
}

void Matcher::ShowInfo() const
{
    // Display statistics
    LOG(INFO) << _regions_per_img.at(0)->RegionCount() << " #features on image A" << std::endl
              << _regions_per_img.at(1)->RegionCount() << " #features on image B" << std::endl
              << _vec_putative_matches.size() << " #matches with distance ratio filter" << std::endl;
}

bool Matcher::ReadIntrinsic(const size_t& i, Mat3& K)
{
    size_t intrinsic_id = _sfm_data.GetViews().at(i)->id_intrinsic;

    std::shared_ptr<cameras::IntrinsicBase> intrinsic_base = 
                                        _sfm_data.GetIntrinsics().at(intrinsic_id);
    cameras::Pinhole_Intrinsic* intrinsic = 
                                dynamic_cast<cameras::Pinhole_Intrinsic*>(intrinsic_base.get());
    K = intrinsic->K();

    return true;
}

void Matcher::FundamentalFiltering(Eigen::Matrix3d& F, std::vector<uint32_t>& vec_inliers)
{
    // Fundamental geometry filtering of putative matches
    Mat xl(2, _vec_putative_matches.size());
    Mat xr(2, _vec_putative_matches.size());

    for (size_t k = 0; k < _vec_putative_matches.size(); k++) {
        const PointFeature& img_l = _feats_l[_vec_putative_matches[k].i_];
        const PointFeature& img_r = _feats_r[_vec_putative_matches[k].j_];
        xl.col(k) = img_l.coords().cast<double>();
        xr.col(k) = img_r.coords().cast<double>();
    }

    using KernelType = ACKernelAdaptor<openMVG::fundamental::kernel::SevenPointSolver,
                                       openMVG::fundamental::kernel::SymmetricEpipolarDistanceError,
                                       UnnormalizerI,
                                       Mat3>;
    KernelType kernel(xl, _image_l.Width(), _image_l.Height(),
                      xr, _image_r.Width(), _image_r.Height(),
                      true);
    
    const std::pair<double, double> acransac_out = ACRANSAC(kernel, vec_inliers, 1024, &F,
                                                            Square(4.0), true);
    const double& threshold_f = acransac_out.first;

    // Check the fundamental support some point to be considered as valid
    if (vec_inliers.size() > KernelType::MINIMUM_SAMPLES * 2.5) {
        _inliers.assign(vec_inliers.begin(), vec_inliers.end());
        LOG(INFO) << "\nFound a fundamental under the confidence threshold of: "
                  << threshold_f << " pixels\n\twith: " << vec_inliers.size() << " inliers"
                  << " from: " << _vec_putative_matches.size()
                  << " putatives correspondences"
                  << std::endl;
    } else {
        LOG(INFO) << "Unable to estimate a rigid fundamental" << std::endl;
    }
}

void Matcher::HomographyFiltering(Eigen::Matrix3d& H, std::vector<uint32_t>& vec_inliers)
{
    // Homography geometry filtering of putative matches
    Mat xl(2, _vec_putative_matches.size());
    Mat xr(2, _vec_putative_matches.size());

    for (size_t k = 0; k < _vec_putative_matches.size(); k++) {
        const PointFeature& img_l = _feats_l[_vec_putative_matches[k].i_];
        const PointFeature& img_r = _feats_r[_vec_putative_matches[k].j_];
        xl.col(k) = img_l.coords().cast<double>();
        xr.col(k) = img_r.coords().cast<double>();
    }

    // Homography robust estimation
    using KernelType = ACKernelAdaptor<openMVG::homography::kernel::FourPointSolver,
                                       openMVG::homography::kernel::AsymmetricError,
                                       UnnormalizerI,
                                       Mat3>;
    
    KernelType kernel(xl, _image_l.Width(), _image_l.Height(),
                      xr, _image_r.Width(), _image_r.Height(),
                      false);
    
    const std::pair<double, double> acransac_out = ACRANSAC(kernel, vec_inliers, 1024, &H,
                                                            std::numeric_limits<double>::infinity(),
                                                            true);
    const double& threshold_h = acransac_out.first;

    // Check the homography support some point to be considered as valids
    if (vec_inliers.size() > KernelType::MINIMUM_SAMPLES * 2.5) {
        _inliers.assign(vec_inliers.begin(), vec_inliers.end());
        LOG(INFO) << "\nFound a homography under the confidence threshold of: "
                  << threshold_h << " pixels\n\twith: " << vec_inliers.size() << " inliers"
                  << " from: " << _vec_putative_matches.size()
                  << " putatives correspondences"
                  << std::endl;
    } else {
        LOG(INFO) << "Unable to estimate a rigid homography" << std::endl;
    }
}

void Matcher::EssentialFiltering(const size_t& i, 
                                 const size_t& j,
                                 Eigen::Matrix3d& R, 
                                 std::vector<uint32_t>& vec_inliers)
{
    // Essential geometry filtering of putative matches
    Mat3 K_l, K_r;
    if (!this->ReadIntrinsic(i, K_l)) {
        LOG(ERROR) << "Cannot read intrinsic parameters of view " << i << std::endl;
        return;
    }
    if (!this->ReadIntrinsic(j, K_r)) {
        LOG(ERROR) << "Cannot read intrinsic parameters of view " << j << std::endl;
        return;
    }

    const Pinhole_Intrinsic cam_l(_image_l.Width(), _image_l.Height(), K_l(0, 0), K_l(0, 2), K_l(1, 2));
    const Pinhole_Intrinsic cam_r(_image_r.Width(), _image_r.Height(), K_r(0, 0), K_r(0, 2), K_r(1, 2));

    // prepare the corresponding putatives points
    Mat xl(2, _vec_putative_matches.size());
    Mat xr(2, _vec_putative_matches.size());
    for (size_t k = 0; k < _vec_putative_matches.size(); k++) {
        const PointFeature& img_l = _feats_l[_vec_putative_matches[k].i_];
        const PointFeature& img_r = _feats_r[_vec_putative_matches[k].j_];
        xl.col(k) = img_l.coords().cast<double>();
        xr.col(k) = img_r.coords().cast<double>();
    }

    // Compute the relative pose thanks to a essential matrix estimation
    const std::pair<size_t, size_t> size_imgl(_image_l.Width(), _image_l.Height());
    const std::pair<size_t, size_t> size_imgr(_image_r.Width(), _image_r.Height());

    sfm::RelativePose_Info relative_pose_info;
    if (!sfm::robustRelativePose(&cam_l, &cam_r, 
                                 xl, xr, 
                                 relative_pose_info, 
                                 size_imgl, size_imgr,
                                 256)) {
        LOG(ERROR) << "Robust relative pose estimation failure." << std::endl;
        return;
    }

    LOG(INFO) << "\nFound an Essential matrix:\n"
              << "\tprecision: " << relative_pose_info.found_residual_precision << " pixels\n"
              << "\tinliers: " << relative_pose_info.vec_inliers.size() << "\n"
              << "\tmatches: " << _vec_putative_matches.size() << std::endl;

    // Extract rotation matrix
    R = relative_pose_info.relativePose.rotation();
    vec_inliers.assign(relative_pose_info.vec_inliers.begin(), 
                       relative_pose_info.vec_inliers.end());
    _inliers.assign(vec_inliers.begin(), vec_inliers.end());
}

void Matcher::CacheValidMatches(size_t i, size_t j)
{
    IndMatches geometric_matches;
    for (auto index : _inliers) {
        geometric_matches.push_back(_vec_putative_matches[index]);
    }
    LOG(INFO) << "_inliers number is: " << _inliers.size() << std::endl;
    LOG(INFO) << "geometric matches number is: " << geometric_matches.size() << std::endl;
    if (!geometric_matches.empty()) {
        _matches.insert({{i, j}, std::move(geometric_matches)});
    }
    LOG(INFO) << "Validated matches number is: " << _matches.size() << std::endl;
}

PairWiseMatches Matcher::GetMatches() const
{
    return _matches;
}

size_t Matcher::TotalMatchesNumber() const
{
    size_t sum = 0;
    for (auto it = _matches.begin(); it != _matches.end(); it++) {
        sum += it->second.size();
    }
    return sum;
}

bool Matcher::ExportMatches(const std::string& dir, const std::string& filename) const
{
    if (!openMVG::matching::Save(_matches, std::string(dir + "/matches.f.txt"))) {
        LOG(ERROR) << " Cannot Save Computed Matches in: "
                  << std::string(dir + "/matches.f.txt");
        return false;
    } 

    // export view pair graph once geometric filter have been done
    std::set<IndexT> set_ViewIds;
    std::transform(_sfm_data.GetViews().begin(), _sfm_data.GetViews().end(),
                   std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
    openMVG::graph::indexedGraph putative_graph(set_ViewIds, getPairs(_matches));
    openMVG::graph::exportToGraphvizData(stlplus::create_filespec(dir, filename),
                                         putative_graph);

    LOG(INFO) << "matches file saved in " << dir + "/matches.f.txt" << std::endl;
    return true;
}

void Matcher::Clear()
{
    std::vector<PointFeature>().swap(_feats_l);
    std::vector<PointFeature>().swap(_feats_r);
    _regions_per_img.clear();
    std::vector<uint32_t>().swap(_inliers);
    std::vector<openMVG::matching::IndMatch>().swap(_vec_putative_matches);
}

} // namespace GraphSfM
} // namespace matching