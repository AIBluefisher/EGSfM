#ifndef SFM_SFM_RECONSTRUCTION_H
#define SFM_SFM_RECONSTRUCTION_H

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <string>
#include <atomic>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <cstdio>
#include <thread>

#include "openMVG/cameras/cameras.hpp"
#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/exif/sensor_width_database/ParseDatabase.hpp"
#include "openMVG/features/akaze/image_describer_akaze_io.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer_io.hpp"
#include "openMVG/features/regions_factory_io.hpp"
#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/features/svg_features.hpp"
#include "openMVG/geodesy/geodesy.hpp"
#include "openMVG/geometry/frustum.hpp"
#include "openMVG/graph/graph.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust_Angular.hpp"
#include "openMVG/matching_image_collection/Eo_Robust.hpp"
#include "openMVG/matching_image_collection/H_ACRobust.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM2.hpp"
#include "openMVG/sfm/pipelines/sequential/SfmSceneInitializerMaxPair.hpp"
#include "openMVG/sfm/pipelines/sequential/SfmSceneInitializerStellar.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/pipelines/structure_from_known_poses/structure_estimator.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_BA.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_colorization.hpp"
#include "openMVG/sfm/sfm_data_filters.hpp"
#include "openMVG/sfm/sfm_data_filters_frustum.hpp"
#include "openMVG/sfm/sfm_data_triangulation.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/sfm/sfm_view_priors.hpp"
#include "openMVG/stl/stl.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/tracks/tracks.hpp"
#include "openMVG/types.hpp"

#include "third_party/progress/progress_display.hpp"
// #include "stlplus3/filesystemSimplified/file_system.hpp"
#include "stlplus3/file_system.hpp"
#include "nonFree/sift/SIFT_describer_io.hpp"

#include "geometry/image_cluster.h"
#include "geometry/sfm_aligner.h"

#include <glog/logging.h>
#include <cereal/archives/json.hpp>
#include <cereal/details/helpers.hpp>

#include "libvot/src/libvot_config.h"
#include "libvot/src/vocab_tree/vot_pipeline.h"
#include "libvot/src/utils/io_utils.h"

#ifdef _USE_OPENMP
#include <omp.h>
#endif

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::features;
using namespace openMVG::geodesy;
using namespace openMVG::image;
using namespace openMVG::matching;
using namespace openMVG::matching_image_collection;
using namespace openMVG::robust;
using namespace openMVG::sfm;
using namespace GraphSfM::geometry;

namespace GraphSfM {
namespace sfm {

static void ExportTracks(const PairWiseMatches& map_matches, std::string dir)
{
    tracks::TracksBuilder tracks_builder;
    tracks::STLMAPTracks map_tracks;

    LOG(INFO) << "Track building";
    tracks_builder.Build(map_matches);
    LOG(INFO) << "Track filtering";
    tracks_builder.Filter();
    LOG(INFO) << "Track export to internal struct";
    //-- Build tracks with STL compliant type :
    tracks_builder.ExportToSTL(map_tracks);

    LOG(INFO) << "Track stats";
    {
        std::ostringstream os_track;
        //-- Display stats :
        //    - number of images
        //    - number of tracks
        std::set<uint32_t> set_images_id;
        tracks::TracksUtilsMap::ImageIdInTracks(map_tracks, set_images_id);
        os_track << "------------------" << "\n"
                << "-- Tracks Stats --" << "\n"
                << " Tracks number: " << tracks_builder.NbTracks() << "\n"
                << " Images Id: " << "\n";
        std::copy(set_images_id.begin(),
                  set_images_id.end(),
                  std::ostream_iterator<uint32_t>(os_track, ", "));
        os_track << "\n------------------" << "\n";

        std::map<uint32_t, uint32_t> map_occur_track_length;
        tracks::TracksUtilsMap::TracksLength(map_tracks, map_occur_track_length);
        os_track << "TrackLength, Occurrence" << "\n";
        for (const auto & it : map_occur_track_length)  {
            os_track << "\t" << it.first << "\t" << it.second << "\n";
        }
        os_track << "\n";
        std::cout << os_track.str();
    }

    std::string filename = dir + "/tracks.txt";
    tracks_builder.ExportToFile(filename, map_tracks);
}

enum EGeometricModel
{
  FUNDAMENTAL_MATRIX = 0,
  ESSENTIAL_MATRIX   = 1,
  HOMOGRAPHY_MATRIX  = 2,
  ESSENTIAL_MATRIX_ANGULAR = 3,
  ESSENTIAL_MATRIX_ORTHO = 4
};

enum EPairMode
{
  PAIR_EXHAUSTIVE = 0,
  PAIR_CONTIGUOUS = 1,
  PAIR_FROM_FILE  = 2
};

enum class ESfMSceneInitializer
{
  INITIALIZE_EXISTING_POSES,
  INITIALIZE_MAX_PAIR,
  INITIALIZE_AUTO_PAIR,
  INITIALIZE_STELLAR
};

struct SimilaritySearchOption
{
    size_t top_k_images;
};

struct SfMOption
{
    int thread_num;
    std::string image_dir;
    std::string output_dir;
    std::string features_dir;
    std::string matches_dir;

    /////////////////////////////////////////////
    ////   configuration for image listing   ////
    /////////////////////////////////////////////

    // Specify the type of camera model (namespace: openMVG::cameras::EINTRINSIC)
    //  - static_cast<int>(PINHOLE_CAMERA): Pinhole
    //  - static_cast<int>(PINHOLE_CAMERA_RADIAL1): Pinhole radial 1
    //  - static_cast<int>(PINHOLE_CAMERA_RADIAL3): Pinhole radial 3 (default)
    //  - static_cast<int>(PINHOLE_CAMERA_BROWN): Pinhole brown 2
    //  - static_cast<int>(PINHOLE_CAMERA_FISHEYE): Pinhole with a simple Fish-eye distortion
    //  - static_cast<int>(CAMERA_SPHERICAL): Spherical camera
    int camera_model_type;

    // Group camera that share common properties if desired (leads to more faster & stable BA)
    //  - 0: each view have it's own camera intrinsic parameters,
    //  - 1: (default) view can share some camera intrinsic parameters
    bool group_camera_model; 

    // the path that store the sensor width database file
    std::string sensor_width_database;

    ///////////////////////////////////////////////////////
    ////    configuration for feature extraction     /////
    /////////////////////////////////////////////////////

    // used to control the Image_describer configuration):
    //  NORMAL_PRESET (default), HIGH_PRESET, ULTRA_PRESET (Can take long time!)
    int feature_quality;

    /////////////////////////////////////////////
    ////    configuration for matching     /////
    ///////////////////////////////////////////

    // Use a regions cache (only cache_size regions will be stored in memory
    // If not used, all regions will be load in memory (0 by default)
    int ui_max_cache_size;

    // Distance ratio to discard non meaningful matches (0.8f by default)
    float distance_ratio;

    // maximum number of iterations (2048 by default)
    int ransac_max_iteration;

    // use the found geometric model to improve the 
    // pairwise correspondences(false by default)
    bool perform_guided_matching;

    // (pairwise correspondences filtering thanks to robust model estimation):
    //  - f: (default) fundamental matrix
    //  - e: essential matrix
    //  - h: homography matrix
    //  - a: essential matrix with an angular parametrization
    //  - o: orthographic essential matrix
    std::string geometric_model;

    // AUTO: auto choice from regions type
    // For Scalar based regions descriptor:
    //  - BRUTEFORCEL2: L2 BruteForce matching
    //  - ANNL2: L2 Approximate Nearest Neighbor matching
    //  - CASCADEHASHINGL2: L2 Cascade Hashing matching
    //  - FASTCASCADEHASHINGL2: (default)
    //  - L2 Cascade Hashing with precomputed hashed regions
    //    (faster than CASCADEHASHINGL2 but use more memory)
    // For Binary based descriptor:
    //  - BRUTEFORCEHAMMING: BruteForce Hamming matching
    std::string nearest_matching_method;

    std::string predefined_pair_list;

    // Sequence matching with an overlap of X images:
    //  - X: with match 0 with (1->X), ...]
    //  - 2: will match 0 with (1,2), 1 with (2,3), ...
    //  - 3: will match 0 with (1,2,3), 1 with (2,3,4), ...
    int video_mode_matching;


    //////////////////////////////////////////////////
    ////   configuration for similarity search    ////
    //////////////////////////////////////////////////
    bool use_similarity_search;

    // vocabulary tree working directory
    std::string vot_dir;

    // filename that stores all the path of sift file
    std::string vot_sifts_file;

    // sift feature type used in vocabulary tree
    //  - E3D_SIFT
    //  - OPENMVG_FEAT(default)
    vot::SiftType vot_sift_type;

    // depth of vocabulary tree
    int vot_depth;

    // branch number of vocabulary tree
    int vot_branch_num;


    // |--refineIntrinsics] Intrinsic parameters refinement option
    //  - ADJUST_ALL -> refine all existing parameters (default)
    //  - NONE -> intrinsic parameters are held as constant
    //  - ADJUST_FOCAL_LENGTH -> refine only the focal length
    //  - ADJUST_PRINCIPAL_POINT -> refine only the principal point position
    //  - ADJUST_DISTORTION -> refine only the distortion coefficient(s) (if any)
    // -> NOTE: options can be combined thanks to '|'
    //  - ADJUST_FOCAL_LENGTH | ADJUST_PRINCIPAL_POINT
    // -> refine the focal length & the principal point position
    //  - ADJUST_FOCAL_LENGTH | ADJUST_DISTORTION
    // -> refine the focal length & the distortion coefficient(s) (if any)
    //  - ADJUST_PRINCIPAL_POINT|ADJUST_DISTORTION
    // -> refine the principal point position & the distortion coefficient(s) (if any)
    std::string intrinsic_refinement_option;


    ////////////////////////////////////////////////
    ////    configuration for image cluster    ////
    //////////////////////////////////////////////

    // filename that stores all the images' path
    // std::string img_list;

    // stores the weight of cluster graph
    // std::string weight_file;

    // image cluster option:
    //  - naive: divide images into clusters with no intersection
    //  - expansion: divide images into clusters with connected images
    std::string cluster_option;

    // maximum number of images in each cluster
    int max_cluster_size;

    // the repeated ratio of images
    double completeness_ratio;

    float relax_ratio;

    // define whether to copy images into sub-folders after image clustering
    bool copy_images;

    // Choose the SfM initializer method:
    //  - 'EXISTING_POSE'-> Initialize the reconstruction from the existing sfm_data camera poses
    //  - 'MAX_PAIR'-> Initialize the reconstruction from the pair that has the most of matches
    //  - 'AUTO_PAIR'-> Initialize the reconstruction with a pair selected automatically
    //  - 'STELLAR'-> Initialize the reconstruction with a 'stellar' reconstruction.
    std::string batched_sfm_initilizer;

    // maximal pixels reprojection error that will be considered for triangulations(4.0 by default)
    double max_triangulation_repro_err;

    /////////////////////////////////////////
    ////   configuration for sfm align   ////
    ////////////////////////////////////////
    // std::string sfm_datas_path;

    bool align_use_common_cameras;

    bool align_use_common_structures;

    bool perform_global_ba;
    
    // constructor
    SfMOption()
    {
        thread_num = -1;
        feature_quality = 0;
        ui_max_cache_size = 0;
        nearest_matching_method = "FASTCASCADEHASHINGL2";
        distance_ratio = 0.8f;
        ransac_max_iteration = 2048;
        perform_guided_matching = false;
        geometric_model = "f";    
        predefined_pair_list = "";
        video_mode_matching = -1;
        use_similarity_search = false;
        vot_sift_type = vot::SiftType::OPENMVG_FEAT;
        vot_depth = 6;
        vot_branch_num = 8;
        intrinsic_refinement_option = "ADJUST_ALL";
        cluster_option = "expansion";
        max_cluster_size = 100;
        completeness_ratio = 0.7;
        relax_ratio = 0.35;
        copy_images = false;
        batched_sfm_initilizer = "AUTO_PAIR";
        max_triangulation_repro_err = 4.0;
        align_use_common_cameras = true;
        align_use_common_structures = false;
        perform_global_ba = false;
    }

    // copy constructor
    SfMOption(const SfMOption& sfm_option)
    {
        thread_num = sfm_option.thread_num;
        image_dir = sfm_option.image_dir;
        output_dir = sfm_option.output_dir;
        camera_model_type = sfm_option.camera_model_type;
        sensor_width_database = sfm_option.sensor_width_database;
        feature_quality = sfm_option.feature_quality;
        ui_max_cache_size = sfm_option.ui_max_cache_size;
        distance_ratio = sfm_option.distance_ratio;
        nearest_matching_method = sfm_option.nearest_matching_method;
        ransac_max_iteration = sfm_option.ransac_max_iteration;
        perform_guided_matching = sfm_option.perform_guided_matching;
        predefined_pair_list = sfm_option.predefined_pair_list;
        video_mode_matching = sfm_option.video_mode_matching;
        use_similarity_search = sfm_option.use_similarity_search;
        vot_dir = sfm_option.vot_dir;
        vot_sift_type = sfm_option.vot_sift_type;
        vot_sifts_file = sfm_option.vot_sifts_file;
        vot_depth = sfm_option.vot_depth;
        vot_branch_num = sfm_option.vot_branch_num;
        intrinsic_refinement_option = sfm_option.intrinsic_refinement_option;
        // img_list = sfm_option.img_list;
        // weight_file = sfm_option.weight_file;
        cluster_option = sfm_option.cluster_option;
        max_cluster_size = sfm_option.max_cluster_size;
        completeness_ratio = sfm_option.completeness_ratio;
        relax_ratio = sfm_option.relax_ratio;
        copy_images = sfm_option.copy_images;
        batched_sfm_initilizer = sfm_option.batched_sfm_initilizer;
        max_triangulation_repro_err = sfm_option.max_triangulation_repro_err;
        align_use_common_cameras = sfm_option.align_use_common_cameras;
        align_use_common_structures = sfm_option.align_use_common_structures;
        perform_global_ba = sfm_option.perform_global_ba;
    }
};


class SfMReconstructor
{
private:
    openMVG::sfm::SfM_Data _global_sfm_data; // global sfm data
    SfMOption _sfm_option;

    // Data with openMVG format
    std::vector<std::string> _image_list;
    PairWiseMatches _map_geometric_matches;

    // local information
    std::vector<std::string> _clusters_dir_list;
    std::vector<std::string> _local_sfm_datas_path;
    std::vector<SfM_Data> _local_sfm_datas;
    std::vector<PairWiseMatches> _local_matches;

public:
    SfMReconstructor();
    SfMReconstructor(const SfMOption& sfm_option);
    ~SfMReconstructor();

    void CheckSfMConfigurations() const;

    // Get functions
    PairWiseMatches GetGlobalMatches() const;
    std::vector<std::string> GetClustersDir() const;
    std::vector<std::string> GetLocalSfMDatasPath() const;
    std::vector<SfM_Data> GetLocalSfMDatas() const;
    SfM_Data GetGlobalSfMData() const;

    void InsertMatchesData(const PairWiseMatches& matches);
    void InsertSfMData(const SfM_Data& sfm_data);
    void InsertClusterDir(std::string dir);

    bool ReadSfMConfiguration(const std::string& filename);

    void InitImageListing();

    void ComputeFeatures();

    void SimilaritySearch();

    void ComputeMatches();

    void ImageClustering();

    void LocalIncrementalSfM(const size_t& index);

    void BatchedIncrementalSfM();

    void ComputeStructureFromKnowPoses(const size_t& index);

    void SfMAlign();

    // void ComputeRelativeMotions();

    void ExportLocalSfMDatasPath() const;
    void ImportLocalSfMDatasPath();

    void ComputeSfMDataColor(const size_t& index);

    static bool CreateLineCameraFile(const IndexT camera_id, 
                                 std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic,
                                 std::string& camera_linie);
    static bool CreateCameraFile(const SfM_Data& sfm_data, const std::string& cameras_filename);
    static bool CreateImageFile(const SfM_Data& sfm_data, const std::string& images_filename);
    static bool CreatePoint3DFile(const SfM_Data& sfm_data, const std::string& points3d_filename);
    static bool CreateColmapFolder(const SfM_Data& sfm_data,
                            const std::string& output_dir,
                            const std::string& cameras_filename,
                            const std::string& images_filename, 
                            const std::string& points3d_filename);
    static bool ExportToColmap(const SfM_Data& sfm_data, const std::string& output_dir);

    static bool OpenMVGData2ColmapData(const std::string& sfm_data_file, 
                                       const std::string& output_dir);
    static bool OpenMVGData2ColmapData(const openMVG::sfm::SfM_Data& sfm_data, 
                                       const std::string& output_dir);

private:
    
    void UpdateGlobalMatches(const openMVG::matching::PairWiseMatches& matches);

    std::pair<bool, Vec3> CheckGPS(const std::string& filename,
                                   const int& GPS_to_XYZ_method = 0);
    bool ComputeIndexFromImageNames(const SfM_Data& sfm_data,
                                    const std::pair<std::string,std::string>& initial_pair_name,
                                    Pair& initial_pair_index);

    // bool PairedIndMatchToStream(const PairWiseMatches& map_indexedMatches,
                                // std::ostream& os);
    vector<PairWiseMatches> BuildClusterMatches(const PairWiseMatches& mapMatches,
                                                const std::vector<shared_ptr<ImageGraph>>& igList);
    // void OutputClusterMatches(const std::vector<PairWiseMatches>& matchesList, std::string dir);
    // PairWiseMatches RecoverMatches(std::string fileName);

    // Build a list of pair from the camera frusta intersections
    Pair_Set BuildPairsFromFrustumsIntersections(const SfM_Data& sfm_data,
                                                 const double z_near = -1.,  // default near plane
                                                 const double z_far = -1.);  // default far plane
    
    // Export camera poses positions as a Vec3 vector
    void GetCameraPositions(const SfM_Data& sfm_data, std::vector<Vec3>& cam_positions);

    bool StringToEnumESfMSceneInitializer(const std::string& str,
                                          ESfMSceneInitializer& scene_initializer);

};

}   // namespace sfm
}   // namespace retina

#endif