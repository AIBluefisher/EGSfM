#include "sfm_reconstructor.h"
#include "util/ply_helper.h"

#include "openMVG/sfm/sfm_data_colorization.hpp"
#include "openMVG/sfm/pipelines/relative_pose_engine.hpp"

namespace GraphSfM {
namespace sfm {

SfMReconstructor::SfMReconstructor()
{

}

SfMReconstructor::SfMReconstructor(const SfMOption& sfm_option)
{
    _sfm_option = sfm_option;
}

SfMReconstructor::~SfMReconstructor()
{

}

void SfMReconstructor::CheckSfMConfigurations() const
{
    if (!stlplus::folder_exists(_sfm_option.image_dir)) {
        LOG(ERROR) << "image directory doesn't exist!";
        return;
    }

    if (!stlplus::folder_exists(_sfm_option.output_dir)) {
        if (stlplus::folder_create(_sfm_option.output_dir)) {
            LOG(INFO) << "output directory doesn't exist, and is created with the given value.";
        } else {
            LOG(ERROR) << "output directory doesn't and created failed!";
            return;
        }
    }

    if (!stlplus::folder_exists(_sfm_option.features_dir)) {
        if (stlplus::folder_create(_sfm_option.features_dir)) {
            LOG(INFO) << "features directory doesn't exist, and is created with the given value.";
        } else {
            LOG(ERROR) << "features directory doesn't and created failed!";
            return;
        }
    }

    if (!stlplus::folder_exists(_sfm_option.matches_dir)) {
        if (stlplus::folder_create(_sfm_option.matches_dir)) {
            LOG(INFO) << "matches directory doesn't exist, and is created with the given value.";
        } else {
            LOG(ERROR) << "matches directory doesn't and created failed!";
            return;
        }
    }

    if (!stlplus::file_exists(_sfm_option.sensor_width_database)) {
        LOG(ERROR) << "sensor database file doesn't exist!";
        return;
    }

    if (!isValid(openMVG::cameras::EINTRINSIC(_sfm_option.camera_model_type)))  {
        LOG(ERROR) << "\n Invalid camera type";
        return;
    }
}

PairWiseMatches SfMReconstructor::GetGlobalMatches() const
{
    return _map_geometric_matches;
}

std::vector<std::string> SfMReconstructor::GetClustersDir() const
{
    return _clusters_dir_list;
}

std::vector<std::string> SfMReconstructor::GetLocalSfMDatasPath() const
{
    return _local_sfm_datas_path;
}

std::vector<SfM_Data> SfMReconstructor::GetLocalSfMDatas() const
{
    return _local_sfm_datas;
}

SfM_Data SfMReconstructor::GetGlobalSfMData() const
{
    return _global_sfm_data;
}

void SfMReconstructor::InsertMatchesData(const PairWiseMatches& matches)
{
    _local_matches.emplace_back(matches);
}

void SfMReconstructor::InsertSfMData(const SfM_Data& sfm_data)
{
    _local_sfm_datas.emplace_back(sfm_data);
}

void SfMReconstructor::InsertClusterDir(std::string dir)
{
    _clusters_dir_list.push_back(dir);
}

bool SfMReconstructor::ReadSfMConfiguration(const std::string& filename)
{
    // TODO:
    return false;
}

void SfMReconstructor::InitImageListing()
{
    // Expected properties for each image
    double width = -1, height = -1, focal = -1, ppx = -1,  ppy = -1;

    const EINTRINSIC camera_model = EINTRINSIC(_sfm_option.camera_model_type);

    if (!stlplus::folder_exists(_sfm_option.image_dir) ) {
        LOG(ERROR) << "\nThe image directory doesn't exist";
        return;
    }

    if (_sfm_option.output_dir.empty()) {
        LOG(ERROR) << "\nInvalid output directory";
        return;
    }

    if (!stlplus::folder_exists(_sfm_option.output_dir)) {
        if (!stlplus::folder_create(_sfm_option.output_dir)) {
            LOG(ERROR) << "\nCannot create output directory";
            return;
        }
    }

    std::vector<Datasheet> vec_database;
    if (!_sfm_option.sensor_width_database.empty()) {
        if (!parseDatabase(_sfm_option.sensor_width_database, vec_database)) {
            LOG(ERROR) << "\nInvalid input database: " << _sfm_option.sensor_width_database
                       << ", please specify a valid file.";
            return;
        }
    }

    std::vector<std::string> vec_image = stlplus::folder_files(_sfm_option.image_dir);
    std::sort(vec_image.begin(), vec_image.end());

    // Configure an empty scene with Views and their corresponding cameras
    _global_sfm_data.s_root_path = _sfm_option.image_dir;
    Views& views = _global_sfm_data.views;
    Intrinsics& intrinsics = _global_sfm_data.intrinsics;

    C_Progress_display progress_bar( vec_image.size(), std::cout, "\n- Image Listing -\n");
    std::ostringstream error_report_stream;
    for (auto ite_img = vec_image.begin(); ite_img != vec_image.end(); ++ite_img, ++progress_bar) {
        // Read meta data to fill camera parameter (w,h,focal,ppx,ppy) fields.
        width = height = ppx = ppy = focal = -1.0;

        const std::string image_filename = stlplus::create_filespec(_sfm_option.image_dir, *ite_img);
        _image_list.push_back(image_filename);

        const std::string sImageFilename = stlplus::create_filespec(_sfm_option.image_dir, *ite_img );
        const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

        // Test if the image format is supported:
        if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown) {
            error_report_stream << sImFilenamePart << ": Unkown image file format." << "\n";
            continue; // image cannot be opened
        }

        if (sImFilenamePart.find("mask.png") != std::string::npos || 
            sImFilenamePart.find("_mask.png") != std::string::npos) {
            error_report_stream << sImFilenamePart << " is a mask image" << "\n";
            continue;
        }

        ImageHeader img_header;
        if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &img_header)) {
            continue; // image cannot be read
        }

        width = img_header.width;
        height = img_header.height;
        ppx = width / 2.0;
        ppy = height / 2.0;

        // Consider the case where the focal is provided manually
        // if (sKmatrix.size() > 0) { // Known user calibration K matrix
        //     if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
        //         focal = -1.0;
        // } else { // User provided focal length value
        //     if (focal_pixels != -1 ) focal = focal_pixels;
        // }

        // If not manually provided or wrongly provided
        if (focal == -1) {
            std::unique_ptr<Exif_IO> exif_reader(new Exif_IO_EasyExif);
            exif_reader->open(sImageFilename);

            const bool bHaveValidExifMetadata = exif_reader->doesHaveExifInfo() && 
                                                !exif_reader->getModel().empty();

            if (bHaveValidExifMetadata) { // If image contains meta data
                const std::string sCamModel = exif_reader->getModel();

                // Handle case where focal length is equal to 0
                if (exif_reader->getFocal() == 0.0f) {
                  error_report_stream << stlplus::basename_part(sImageFilename) 
                                      << ": Focal length is missing.\n";
                  focal = -1.0;
                } else { // Create the image entry in the list file
                    Datasheet datasheet;
                    if (getInfo( sCamModel, vec_database, datasheet)) {
                        // The camera model was found in the database so we can compute 
                        // it's approximated focal length
                        const double ccdw = datasheet.sensorSize_;
                        focal = std::max(width, height) * exif_reader->getFocal() / ccdw;
                    } else {
                        error_report_stream << stlplus::basename_part(sImageFilename)
                                            << "\" model \"" << sCamModel << "\" doesn't exist in the database\n"
                                            << "Please consider add your camera model and sensor width in the database.\n";
                    }
                }
            }
        }
        // Build intrinsic parameter related to the view
        std::shared_ptr<IntrinsicBase> intrinsic;
        if(focal < 0) {
            focal = std::max(width, height);
            error_report_stream << "Image " << sImageFilename 
                                << " focal length doesn't exist, set it to : " << focal << "\n";
        }

        if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0) {
            // Create the desired camera type
            switch (camera_model) {
                case PINHOLE_CAMERA:
                    intrinsic = std::make_shared<Pinhole_Intrinsic>(width, height, focal, ppx, ppy);
                    break;
                case PINHOLE_CAMERA_RADIAL1:
                    // setup no distortion as initial guess
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K1>
                        (width, height, focal, ppx, ppy, 0.0);
                    break;
                case PINHOLE_CAMERA_RADIAL3:
                    // setup no distortion as initial guess
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
                        (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);  
                    break;
                case PINHOLE_CAMERA_BROWN:
                    // setup no distortion as initial guess
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Brown_T2>
                      (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0, 0.0); 
                    break;
                case PINHOLE_CAMERA_FISHEYE:
                    // setup no distortion as initial guess
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Fisheye>
                      (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0); 
                    break;
                case CAMERA_SPHERICAL:
                    intrinsic = std::make_shared<Intrinsic_Spherical>(width, height);
                    break;
                default:
                    LOG(ERROR) << "Error: unknown camera model: " << (int)camera_model;
                    return;
            }
        }

        // Build the view corresponding to the image
        View v(*ite_img, views.size(), views.size(), views.size(), width, height);

        // Add intrinsic related to the image (if any)
        if (intrinsic == nullptr) {
            // Since the view have invalid intrinsic data
            // (export the view, with an invalid intrinsic field value)
            v.id_intrinsic = UndefinedIndexT;
        } else {
            // Add the defined intrinsic to the sfm_container
            intrinsics[v.id_intrinsic] = intrinsic;
        }

        // Add the view to the sfm_container
        views[v.id_view] = std::make_shared<View>(v);
    }

    // Display saved warning & error messages if any.
    if (!error_report_stream.str().empty()) {
        LOG(WARNING) << "\nWarning & Error messages:\n"
                   << error_report_stream.str();
    }

    // Group camera that share common properties if desired (leads to more faster & stable BA)
    if (_sfm_option.group_camera_model) {
        GroupSharedIntrinsics(_global_sfm_data);
    }

    LOG(INFO) << "\nSfM InitImageListing Report:"
              << "\nlisted #File(s): " << vec_image.size()
              << "\nusable #File(s) listed in sfm_data: " << _global_sfm_data.GetViews().size()
              << "\nusable #Intrinsic(s) listed in sfm_data: " << _global_sfm_data.GetIntrinsics().size();
}

void SfMReconstructor::ComputeFeatures()
{
    bool up_right = false; // Use Upright feature 0 or 1
    bool force = false;

    // a. Init the image_describer
    // - retrieve the used one in case of pre-computed features
    // - else create the desired one
    using namespace openMVG::features;
    std::unique_ptr<Image_describer> image_describer;

    const std::string image_describer_file = 
        stlplus::create_filespec(_sfm_option.features_dir, "image_describer", "json");
    if (!force && stlplus::is_file(image_describer_file)) {
        // Dynamically load the image_describer from the file (will restore old used settings)
        std::ifstream stream(image_describer_file.c_str());
        if (!stream.is_open()) return;    
        try {
            cereal::JSONInputArchive archive(stream);
            archive(cereal::make_nvp("image_describer", image_describer));
        } catch (const cereal::Exception & e) {
            LOG(ERROR) << e.what()
                       << "\nCannot dynamically allocate the Image_describer interface.";
            return;
        }
    } else {
        // Create the desired Image_describer method.
        // Don't use a factory, perform direct allocation
        image_describer.reset(new SIFT_Image_describer(SIFT_Image_describer::Params(), !up_right));
        if (!image_describer) {
            LOG(ERROR) << "Cannot create the SIFT Image_describer:";
            return;
        } else {
            if (!image_describer->Set_configuration_preset((features::EDESCRIBER_PRESET)_sfm_option.feature_quality)) {
                LOG(ERROR) << "Preset configuration failed.";
                return;
            }
        }

        // Export the used Image_describer and region type for:
        // - dynamic future regions computation and/or loading
        {
          std::ofstream stream(image_describer_file.c_str());
          if (!stream.is_open()) return;

          cereal::JSONOutputArchive archive(stream);
          archive(cereal::make_nvp("image_describer", image_describer));
          auto regionsType = image_describer->Allocate();
          archive(cereal::make_nvp("regions_type", regionsType));
        }
    }

    // Feature extraction routines
    // For each View of the SfM_Data container:
    // - if regions file exists continue,
    // - if no file, compute features
    {
        system::Timer timer;
        Image<unsigned char> imageGray;

        C_Progress_display progress_bar(_global_sfm_data.GetViews().size(), std::cout, "\n- EXTRACT FEATURES -\n" );

        // Use a boolean to track if we must stop feature extraction
        std::atomic<bool> preemptive_exit(false);
        #ifdef _USE_OPENMP
            if (_sfm_option.thread_num < 0) {
                _sfm_option.thread_num = omp_get_max_threads();
            }
            omp_set_num_threads(_sfm_option.thread_num);

            #pragma omp parallel for schedule(dynamic) if (iNumThreads > 0) private(imageGray)
        #endif

        for (int i = 0; i < static_cast<int>(_global_sfm_data.views.size()); ++i) {
            Views::const_iterator ite_views = _global_sfm_data.views.begin();
            std::advance(ite_views, i);
            const View* view = ite_views->second.get();
            const std::string sView_filename = stlplus::create_filespec(_global_sfm_data.s_root_path, view->s_Img_path),
              sFeat = stlplus::create_filespec(_sfm_option.features_dir, stlplus::basename_part(sView_filename), "feat"),
              sDesc = stlplus::create_filespec(_sfm_option.features_dir, stlplus::basename_part(sView_filename), "desc");

            // If features or descriptors file are missing, compute them
            if (!preemptive_exit && 
                (force || !stlplus::file_exists(sFeat) || !stlplus::file_exists(sDesc))) {
                if (!ReadImage(sView_filename.c_str(), &imageGray)) continue;

                // Look if there is occlusion feature mask
                Image<unsigned char> * mask = nullptr; // The mask is null by default

                const std::string mask_filename_local = stlplus::create_filespec(_global_sfm_data.s_root_path, 
                                                        stlplus::basename_part(sView_filename) + "_mask", "png");
                const std::string mask__filename_global = stlplus::create_filespec(_global_sfm_data.s_root_path, "mask", "png");

                Image<unsigned char> image_mask;
                // Try to read the local mask
                if (stlplus::file_exists(mask_filename_local)) {
                    if (!ReadImage(mask_filename_local.c_str(), &image_mask)) {
                        LOG(ERROR) << "Invalid mask: " << mask_filename_local
                                   << "\nStopping feature extraction.";
                        preemptive_exit = true;
                        continue;
                    }
                    // Use the local mask only if it fits the current image size
                    if (image_mask.Width() == imageGray.Width() && image_mask.Height() == imageGray.Height())
                        mask = &image_mask;
                } else {
                    // Try to read the global mask
                    if (stlplus::file_exists(mask__filename_global)) {
                        if (!ReadImage(mask__filename_global.c_str(), &image_mask)) {
                            LOG(ERROR) << "Invalid mask: " << mask__filename_global
                                      << "\nStopping feature extraction.";
                            preemptive_exit = true;
                            continue;
                        }
                        // Use the global mask only if it fits the current image size
                        if (image_mask.Width() == imageGray.Width() && image_mask.Height() == imageGray.Height())
                            mask = &image_mask;
                    }
                }

                // Compute features and descriptors and export them to files
                auto regions = image_describer->Describe(imageGray, mask);
                if (regions && !image_describer->Save(regions.get(), sFeat, sDesc)) {
                    LOG(ERROR) << "Cannot save regions for images: " << sView_filename
                               << "\nStopping feature extraction.";
                    preemptive_exit = true;
                    continue;
                }
            }
            ++progress_bar;
        }
        LOG(INFO) << "Task done in (s): " << timer.elapsed();
    }
}

void SfMReconstructor::ComputeMatches()
{
    EPairMode pair_mode = (_sfm_option.video_mode_matching == -1 ) ? PAIR_EXHAUSTIVE : PAIR_CONTIGUOUS;

    if (_sfm_option.predefined_pair_list.length()) {
        pair_mode = PAIR_FROM_FILE;
        if (_sfm_option.video_mode_matching > 0) {
            LOG(ERROR) << "\nIncompatible options: --videoModeMatching and --pairList";
            return;
        }
    }

    // if (_sfm_option.matches_dir.empty() || !stlplus::is_folder(_sfm_option.matches_dir))  {
    //     LOG(ERROR) << "It is an invalid output directory";
    //     return;
    // }

    EGeometricModel eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
    std::string s_geometric_matches_filename = "";
    switch (_sfm_option.geometric_model[0]) {
        case 'f': 
        case 'F':
            eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
            s_geometric_matches_filename = "matches.f.bin";
            break;
        case 'e': 
        case 'E':
            eGeometricModelToCompute = ESSENTIAL_MATRIX;
            s_geometric_matches_filename = "matches.e.bin";
            break;
        case 'h': 
        case 'H':
            eGeometricModelToCompute = HOMOGRAPHY_MATRIX;
            s_geometric_matches_filename = "matches.h.bin";
            break;
        case 'a': 
        case 'A':
            eGeometricModelToCompute = ESSENTIAL_MATRIX_ANGULAR;
            s_geometric_matches_filename = "matches.f.bin";
            break;
        case 'o': 
        case 'O':
            eGeometricModelToCompute = ESSENTIAL_MATRIX_ORTHO;
            s_geometric_matches_filename = "matches.o.bin";
            break;
        default:
            LOG(ERROR) << "Unknown geometric model";
            return;
    }

    // a. Compute putative descriptor matches
    // b. Geometric filtering of putative matches
    // + Export some statistics

    // Load SfM Scene regions
    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    const std::string image_describer = 
        stlplus::create_filespec(_sfm_option.features_dir, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(image_describer);
    if (!regions_type) {
        LOG(ERROR) << "Invalid: " << image_describer << " regions type file.";
        return;
    }

    //---------------------------------------
    // a. Compute putative descriptor matches
    //    - Descriptor matching (according user method choice)
    //    - Keep correspondences only if NearestNeighbor ratio is ok
    //---------------------------------------

    // Load the corresponding view regions
    std::shared_ptr<Regions_Provider> regions_provider;
    if (_sfm_option.ui_max_cache_size == 0) {
        // Default regions provider (load & store all regions in memory)
        regions_provider = std::make_shared<Regions_Provider>();
    } else {
        // Cached regions provider (load & store regions on demand)
        regions_provider = std::make_shared<Regions_Provider_Cache>(_sfm_option.ui_max_cache_size);
    }

    C_Progress_display progress;
    if (!regions_provider->load(_global_sfm_data, _sfm_option.features_dir, regions_type, &progress)) {
        LOG(ERROR) << "\nInvalid regions.";
        return;
    }

    PairWiseMatches map_putative_matches;

    // Build some alias from SfM_Data Views data:
    // - List views as a vector of filenames & image sizes
    std::vector<std::string> vec_file_names;
    std::vector<std::pair<size_t, size_t>> vec_images_size;
    {
        vec_file_names.reserve(_global_sfm_data.GetViews().size());
        vec_images_size.reserve(_global_sfm_data.GetViews().size());
        for (auto iter = _global_sfm_data.GetViews().begin(); iter != _global_sfm_data.GetViews().end(); ++iter) {
            const View* v = iter->second.get();
            vec_file_names.push_back(stlplus::create_filespec(_global_sfm_data.s_root_path, v->s_Img_path));
            vec_images_size.push_back( std::make_pair(v->ui_width, v->ui_height));
        }
    }

    LOG(INFO) << "\n - PUTATIVE MATCHES - ";
    // If the matches already exists, reload them
    if (stlplus::file_exists(_sfm_option.matches_dir + "/matches.putative.txt") || 
        stlplus::file_exists(_sfm_option.matches_dir + "/matches.putative.bin")) {
        if (!(Load(map_putative_matches, _sfm_option.matches_dir + "/matches.putative.bin") ||
            Load(map_putative_matches, _sfm_option.matches_dir + "/matches.putative.txt"))) {
            LOG(ERROR) << "Cannot load input matches file";
            return;
        }
        LOG(INFO) << "\t PREVIOUS RESULTS LOADED;" << " #pair: " << map_putative_matches.size();
    } else { // Compute the putative matches
        LOG(INFO) << "Use: ";
        switch (pair_mode) {
            case PAIR_EXHAUSTIVE: 
                LOG(INFO) << "exhaustive pairwise matching";
                break;
            case PAIR_CONTIGUOUS: 
                LOG(INFO) << "sequence pairwise matching"; 
                break;
            case PAIR_FROM_FILE:  
                LOG(INFO) << "user defined pairwise matching"; 
                break;
        }

        // Allocate the right Matcher according the Matching requested method
        std::unique_ptr<Matcher> collection_matcher;
        if (_sfm_option.nearest_matching_method == "AUTO") {
            if (regions_type->IsScalar()) {
                LOG(INFO) << "Using FAST_CASCADE_HASHING_L2 matcher";
                collection_matcher.reset(new Cascade_Hashing_Matcher_Regions(_sfm_option.distance_ratio));
            }
            else if (regions_type->IsBinary()) {
                LOG(INFO) << "Using BRUTE_FORCE_HAMMING matcher";
                collection_matcher.reset(new Matcher_Regions(_sfm_option.distance_ratio, BRUTE_FORCE_HAMMING));
            }
        } else if (_sfm_option.nearest_matching_method == "BRUTE_FORCE_L2") {
            LOG(INFO) << "Using BRUTE_FORCE_L2 matcher";
            collection_matcher.reset(new Matcher_Regions(_sfm_option.distance_ratio, BRUTE_FORCE_L2));
        } else if (_sfm_option.nearest_matching_method == "BRUTE_FORCE_HAMMING") {
            LOG(INFO) << "Using BRUTE_FORCE_HAMMING matcher";
            collection_matcher.reset(new Matcher_Regions(_sfm_option.distance_ratio, BRUTE_FORCE_HAMMING));
        } else if (_sfm_option.nearest_matching_method == "ANN_L2") {
            LOG(INFO) << "Using ANN_L2 matcher";
            collection_matcher.reset(new Matcher_Regions(_sfm_option.distance_ratio, ANN_L2));
        } else if (_sfm_option.nearest_matching_method == "CASCADE_HASHING_L2") {
            LOG(INFO) << "Using CASCADE_HASHING_L2 matcher";
            collection_matcher.reset(new Matcher_Regions(_sfm_option.distance_ratio, CASCADE_HASHING_L2));
        } else if (_sfm_option.nearest_matching_method == "FAST_CASCADE_HASHING_L2") {
            LOG(INFO) << "Using FAST_CASCADE_HASHING_L2 matcher";
            collection_matcher.reset(new Cascade_Hashing_Matcher_Regions(_sfm_option.distance_ratio));
        }
        if (!collection_matcher) {
            LOG(ERROR) << "Invalid Nearest Neighbor method: " << _sfm_option.nearest_matching_method;
            return;
        }
        // Perform the matching
        system::Timer timer;
        {
            // From matching mode compute the pair list that have to be matched:
            Pair_Set pairs;
            switch (pair_mode) {
                case PAIR_EXHAUSTIVE: 
                    pairs = exhaustivePairs(_global_sfm_data.GetViews().size()); 
                    break;
                case PAIR_CONTIGUOUS: 
                    pairs = contiguousWithOverlap(_global_sfm_data.GetViews().size(), _sfm_option.video_mode_matching); 
                    break;
                case PAIR_FROM_FILE:
                    if (!loadPairs(_global_sfm_data.GetViews().size(), _sfm_option.predefined_pair_list, pairs)) {
                        return;
                    }
                    break;
            }
            // Photometric matching of putative pairs
            collection_matcher->Match(regions_provider, pairs, map_putative_matches, &progress);
            //---------------------------------------
            //-- Export putative matches
            //---------------------------------------
            if (!Save(map_putative_matches, std::string(_sfm_option.matches_dir + "/matches.putative.bin"))) {
                LOG(ERROR) << "Cannot save computed matches in: "
                          << std::string(_sfm_option.matches_dir + "/matches.putative.bin");
                return;
            }
        }
        LOG(INFO) << "Task (Regions Matching) done in (s): " << timer.elapsed();
    }
    //-- export putative matches Adjacency matrix
    PairWiseMatchingToAdjacencyMatrixSVG(vec_file_names.size(),
      map_putative_matches,
      stlplus::create_filespec(_sfm_option.matches_dir, "PutativeAdjacencyMatrix", "svg"));
    //-- export view pair graph once putative graph matches have been computed
    {
        std::set<IndexT> set_ViewIds;
        std::transform(_global_sfm_data.GetViews().begin(), _global_sfm_data.GetViews().end(),
          std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
        openMVG::graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_putative_matches));
        openMVG::graph::exportToGraphvizData(
          stlplus::create_filespec(_sfm_option.matches_dir, "putative_matches"),
          putativeGraph);
    }

    //---------------------------------------
    // b. Geometric filtering of putative matches
    //    - AContrario Estimation of the desired geometric model
    //    - Use an upper bound for the a contrario estimated threshold
    //---------------------------------------

    std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(
      new ImageCollectionGeometricFilter(&_global_sfm_data, regions_provider));

    if (filter_ptr) {
        system::Timer timer;
        if (stlplus::file_exists(_sfm_option.matches_dir + "/matches.f.txt") || 
            stlplus::file_exists(_sfm_option.matches_dir + "/matches.f.bin")) {
            if (!(Load(_map_geometric_matches , _sfm_option.matches_dir + "/matches.f.bin") ||
                Load(_map_geometric_matches , _sfm_option.matches_dir + "/matches.f.txt"))) {
                LOG(ERROR) << "Cannot load input matches file";
                return;
            }
            LOG(INFO) << "\t PREVIOUS GEOMETRIC FILTERED RESULTS LOADED;" 
                      << " #pair: " << _map_geometric_matches.size();
        } else {
            const double d_distance_ratio = 0.6;

            // PairWiseMatches map_geometric_matches;
            switch (eGeometricModelToCompute) {
                case HOMOGRAPHY_MATRIX:
                {
                    const bool geometric_only_guided_matching = true;
                    filter_ptr->Robust_model_estimation(
                                GeometricFilter_HMatrix_AC(4.0, _sfm_option.ransac_max_iteration),
                                map_putative_matches,
                                _sfm_option.perform_guided_matching,
                                geometric_only_guided_matching ? -1.0 : d_distance_ratio, 
                                &progress);
                    _map_geometric_matches = filter_ptr->Get_geometric_matches();
                    break;
                }
                case FUNDAMENTAL_MATRIX:
                {
                    filter_ptr->Robust_model_estimation(
                                GeometricFilter_FMatrix_AC(4.0, _sfm_option.ransac_max_iteration),
                                map_putative_matches,
                                _sfm_option.perform_guided_matching, 
                                d_distance_ratio, 
                                &progress);
                    _map_geometric_matches = filter_ptr->Get_geometric_matches();
                    break;
                }
                case ESSENTIAL_MATRIX:
                {
                    filter_ptr->Robust_model_estimation(
                                GeometricFilter_EMatrix_AC(4.0, _sfm_option.ransac_max_iteration),
                                map_putative_matches,
                                _sfm_option.perform_guided_matching, 
                                d_distance_ratio, 
                                &progress);
                    _map_geometric_matches = filter_ptr->Get_geometric_matches();

                    //-- Perform an additional check to remove pairs with poor overlap
                    std::vector<PairWiseMatches::key_type> vec_toRemove;
                    for (const auto & pairwisematches_it : _map_geometric_matches) {
                        const size_t putativePhotometricCount = 
                          map_putative_matches.find(pairwisematches_it.first)->second.size();
                        const size_t putativeGeometricCount = pairwisematches_it.second.size();
                        const float ratio = putativeGeometricCount / static_cast<float>(putativePhotometricCount);
                        if (putativeGeometricCount < 50 || ratio < .3f)  {
                          // the pair will be removed
                          vec_toRemove.push_back(pairwisematches_it.first);
                        }
                    }
                    //-- remove discarded pairs
                    for (const auto & pair_to_remove_it : vec_toRemove) {
                        _map_geometric_matches.erase(pair_to_remove_it);
                    }
                    break;
                }
                case ESSENTIAL_MATRIX_ANGULAR:
                {
                    filter_ptr->Robust_model_estimation(
                                GeometricFilter_ESphericalMatrix_AC_Angular(4.0, _sfm_option.ransac_max_iteration),
                                map_putative_matches,
                                _sfm_option.perform_guided_matching);
                    _map_geometric_matches = filter_ptr->Get_geometric_matches();
                    break;
                }
                case ESSENTIAL_MATRIX_ORTHO:
                {
                    filter_ptr->Robust_model_estimation(
                                GeometricFilter_EOMatrix_RA(2.0, _sfm_option.ransac_max_iteration),
                                map_putative_matches,
                                _sfm_option.perform_guided_matching, 
                                d_distance_ratio, 
                                &progress);
                    _map_geometric_matches = filter_ptr->Get_geometric_matches();
                    break;
                }
            }
        }

        //-- Export geometric filtered matches
        if (!Save(_map_geometric_matches,
            std::string(_sfm_option.matches_dir + "/" + s_geometric_matches_filename))) {
            LOG(ERROR) << "Cannot save computed matches in: "
                       << std::string(_sfm_option.matches_dir + "/" + s_geometric_matches_filename);
            return;
        }

        LOG(INFO) << "Task done in (s): " << timer.elapsed();

        ExportTracks(_map_geometric_matches, _sfm_option.matches_dir);

        //-- export Adjacency matrix
        LOG(INFO) << "\n Export Adjacency Matrix of the pairwise's geometric matches";
        PairWiseMatchingToAdjacencyMatrixSVG(vec_file_names.size(),
            _map_geometric_matches,
            stlplus::create_filespec(_sfm_option.matches_dir, "GeometricAdjacencyMatrix", "svg"));

        //-- export view pair graph once geometric filter have been done
        {
            std::set<IndexT> set_ViewIds;
            std::transform(_global_sfm_data.GetViews().begin(), _global_sfm_data.GetViews().end(),
              std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
            openMVG::graph::indexedGraph putativeGraph(set_ViewIds, getPairs(_map_geometric_matches));
            openMVG::graph::exportToGraphvizData(
              stlplus::create_filespec(_sfm_option.matches_dir, "geometric_matches"),
              putativeGraph);
        }
    }
}

void SfMReconstructor::SimilaritySearch()
{
	// optional parameters
	int thread_num = std::thread::hardware_concurrency();
	int start_id = 0;
	int num_matches = 100;

    _sfm_option.vot_dir = _sfm_option.output_dir + "/vot";
    if (!stlplus::folder_create(_sfm_option.vot_dir)) {
        LOG(ERROR) << _sfm_option.vot_dir << "cannot be created!";
        return;
    }

    std::string tree_output = _sfm_option.vot_dir + std::string("/tree.out");
	std::string db_output = _sfm_option.vot_dir + std::string("/db.out");
	std::string match_output = _sfm_option.vot_dir + std::string("/match.out");
	std::string filtered_output = _sfm_option.vot_dir + std::string("/match_pairs");
	const std::string svg_adjacency_matrix = _sfm_option.vot_dir + std::string("/adjacency_matrix.svg");

	if (!vot::BuildVocabTree(_sfm_option.vot_sifts_file.c_str(), 
                             tree_output.c_str(), 
                             _sfm_option.vot_depth, 
                             _sfm_option.vot_branch_num, 
                             _sfm_option.vot_sift_type, 
                             _sfm_option.thread_num))
		return;
	if (!vot::BuildImageDatabase(_sfm_option.vot_sifts_file.c_str(), 
                                 tree_output.c_str(), 
                                 db_output.c_str(), 
                                 _sfm_option.vot_sift_type, 
                                 start_id, 
                                 _sfm_option.thread_num))
		return;
	if (!vot::QueryDatabase(db_output.c_str(), 
                            _sfm_option.vot_sifts_file.c_str(), 
                            match_output.c_str(), 
                            _sfm_option.vot_sift_type, 
                            _sfm_option.thread_num))
		return;
	if(!vot::FilterMatchList(_sfm_option.vot_sifts_file.c_str(), 
                             match_output.c_str(), 
                             filtered_output.c_str(), 
                             num_matches, 
                             svg_adjacency_matrix.c_str()))
		return;
}

void SfMReconstructor::ImageClustering()
{
    LOG(INFO) << "\nImage Clusering";

    ImageClusterOption image_cluster_option((size_t)_sfm_option.max_cluster_size, 
                                            _sfm_option.completeness_ratio, 
                                            _sfm_option.relax_ratio, 
                                            _sfm_option.copy_images);

    ImageCluster image_cluster(image_cluster_option);

    if (_image_list.empty()) {
        LOG(ERROR) << "No image listed!";
        return;
    }

    ImageGraph img_graph;
    for (int i = 0; i < _image_list.size(); i++) {
        graph::ImageNode inode(i, _image_list[i]);
        img_graph.AddNode(inode);
    }

    for (auto iter = _map_geometric_matches.begin(); iter!= _map_geometric_matches.end(); ++iter) {
        size_t src = iter->first.first, dst = iter->first.second;
        double weight = (double)iter->second.size();
        img_graph.AddEdge(Edge(src, dst, weight));
        img_graph.AddEdge(Edge(dst, src, weight));
    }

    LOG(INFO) << "Node size(Before remove singleton nodes): " << img_graph.GetSize();
    img_graph.CountInDegrees();
    img_graph.CountOutDegrees();
    img_graph.CountDegrees();
    img_graph.RemoveSingletonNodes();
    LOG(INFO) << "Node size(After remove singleton nodes): " << img_graph.GetSize();

#ifdef __DEBUG__
    img_graph.ShowInfo();
#endif

    size_t clust_num = 1;
    LOG(INFO) << "nodes: " << img_graph.GetNodesNum();
    LOG(INFO) << "cluster capacity: " << image_cluster_option.cluster_upper_size;
    if(img_graph.GetNodesNum() < image_cluster_option.cluster_upper_size) {
        LOG(ERROR) << "size of graphs less than cluster size, camera cluster is the origin one";
    } else {
        // clust_num = img_graph.GetNodesNum() / image_cluster_option.cluster_upper_size;
        clust_num = 2;
    }

    std::unordered_map<int, int> cluster_map = img_graph.NormalizedCut(clust_num);
    queue<shared_ptr<ImageGraph>> sub_image_graphs = 
        image_cluster.ConstructSubGraphs(img_graph, cluster_map, clust_num);
    image_cluster.CollectDiscardedEdges(img_graph, sub_image_graphs);

    if(_sfm_option.cluster_option == "naive") {
        image_cluster.NaiveImageCluster(sub_image_graphs, _sfm_option.image_dir, clust_num);
    } else if(_sfm_option.cluster_option == "expansion") {
        vector<shared_ptr<ImageGraph>> insize_graphs = 
            image_cluster.ExpanImageCluster(img_graph, sub_image_graphs);

        // image_cluster.MoveImages(insize_graphs, _sfm_option.image_dir, _sfm_option.output_dir);
        for (int i = 0; i < insize_graphs.size(); i++) {
            auto img_nodes = insize_graphs[i]->GetNodes();
            string cluster_dir = _sfm_option.image_dir + "/cluster_part_" + std::to_string(i);
            _clusters_dir_list.push_back(cluster_dir);
            // sfmdata_list << partial_dir + "/reconstruction_sequential/robust.json\n";
            _local_sfm_datas.push_back(image_cluster.GenerateSfMData(_global_sfm_data, img_nodes, cluster_dir));

            if(!stlplus::folder_create(cluster_dir)) {
                LOG(WARNING) << "cluster part " << i << " cannot be created!";
            }
            insize_graphs[i]->ShowInfo(cluster_dir + "/image_cluster.txt");
        }
        
        _local_matches = BuildClusterMatches(_map_geometric_matches, insize_graphs);
        // OutputClusterMatches(_local_matches, _sfm_option.image_dir);
    } else {
        LOG(ERROR) << "cluster_option must be 'naive' or 'expansion'\n";
    }
}

void SfMReconstructor::LocalIncrementalSfM(const size_t& index)
{
    LOG(INFO) << "\nPerforming " << index << "-th reconstruction"; 

    SfM_Data sfm_data = _local_sfm_datas[index];
    PairWiseMatches matches = _local_matches[index];
    std::pair<std::string, std::string> initialPairString("", "");
    std::string local_output_dir = _clusters_dir_list[index];

    if (!stlplus::folder_exists(local_output_dir)) {
        stlplus::folder_create(local_output_dir);
    }

    const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
      cameras::StringTo_Intrinsic_Parameter_Type(_sfm_option.intrinsic_refinement_option);
    if (intrinsic_refinement_options == static_cast<cameras::Intrinsic_Parameter_Type>(0)) {
        LOG(ERROR) << "Invalid input for Bundle Adjusment Intrinsic parameter refinement option";
        return;
    }

    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    const std::string sImage_describer = 
        stlplus::create_filespec(_sfm_option.features_dir, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type) {
        LOG(ERROR) << "Invalid: " << sImage_describer << " regions type file.";
        return;
    }

    // Features reading
    std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
    if (!feats_provider->load(sfm_data, _sfm_option.features_dir, regions_type)) {
        LOG(ERROR) << "\nInvalid features.";
        return;
    }
    // Matches reading
    std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
    matches_provider->pairWise_matches_ = matches;

    // Sequential reconstruction process
    openMVG::system::Timer timer;
    SequentialSfMReconstructionEngine sfm_engine(
        sfm_data,
        local_output_dir,
        stlplus::create_filespec(local_output_dir, "Reconstruction_Report.html"));

    // Configure the features_provider & the matches_provider
    sfm_engine.SetFeaturesProvider(feats_provider.get());
    sfm_engine.SetMatchesProvider(matches_provider.get());
    sfm_engine.SetMatchesDir(_sfm_option.matches_dir);

    // Configure reconstruction parameters
    sfm_engine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    sfm_engine.SetUnknownCameraType(EINTRINSIC(_sfm_option.camera_model_type));

    // Handle Initial pair parameter
    if (!initialPairString.first.empty() && !initialPairString.second.empty()) {
        Pair initial_pair_index;
        if (!ComputeIndexFromImageNames(sfm_data, initialPairString, initial_pair_index)) {
            LOG(ERROR) << "Could not find the initial pairs <" 
                       << initialPairString.first
                       <<  ", " << initialPairString.second << ">!";
            return;
        }
        sfm_engine.setInitialPair(initial_pair_index);
    }

    if (sfm_engine.Process()) {
        // sfm_engine.UpdateGlobalMatches();
        // this->UpdateGlobalMatches(sfm_engine.GetMatches());

        LOG(INFO) << "\n Total Ac-Sfm took (s): " << timer.elapsed(); 
        LOG(INFO) << "Generating SfM_Report.html...";

        // reset sfm_data
        _local_sfm_datas[index] = sfm_engine.Get_SfM_Data();

        Generate_SfM_Report(sfm_engine.Get_SfM_Data(),
            stlplus::create_filespec(local_output_dir, "SfMReconstruction_Report.html"));    
        // Export to disk computed scene (data & visualizable results)
        LOG(INFO) << "Export SfM_Data to disk...";
        Save(sfm_engine.Get_SfM_Data(),
             stlplus::create_filespec(local_output_dir, "sfm_data", ".bin"),
             ESfM_Data(ALL));    
        Save(sfm_engine.Get_SfM_Data(), 
             stlplus::create_filespec(local_output_dir, "cloud_and_poses", ".ply"),
             ESfM_Data(ALL));
    }
}

void SfMReconstructor::BatchedIncrementalSfM()
{
    if (!isValid(openMVG::cameras::EINTRINSIC(_sfm_option.camera_model_type)) )  {
        LOG(ERROR) << "\n Invalid camera type";
        return;
    }

    const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
      cameras::StringTo_Intrinsic_Parameter_Type(_sfm_option.intrinsic_refinement_option);
    if (intrinsic_refinement_options == static_cast<cameras::Intrinsic_Parameter_Type>(0)) {
        LOG(ERROR) << "Invalid input for the Bundle Adjusment Intrinsic parameter refinement option";
        return;
    }

    ESfMSceneInitializer scene_initializer_enum;
    if (!StringToEnumESfMSceneInitializer(_sfm_option.batched_sfm_initilizer, 
                                          scene_initializer_enum)) {
        LOG(ERROR) << "Invalid input for the SfM initializer option";
        return;
    }

    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    const std::string sImage_describer = 
        stlplus::create_filespec(_sfm_option.features_dir, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type) {
        LOG(ERROR) << "Invalid: " << sImage_describer << " regions type file.";
        return;
    }

    // Features reading
    std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
    if (!feats_provider->load(_global_sfm_data, _sfm_option.features_dir, regions_type)) {
        std::cerr << "Invalid features.";
        return;
    }
    // Matches reading
    std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
    if ( // Try to read the provided match filename or the default one (matches.f.txt/bin)
      !(matches_provider->load(_global_sfm_data, stlplus::create_filespec(_sfm_option.matches_dir, "matches.f.txt")) ||
        matches_provider->load(_global_sfm_data, stlplus::create_filespec(_sfm_option.matches_dir, "matches.f.bin")))) {
        LOG(ERROR) << "Invalid matches file.";
        return;
    }

    if (!stlplus::folder_exists(_sfm_option.output_dir)) {
        if (!stlplus::folder_create(_sfm_option.output_dir)) {
            LOG(ERROR) << "\nCannot create the output directory";
        }
    }

    //---------------------------------------
    // Sequential reconstruction process
    //---------------------------------------

    openMVG::system::Timer timer;

    std::unique_ptr<SfMSceneInitializer> scene_initializer;
    switch(scene_initializer_enum) {
        case ESfMSceneInitializer::INITIALIZE_AUTO_PAIR:
            LOG(ERROR) << "Not yet implemented.";
            return;
            break;
        case ESfMSceneInitializer::INITIALIZE_MAX_PAIR:
            scene_initializer.reset(new SfMSceneInitializerMaxPair(_global_sfm_data,
              feats_provider.get(),
              matches_provider.get()));
            break;
        case ESfMSceneInitializer::INITIALIZE_EXISTING_POSES:
            scene_initializer.reset(new SfMSceneInitializer(_global_sfm_data,
              feats_provider.get(),
              matches_provider.get()));
            break;
        case ESfMSceneInitializer::INITIALIZE_STELLAR:
            scene_initializer.reset(new SfMSceneInitializerStellar(_global_sfm_data,
              feats_provider.get(),
              matches_provider.get()));
            break;
        default: return;
    }
    if (!scene_initializer) {
        LOG(ERROR) << "Invalid scene initializer.";
        return;
    }   

    SequentialSfMReconstructionEngine2 sfm_engine(scene_initializer.get(),
                                                  _global_sfm_data,
                                                  _sfm_option.output_dir,
                                                  stlplus::create_filespec(_sfm_option.output_dir, 
                                                  "Reconstruction_Report.html"));

    // Configure the features_provider & the matches_provider
    sfm_engine.SetFeaturesProvider(feats_provider.get());
    sfm_engine.SetMatchesProvider(matches_provider.get());

    // Configure reconstruction parameters
    sfm_engine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    sfm_engine.SetUnknownCameraType(EINTRINSIC(_sfm_option.camera_model_type));

    if (sfm_engine.Process()) {
        LOG(INFO) << " Total Ac-Sfm took (s): " << timer.elapsed();

        LOG(INFO) << "...Generating SfM_Report.html";
        Generate_SfM_Report(sfm_engine.Get_SfM_Data(),
          stlplus::create_filespec(_sfm_option.output_dir, "SfMReconstruction_Report.html"));

        //-- Export to disk computed scene (data & visualizable results)
        LOG(INFO) << "...Export SfM_Data to disk.";
        Save(sfm_engine.Get_SfM_Data(),
             stlplus::create_filespec(_sfm_option.output_dir, "sfm_data", ".bin"),
             ESfM_Data(ALL));

        Save(sfm_engine.Get_SfM_Data(),
             stlplus::create_filespec(_sfm_option.output_dir, "cloud_and_poses", ".ply"),
             ESfM_Data(ALL));
    }
}

void SfMReconstructor::ComputeStructureFromKnowPoses(const size_t& index)
{
    LOG(INFO) << "\nCompute Structure From Known Poses";

    SfM_Data& sfm_data = _local_sfm_datas[index];
    std::string local_output_dir = _clusters_dir_list[index];

    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    const std::string sImage_describer = 
        stlplus::create_filespec(_sfm_option.features_dir, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type) {
        LOG(ERROR) << "Invalid: " << sImage_describer << " regions type file.";
        return;
    }

    // Prepare the Regions provider
    std::shared_ptr<Regions_Provider> regions_provider;
    if (_sfm_option.ui_max_cache_size == 0) {
        // Default regions provider (load & store all regions in memory)
        regions_provider = std::make_shared<Regions_Provider>();
    } else {
        // Cached regions provider (load & store regions on demand)
        regions_provider = std::make_shared<Regions_Provider_Cache>(_sfm_option.ui_max_cache_size);
    }

    // Show the progress on the command line:
    C_Progress_display progress;
    if (!regions_provider->load(sfm_data, _sfm_option.features_dir, regions_type, &progress)) {
        LOG(ERROR) << "\nInvalid regions.";
        return;
    }

    LOG(INFO) << "Loaded a sfm_data scene with:\n"
              << " - views: " << sfm_data.GetViews().size() << "\n"
              << " - poses: " << sfm_data.GetPoses().size() << "\n"
              << " - intrinsics: " << sfm_data.GetIntrinsics().size() << "\n"
              << " - tracks: " << sfm_data.GetLandmarks().size();

    std::string match_file = _sfm_option.matches_dir + "/matches.f.bin";
    LOG(INFO) << "=============================================================\n"
              << "Robust triangulation of the tracks\n"
              << " - Triangulation of guided epipolar geometry matches\n"
              << "=============================================================";
    //- Pair selection method:
    //  - geometry guided -> camera frustum intersection,
    //  - putative matches guided (photometric matches)
    //     (keep pairs that have valid Intrinsic & Pose ids).
    Pair_Set pairs;
    if (stlplus::file_exists(match_file)) {
        // no provided pair, use camera frustum intersection
        pairs = BuildPairsFromFrustumsIntersections(sfm_data);
    } else {
        LOG(ERROR) << match_file << "does not exist";
    } 

    openMVG::system::Timer timer;

    // Compute Structure from known camera poses
    SfM_Data_Structure_Estimation_From_Known_Poses 
        structure_estimator(_sfm_option.max_triangulation_repro_err);
    structure_estimator.run(sfm_data, pairs, regions_provider);
    LOG(INFO) << "\nStructure estimation took(s): " << timer.elapsed();

    regions_provider.reset(); // Regions are not longer needed.
    RemoveOutliers_AngleError(sfm_data, 2.0);

    LOG(INFO) << "\n- landmark found: " << sfm_data.GetLandmarks().size();

    LOG(INFO) << "Generating SfM_Report.html...";
    Generate_SfM_Report(sfm_data,
      stlplus::create_filespec(_sfm_option.output_dir, "SfM_StructureFromKnownPoses_Report.html"));

    LOG(INFO) << "Found a sfm_data scene with:\n"
              << " - views: " << sfm_data.GetViews().size() << "\n"
              << " - poses: " << sfm_data.GetPoses().size() << "\n"
              << " - intrinsics: " << sfm_data.GetIntrinsics().size() <<  "\n"
              << " - tracks: " << sfm_data.GetLandmarks().size();

    std::string out_file = local_output_dir + "/robust.bin";
    _local_sfm_datas_path.push_back(out_file);
    if (stlplus::extension_part(out_file) != "ply") {
        Save(sfm_data,
             stlplus::create_filespec(stlplus::folder_part(out_file),
                                      stlplus::basename_part(out_file), "ply"),
             ESfM_Data(ALL));
    }

    Save(sfm_data, out_file, ESfM_Data(ALL));
}

void SfMReconstructor::SfMAlign()
{
    AlignOption align_option(_sfm_option.align_use_common_cameras, 
                             _sfm_option.align_use_common_structures, 
                             _sfm_option.perform_global_ba);
    align_option.features_dir = _sfm_option.features_dir;
    align_option.tracks_filename = _sfm_option.matches_dir + "/tracks.txt";

    SfMAligner sfm_aligner;
    sfm_aligner.SetAlignOption(align_option);
    LOG(INFO) << "\nalign option: "
              << "\n--use common cameras: " << align_option.use_common_cameras
              << "\n--use common structures: " << align_option.use_common_structures              
              << "\n-- features directory: " << align_option.features_dir
              << "\n-- tracks filename: " << align_option.tracks_filename;

    

    // std::vector<std::string> local_sfm_datas_path;
    // for (int i = 0; i < _clusters_dir_list.size(); i++) {
    //     std::string cluster_dir = _clusters_dir_list[i];
    //     if (stlplus::file_exists(cluster_dir + "/robust.bin")) {
    //         _local_sfm_datas_path.push_back(cluster_dir + "/robust.bin");
    //     } else if (stlplus::file_exists(cluster_dir + "/sfm_data.bin")) {
    //         _local_sfm_datas_path.push_back(cluster_dir + "/sfm_data.bin");
    //     } else {
    //         LOG(WARNING) << "Cluster " << i << "doesn't contain a sfm_data file, "
    //                      << "please check whether reconstruction unfinished or failed!";
    //     }
    // }

    if(!sfm_aligner.InitializeGraph(_local_sfm_datas_path)) {
        LOG(ERROR) << "cluster graph cannot be initialized!\n";
        return;
    }

    sfm_aligner.GetClusterGraph().ShowInfo();

    // sfm_aligner.SimilarityAveraging();

    sfm_aligner.UpdateGraph();

    // Merge clusters
    sfm_aligner.MergeClusters(_sfm_option.image_dir);

    sfm_aligner.ExportReport(_sfm_option.output_dir);
    SfMReconstructor::ExportToColmap(sfm_aligner.GetGlobalSfMData(), 
                                     _sfm_option.output_dir + "/colmap");
}

void SfMReconstructor::ExportLocalSfMDatasPath() const
{
    ofstream fout(_sfm_option.output_dir + "/sfm_datas.txt");

    if (fout.is_open()) {
        for (auto sfm_data_path : _local_sfm_datas_path) {
            fout << sfm_data_path << std::endl;
        }
    }
    fout.close();
}

void SfMReconstructor::ImportLocalSfMDatasPath()
{
    std::string filename = _sfm_option.output_dir + "/sfm_datas.txt";
    if (!stlplus::file_exists(filename)) {
        LOG(ERROR) << filename << " doesn't exist";
        return;
    }

    ifstream fin(filename);
    if (fin.is_open()) {
        std::string path;
        while (fin >> path) { _local_sfm_datas_path.push_back(path); }
    }
    fin.close();
}

void SfMReconstructor::ComputeSfMDataColor(const size_t& index) 
{
    LOG(INFO) << "\nCompute Color for Point Clouds";

    std::string cluster_dir = _clusters_dir_list[index];
    SfM_Data sfm_data = _local_sfm_datas[index];

    SfMReconstructor::ExportToColmap(sfm_data, cluster_dir + "/colmap");

    std::string output_ply = cluster_dir + "/sparse_point_cloud.ply";

    // Compute the scene structure color
    std::vector<Vec3> points, tracks_colors, cam_positions;
    if (openMVG::sfm::ColorizeTracks(sfm_data, points, tracks_colors)) {
        GetCameraPositions(sfm_data, cam_positions);    

        // Export the SfM_Data scene in the expected format
        if (GraphSfM::plyHelper::ExportToPly(points, cam_positions, output_ply, &tracks_colors)) {
            return;
        }
    }
}

void SfMReconstructor::UpdateGlobalMatches(const openMVG::matching::PairWiseMatches& matches)
{
    // for (auto it = matches.begin(); it != matches.end(); ++it) {
        // auto pair = it->first;
        // IndMatches ind_matches = it->second;
        // _map_geometric_matches[pair].clear();
        // _map_geometric_matches[pair].assign(ind_matches.begin(), ind_matches.end());
    // }
    //-- export view pair graph once geometric filter have been done
    {
        std::set<IndexT> set_ViewIds;
        std::transform(_global_sfm_data.GetViews().begin(), _global_sfm_data.GetViews().end(),
          std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
        openMVG::graph::indexedGraph putativeGraph(set_ViewIds, getPairs(matches));
        openMVG::graph::exportToGraphvizData(
          stlplus::create_filespec(_sfm_option.matches_dir, "optimized_matches"),
          putativeGraph);
    }
    _map_geometric_matches = matches;
}

std::pair<bool, Vec3> SfMReconstructor::CheckGPS(const std::string & filename,
                                                 const int & GPS_to_XYZ_method)
{
    std::pair<bool, Vec3> val(false, Vec3::Zero());
    std::unique_ptr<Exif_IO> exif_reader(new Exif_IO_EasyExif);
    if (exif_reader) {
        // Try to parse EXIF metada & check existence of EXIF data
        if ( exif_reader->open(filename) && exif_reader->doesHaveExifInfo()) {
            // Check existence of GPS coordinates
            double latitude, longitude, altitude;
            if (exif_reader->GPSLatitude(&latitude) &&
                exif_reader->GPSLongitude(&longitude) &&
                exif_reader->GPSAltitude(&altitude)) {
                // Add ECEF or UTM XYZ position to the GPS position array
                val.first = true;
                switch (GPS_to_XYZ_method) {
                    case 1:
                        val.second = lla_to_utm( latitude, longitude, altitude );
                        break;
                    case 0:
                    default:
                        val.second = lla_to_ecef( latitude, longitude, altitude );
                        break;
                }
            }
        }
    }
    return val;
}

bool SfMReconstructor::ComputeIndexFromImageNames(
                                const SfM_Data& sfm_data,
                                const std::pair<std::string,std::string>& initial_pair_name,
                                Pair& initial_pair_index)
{
    if (initial_pair_name.first == initial_pair_name.second) {
      LOG(ERROR) << "\nInvalid image names. You cannot use the same image to initialize a pair.";
      return false;
    }

    initial_pair_index = {UndefinedIndexT, UndefinedIndexT};

    // List views filenames and find the one that correspond to the user ones:
    for (auto it = sfm_data.GetViews().begin(); it != sfm_data.GetViews().end(); ++it) {
        const View* v = it->second.get();
        const std::string filename = stlplus::filename_part(v->s_Img_path);
        if (filename == initial_pair_name.first) {
            initial_pair_index.first = v->id_view;
        } else {
            if (filename == initial_pair_name.second) {
              initial_pair_index.second = v->id_view;
            }
        }
    }
    return (initial_pair_index.first != UndefinedIndexT &&
            initial_pair_index.second != UndefinedIndexT);
}

vector<PairWiseMatches> SfMReconstructor::BuildClusterMatches(const PairWiseMatches& mapMatches,
                                            const std::vector<shared_ptr<ImageGraph>>& igList)
{
    std::vector<PairWiseMatches> matches_list;
    std::map<Pair, IndMatches>::iterator ite;
    
    for (auto ig : igList) {
        PairWiseMatches pm_matches;
        // std::vector<EdgeMap> adj_maps = ig->SerializeEdges();
        auto adj_maps = ig->GetEdges();
        // std::vector<ImageNode> nodes = ig->SerializeNodes();
        auto nodes = ig->GetNodes();
        for (auto em_it = adj_maps.begin(); em_it != adj_maps.end(); ++em_it) {
            auto edge_map = em_it->second;
            for (auto ite = edge_map.begin(); ite != edge_map.end(); ++ite) {
                // Pair pair(ite->second.src, ite->second.dst);
                int src = nodes[ite->second.src].id;
                int dst = nodes[ite->second.dst].id;
                Pair pair1(src, dst), pair2(dst, src);
                auto match_ite1 = mapMatches.find(pair1);
                auto match_ite2 = mapMatches.find(pair2);
                if (match_ite1 != mapMatches.end()) {
                    IndMatches ind_matches = match_ite1->second;
                    pm_matches.insert(make_pair(pair1, ind_matches));
                } else if (match_ite2 != mapMatches.end()) {
                    IndMatches ind_matches = match_ite2->second;
                    pm_matches.insert(make_pair(pair2, ind_matches));
                }
            }
        }
        matches_list.push_back(pm_matches);
    }
    return matches_list;
}

Pair_Set SfMReconstructor::BuildPairsFromFrustumsIntersections(const SfM_Data& sfm_data,
                                                               const double z_near,
                                                               const double z_far)
{
    const Frustum_Filter frustum_filter(sfm_data, z_near, z_far);
    return frustum_filter.getFrustumIntersectionPairs();
}

// Export camera poses positions as a Vec3 vector
void SfMReconstructor::GetCameraPositions(const SfM_Data& sfm_data, std::vector<Vec3>& cam_positions)
{
    for (const auto& view : sfm_data.GetViews()) {
        if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get())) {
            const openMVG::geometry::Pose3 pose = sfm_data.GetPoseOrDie(view.second.get());
            cam_positions.push_back(pose.center());
        }
    }
}

bool SfMReconstructor::StringToEnumESfMSceneInitializer(const std::string& str,
                                                         ESfMSceneInitializer& scene_initializer)
{
    const std::map<std::string, ESfMSceneInitializer> string_to_enum_mapping =
    {
        {"EXISTING_POSE", ESfMSceneInitializer::INITIALIZE_EXISTING_POSES},
        {"MAX_PAIR", ESfMSceneInitializer::INITIALIZE_MAX_PAIR},
        {"AUTO_PAIR", ESfMSceneInitializer::INITIALIZE_AUTO_PAIR},
        {"STELLAR", ESfMSceneInitializer::INITIALIZE_STELLAR},
    };
    auto it = string_to_enum_mapping.find(str);
    if (it == string_to_enum_mapping.end()) {
        return false;
    }
    scene_initializer = it->second;
    return true;
}

bool SfMReconstructor::CreateLineCameraFile(const IndexT camera_id, 
                                 std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic,
                                 std::string& camera_linie)
{
    std::stringstream came_line_ss;
    EINTRINSIC current_type = intrinsic->getType();
    switch(current_type) {
        case PINHOLE_CAMERA: 
            // OpenMVG's PINHOLE_CAMERA corresponds to Colmap's SIMPLE_PINHOLE
            // Parameters: f, cx, cy  
            {
                std::shared_ptr<openMVG::cameras::Pinhole_Intrinsic> pinhole_intrinsic(
                  dynamic_cast<openMVG::cameras::Pinhole_Intrinsic * >(intrinsic->clone()));

                came_line_ss << camera_id << " " 
                             << "SIMPLE_PINHOLE" << " " 
                             << pinhole_intrinsic->w() << " " 
                             << pinhole_intrinsic->h() << " " 
                             << pinhole_intrinsic->focal() << " " 
                             << pinhole_intrinsic->principal_point().x() << " " 
                             << pinhole_intrinsic->principal_point().y() << "\n";
            }
            break;
        case PINHOLE_CAMERA_RADIAL1:
            // OpenMVG's PINHOLE_CAMERA_RADIAL1 corresponds to Colmap's SIMPLE_RADIAL
            // Parameters: f, cx, cy, k1   
            {
              std::shared_ptr<openMVG::cameras::Pinhole_Intrinsic_Radial_K1> pinhole_intrinsic_radial(
                  dynamic_cast<openMVG::cameras::Pinhole_Intrinsic_Radial_K1 * >(intrinsic->clone()));

              came_line_ss << camera_id << " " 
                           << "SIMPLE_RADIAL" << " " 
                           << pinhole_intrinsic_radial->w() << " " 
                           << pinhole_intrinsic_radial->h() << " " 
                           << pinhole_intrinsic_radial->focal() << " " 
                           << pinhole_intrinsic_radial->principal_point().x() << " " 
                           << pinhole_intrinsic_radial->principal_point().y() << " " 
                           << pinhole_intrinsic_radial->getParams().at(3) << "\n";   //k1
            }
            break;      
        case PINHOLE_CAMERA_RADIAL3: 
            // OpenMVG's PINHOLE_CAMERA_RADIAL3 corresponds to Colmap's FULL_OPENCV
            // Parameters: fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
            {
                std::shared_ptr<openMVG::cameras::Pinhole_Intrinsic_Radial_K3> pinhole_intrinsic_radial(
                    dynamic_cast<openMVG::cameras::Pinhole_Intrinsic_Radial_K3 * >(intrinsic->clone()));

                came_line_ss << camera_id << " " 
                             << "FULL_OPENCV" << " " 
                             << pinhole_intrinsic_radial->w() << " " 
                             << pinhole_intrinsic_radial->h() << " " 
                             << pinhole_intrinsic_radial->focal() << " " 
                             << pinhole_intrinsic_radial->focal() << " " 
                             << pinhole_intrinsic_radial->principal_point().x() << " " 
                             << pinhole_intrinsic_radial->principal_point().y() << " " 
                             << pinhole_intrinsic_radial->getParams().at(3) << " " // k1
                             << pinhole_intrinsic_radial->getParams().at(4) << " " // k2
                             << 0.0 << " " // p1
                             << 0.0 << " " // p2
                             << pinhole_intrinsic_radial->getParams().at(5) << " " // k3
                             << 0.0 << " " // k4
                             << 0.0 << " " // k5
                             << 0.0 << "\n";  // k6
            }
            break; 
        case PINHOLE_CAMERA_BROWN: 
            LOG(INFO) << "PINHOLE_CAMERA_BROWN is not supported. Aborting ...";
            return false;
            break;      
        case PINHOLE_CAMERA_FISHEYE: 
            LOG(INFO) << "PINHOLE_CAMERA_FISHEYE is not supported. Aborting ...";
            return false;
            break; 
        default: 
            LOG(INFO) << "Camera Type " << current_type << " is not supported. Aborting ...";
            return false;
    }
    camera_linie = came_line_ss.str();
    return true;
}

bool SfMReconstructor::CreateCameraFile(const SfM_Data& sfm_data, const std::string& cameras_filename)
{
    /* cameras.txt
        # Camera list with one line of data per camera:
        #   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
        # Number of cameras: X
    */ 
    std::ofstream camera_file( cameras_filename );
    if (!camera_file) {
        std::cerr << "Cannot write file" << cameras_filename << std::endl;
        return false;
    }
    camera_file << "# Camera list with one line of data per camera:\n";
    camera_file << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
    camera_file << "# Number of cameras: X\n";

    std::vector<std::string> camera_lines;
    C_Progress_display progress_bar( sfm_data.GetIntrinsics().size(), std::cout, "\n- CREATE CAMERA FILE -\n" );
    for (auto iter = sfm_data.GetIntrinsics().begin();
        iter != sfm_data.GetIntrinsics().end(); ++iter, ++progress_bar) {
        const IndexT camera_id = iter->first;
        std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic = iter->second;
        std::string camera_line;
        if (CreateLineCameraFile(camera_id, intrinsic, camera_line)) {
            camera_lines.push_back(camera_line);
        } else {
            return false;
        }
    }
    for (auto const& camera_line: camera_lines) {
        camera_file << camera_line << "\n";
    }
    return true;
}

bool SfMReconstructor::CreateImageFile(const SfM_Data& sfm_data, const std::string& images_filename)
{
    /* images.txt
       # Image list with two lines of data per image:
       #   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
       #   POINTS2D[] as (X, Y, POINT3D_ID)
       # Number of images: X, mean observations per image: Y
     */

    // Header
    std::ofstream images_file(images_filename);

    if (!images_file) {
        std::cerr << "Cannot write file" << images_filename << std::endl;
        return false;
    }
    images_file << "# Image list with two lines of data per image:\n";
    images_file << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n";
    images_file << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";
    images_file << "# Number of images: X, mean observations per image: Y\n";

    std::map< IndexT, std::vector< std::tuple<double, double, IndexT> > > viewIdToPoints2D;
    const Landmarks & landmarks = sfm_data.GetLandmarks();
    
    for (auto ite_landmarks = landmarks.begin();
          ite_landmarks != landmarks.end(); ++ite_landmarks) {
        const IndexT point3d_id = ite_landmarks->first; 
        // Tally set of feature observations
        const Observations& obs = ite_landmarks->second.obs;
        for (auto itObs = obs.begin(); itObs != obs.end(); ++itObs ) {
            const IndexT currentViewId = itObs->first;
            const Observation& ob = itObs->second;
            viewIdToPoints2D[currentViewId].push_back(std::make_tuple(ob.x( 0 ), ob.x( 1 ), point3d_id));
        }
    }
    

    
    C_Progress_display progress_bar( sfm_data.GetViews().size(), std::cout, "\n- CREATE IMAGE FILE -\n" );

    for (auto iter = sfm_data.GetViews().begin();
        iter != sfm_data.GetViews().end(); ++iter, ++progress_bar) {
        const View * view = iter->second.get();

        if (!sfm_data.IsPoseAndIntrinsicDefined(view)) {
            continue;
        }

        const Pose3 pose = sfm_data.GetPoseOrDie( view );
        const Mat3 rotation = pose.rotation();
        const Vec3 translation = pose.translation();

        const double Tx = translation[0];
        const double Ty = translation[1];
        const double Tz = translation[2];
        Eigen::Quaterniond q( rotation );
        const double Qx = q.x();
        const double Qy = q.y();
        const double Qz = q.z();
        const double Qw = q.w();

        const IndexT image_id = view->id_view;
        // Colmap's camera_ids correspond to openMVG's intrinsic ids
        const IndexT camera_id = view->id_intrinsic;                           
        const std::string image_name = view->s_Img_path;

        // first line per image
        // IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        images_file << image_id << " "
                    << Qw << " "
                    << Qx << " "
                    << Qy << " "
                    << Qz << " "
                    << Tx << " "
                    << Ty << " "
                    << Tz << " "
                    << camera_id << " "
                    << image_name << " "     
                    << "\n";

        // second line per image 
        // POINTS2D[] as (X, Y, POINT3D_ID)
        for (auto point2D: viewIdToPoints2D[image_id]) {
            images_file << std::get<0>(point2D) << " " 
                        << std::get<1>(point2D) << " " 
                        << std::get<2>(point2D) << " ";
        }
        images_file << "\n";
        
    }
    return true;
}

bool SfMReconstructor::CreatePoint3DFile(const SfM_Data& sfm_data, const std::string& points3d_filename)
{
    /* points3D.txt
        # 3D point list with one line of data per point:
        #   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
        # Number of points: X, mean track length: Y
     */

    std::ofstream points3D_file(points3d_filename);

    if (!points3D_file) {
        std::cerr << "Cannot write file" << points3d_filename << std::endl;
        return false;
    }
    points3D_file << "# 3D point list with one line of data per point:\n";
    points3D_file << "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n";
    points3D_file << "# Number of points: X, mean track length: Y\n";

    const Landmarks & landmarks = sfm_data.GetLandmarks();

    std::vector<Vec3> vec_3dPoints, vec_tracksColor;
    if (!ColorizeTracks(sfm_data, vec_3dPoints, vec_tracksColor)) {
        return false;
    }

    C_Progress_display progress_bar( landmarks.size(), std::cout, "\n- CREATE POINT3D FILE  -\n" );
    int point_index = 0;
    for (auto ite_landmarks = landmarks.begin();
         ite_landmarks != landmarks.end(); ++ite_landmarks, ++progress_bar ) {
        const Vec3 exportPoint = ite_landmarks->second.X;
        const IndexT point3d_id = ite_landmarks->first;
        points3D_file << point3d_id << " "
                      << exportPoint.x() << " " 
                      << exportPoint.y() << " " 
                      << exportPoint.z() << " "
                      << static_cast<int>(vec_tracksColor.at(point_index)(0)) << " " 
                      << static_cast<int>(vec_tracksColor.at(point_index)(1)) << " " 
                      << static_cast<int>(vec_tracksColor.at(point_index)(2)) << " ";

        ++point_index;

        const double error = 0.0;     // Some error
        points3D_file << error;

        const Observations & obs = ite_landmarks->second.obs;
        for (auto itObs = obs.begin(); itObs != obs.end(); ++itObs) {
            const IndexT viewId = itObs->first;
            const IndexT featId = itObs->second.id_feat;

            points3D_file << " " << viewId << " " << featId;
        }
        points3D_file << "\n";
    }
    return true;
}

/**
* @brief Convert OpenMVG reconstruction result to Colmap's reconstruction format. 
* @param sfm_data Structure from Motion file
* @param output_dir Output directory
* @param cameras_filename File name of the camera file
* @param images_filename File name of the image file
* @param points3d_filename File name of the point3D file
*/
bool SfMReconstructor::CreateColmapFolder(const SfM_Data& sfm_data,
                                          const std::string& output_dir,
                                          const std::string& cameras_filename,
                                          const std::string& images_filename, 
                                          const std::string& points3d_filename)
{
    /* Colmap Output structure:
        cameras.txt
        images.txt
        points3D.txt
    */
    if (!CreateCameraFile(sfm_data, cameras_filename)) {
        return false;
    }
    if (!CreateImageFile(sfm_data, images_filename)) {
        return false;
    }
    if (!CreatePoint3DFile(sfm_data, points3d_filename)) {
        return false;
    }
    return true;
}

/**
* @brief Main function used to export a Colmap reconstruction folder
* @param sfm_data Structure from Motion file to export
* @param output_dir Output directory
*/
bool SfMReconstructor::ExportToColmap(const SfM_Data& sfm_data, const std::string& output_dir)
{
    // Create output directory
    bool bOk = false;
    if (!stlplus::is_folder(output_dir)) {
        LOG(INFO) << "Creating directory:  " << output_dir;
        stlplus::folder_create( output_dir );
        bOk = stlplus::is_folder( output_dir );
    }
    else { bOk = true; }

    if (!bOk) {
        LOG(ERROR) << "Cannot access one of the desired output directories";
        return false;
    }
    const std::string cameras_filename = stlplus::create_filespec(output_dir, "cameras.txt");
    const std::string images_filename = stlplus::create_filespec(output_dir, "images.txt");
    const std::string points3d_filename = stlplus::create_filespec(output_dir, "points3D.txt");
    if (!CreateColmapFolder(sfm_data, output_dir, cameras_filename, images_filename, points3d_filename)) {
        LOG(ERROR) << "There was an error exporting project";
        return false;
    }
    return true;
}

bool SfMReconstructor::OpenMVGData2ColmapData(const std::string& sfm_data_file, const std::string& output_dir)
{
    if (!stlplus::folder_exists(output_dir)) {
        stlplus::folder_create(output_dir);
    }

    // Read the input SfM scene
    SfM_Data sfm_data;
    if (!openMVG::sfm::Load(sfm_data, sfm_data_file, ESfM_Data(ALL))) {
        LOG(ERROR) << "The input SfM_Data file \"" << sfm_data_file << "\" cannot be read.";
        return false;
    }

    return OpenMVGData2ColmapData(sfm_data, output_dir);
}

bool SfMReconstructor::OpenMVGData2ColmapData(const openMVG::sfm::SfM_Data& sfm_data, 
                                   const std::string& output_dir)
{
    if (!ExportToColmap(sfm_data, output_dir)) {
        LOG(ERROR) << "There was an error during export of the file";
        return false;
    }
    return true;
}

}   // namespace sfm
}   // namespace retina