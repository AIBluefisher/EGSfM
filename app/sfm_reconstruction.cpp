#include <fstream>

#include "sfm/sfm_reconstructor.h"
#include "yaml-cpp/yaml.h"

#include <glog/logging.h>
#include <gflags/gflags.h>

using namespace GraphSfM;
using namespace GraphSfM::sfm;

DEFINE_string(config_filename, "", "the configuration file of structure from motion");

void ReadSfMConfigurations(const std::string& filename, SfMOption& sfm_option)
{
    YAML::Node config = YAML::LoadFile(filename);

    sfm_option.thread_num = config["thread_num"].as<int>();
    sfm_option.image_dir = config["image_dir"].as<std::string>();
    sfm_option.output_dir = config["output_dir"].as<std::string>();
    sfm_option.features_dir = config["features_dir"].as<std::string>();
    sfm_option.matches_dir = config["matches_dir"].as<std::string>();

    sfm_option.camera_model_type = config["camera_model_type"].as<int>();
    sfm_option.group_camera_model = config["group_camera_model"].as<bool>();
    sfm_option.sensor_width_database = config["sensor_width_database"].as<std::string>();

    sfm_option.feature_quality = config["feature_quality"].as<int>();
    sfm_option.ui_max_cache_size = config["ui_max_cache_size"].as<int>();
    sfm_option.distance_ratio = config["distance_ratio"].as<float>();
    sfm_option.ransac_max_iteration = config["ransac_max_iteration"].as<int>();
    sfm_option.perform_guided_matching = config["perform_guided_matching"].as<bool>();
    sfm_option.geometric_model = config["geometric_model"].as<std::string>();
    sfm_option.nearest_matching_method = config["nearest_matching_method"].as<string>();

    if (config["predefined_pair_list"]) {
        sfm_option.predefined_pair_list = config["predefined_pair_list"].as<string>();        
    }

    if (config["video_mode_matching"]) {
        sfm_option.video_mode_matching = config["video_mode_matching"].as<int>();
    }
    sfm_option.use_similarity_search = config["use_similarity_search"].as<bool>();
    sfm_option.vot_dir = config["vot_dir"].as<string>();
    sfm_option.vot_sifts_file = config["vot_sifts_file"].as<string>();
    sfm_option.vot_depth = config["vot_depth"].as<int>();
    sfm_option.vot_branch_num = config["vot_branch_num"].as<int>();

    sfm_option.intrinsic_refinement_option = config["intrinsic_refinement_option"].as<string>();

    sfm_option.cluster_option = config["cluster_option"].as<string>();
    sfm_option.max_cluster_size = config["max_cluster_size"].as<int>();
    sfm_option.completeness_ratio = config["completeness_ratio"].as<double>();
    sfm_option.relax_ratio = config["relax_ratio"].as<float>();
    sfm_option.copy_images = config["copy_images"].as<bool>();

    if (config["batched_sfm_initilizer"]) {
        sfm_option.batched_sfm_initilizer = config["batched_sfm_initilizer"].as<string>();
    }
    
    sfm_option.max_triangulation_repro_err = config["max_triangulation_repro_err"].as<double>();

    sfm_option.align_use_common_cameras = config["align_use_common_cameras"].as<bool>();
    sfm_option.align_use_common_structures = config["align_use_common_structures"].as<bool>();
    sfm_option.perform_global_ba = config["perform_global_ba"].as<bool>();
}

int main(int argc, char* argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    std::string config_filename = FLAGS_config_filename;

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    SfMOption sfm_option;
    ReadSfMConfigurations(config_filename, sfm_option);

    SfMReconstructor sfm_reconstructor(sfm_option);
    sfm_reconstructor.CheckSfMConfigurations();

    std::vector<SfM_Data> local_sfm_datas;

    bool pre_results_exist = stlplus::file_exists(sfm_option.output_dir + "/sfm_datas.txt");
        // stlplus::file_exists(sfm_option.output_dir + "/final_sfm_data.json") ||
        // stlplus::file_exists(sfm_option.output_dir + "/final_sfm_data.bin");

    if (pre_results_exist) {
        LOG(WARNING) << "Previous results exist";
    } else {
        sfm_reconstructor.InitImageListing();
        sfm_reconstructor.ComputeFeatures();
        sfm_reconstructor.ComputeMatches();

        sfm_reconstructor.ImageClustering();

        local_sfm_datas = sfm_reconstructor.GetLocalSfMDatas();
        if (local_sfm_datas.empty()) {
            SfM_Data sfm_data = sfm_reconstructor.GetGlobalSfMData();
            PairWiseMatches matches = sfm_reconstructor.GetGlobalMatches();

            sfm_reconstructor.InsertMatchesData(matches);
            sfm_reconstructor.InsertSfMData(sfm_data);
            sfm_reconstructor.InsertClusterDir(sfm_option.output_dir);
        }

        local_sfm_datas = sfm_reconstructor.GetLocalSfMDatas();
        for (int i = 0; i < local_sfm_datas.size(); i++) {
            sfm_reconstructor.LocalIncrementalSfM(i);
            sfm_reconstructor.ComputeStructureFromKnowPoses(i);
            sfm_reconstructor.ComputeSfMDataColor(i);
        }

        sfm_reconstructor.ExportLocalSfMDatasPath();
    }

    if (local_sfm_datas.empty()) {
        sfm_reconstructor.ImportLocalSfMDatasPath();
    }
    if (local_sfm_datas.size() > 1 || 
        !sfm_reconstructor.GetLocalSfMDatasPath().empty()) {
        sfm_reconstructor.SfMAlign();
        sfm_reconstructor.ExportLocalSfMDatasPath();
    }

    std::cout << std::endl;
    LOG(INFO) << "Structure from Motion task done!";
}