#include <string>
#include <iostream>
#include <fstream>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include "geometry/sfm_aligner.h"

using namespace std;
using namespace openMVG;
using namespace GraphSfM::geometry;

#define USE_CAMERA
// #define USE_OBSERVATION

DEFINE_string(sfm_datas_path, "", "The file that stores all path of sfm_data.json which in openMVG format");
DEFINE_string(img_path, "", "The directory that stores images");
DEFINE_string(features_dir, "", "The directory that stores features");
DEFINE_string(tracks_filename, "", "Absolute filename of tracks");
DEFINE_string(output_dir, "", "The directory that saves reconstruction result");
DEFINE_bool(use_common_cameras, false, "Use common cameras to compute similarity transform");
DEFINE_bool(use_common_structures, false, "Use common structures to compute similarity transform");
DEFINE_bool(perform_global_ba, false, "Set true the enable global bundle adjustment after merging");

int main(int argc, char* argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    if(argc < 3) {
        LOG(ERROR) << "Please input the file path of sfm_data and image path\n";
        return 0;
    }

    string sfm_datas_path = FLAGS_sfm_datas_path;
    string img_path = FLAGS_img_path;
    string features_dir = FLAGS_features_dir;
    string tracks_filename = FLAGS_tracks_filename;
    string output_dir = FLAGS_output_dir;
    bool uc = FLAGS_use_common_cameras;
    bool us = FLAGS_use_common_structures;
    bool pgb = FLAGS_perform_global_ba;
    
    cout << "---------------- Structure from Motion Aligner --------------\n";
    AlignOption align_option(uc, us, pgb);
    align_option.features_dir = features_dir;
    align_option.tracks_filename = tracks_filename;

    SfMAligner sfm_aligner;
    sfm_aligner.SetAlignOption(align_option);
    LOG(INFO) << "\nalign option: "
              << "\n-- use common cameras: " << align_option.use_common_cameras
              << "\n-- use common structures: " << align_option.use_common_structures
              << "\n-- features directory: " << align_option.features_dir
              << "\n-- tracks filename: " << align_option.tracks_filename;

    // initilization
    std::vector<std::string> sfm_datas_filename;
    ifstream file_in(sfm_datas_path);
    int idx = 0;
    string path;

    if(!file_in.is_open()) {
        LOG(ERROR) << "file " << sfm_datas_path << " cannot be opened! Please check the path!";
        return false;
    }

    // Add nodes
    while(file_in >> path) {
        if (stlplus::file_exists(path)) {
            sfm_datas_filename.push_back(path);
        } else {
            LOG(WARNING) << "file " << path << "doesn't exists";
        }
    }
    file_in.close();

    if(!sfm_aligner.InitializeGraph(sfm_datas_filename)) {
        LOG(ERROR) << "cluster graph cannot be initialized!\n";
        return 0;
    }

    sfm_aligner.GetClusterGraph().ShowInfo();

    sfm_aligner.SimilarityAveraging();

    sfm_aligner.UpdateGraph();

    // Merge clusters
    string dir = stlplus::folder_part(sfm_datas_path);
    // sfm_aligner.MergeClusters(dir, img_path);
    sfm_aligner.MergeClusters(img_path);
    
    if (!align_option.features_dir.empty()) {
        sfm_aligner.Retriangulate();
    }

    sfm_aligner.ExportReport(output_dir);
}