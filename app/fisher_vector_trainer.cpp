#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "feature/fisher_vector_extractor.h"
#include "stlplus3/stlplus.h"

#include <glog/logging.h>
#include <gflags/gflags.h>

using namespace std;
using namespace GraphSfM;
using namespace GraphSfM::feature;


DEFINE_string(filename, "", "The filename that stores the path of sift descriptors");
DEFINE_string(output_dir, "", "Output directory that stores the gmm");
DEFINE_string(gmm_file, "", "output GMM filename");

bool ReadBinaryOpenMVGDesc(string desc_file, vector<Eigen::VectorXf>& descs)
{
    ifstream file_in(desc_file, ios::in);
    if (!file_in.is_open()) {
        LOG(ERROR) << "Fail to load OpenMVG descriptor file.\n";
        return false;
    }
    
    // Read the number of descriptor in the file
    size_t card_desc = 0;
    file_in.read((char *) &card_desc, sizeof(size_t));

    const int desc_dim = 128;
    unsigned char* dp = new unsigned char [card_desc * desc_dim];
    file_in.read((char *)dp, desc_dim * sizeof(unsigned char) * card_desc);

    for (int i = 0; i < card_desc; i ++) {
        Eigen::VectorXf desc = Eigen::VectorXf::Zero(desc_dim);
        for (int j = 0; j < desc_dim; j++) {
            desc[j] = dp[i * desc_dim + j];
        }
        descs.push_back(desc);
    }

    bool b_ok = !file_in.bad();
    file_in.close();

    return b_ok;
}

bool ReadTextOpenMVGDesc(string desc_file, Eigen::VectorXf& desc)
{
    // TODO:
}

void ReadOpenMVGDescs(string filename, vector<vector<Eigen::VectorXf>>& descs_list)
{
    string sift_file;
    ifstream in(filename, ios::in);
    while (in >> sift_file) {
        vector<Eigen::VectorXf> descs;
        if (ReadBinaryOpenMVGDesc(sift_file, descs)) {
            descs_list.push_back(descs);
        } else {
            LOG(WARNING) << "Fail to load OpenMVG decriptor file\n";
        }
    }
}

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    string filename = FLAGS_filename;
    string out_dir = FLAGS_output_dir;
    string gmm_file = FLAGS_gmm_file;

    FLAGS_log_dir = out_dir;
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    if (!stlplus::folder_exists(out_dir)) stlplus::folder_create(out_dir);

    vector<Eigen::VectorXf> fisher_vectors;
    vector<vector<Eigen::VectorXf>> descs_list;

    ReadOpenMVGDescs(filename, descs_list);

    FisherVectorExtractor::Options option;
    FisherVectorExtractor fv_extractor(option);

    #pragma omp parallel 
    {
        #pragma omp for
        for (int i = 0; i < descs_list.size(); i++) {
            vector<Eigen::VectorXf> descs = descs_list[i];
            fv_extractor.AddFeaturesForTraining(descs);
        }
    }

    fv_extractor.Train();

    fv_extractor.ExportGaussianMixtureModel(out_dir + "/" + gmm_file);

    LOG(INFO) << "GMM saved in " << out_dir + "/" + gmm_file << endl;
}
