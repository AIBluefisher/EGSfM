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

enum DISTANCE_METRIC { L2_NORM, DOT_PRODUCT, COSINE_SIMILARITY };

DEFINE_string(filename, "", "The filename that stores the path of gmm");
DEFINE_string(output_dir, "", "Output directory that stores the fisher vector similarity scores");
DEFINE_uint32(metric_option, 0, "Option used in fisher vector similarity metric");

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

float ComputeFisherDistance(Eigen::VectorXf fv_l, Eigen::VectorXf fv_r, DISTANCE_METRIC metric)
{
    double distance = 0.0;
    switch (metric) {
        case L2_NORM: {
            distance = (fv_l - fv_r).norm();
            break;
        }

        case DOT_PRODUCT: {
            distance = fv_l.transpose() * fv_r;
            break;
        }

        case COSINE_SIMILARITY: {
            float dp = fv_l.transpose() * fv_r;
            distance = abs(dp / (fv_l.norm() * fv_r.norm()));
            break;
        }
    }
    return distance;
}

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    string filename = FLAGS_filename;
    string out_dir = FLAGS_output_dir;

    FLAGS_log_dir = out_dir;
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    DISTANCE_METRIC metric = COSINE_SIMILARITY;
    switch (FLAGS_metric_option) {
        case 0: 
            metric = L2_NORM; 
            break;
        case 1:
            metric = DOT_PRODUCT; 
            break;
        case 2: 
            metric = COSINE_SIMILARITY; 
            break;
        default: 
            LOG(WARNING) << "Metric option undefined! Set to default!";
            break;
    }

    if (!stlplus::folder_exists(out_dir)) stlplus::folder_create(out_dir);

    vector<Eigen::VectorXf> fisher_vectors;
    vector<vector<Eigen::VectorXf>> descs_list;

    ReadOpenMVGDescs(filename, descs_list);

    FisherVectorExtractor::Options option;
    FisherVectorExtractor fv_extractor(option);

    // #pragma omp parallel 
    // {
    //     #pragma omp for
    //     for (int i = 0; i < descs_list.size(); i++) {
    //         vector<Eigen::VectorXf> descs = descs_list[i];
    //         fv_extractor.AddFeaturesForTraining(descs);
    //     }
    // }

    // fv_extractor.Train();
    fv_extractor.ImportGaussianMixtureModel(filename);

    #pragma omp parallel 
    {
        #pragma omp for
        for (int i = 0; i < descs_list.size(); i++) {
            vector<Eigen::VectorXf> descs = descs_list[i];
            Eigen::VectorXf fisher_vector = fv_extractor.ExtractGlobalDescriptor(descs);
            fisher_vectors.push_back(fisher_vector);
        }
    }

    string fisher_file = out_dir + "/fisher_similarity.out";
    ofstream out(fisher_file);
    if (!out.is_open()) {
        LOG(ERROR) << "Fisher vector similary file " 
                   << fisher_file
                   << "cannot be created!";
        return 0;
    }

    
    vector<unordered_map<size_t, float>> distances(fisher_vectors.size());
    #pragma omp parallel
    {
        #pragma omp for
        for (int i = 0; i < fisher_vectors.size(); i++) {
            for (int j = i + 1; j < fisher_vectors.size(); j++) {
                float distance = ComputeFisherDistance(fisher_vectors[i], fisher_vectors[j], metric);
                distances[i][j] = distance;
            }
        }
    }

    for (int i = 0; i < fisher_vectors.size(); i++) {
        for (int j = i + 1; j < fisher_vectors.size(); j++) {
            // to keep consistent with vocabulary tree, where larger similarity score
            // suggests higher priority level; for fisher vector, less distance score
            // suggests higher priority level - thus we divided it by 1.0
            out << i << " " << j << " " << 1.0 / distances[i][j] << std::endl;
        }
    }

    out.close();
    cout << "Fisher distance file saved in " << fisher_file << endl;
}
