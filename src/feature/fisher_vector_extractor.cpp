extern "C" {
    #include <vlfeat/fisher.h>
    #include <vlfeat/gmm.h>
}

#include <Eigen/Core>
#include <glog/logging.h>
#include <memory>
#include <vector>

#include "fisher_vector_extractor.h"

namespace GraphSfM {
namespace feature {
Eigen::MatrixXf ConvertVectorOfFeaturesToMatrix(const std::vector<Eigen::VectorXf>& features) 
{
    Eigen::MatrixXf feature_table(features[0].size(), features.size());
    for (int i = 0; i < feature_table.cols(); i++) {
        feature_table.col(i) = features[i];
    }
    return feature_table;
}

FisherVectorExtractor::FisherVectorExtractor(const Options& options)
    : gmm_(new GaussianMixtureModel(options.num_gmm_clusters)),
      training_feature_sampler_(options.max_num_features_for_training) 
{

}

FisherVectorExtractor::~FisherVectorExtractor() 
{

}

void FisherVectorExtractor::AddFeaturesForTraining(const std::vector<Eigen::VectorXf>& features) 
{
    for (const Eigen::VectorXf& feature : features) {
      CHECK(!feature.hasNaN()) << "Feature: " << feature.transpose();
      training_feature_sampler_.AddElementToSampler(feature);
    }
}

bool FisherVectorExtractor::Train() 
{
    // Get the features randomly sampled for training.
    const auto& sampled_features = training_feature_sampler_.GetAllSamples();
    CHECK_GT(sampled_features.size(), 0);
    LOG(INFO) << "Training GMM for Fisher Vector extractin with "
              << sampled_features.size() << " features sampled from "
              << training_feature_sampler_.NumElementsAdded()
              << " total features.";    
    // Train the GMM using the training feaures.
    const Eigen::MatrixXf feature_table = ConvertVectorOfFeaturesToMatrix(sampled_features);
    return gmm_->Compute(feature_table);
}

Eigen::VectorXf 
FisherVectorExtractor::ExtractGlobalDescriptor(const std::vector<Eigen::VectorXf>& features) 
{
    // Ensure there are input features and they are not zero dimensions.
    CHECK_GT(features.size(), 0);
    CHECK_GT(features[0].size(), 0);    
    // Convert the features into a continuous memory block. The matrix is of size
    // D x N where D is the number of descrip
    const Eigen::MatrixXf feature_table = ConvertVectorOfFeaturesToMatrix(features);
    // Compute the fisher vector encoding.
    Eigen::VectorXf fisher_vector(2 * feature_table.rows() * gmm_->num_clusters());
    vl_fisher_encode(fisher_vector.data(),
                     VL_TYPE_FLOAT,
                     gmm_->GetMeans(),
                     feature_table.rows(),
                     gmm_->num_clusters(),
                     gmm_->GetCovariances(),
                     gmm_->GetPriors(),
                     feature_table.data(),
                     feature_table.cols(),
                     VL_FISHER_FLAG_IMPROVED);
    DCHECK(std::isfinite(fisher_vector.sum()));
    return fisher_vector;
}

void FisherVectorExtractor::ExportGaussianMixtureModel(std::string filename)
{
    size_t dimension = gmm_->GetDimension();
    size_t num_clusters = gmm_->GetNumClusters();
    float const *means = (float const *)gmm_->GetMeans();
    float const *covariances = (float const *)gmm_->GetCovariances();
    float const *priors = (float const *)gmm_->GetPriors();

    std::ofstream out(filename);
    if (!out.is_open()) {
        LOG(ERROR) << filename << " cannot be created!";
        return;
    }
    out << dimension << " " << num_clusters << std::endl;

    for (int i = 0; i < dimension * num_clusters; i++) {
        out << means[i] << " ";
    }
    out << std::endl;

    for (int i = 0; i < dimension * num_clusters; i++) {
        out << covariances[i] << " ";
    }
    out << std::endl;

    for (int i = 0;  i < num_clusters; i++) {
        out << priors[i] << " ";
    }
    out << std::endl;
    out.close();
}

void FisherVectorExtractor::ImportGaussianMixtureModel(std::string filename)
{
    std::ifstream in(filename);
    if (!in.is_open()) {
        LOG(ERROR) << filename << " cannot be created!";
    }
    
    size_t dimension, num_clusters;
    float *means = new float [dimension * num_clusters];
    float *covariances = new float [dimension * num_clusters];
    float *priors = new float [num_clusters];

    in >> dimension >> num_clusters;

    for (int i = 0; i < dimension * num_clusters; i++) {
        in >> means[i];
    }
    for (int i = 0; i < dimension * num_clusters; i++) {
        in >> covariances[i];
    }
    for (int i = 0; i < num_clusters; i++) {
        in >> priors[i];
    }
    in.close();

    // gmm_.reset(vl_gmm_new(VL_TYPE_FLOAT, dimension, num_clusters));
    if (!gmm_->Init(dimension, num_clusters)) {
        delete [] means;
        delete [] covariances;
        delete [] priors;
        LOG(ERROR) << "Gaussian Mixture Model import failed!";
        return;
    }
    gmm_->SetMeans(means);
    gmm_->SetCovariances(covariances);
    gmm_->SetPriors(priors);

    delete [] means;
    delete [] covariances;
    delete [] priors;
}

}  // namespace feature
}  // namespace GraphSfM
