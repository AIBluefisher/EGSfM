#ifndef XFEATURES_MATCHING_FISHER_VECTOR_EXTRACTOR_H
#define XFEATURES_MATCHING_FISHER_VECTOR_EXTRACTOR_H

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <fstream>
#include <glog/logging.h>

extern "C" {
    #include <vlfeat/fisher.h>
    #include <vlfeat/gmm.h>
}

#include "global_descriptor_extractor.h"
#include "math/reservoir_sampler.h"

namespace GraphSfM {
namespace feature {
// A Fisher Vector is an image representation obtained by pooling local image
// features. It is frequently used as a global image descriptor in visual
// classification. A Gaussian Mixture Model is fitted to the training data, and
// the Fisher Vector is computed as as the mean and covariance deviation vectors
// from the modes of the distribution of the GMM.
class FisherVectorExtractor : public GlobalDescriptorExtractor 
{
public:
    struct Options 
    {
      // The number of cluster to use for the Gaussian Mixture Model that power
      // the Fisher Kernel.
      int num_gmm_clusters = 16;    
      // If more than this number of features are added to the
      // FisherVectorExtractor then we randomly sample
      // max_num_features_for_training using a memory efficient Reservoir sampler
      // to avoid holding all features in memory.
      int max_num_features_for_training = 100000;
    };

    // The number of clusters to use for the GMM.
    FisherVectorExtractor(const Options& options);  
    ~FisherVectorExtractor();   
    // Add features to the descriptor extractor for training. This method may be
    // called multiple times to add multiple sets of features (e.g., once per
    // image) to the global descriptor extractor for training.
    void AddFeaturesForTraining(const std::vector<Eigen::VectorXf>& features) override; 
    // Train the global descriptor extracto with the given set of feature
    // descriptors added with AddFeaturesForTraining. It is assumed that all
    // descriptors have the same length.
    bool Train() override;  
    // Compute a global image descriptor for the set of input features.
    Eigen::VectorXf ExtractGlobalDescriptor(const std::vector<Eigen::VectorXf>& features) override;

    void ExportGaussianMixtureModel(std::string filename);
    void ImportGaussianMixtureModel(std::string filename);

private:
    // A Gaussian Mixture Model is used to compute the Fisher Kernel.
    class GaussianMixtureModel;
    std::unique_ptr<GaussianMixtureModel> gmm_; 
    // The GMM is trained from a set of feature descriptors. A reservoir sampler
    // is used to randomly sample features from an unknown number of input
    // features for training.
    ReservoirSampler<Eigen::VectorXf> training_feature_sampler_;
};

// Wrapper for the VlFeat gmm.
class FisherVectorExtractor::GaussianMixtureModel 
{
public:
    // NOTE: We need the unique_ptr to use vlfeat's delete function for the GMM.
    GaussianMixtureModel(const int num_clusters)
        : num_clusters_(num_clusters), gmm_(nullptr, vl_gmm_delete) {}  

    int num_clusters() const { return num_clusters_; }  

    // Computes the Gaussian mixture model based on the input training features.
    bool Compute(const Eigen::MatrixXf& data_points) 
    {
        CHECK(!data_points.hasNaN()); 
        // Create the GMM instance.
        gmm_.reset(vl_gmm_new(VL_TYPE_FLOAT, data_points.rows(), num_clusters_));
        if (!gmm_) { return false; }  
        // Compute the model
        const double distortion =
            vl_gmm_cluster(gmm_.get(), data_points.data(), data_points.cols());
        CHECK(!std::isnan(distortion));
        return true;
    }

    bool Init(const size_t dimension, const size_t num_clusters)
    {
        gmm_.reset(vl_gmm_new(VL_TYPE_FLOAT, dimension, num_clusters));
        if (!gmm_) { return false; }
        return true;
    }

    void SetMeans(void const* means)
    {
        vl_gmm_set_means(gmm_.get(), means);
    }

    void SetCovariances(void const* covariances)
    {
        vl_gmm_set_covariances(gmm_.get(), covariances);
    }

    void SetPriors(void const* priors)
    {
        vl_gmm_set_priors(gmm_.get(), priors);
    }

    // Helper accessor methods for retreiving VLFeat types.
    void const* GetMeans() const 
    { 
        return vl_gmm_get_means(gmm_.get()); 
    }

    void const* GetCovariances() const 
    {
        return vl_gmm_get_covariances(gmm_.get());
    }

    void const* GetPriors() const 
    { 
        return vl_gmm_get_priors(gmm_.get()); 
    }

    size_t GetDimension() const 
    { 
        return vl_gmm_get_dimension(gmm_.get()); 
    }

    size_t GetNumClusters() const 
    { 
        return vl_gmm_get_num_clusters(gmm_.get()); 
    }

private:
    const int num_clusters_;                        // Number of clusters.
    std::unique_ptr<VlGMM, void (*)(VlGMM*)> gmm_;  // The VlFeat GMM model.
};

}  // namespace feature
}  // namespace GraphSfM
#endif
