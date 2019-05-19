#ifndef XFEATURES_MATCHING_GLOBAL_DESCRIPTOR_EXTRACTOR_H
#define XFEATURES_MATCHING_GLOBAL_DESCRIPTOR_EXTRACTOR_H

#include <Eigen/Core>
#include <vector>

namespace GraphSfM {

// Global descriptors provide a summary of an entire image into a single feature
// descriptor. These descriptors may be formed using training data (e.g., SIFT
// features) or may be directly computed from the image itself. Global
// descriptors provide an efficient mechanism for determining the image
// similarity between two images.
class GlobalDescriptorExtractor 
{
public:
  virtual ~GlobalDescriptorExtractor() {}

  // Add features to the descriptor extractor for training. This method may be
  // called multiple times to add multiple sets of features (e.g., once per
  // image) to the global descriptor extractor for training.
  virtual void AddFeaturesForTraining(const std::vector<Eigen::VectorXf>& features) = 0;

  // Train the global descriptor extracto with the given set of feature
  // descriptors added with AddFeaturesForTraining. It is assumed that all
  // descriptors have the same length.
  virtual bool Train() = 0;

  // Compute a global image descriptor for the set of input features.
  virtual Eigen::VectorXf ExtractGlobalDescriptor(const std::vector<Eigen::VectorXf>& features) = 0;
};

}  // namespace GraphSfM
#endif  // XFEATURES_MATCHING_GLOBAL_DESCRIPTOR_EXTRACTOR_H
