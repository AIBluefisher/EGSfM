#ifndef XFEATURES_MATH_RESERVOIR_SAMPLER_H
#define XFEATURES_MATH_RESERVOIR_SAMPLER_H

#include <vector>

#include "util/random.h"

namespace GraphSfM {

// A reservoir sampler is a memory-constrained probabilistic sampler. It is
// suitable for drawing random samples from a sequence with a large or unknown
// number of items. As such, this class is suitable for drawing K random samples
// from a sequence of N >> K elements without having to hold all N elements in
// memory.
//
// The sample proceeds by choosing to keep the i-th element added to the sampler
// with a probability of K / i.
template <typename ElementType>
class ReservoirSampler 
{
public:
  // TODO(sweeneychris): Add a constructor to set the RNG or the seed.

  // The number of elements we would like to sample from the entire sequence.
  explicit ReservoirSampler(const int num_elements_to_sample)
      : num_elements_to_sample_(num_elements_to_sample), num_elements_added_(0)
  {
    randomly_sampled_elements_.reserve(num_elements_to_sample_);
  }

  // Add a single element to the sampler. The element will be retained (i.e.
  // will be considered a part of the random K samples) with probability K / i,
  // where i is the # of elements added to the sampler so far.
  void AddElementToSampler(const ElementType& element) 
  {
    // If we do not have enough samples yet, add the element to the sampling
    // with probabiliy of 1.
    if (num_elements_added_ < num_elements_to_sample_) {
      randomly_sampled_elements_.push_back(element);
    } else {
      // Otherwise, we want to add the new element to our sampling with if
      //   Rand(0.0, 1.0) < K / i.
      // This is equivalent to evaluating the probability that Random(0, N) < K
      // where N is the number of elements added so far, but this version avoids
      // costly division operators for each sample.
      const int modified_sample_probability = rng_.RandInt(0, num_elements_added_);
      if (modified_sample_probability < num_elements_to_sample_) {
        randomly_sampled_elements_[modified_sample_probability] = element;
      }
    }
    ++num_elements_added_;
  }

  // Returns all of the samples
  const std::vector<ElementType>& GetAllSamples() const 
  {
    return randomly_sampled_elements_;
  }

  // Return the total number of elements added to the reservoir.
  int NumElementsAdded() const { return num_elements_added_; }

private:
  // The number of elements we would like to sample from the entire sequence.
  const int num_elements_to_sample_;
  // The number of elements currently added to the sampler. This informs how to
  // probabilistically sample new data as it is added.
  int num_elements_added_;
  RandomNumberGenerator rng_;

  // The current random sampling of elements.
  std::vector<ElementType> randomly_sampled_elements_;
};
}  // namespace GraphSfM

#endif  // XFEATURES_MATH_RESERVOIR_SAMPLER_H
