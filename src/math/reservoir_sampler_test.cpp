#include <glog/logging.h>
#include <gtest/gtest.h>
#include <set>

#include "reservoir_sampler.h"
#include "util/random.h"

namespace GraphSfM {

TEST(ReservoirSampler, Sanity) 
{
  constexpr int kNumFeatures = 1000000;
  constexpr int kNumSampledFeatures = 1000;
  ReservoirSampler<int> reservoir_sampler(kNumSampledFeatures);

  // Add features ranging from 0 to 100. This should yield a random sample which
  // roughly corresponds to 0,1,....,99.
  for (int i = 0; i < kNumFeatures; i++) {
    reservoir_sampler.AddElementToSampler(i % kNumSampledFeatures);
  }

  RandomNumberGenerator rng;
  std::set<int> rand_samples;
  for (int i = 0; i < kNumSampledFeatures; i++) {
    rand_samples.insert(rng.RandInt(0, kNumSampledFeatures));
  }

  auto samples = reservoir_sampler.GetAllSamples();
  std::set<int> sorted_samples(samples.begin(), samples.end());
  LOG(INFO) << "Num unique reservoir samples: " << sorted_samples.size();
  LOG(INFO) << "Num unique random samples: " << rand_samples.size();

}

}  // namespace GraphSfM
