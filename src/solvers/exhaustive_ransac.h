#ifndef SOLVERS_EXHAUSTIVE_RANSAC_H
#define SOLVERS_EXHAUSTIVE_RANSAC_H

#include "solvers/exhaustive_sampler.h"
#include "solvers/sample_consensus_estimator.h"
#include "solvers/sampler.h"

namespace GraphSfM {

template <class ModelEstimator>
class ExhaustiveRansac : public SampleConsensusEstimator<ModelEstimator> 
{
public:
    typedef typename ModelEstimator::Datum Datum;
    typedef typename ModelEstimator::Model Model;

    ExhaustiveRansac(const RansacParameters& ransac_params,
                     const ModelEstimator& estimator)
        : SampleConsensusEstimator<ModelEstimator>(ransac_params, estimator) {}
    virtual ~ExhaustiveRansac() {}

    // Initializes the random sampler and inlier support measurement.
    bool Initialize() {
        Sampler* random_sampler = new ExhaustiveSampler(
            this->ransac_params_.rng, this->estimator_.SampleSize());
        return SampleConsensusEstimator<ModelEstimator>::Initialize(random_sampler);
    }
};

}  // namespace theia

#endif  // THEIA_SOLVERS_EXHAUSTIVE_RANSAC_H_
