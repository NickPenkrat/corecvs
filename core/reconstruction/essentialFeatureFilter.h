#ifndef ESSENTIALFEATUREFILTER
#define ESSENTIALFEATUREFILTER

#include <random>
#include <vector>

#include "matrix33.h"
#include "essentialEstimator.h"
#include "tbbWrapper.h"
#ifdef WITH_TBB
#include "tbb/mutex.h"
#endif

namespace corecvs
{
class EssentialFeatureFilter
{
public:
    EssentialFeatureFilter(const Matrix33 &K1, const Matrix33 &K2, std::vector<std::array<corecvs::Vector2dd, 2>> &features, std::vector<std::array<corecvs::Vector2dd, 2>> &featuresInlierCheck, double inlierRadius = 2, double targetGamma = 1e-2, int maxIter = 16000, int batch = 100, int batches = 16);

    void estimate();

    corecvs::EssentialDecomposition bestDecomposition;
    std::vector<int> inlierIdx;


    struct Estimator
    {
        Estimator(EssentialFeatureFilter* filter, double inlierRadius, int batch) : batch(batch), filter(filter), inlierRadius(inlierRadius)
        {
        }
        void operator() (const corecvs::BlockedRange<int> &r);

        void makeHypo();
        void selectInliers();

        std::vector<Correspondence> hypoBase;
        std::vector<Correspondence*> hypo;
        std::vector<int> inliers;
        std::vector<EssentialMatrix> model;
        std::mt19937 rng;
        corecvs::EssentialDecomposition ed;
        int localMax = 0;
        int batch;
        double inlierRadius;
        EssentialFeatureFilter* filter;
    };
    struct ParallelEstimator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const;
        ParallelEstimator(EssentialFeatureFilter* filter, double inlierRadius, int batch) : batch(batch), filter(filter), inlierRadius(inlierRadius)
        {
        }
        int batch;
        double inlierRadius;
        EssentialFeatureFilter* filter;
    };

    void accept(const EssentialDecomposition &ed, const std::vector<int> &inlierIdx);
    double nForGamma();

    corecvs::Matrix33 K1, K2;
    std::vector<std::array<corecvs::Vector2dd, 2>> features;
    std::vector<std::array<corecvs::Vector2dd, 2>> featuresInlierCheck;
    int maxIter, batch, batches;
    double targetGamma, inlierRadius;
#ifdef WITH_TBB
    tbb::mutex mutex;
#endif
};
}

#endif
