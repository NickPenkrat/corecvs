#ifndef ESSENTIALFEATUREFILTER
#define ESSENTIALFEATUREFILTER

#include <random>
#include <vector>
#include <array>

#include "matrix33.h"
#include "essentialEstimator.h"
#include "tbbWrapper.h"
#ifdef WITH_TBB
#include "tbb/mutex.h"
#endif

namespace corecvs
{

struct EssentialFilterParams
{
    double b2bThreshold  = 0.9,
           inlierRadius  = 2.0,
           targetGamma   = 1e-2;
    int    maxIterations = 200000;
};

class EssentialFeatureFilter
{
public:
    EssentialFeatureFilter(
            const Matrix33 &K1,
            const Matrix33 &K2,
            std::vector<std::array<corecvs::Vector2dd, 2>> &features,
            std::vector<std::array<corecvs::Vector2dd, 2>> &featuresInlierCheck,
            EssentialFilterParams params,
            int batch = 200,
            int batches = 4
    );

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
        EssentialFeatureFilter* filter;
        double inlierRadius;
    };

    struct ParallelEstimator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const;
        ParallelEstimator(EssentialFeatureFilter* filter, double inlierRadius, int batch) :
            batch(batch),
            inlierRadius(inlierRadius),
            filter(filter)
        {
        }
        int batch;
        double inlierRadius;
        EssentialFeatureFilter* filter;
    };

    void accept(const EssentialDecomposition &ed, const std::vector<int> &inlierIdx);
    double nForGamma();
    double getGamma();

    corecvs::Matrix33 K1, K2;
    std::vector<std::array<corecvs::Vector2dd, 2>> features;
    std::vector<std::array<corecvs::Vector2dd, 2>> featuresInlierCheck;
    int maxIter, batch, batches, usedIter;
    double targetGamma;
    double inlierRadius;
    static const int FEATURE_POINTS_FOR_MODEL = 5;
#ifdef WITH_TBB
    tbb::mutex mutex;
#endif
};
}

#endif
