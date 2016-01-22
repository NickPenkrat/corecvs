#include "essentialFeatureFilter.h"

using namespace corecvs;

EssentialFeatureFilter::EssentialFeatureFilter(const Matrix33 &K1, const Matrix33 &K2, std::vector<std::array<corecvs::Vector2dd, 2>> &features, std::vector<std::array<corecvs::Vector2dd, 2>> &featuresInlierCheck, double inlierRadius, double targetGamma, int maxIter, int batch, int batches) : K1(K1.inv()), K2(K2.inv()), features(features), inlierRadius(inlierRadius), targetGamma(targetGamma), maxIter(maxIter), batch(batch), batches(batches), featuresInlierCheck(featuresInlierCheck)
{
}

void EssentialFeatureFilter::accept(const EssentialDecomposition &ed, const std::vector<int> &inliers)
{
    tbb::mutex::scoped_lock lock(mutex);
    if (inliers.size() <= this->inlierIdx.size())
        return;
    inlierIdx = inliers;
    bestDecomposition = ed;
}

void EssentialFeatureFilter::estimate()
{
    if (!features.size())
        return;
    int iter = 1;
    while (iter < nForGamma() && iter < maxIter)
    {
        parallelable_for(0, batches, ParallelEstimator(this, inlierRadius, batch));
        iter += batch * batches;
    }
}

double EssentialFeatureFilter::nForGamma()
{
    double inl = std::max(1.0, (double)inlierIdx.size());
    double total = featuresInlierCheck.size();
    double alpha = inl / total;
    double N = std::log(targetGamma) / std::log(1.0 - std::pow(alpha, 5));
    return N;
}


void EssentialFeatureFilter::Estimator::operator() (const corecvs::BlockedRange<int> &r)
{
    rng = std::mt19937(std::random_device()());
    for (int i = r.begin() * batch; i < r.end() * batch; ++i)
    {
        makeHypo();
        selectInliers();
    }
}

void EssentialFeatureFilter::ParallelEstimator::operator() (const corecvs::BlockedRange<int> &r) const
{
    EssentialFeatureFilter::Estimator es(filter, inlierRadius, batch);
    es(r);
}

void EssentialFeatureFilter::Estimator::makeHypo()
{
    auto& features = filter->features;
    auto& K1 = filter->K1;
    auto& K2 = filter->K2;
    int N = features.size();
    if (N == 0)
        return;
    int idx[5];
    for (int i = 0; i < 5;)
    {
        idx[i] = rng() % N;
        bool ok = true;
        for (int j = 0; j < i && ok; ++j)
            if (idx[i] == idx[j])
                ok = false;
        if (ok)
            i++;
    }
    hypoBase.resize(5);
    hypo.resize(5);
    for (int i = 0; i < 5; ++i)
    {
        hypoBase[i].start = K1 * features[idx[i]][0];
        hypoBase[i].end   = K2 * features[idx[i]][1];
        hypo[i] = &hypoBase[i];
    }
    model = EssentialEstimator().getEssential5point(hypo);
}

void EssentialFeatureFilter::Estimator::selectInliers()
{
    auto& features = filter->features;
    auto& featuresInlierCheck = filter->featuresInlierCheck;
    auto& K1 = filter->K1;
    auto& K2 = filter->K2;
    inliers.clear();
    std::vector<int> local;
    EssentialDecomposition ed;

    EssentialDecomposition decompositions[4];
    std::vector<int> inliers[4];
    int inlierCnt[4] = {0};

    for (auto& F: model)
    {
        Matrix33 FM = K1.transposed() * F * K2;
        F.decompose(decompositions);
        for (int i = 0; i < 4; ++i)
        {
            inliers[i].clear();
            inlierCnt[i] = 0;
        }
        for (auto& m: featuresInlierCheck)
        {
            auto L = m[0];
            auto R = m[1];
            Line2d LL = FM.mulBy2dRight(R);
            Line2d RL = FM.mulBy2dLeft (L);
            double diff = std::max(LL.distanceTo(L), RL.distanceTo(R));
            if (diff < inlierRadius)
            {
                for (int i = 0; i < 4; ++i)
                {
                    double scaleL, scaleR, foo;
                    auto R1 = K2 * R;
                    auto L1 = K1 * L;

                    decompositions[i].getScaler(L1, R1, scaleL, scaleR, foo);
                    if (scaleL > 0.0 && scaleR > 0.0)
                    {
                        ++inlierCnt[i];
                        inliers[i].push_back(&m - &featuresInlierCheck[0]);
                    }
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            if (inlierCnt[i] > local.size())
            {
                local = std::move(inliers[i]);
                ed = decompositions[i];
            }
    }
    if (local.size())
       filter->accept(ed, local);
}