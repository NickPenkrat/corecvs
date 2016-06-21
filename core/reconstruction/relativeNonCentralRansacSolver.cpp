#include "relativeNonCentralRansacSolver.h"
#include "relativeNonCentralP6PSolver.h"
#include "relativeNonCentralO3PSolver.h"
#include "levenmarq.h"

#include <chrono>
#include <cmath>

corecvs::RelativeNonCentralRansacSolver::RelativeNonCentralRansacSolver(CameraFixture *query
    , const MatchContainer &matchesRansac
    , const MatchContainer &matchesAll
    , const RelativeNonCentralRansacSolverSettings &settings)
    : RelativeNonCentralRansacSolverSettings(settings)
    , query(query)
    , matchesRansac(matchesRansac)
    , matchesAll(matchesAll)
{
    buildDependencies();
}


corecvs::RelativeNonCentralRansacSolver::RelativeNonCentralRansacSolver(CameraFixture *query, const Affine3DQ firstTry, const MatchContainer &matchesRansac, const MatchContainer &matchesAll, const RelativeNonCentralRansacSolverSettings &settings)
    : RelativeNonCentralRansacSolverSettings(settings), query(query), matchesRansac(matchesRansac), matchesAll(matchesAll)
{
    buildDependencies();
    makeTry(firstTry);
}


void corecvs::RelativeNonCentralRansacSolver::makeTry(const corecvs::Affine3DQ &hypo)
{
    std::cout << "RNCRS::makeTry()" << std::endl;
    Estimator es(this, inlierThreshold, restrictions, shift, scale);
    es.hypothesis.push_back(hypo);
    es.selectInliers();
    std::cout << "RNCRS::makeTry()" << std::endl;
}


void corecvs::RelativeNonCentralRansacSolver::run()
{
    if (matchesRansac.size() < FEATURES_FOR_MODEL)
        return;

    do
    {
        corecvs::parallelable_for(0, batches, ParallelEstimator(this, batch));
        usedEvals += batches * batch;
    } while (usedEvals < nForGamma() && usedEvals < (int)maxIterations);

    maxIterations = usedEvals;
    std::cout << "Finishing after " << usedEvals << " with gamma" << getGamma() << std::endl;
}

double corecvs::RelativeNonCentralRansacSolver::nForGamma()
{
    double alpha = ((double)maxInliers) / matchesAll.size();
    double N = std::log(gamma) / std::log(1.0 - std::pow(alpha, sampleSize()));
    if (alpha == 0.0)
        return maxIterations;
    return N;
}

void corecvs::RelativeNonCentralRansacSolver::ParallelEstimator::operator() (const corecvs::BlockedRange<int> &r) const
{
    Estimator estimator(solver, solver->inlierThreshold, solver->restrictions, solver->shift, solver->scale);
    estimator.localMax = solver->maxInliers;
    estimator(corecvs::BlockedRange<int>(r.begin() * batch, r.end() * batch));
}

void corecvs::RelativeNonCentralRansacSolver::Estimator::operator() (const corecvs::BlockedRange<int> &r)
{
    if ((int)solver->matchesRansac.size() < solver->sampleSize())
        return;

    for (int i = r.begin(); i < r.end(); ++i)
    {
        sampleModel();
        makeHypo();
        selectInliers();
    }
}

void corecvs::RelativeNonCentralRansacSolver::Estimator::sampleModel()
{
    int ss = solver->sampleSize();
    CameraFixture queryCopy = *solver->query;
    queryCopy.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    queryCopy.location.shift = corecvs::Vector3dd(0, 0, 0);

    auto& matchesRansac = solver->matchesRansac;

    int N = (int)matchesRansac.size();

    for (int rdy = 0; rdy < ss;)
    {
        idxs[rdy] = rng() % N;
        bool isOk = true;
        for (int i = 0; i < rdy && isOk; ++i)
            if (idxs[i] == idxs[rdy])
                isOk = false;
        if (isOk) ++rdy;
    }

    for (int i = 0; i < ss; ++i)
    {
        auto t = matchesRansac[idxs[i]];
        auto r1 = std::get<0>(t).u->rayFromPixel(std::get<0>(t).v, std::get<1>(t));
        auto r2 = std::get<2>(t).u->rayFromPixel(std::get<2>(t).v, std::get<3>(t));
        pluckerRef[i] = r1.pluckerize();
        pluckerQuery[i] = r2.pluckerize();
    }
}


void corecvs::RelativeNonCentralRansacSolver::Estimator::makeHypo()
{
    if (restrictions == RelativeNonCentralRansacSolverSettings::Restrictions::SHIFT)
        hypothesis = corecvs::RelativeNonCentralO3PSolver::SolveRelativeNonCentralO3P(pluckerRef, pluckerQuery, shift);
    else
        hypothesis = corecvs::RelativeNonCentralP6PSolver::SolveRelativeNonCentralP6P(pluckerRef, pluckerQuery);
}

void corecvs::RelativeNonCentralRansacSolver::Estimator::selectInliers()
{
    std::vector<int> currentInliers, bestInliers;
    int id = 0;
    for (auto a: hypothesis)
    {
        if ((!a.shift) < 1e-6)
            continue;
        switch (restrictions)
        {
            case RelativeNonCentralRansacSolverSettings::Restrictions::SCALE:
                a.shift = a.shift * scale / !a.shift;
                break;
            case RelativeNonCentralRansacSolverSettings::Restrictions::SHIFT:
                a.shift = shift;
                break;
            default:
                break;
        }
        hypothesis[id++] = a;
    }
    hypothesis.resize(id);
    CameraFixture query = *solver->query;
    auto& fundamentalsCacheId = solver->fundamentalsCacheId;
    auto& matchesAll = solver->matchesAll;
    auto& dependencyList = solver->dependencyList;
    corecvs::Affine3DQ bestHypothesis;

    for (size_t i = 0; i < hypothesis.size(); ++i)
    {
        currentInliers.clear();
        query.location = hypothesis[i];
        for (size_t j = 0; j < fundamentalsCacheId.size(); ++j)
        {
            auto& wpp = fundamentalsCacheId[j];
            fundamentalsCache[j] = wpp.first.u->fundamentalTo(wpp.first.v, &query, wpp.second.v);
            essentialsCache[j] = wpp.first.u->essentialTo(wpp.first.v, &query, wpp.second.v);
        }

        for (size_t j = 0; j < matchesAll.size(); ++j)
        {
            auto t = matchesAll[j];
            auto ptRef  = std::get<1>(t);
            auto ptQuery= std::get<3>(t);

            CORE_ASSERT_TRUE_S(j < dependencyList.size());
            CORE_ASSERT_TRUE_S((size_t)dependencyList[j] < fundamentalsCache.size());
            auto F = fundamentalsCache[dependencyList[j]];
            corecvs::Line2d l = F.mulBy2dRight(ptQuery), r = F.mulBy2dLeft(ptRef);
            double score = std::max(l.distanceTo(ptRef), r.distanceTo(ptQuery));
            if (score > inlierThreshold || std::isnan(score) || std::isinf(score))
                continue;
//            std::cout << score << " ";
            auto E = essentialsCache[dependencyList[j]];
            auto camRef  = std::get<0>(t).v,
                 camQuery= std::get<2>(t).v;
            auto ptER = camRef->intrinsics.reverse(ptRef);
            auto ptEQ = camQuery->intrinsics.reverse(ptQuery);
            double sL, sR, foo;
            E.getScaler(ptER, ptEQ, sL, sR, foo);
            if (sL >= 0.0 && sR >= 0.0)
            {
                currentInliers.push_back((int)j);
            }
        }
//            std::cout << std::endl;
        if (currentInliers.size() > localMax && currentInliers.size() > bestInliers.size())
        {
            std::swap(bestInliers, currentInliers);
            bestHypothesis = hypothesis[i];
        }

    }
    solver->accept(bestHypothesis, bestInliers);
}

void corecvs::RelativeNonCentralRansacSolver::accept(const corecvs::Affine3DQ& hypo, const std::vector<int> &inliers)
{
#ifdef WITH_TBB
    tbb::mutex::scoped_lock lock(mutex);
#endif
    if (inliers.size() <= bestInliers.size())
        return;
    std::cout << "ACCEPTING " << hypo << " cause it has " << inliers.size() << " inliers" << std::endl;
    bestInliers = inliers;
    bestHypothesis = hypo;
    maxInliers = (int)inliers.size();
    std::cout << "RP6P: " << maxInliers << " " << ((double)maxInliers) / ((double)matchesAll.size()) * 100.0 << "%" << std::endl;
}

double corecvs::RelativeNonCentralRansacSolver::getGamma()
{
    return std::pow((1.0 - std::pow(maxInliers * 1.0 / matchesAll.size(), sampleSize())), maxIterations);
}


void corecvs::RelativeNonCentralRansacSolver::buildDependencies()
{
    dependencyList.clear();
//    essentialsCache.clear();
//    fundamentalsCache.clear();
    fundamentalsCacheId.clear();

    for (size_t i = 0; i < matchesAll.size(); ++i)
    {
        auto& m = matchesAll[i];
        WPP ref = std::get<0>(m), query = std::get<2>(m);
        size_t j = 0;
        for (; j < fundamentalsCacheId.size(); ++j)
            if (fundamentalsCacheId[j].first == ref && fundamentalsCacheId[j].second == query)
                break;
        dependencyList.push_back((int)j);
        if (j < fundamentalsCacheId.size())
            continue;
        fundamentalsCacheId.emplace_back(ref, query);
    }
  //  essentialsCache.resize(fundamentalsCacheId.size());
  //  fundamentalsCache.resize(fundamentalsCacheId.size());
}

corecvs::Affine3DQ corecvs::RelativeNonCentralRansacSolver::getBestHypothesis() const
{
    return bestHypothesis;
}


void corecvs::RelativeNonCentralRansacSolver::computeError(double out[])
{
    query->location = bestHypothesis;

    int outIdx = 0;
    for (size_t ji= 0; ji< bestInliers.size(); ++ji)
    {
        int j = bestInliers[ji];
        auto t = matchesAll[j];
        auto fixtureRef = std::get<0>(t).u;
        auto camRef  = std::get<0>(t).v,
             camQuery= std::get<2>(t).v;
        auto ptRef  = std::get<1>(t);
        auto ptQuery= std::get<3>(t);

        out[outIdx++] = fixtureRef->scoreFundamental(camRef, ptRef, query, camQuery, ptQuery);
    }
}

size_t corecvs::RelativeNonCentralRansacSolver::getInliersCount()
{
    return bestInliers.size();
}

void corecvs::RelativeNonCentralRansacSolver::readParams(const double in[])
{
    corecvs::Quaternion q(in[0], in[1], in[2], in[3]);
    corecvs::Vector3dd v(in[4], in[5], in[6]);
    bestHypothesis.rotor = q.normalised();
    bestHypothesis.shift = v;
}

void corecvs::RelativeNonCentralRansacSolver::writeParams(double in[])
{
    corecvs::Quaternion q = bestHypothesis.rotor;
    corecvs::Vector3dd v = bestHypothesis.shift;
    int argout = 0;
    for (int i = 0; i < 4; ++i)
        in[argout++] = q[i];
    for (int i = 0; i < 3; ++i)
        in[argout++] = v[i];
}


void corecvs::RelativeNonCentralRansacSolver::fit(double distanceGuess)
{
    if (getInliersCount() < sampleSize())
        return;
    bestHypothesis.shift.normalise();
    bestHypothesis.shift *= distanceGuess;
    corecvs::LevenbergMarquardt lm;

    FunctorCost cost(this);
    FunctorCostNorm norm(this);

    lm.f = &cost;
    lm.normalisation = &norm;
    lm.maxIterations = 10000;
    std::vector<double> input(7);
    std::vector<double> output(getInliersCount());
    writeParams(&input[0]);
    auto res = lm.fit(input, output);
    readParams(&res[0]);
    updateInliers();
}

void corecvs::RelativeNonCentralRansacSolver::updateInliers()
{
    std::cout << "Prev inliers: " << maxInliers << std::endl;
    Estimator estimator(this, inlierThreshold, restrictions, shift, scale);
    maxInliers = 0;
    estimator.hypothesis.push_back(bestHypothesis);
    estimator.selectInliers();
    std::cout << "New inliers: " << maxInliers << std::endl;
}

std::vector<int> corecvs::RelativeNonCentralRansacSolver::getBestInliers() const
{
    return bestInliers;
}
