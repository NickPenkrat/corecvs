#include "absoluteNonCentralRansacSolver.h"

#include "pnpSolver.h"
#include <random>

void corecvs::AbsoluteNonCentralRansacSolver::run()
{
    do
    {
        corecvs::parallelable_for(0, batches, ParallelEstimator(this, batch));
        usedEvals += batches * batch;
    } while (usedEvals < nForGamma() && usedEvals < maxIterations);
}

double corecvs::AbsoluteNonCentralRansacSolver::nForGamma()
{
    double alpha = ((double)bestInlierCnt) / cloudMatches.size();
    double N = std::log(gamma) / std::log(1.0 - std::pow(alpha, Estimator::SAMPLESIZE));
    return N;
}

void corecvs::AbsoluteNonCentralRansacSolver::ParallelEstimator::operator() (const corecvs::BlockedRange<int> &r) const
{
    Estimator estimator(solver, solver->reprojectionInlierThreshold);
    estimator.localMax = solver->bestInlierCnt;
    estimator(corecvs::BlockedRange<int>(r.begin() * batch, r.end() * batch));
}

void corecvs::AbsoluteNonCentralRansacSolver::Estimator::operator() (const corecvs::BlockedRange<int> &r)
{
    if (solver->cloudMatches.size() < SAMPLESIZE)
        return;
    for (int i = r.begin(); i < r.end(); ++i)
    {
        sampleModel();
        makeHypo();
        selectInliers();
    }
}

void corecvs::AbsoluteNonCentralRansacSolver::runInliersRE()
{
    ReprojectionError err(this);
    ReprojectionErrorNormalizer norm(this);

    corecvs::LevenbergMarquardt lm;
    lm.f = &err;
    lm.normalisation = &norm;
    lm.maxIterations = fitIterations;
    lm.trace = false;//true;

    ps->location = bestHypothesis;
    std::vector<double> in(forcePosition ? 4 : 7), out(bestInlierCnt * 2);
    readParams(&in[0]);
    writeParams(&in[0]);
    auto res = lm.fit(in, out);
    readParams(&res[0]);
}

void corecvs::AbsoluteNonCentralRansacSolver::computeReprojectionErrors(double *out)
{
    int argout = 0;
    for (auto i: inliers)
    {
        auto t = cloudMatches[i];
        FixtureCamera* cam = std::get<0>(t);
        auto pt = std::get<1>(t);
        auto ptw= std::get<2>(t);

        auto diff = (pt - ps->project(ptw, cam));
        out[argout++] = diff[0];
        out[argout++] = diff[1];
    }
}

std::vector<int> corecvs::AbsoluteNonCentralRansacSolver::selectInliers(const corecvs::Affine3DQ &hypo)
{
    ps->location = hypo;
    int M = (int)cloudMatches.size();
    std::vector<int> inliers;
    ps->location = hypo;
    if (forcePosition)
        ps->location.shift = forcedPosition;
    for (int i = 0; i < M; ++i)
    {
        auto t = cloudMatches[i];
        auto cam = std::get<0>(t);
        auto  pt = std::get<1>(t);
        auto  ptw= std::get<2>(t);
        double diff = !(pt - ps->project(ptw, cam));
        if (diff < reprojectionInlierThreshold && ps->isVisible(ptw, cam))
            inliers.push_back(i);
    }

    return inliers;
}

void corecvs::AbsoluteNonCentralRansacSolver::Estimator::sampleModel()
{
    int N = (int)solver->cloudMatches.size();
    for(int rdy = 0; rdy < SAMPLESIZE;)
    {
        idxs[rdy] = rng() % N;
        bool isOk = true;
        for (int j = 0; j < rdy && isOk; ++j)
            if (idxs[j] == idxs[rdy])
                isOk = false;
        if (isOk) rdy++;
    }
}

void corecvs::AbsoluteNonCentralRansacSolver::Estimator::makeHypo()
{
    CameraFixture ps = *solver->ps;
    ps.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    ps.location.shift = corecvs::Vector3dd(0, 0, 0);

    std::vector<corecvs::Vector3dd> centers, directions, points3d;
    auto& cloudMatches = solver->cloudMatches;
    for (int i: idxs)
    {
        auto t = cloudMatches[i];
        FixtureCamera* cam = std::get<0>(t);
        auto pt = std::get<1>(t);
        auto ptw= std::get<2>(t);

        centers.push_back(ps.getWorldCamera(cam).extrinsics.position);
        directions.push_back(ps.getWorldCamera(cam).rayFromPixel(pt).a);
        points3d.push_back(ptw);
    }

    hypothesis = corecvs::PNPSolver::solvePNP(centers, directions, points3d);

}

void corecvs::AbsoluteNonCentralRansacSolver::Estimator::selectInliers()
{
    std::vector<int> bestInliers;
    corecvs::Affine3DQ bestHypothesis;
    double bestScore = 0.0;

    CameraFixture ps = *solver->ps;

    for (auto& hypo: hypothesis)
    {
        ps.location = hypo;
        auto& cloudMatches = solver->cloudMatches;
        int M = (int)cloudMatches.size();
        std::vector<int> inliers;
        double score = 0.0;
        if (solver->forcePosition)
            ps.location.shift = solver->forcedPosition;
        for (int i = 0; i < M; ++i)
        {
            auto t = cloudMatches[i];
            FixtureCamera* cam = std::get<0>(t);
            auto pt = std::get<1>(t);
            auto ptw= std::get<2>(t);

            double diff = !(pt - ps.project(ptw, cam));
            if (diff < inlierThreshold && ps.isVisible(ptw, cam))
            {
                inliers.push_back(i);
                score += diff * diff;
            }
        }
        if (bestInliers.size() < inliers.size() || (bestInliers.size() == inliers.size() && score < bestScore))
        {
            bestScore = score;
            bestInliers = inliers;
            bestHypothesis = hypo;
        }
    }
    if (localMax < bestInliers.size())
    {
        localMax = (int)bestInliers.size();
        solver->accept(bestHypothesis, bestInliers, bestScore);
        localMax = (int)bestInliers.size();
    }
}

void corecvs::AbsoluteNonCentralRansacSolver::runInliersPNP()
{
    ps->location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    ps->location.shift = corecvs::Vector3dd(0, 0, 0);
    int N = (int)cloudMatches.size();

    std::vector<corecvs::Vector3dd> centers, directions, points3d;
    for (int i: inliers)
    {
        auto t = cloudMatches[i];
        FixtureCamera* cam = std::get<0>(t);
        auto pt = std::get<1>(t);
        auto ptw= std::get<2>(t);

        centers.push_back(ps->getWorldCamera(cam).extrinsics.position);
        directions.push_back(ps->getWorldCamera(cam).rayFromPixel(pt).a);
        points3d.push_back(ptw);
    }

    hypothesis = corecvs::PNPSolver::solvePNP(centers, directions, points3d);

    int currentBest = 0;
    corecvs::Affine3DQ currentH;
    std::vector<int> currentInliers;
    for (auto& h: hypothesis)
    {
        auto ih = selectInliers(h);
        if ((int)ih.size() < currentBest)
            continue;
        currentBest    = (int)ih.size();
        currentInliers = ih;
        currentH       = h;
    }

    // even if total solution has less inliers it is better
    //if (currentBest < bestInlierCnt)
    //    return;

    bestInlierCnt = currentBest;
    inliers = currentInliers;
    bestHypothesis = currentH;
    std::cout << "PNP: " << bestInlierCnt << " " << ((double)bestInlierCnt) / ((double)N) * 100.0 << "%" << std::endl;
}


std::vector<int> corecvs::AbsoluteNonCentralRansacSolver::getInliers()
{
    return inliers;
}

corecvs::Affine3DQ corecvs::AbsoluteNonCentralRansacSolver::getBestHypothesis()
{
    return bestHypothesis;
}
