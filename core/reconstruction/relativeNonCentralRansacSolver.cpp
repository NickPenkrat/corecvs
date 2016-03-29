#include "relativeNonCentralRansacSolver.h"
#include "relativeNonCentralP6PSolver.h"
#include "levenmarq.h"

corecvs::RelativeNonCentralRansacSolver::RelativeNonCentralRansacSolver(CameraFixture *query, const MatchContainer &matchesRansac, const MatchContainer &matchesAll, const RelativeNonCentralRansacSolverSettings &settings)
    : RelativeNonCentralRansacSolverSettings(settings), matchesRansac(matchesRansac), matchesAll(matchesAll), query(query)
{
}
void corecvs::RelativeNonCentralRansacSolver::run()
{
    currentHypothesis.resize(1);
    currentHypothesis[0].rotor = corecvs::Quaternion(0, 0, 0, 1);
    currentHypothesis[0].shift = corecvs::Vector3dd(1, 1, 1);
    currentScores.resize(1);
    scoreCurrent();
    nullInliers = currentScores[0];

    if (matchesRansac.size() < FEATURES_FOR_MODEL)
        return;

    maxInliers = 0;
    size_t reportBy = maxIterations / 20;
    for (size_t i = 0; i < maxIterations; ++i)
    {
        sampleRays();
        estimatePose();
        scoreCurrent();
        selectBest();
        if ((i + 1) % reportBy == 0)
        {
            std::cout << ((double)i) / ((double) maxIterations) * 100.0 << "% complete" << std::endl;
        }
        if (maxInliers > FEATURES_FOR_MODEL)
        {
            double curr = maxInliers / (1.0 * matchesAll.size());
            double N = std::log(0.001) / std::log(1.0 - std::pow(curr, FEATURES_FOR_MODEL));
            if ((i+1)%reportBy ==0)
            std::cout << "MI: " << maxInliers << " curr = " << curr << " N: " << N << " P: " << std::pow(1.0 - std::pow(curr, FEATURES_FOR_MODEL), i) << std::endl;
            if (i > N && N > 0)
            {
                std::cout << "Finished at " << i << "th iteration" << std::endl;
                break;
            }
        }
    }
    query->location = bestHypothesis;
}

double corecvs::RelativeNonCentralRansacSolver::getGamma()
{
    return std::pow((1.0 - std::pow(maxInliers * 1.0 / matchesAll.size(), FEATURES_FOR_MODEL)), maxIterations);
}

void corecvs::RelativeNonCentralRansacSolver::sampleRays()
{
    pluckerRef.resize(FEATURES_FOR_MODEL);
    pluckerQuery.resize(FEATURES_FOR_MODEL);

    CameraFixture queryCopy = *query;
    queryCopy.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    queryCopy.location.shift = corecvs::Vector3dd(0, 0, 0);

    int N = (int)matchesRansac.size();
    if (N == 0) {
        cout << "Error: sampleRays() there's no matches!" << endl;
        return;
    }

    int idxs[FEATURES_FOR_MODEL] = { 0 };
    for (int rdy = 0; rdy < FEATURES_FOR_MODEL;)
    {
        idxs[rdy] = rng() % N;
        bool isOk = true;
        for (int i = 0; i < rdy && isOk; ++i)
            if (idxs[i] == idxs[rdy])
                isOk = false;
        if (isOk) ++rdy;
    }

    for (int i = 0; i < FEATURES_FOR_MODEL; ++i)
    {
        auto t = matchesRansac[idxs[i]];
        auto cam1 = std::get<0>(t).u->getWorldCamera(std::get<0>(t).v);
        auto cam2 = query->getWorldCamera(std::get<2>(t).v);
        auto r1 = cam1.rayFromPixel(std::get<1>(t));
        auto r2 = cam2.rayFromPixel(std::get<3>(t));
        pluckerRef[i] = r1.pluckerize();
        pluckerQuery[i] = r2.pluckerize();
    }
}

void corecvs::RelativeNonCentralRansacSolver::estimatePose()
{
    currentHypothesis = corecvs::RelativeNonCentralP6PSolver::SolveRelativeNonCentralP6P(pluckerRef, pluckerQuery);
    int id = 0;
    for (auto a: currentHypothesis)
        if ((!a.shift) > 1.0)
            currentHypothesis[id++] = a;
    currentHypothesis.resize(id);
    currentScores.clear();
    currentScores.resize(currentHypothesis.size());
}

void corecvs::RelativeNonCentralRansacSolver::scoreCurrent()
{
    int M = (int)query->cameras.size();
    int K = (int)currentHypothesis.size();

    currentInliers.resize(K);

    for (int i = 0; i < K; ++i)
    {
        currentInliers[i].clear();
    }

    for (size_t j = 0; j < matchesAll.size(); ++j)
    {
        auto t = matchesAll[j];
        auto fixtureRef  = std::get<0>(t).u,
             fixtureQuery= query;
        auto camRef  = std::get<0>(t).v,
             camQuery= std::get<2>(t).v;
        auto ptRef  = std::get<1>(t);
        auto ptQuery= std::get<3>(t);
        auto K1 = camRef->intrinsics.getKMatrix33().inv();
        auto K2 = camQuery->intrinsics.getKMatrix33().inv();
        auto ptER = K1 * ptRef;
        auto ptEQ = K2 * ptQuery;

        for (int i = 0; i < K; ++i)
        {
            query->location = currentHypothesis[i];
            auto score = fixtureRef->scoreFundamental(camRef, ptRef, fixtureQuery, camQuery, ptQuery);
            if (score > inlierThreshold)
                continue;
            auto E = fixtureRef->essentialTo(camRef, fixtureQuery, camQuery);
            double sL, sR, foo;
            E.getScaler(ptER, ptEQ, sL, sR, foo);
            if (sL >= 0.0 && sR >= 0.0)
            {
                currentScores[i]++;
                currentInliers[i].push_back((int)j);
            }
        }
    }
}

void corecvs::RelativeNonCentralRansacSolver::selectBest()
{
    size_t maxIdx = 0;
    int maxCnt = currentScores[maxIdx];
    for (size_t i = 1; i < currentScores.size(); ++i)
        if (currentScores[i] > maxCnt)
        {
            maxIdx = i;
            maxCnt = currentScores[i];
        }
    if (maxIdx >= currentScores.size())
        return;
    if (maxCnt > maxInliers)
    {
        maxInliers = maxCnt;
        bestHypothesis = currentHypothesis[maxIdx];
        bestInliers = currentInliers[maxIdx];
    }
}

corecvs::Affine3DQ corecvs::RelativeNonCentralRansacSolver::getBestHypothesis() const
{
    return bestHypothesis;
}


void corecvs::RelativeNonCentralRansacSolver::computeError(double out[])
{
    int M = (int)query->cameras.size();


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

int corecvs::RelativeNonCentralRansacSolver::getInliersCount()
{
    return (int)bestInliers.size();
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
    currentHypothesis.clear();
    maxInliers = 0;
    currentHypothesis.emplace_back(bestHypothesis);
    scoreCurrent();
    selectBest();
    std::cout << "New inliers: " << maxInliers << std::endl;
}

std::vector<int> corecvs::RelativeNonCentralRansacSolver::getBestInliers() const
{
    return bestInliers;
}

