#include "relativeNonCentralRansacSolver.h"
#include "relativeNonCentralP6PSolver.h"
#include "levenmarq.h"

corecvs::RelativeNonCentralRansacSolver::RelativeNonCentralRansacSolver(const corecvs::Photostation &ref, const corecvs::Photostation &query, const MatchContainer &matchesRansac, const MatchContainer &matchesAll, const RelativeNonCentralRansacSolverSettings &settings)
    : RelativeNonCentralRansacSolverSettings(settings), matchesRansac(matchesRansac), matchesAll(matchesAll)
{
    pss[0] = ref;
    pss[1] = query;
}
void corecvs::RelativeNonCentralRansacSolver::run()
{
    currentHypothesis.resize(1);
    currentHypothesis[0].rotor = corecvs::Quaternion(0, 0, 0, 1);
    currentHypothesis[0].shift = corecvs::Vector3dd(1, 1, 1);
    currentScores.resize(1);
    scoreCurrent();
    nullInliers = currentScores[0];

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
    }
}

void corecvs::RelativeNonCentralRansacSolver::sampleRays()
{
    pluckerRef.resize(6);
    pluckerQuery.resize(6);
    
    pss[1].location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    pss[1].location.shift = corecvs::Vector3dd(0, 0, 0);

    int N = matchesRansac.size();

    bool multiCam = true;
    int idxs[6] = { 0 };
    do
    {
        int rdy = 0;
        for(; rdy < 6;)
        {
            int idx = rng() % N;
            auto t = matchesRansac[idx];
            int df = (6+std::get<0>(t) - std::get<2>(t)) % 6;
            df = std::min(df, 6 - df);
            if (df > 1)
                continue;
            for (int i = 0; i < rdy; ++i)
                if (idxs[i] == idx)
                {
                    idx = -1;
                    break;
                }
            if (idx >= 0)
                idxs[rdy++] = idx;
        }
		std::vector<int> usedRef  (pss[0].cameras.size());
		std::vector<int> usedQuery(pss[1].cameras.size());
        for (int i = 0; i < 6; ++i)
        {
            auto t = matchesRansac[idxs[i]];
            int camRef = std::get<0>(t);
            int camQue = std::get<2>(t);
            usedRef[camRef]++;
            usedQuery[camQue]++;
        }
        int cur = 0, cuq = 0;
        for (int i = 0; i < pss[0].cameras.size(); ++i)
            if (usedRef[i]) cur++;
        for (int i = 0; i < pss[1].cameras.size(); ++i)
            if (usedQuery[i]) cuq++;
        multiCam = cur > 4 && cuq > 4;
    } while(!multiCam);

    for (int i = 0; i < 6; ++i)
    {
        auto t = matchesRansac[idxs[i]];
        auto cam1 = pss[0].getRawCamera(std::get<0>(t));
        auto cam2 = pss[1].getRawCamera(std::get<2>(t));
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
    int N = pss[0].cameras.size();
    int M = pss[1].cameras.size();
    int K = currentHypothesis.size();

    fundamentals.resize(K);
    essentials.resize(K);
    currentInliers.resize(K);

    for (int i = 0; i < K; ++i)
    {
        pss[1].location = currentHypothesis[i];
        fundamentals[i].resize(N * M);
        essentials[i].resize(N * M);
        currentInliers[i].clear();

        for (int j = 0; j < N; ++j)
        {
            for (int k = 0; k < M; ++k)
            {
                auto cam1 = pss[0].getRawCamera(j);
                auto cam2 = pss[1].getRawCamera(k);
                fundamentals[i][j * M + k] = cam1.fundamentalTo(cam2);
                essentials  [i][j * M + k] = cam1.essentialDecomposition(cam2);
            }
        }
    }

    for (int j = 0; j < matchesAll.size(); ++j)
    {
        auto t = matchesAll[j];
        int camRef  = std::get<0>(t);
        int camQuery= std::get<2>(t);
        auto ptRef  = std::get<1>(t);
        auto ptQuery= std::get<3>(t);
        auto K1 = pss[0].getRawCamera(camRef).intrinsics.getKMatrix33().inv();
        auto K2 = pss[1].getRawCamera(camQuery).intrinsics.getKMatrix33().inv();
        auto ptER = K1 * ptRef;
        auto ptEQ = K2 * ptQuery;

        for (int i = 0; i < K; ++i)
        {
            auto F = fundamentals[i][camRef * M + camQuery];
            corecvs::Line2d lineLeft(F.mulBy2dRight(ptQuery));
            corecvs::Line2d lineRight(F.mulBy2dLeft(ptRef));
            double left = lineLeft.distanceTo(ptRef);
            double right= lineRight.distanceTo(ptQuery);
            if (std::max(left, right) > inlierThreshold)
                continue;
            auto E = essentials[i][camRef * M + camQuery];
            double sL, sR, foo;
            E.getScaler(ptER, ptEQ, sL, sR, foo);
            if (sL >= 0.0 && sR >= 0.0)
            {
                currentScores[i]++;
                currentInliers[i].push_back(j);
            }
        }
    }
}

void corecvs::RelativeNonCentralRansacSolver::selectBest()
{
    int maxIdx = 0;
    int maxCnt = currentScores[maxIdx];
    for (int i = 1; i < currentScores.size(); ++i)
        if (currentScores[i] > maxCnt)
        {
            maxIdx = i;
            maxCnt = currentScores[i];
        }
    if (maxIdx >= currentScores.size())
        return;
    if (maxCnt > maxInliers)
    {
        int N = pss[0].cameras.size();
        int M = pss[1].cameras.size();
        std::vector<int> inlierStats(N * M);
        maxInliers = maxCnt;
        bestHypothesis = currentHypothesis[maxIdx];
        bestInliers = currentInliers[maxIdx];
        for (auto& id: bestInliers)
        {
            auto t = matchesAll[id];
            int idRef = std::get<0>(t);
            int idQue = std::get<2>(t);
            inlierStats[idRef * M + idQue]++;
        }
        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < M; ++j)
                std::cout << inlierStats[i * M + j] << " ";
            std::cout << std::endl;
        }
        std::cout << "Best L: " << (!bestHypothesis.shift) << " (" << maxInliers << ") [" << nullInliers << "] {" << matchesAll.size() << " / " << matchesRansac.size() << "}" <<  std::endl;
    }
}

corecvs::Affine3DQ corecvs::RelativeNonCentralRansacSolver::getBestHypothesis() const
{
    return bestHypothesis;
}


void corecvs::RelativeNonCentralRansacSolver::computeError(double out[])
{
    int N = pss[0].cameras.size();
    int M = pss[1].cameras.size();

#ifndef WIN32
    corecvs::Matrix33 fundamentals[N * M];
#else
	std::vector<corecvs::Matrix33> fundamentals(N * M);
#endif

    pss[1].location = bestHypothesis;

    for (int j = 0; j < N; ++j)
    {
        for (int k = 0; k < M; ++k)
        {
            auto cam1 = pss[0].getRawCamera(j);
            auto cam2 = pss[1].getRawCamera(k);
            fundamentals[j * M + k] = cam1.fundamentalTo(cam2);
        }
    }

    int outIdx = 0;
    for (int ji= 0; ji< bestInliers.size(); ++ji)
    {
        int j = bestInliers[ji];
        auto t = matchesAll[j];
        int camRef  = std::get<0>(t);
        int camQuery= std::get<2>(t);
        auto ptRef  = std::get<1>(t);
        auto ptQuery= std::get<3>(t);

        auto F = fundamentals[camRef * M + camQuery];
        corecvs::Line2d lineLeft(F.mulBy2dRight(ptQuery));
        corecvs::Line2d lineRight(F.mulBy2dLeft(ptRef));
        double left = lineLeft.distanceTo(ptRef);
        double right= lineRight.distanceTo(ptQuery);
        out[outIdx++] = std::max(left, right);
    }
}

int corecvs::RelativeNonCentralRansacSolver::getInliersCount()
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

