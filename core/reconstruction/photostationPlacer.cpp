#include "photostationPlacer.h"

#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <tuple>
#include <set>

#include "essentialFeatureFilter.h"
#include "relativeNonCentralRansacSolver.h"
#include "absoluteNonCentralRansacSolver.h"
#include "multicameraTriangulator.h"
#include "pnpSolver.h"
#include "calibrationHelpers.h"
#include "reconstructionInitializer.h"
#include "sceneAligner.h"
#include "statusTracker.h"
#include "log.h"

std::string toString(ReconstructionFunctorOptimizationErrorType type)
{
    switch(type)
    {
        case ReconstructionFunctorOptimizationErrorType::REPROJECTION:
            return "REPROJECTION";
        case ReconstructionFunctorOptimizationErrorType::ANGULAR:
            return "ANGULAR";
        case ReconstructionFunctorOptimizationErrorType::CROSS_PRODUCT:
            return "CROSS-PRODUCT";
        case ReconstructionFunctorOptimizationErrorType::RAY_DIFF:
            return "RAY DIFF";
    }
    CORE_ASSERT_FALSE_S(true);
}

void corecvs::PhotostationPlacer::paintTracksOnImages(bool pairs)
{
    std::mt19937 rng;
    std::uniform_real_distribution<double> runif(0, 360.0);
    std::unordered_map<SceneFeaturePoint*, RGBColor> colorizer;
    for (auto& tf: scene->trackedFeatures)
    {
        std::stringstream ss;
        ss << tf << ":";
        size_t cnt = 0;
        for (auto& o: tf->observations__)
        {
            ss << o.first.u->name << o.first.v->nameId;
            if (++cnt != tf->observations__.size())
                ss << "/";
        }
        tf->name = ss.str();
        colorizer[tf] = corecvs::RGBColor::fromHue(runif(rng), 1.0, 1.0);
    }
    std::vector<std::pair<WPP, std::string>> images;
    for (auto& p: scene->images)
    {
        bool isPlaced = false;
        for (auto& cf: scene->placedFixtures)
            if (cf == p.first.u)
                isPlaced = true;
        if (!isPlaced)
            continue;
        images.push_back(std::make_pair(p.first, p.second));
    }
    int N = (int)images.size();
    corecvs::parallelable_for(0, pairs ? N * N : N, ParallelTrackPainter(images, scene, colorizer, pairs));
}

corecvs::Affine3DQ corecvs::PhotostationPlacer::staticInit(CameraFixture *fixture, std::vector<SceneFeaturePoint*> &staticPoints)
{
    fixture->location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    fixture->location.shift = corecvs::Vector3dd(0, 0, 0);

    std::vector<corecvs::Vector3dd> centers, directions, points3d;
    for (auto spt: staticPoints)
    {
        for (auto &op: spt->observations__)
        {
            auto& o = op.second;
            auto cam = o.camera;
            auto ps  = o.cameraFixture;
            if (o.cameraFixture != fixture)
                continue;
            auto pt  = o.observation;
            auto ptw = spt->position;

            centers.push_back(ps->getWorldCamera(cam).extrinsics.position);
            directions.push_back(ps->rayFromPixel(cam, pt).a);
            points3d.push_back(ptw);
        }
    }

    auto hypothesis = corecvs::PNPSolver::solvePNP(centers, directions, points3d);
    int inliers = 0;
    int bestHypo = 0;
    for (auto& hypo: hypothesis)
    {
        fixture->location = hypo;
        int curInliers = 0;
        for (auto spt: staticPoints)
        {
            for (auto &op: spt->observations__)
            {
                auto&o = op.second;
                auto cam = o.camera;
                auto ps  = o.cameraFixture;
                if (o.cameraFixture != fixture)
                    continue;
                auto pt  = o.observation;
                auto ptw = spt->position;

                if ((!(ps->project(ptw, cam) - pt)) < inlierThreshold())
                    curInliers++;
            }
        }
        if (curInliers > inliers)
        {
            inliers = curInliers;
            bestHypo = &hypothesis[0] - &hypo;
        }
    }
    return hypothesis[bestHypo];
}

void corecvs::PhotostationPlacer::create2PointCloud()
{
    auto& placedFixtures = scene->placedFixtures;
    for (size_t A = 0; A < placedFixtures.size(); ++A)
        for (size_t B = A + 1; B < placedFixtures.size(); ++B)
            create2PointCloud(placedFixtures[A], placedFixtures[B]);
}

void corecvs::PhotostationPlacer::create2PointCloud(CameraFixture* psA, CameraFixture* psB)
{
    std::cout << "Creating 2-point tracks from " << psA->name << psB->name << std::endl;
    auto freeFeatures = scene->getUnusedFeatures(psA, psB);

    std::vector<std::tuple<FixtureCamera*, int, FixtureCamera*, int, FixtureCamera*, int>> trackCandidates;

    for (auto& f: freeFeatures)
    {
        auto camA = std::get<0>(f.first);
        auto camB = std::get<1>(f.first);
        auto  ptA = std::get<2>(f.first);
        auto  ptB = f.second;

        WPP wppA(psA, camA), wppB(psB, camB);
        auto& kpA = scene->keyPoints[wppA][ptA].first;
        auto& kpB = scene->keyPoints[wppB][ptB].first;

        double fscore = psA->scoreFundamental(camA, kpA, psB, camB, kpB);
        if (fscore > trackInlierThreshold())
            continue;

        corecvs::MulticameraTriangulator mct;
        mct.addCamera(psA->getMMatrix(camA), kpA);
        mct.addCamera(psB->getMMatrix(camB), kpB);
        auto res = mct.triangulateLM(mct.triangulate());

        bool isVisibleInlierNotTooFar = true;
        isVisibleInlierNotTooFar &= psA->isVisible(res, camA);
        isVisibleInlierNotTooFar &= (!(kpA - psA->project(res, camA))) < trackInlierThreshold();
        isVisibleInlierNotTooFar &= (!(res - psA->getWorldCamera(camA).extrinsics.position)) < distanceLimit();
        isVisibleInlierNotTooFar &= psB->isVisible(res, camB);
        isVisibleInlierNotTooFar &= (!(kpB - psB->project(res, camB))) < trackInlierThreshold();
        isVisibleInlierNotTooFar &= (!(res - psB->getWorldCamera(camB).extrinsics.position)) < distanceLimit();

        if (!isVisibleInlierNotTooFar)
            continue;

        auto track = scene->createFeaturePoint();
        track->reprojectedPosition = res;
        track->hasKnownPosition = false;
        track->type = SceneFeaturePoint::POINT_RECONSTRUCTED;

        SceneObservation soA, soB;
        soA.camera = camA;
        soA.cameraFixture = psA;
        soA.featurePoint = track;
        soA.observation = kpA;
        track->observations[camA] = soA;
        track->observations__[wppA] = soA;
        scene->trackMap[wppA][ptA] = track;
        soB.camera = camB;
        soB.cameraFixture = psB;
        soB.featurePoint = track;
        soB.observation = kpB;
        track->observations[camB] = soB;
        track->observations__[wppB] = soB;
        scene->trackMap[wppB][ptB] = track;

        scene->trackedFeatures.push_back(track);
    }
}

void corecvs::PhotostationPlacer::getErrorSummaryAll()
{
    getErrorSummary(ReconstructionFunctorOptimizationErrorType::REPROJECTION);
    getErrorSummary(ReconstructionFunctorOptimizationErrorType::ANGULAR);
    getErrorSummary(ReconstructionFunctorOptimizationErrorType::CROSS_PRODUCT);
    getErrorSummary(ReconstructionFunctorOptimizationErrorType::RAY_DIFF);
}

void corecvs::PhotostationPlacer::getErrorSummary(ReconstructionFunctorOptimizationErrorType errorType)
{
    double totalsqr = 0.0;
    double totalcnt = 0.0;
    Vector3dd errV;
    double a;
    Vector2dd errP;
    for (auto& t: scene->trackedFeatures)
        for (auto &pp : t->observations__)
        {
            auto& p = pp.second;
            totalcnt++;
            switch(errorType)
            {
                case ReconstructionFunctorOptimizationErrorType::REPROJECTION:
                    errP = p.cameraFixture->reprojectionError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += (!errP) * (!errP);
                    break;
                case ReconstructionFunctorOptimizationErrorType::CROSS_PRODUCT:
                    errV = p.cameraFixture->crossProductError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += !errV * !errV;
                    break;
                case ReconstructionFunctorOptimizationErrorType::ANGULAR:
                    a = p.cameraFixture->angleError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += a * a;
                    break;
                case ReconstructionFunctorOptimizationErrorType::RAY_DIFF:
                    errV = p.cameraFixture->rayDiffError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += !errV * !errV;
                    break;
            }
        }

    std::cout << toString(errorType) << "RMSE [" << scene->trackedFeatures.size() << " features]: " << std::sqrt(totalsqr / totalcnt) << std::endl;
    totalsqr = 0.0;
    totalcnt = 0.0;
    for (auto& t: scene->staticPoints)
        for (auto &pp : t->observations__)
        {
            bool isPlaced = false;
            for (auto& ptr: scene->placedFixtures)
                if (ptr == pp.first.u)
                    isPlaced = true;
            if (!isPlaced)
                continue;
            auto& p = pp.second;
            totalcnt++;
            switch(errorType)
            {
                case ReconstructionFunctorOptimizationErrorType::REPROJECTION:
                    errP = p.cameraFixture->reprojectionError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += (!errP) * (!errP);
                    break;
                case ReconstructionFunctorOptimizationErrorType::CROSS_PRODUCT:
                    errV = p.cameraFixture->crossProductError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += !errV * !errV;
                    break;
                case ReconstructionFunctorOptimizationErrorType::ANGULAR:
                    a = p.cameraFixture->angleError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += a * a;
                    break;
                case ReconstructionFunctorOptimizationErrorType::RAY_DIFF:
                    errV = p.cameraFixture->rayDiffError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += !errV * !errV;
                    break;
            }
        }
    std::cout << toString(errorType) << "RMSE [" << scene->staticPoints.size() << " static points]: " << std::sqrt(totalsqr / totalcnt) << std::endl;

}

void corecvs::PhotostationPlacer::fit(int num)
{
    fit(optimizationParams, num);
}

void corecvs::PhotostationPlacer::fit(const ReconstructionFunctorOptimizationType &params, int num)
{
    if (scene->placedFixtures.size() < 2)
        return;

    scene->printTrackStats();

    auto oldParams = optimizationParams;
    optimizationParams = params;
    getErrorSummaryAll();
    corecvs::LevenbergMarquardtSparse lm(num);
    ReconstructionFunctor orient(scene, errorType, optimizationParams, excessiveQuaternionParametrization, 1.0);
    ReconstructionNormalizationFunctor orientNorm(&orient, alternatingIterations);
    lm.useSchurComplement = true;
    lm.f = &orient;
    lm.normalisation = &orientNorm;
    lm.maxIterations = num;
    lm.trace = false;
    std::vector<double> input(orient.getInputNum());
    std::vector<double> out(orient.getOutputNum());
    if (orient.getOutputNum() <=  orient.getInputNum())
        return;
    orient.writeParams(&input[0]);
    auto res = lm.fit(input, out);
    orient.readParams(&res[0]);

    getErrorSummaryAll();
    static int cnt = 0;
    cnt++;

    std::map<int, int> cntr;
    for (auto& f: scene->trackedFeatures)
        cntr[(int)f->observations__.size()]++;

    std::cout << "Track sizes: " << std::endl;
    for (auto p: cntr)
        std::cout << p.first << "\t" << p.second << std::endl;
    scene->validateAll();
    optimizationParams = oldParams;

    scene->printTrackStats();
}

void corecvs::PhotostationPlacer::appendTracks(const std::vector<int> &inlierIds, CameraFixture* fixture, const std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> &possibleTracks)
{
    scene->validateAll();

    for (auto& iid: inlierIds)
    {
        auto& inlier = possibleTracks[iid];
        auto cam =     std::get<0>(inlier);
        auto feature = std::get<4>(inlier);
        WPP wpp(fixture, cam);
        if (scene->trackMap[wpp].count(feature))
            continue;
        auto track   = std::get<3>(inlier);
        auto proj    = std::get<1>(inlier);
        if ((!fixture->reprojectionError(track->reprojectedPosition, proj, cam)) > trackInlierThreshold() || (!(track->reprojectedPosition - fixture->getWorldCamera(cam).extrinsics.position)) > distanceLimit())
            continue;

        SceneObservation observation;
        observation.featurePoint = track;
        observation.camera = cam;
        observation.cameraFixture = fixture;
        bool found = false;
        for (auto& f: scene->fixtures())
            if (f == fixture)
                found = true;
        CORE_ASSERT_TRUE_S(found);
        observation.observation = proj;
        track->observations[cam] = observation;
        track->observations__[wpp] = observation;
        scene->trackMap[wpp][feature] = track;
        found = false;
        for (auto& fp: scene->featurePoints())
            if (fp == track)
                found = true;
        CORE_ASSERT_TRUE_S(found);
    }
    scene->validateAll();
}

void corecvs::PhotostationPlacer::updateTrackables()
{
    std::cout << "Starting speculative P3P update" << std::endl;
    activeInlierCount.clear();
    corecvs::Vector3dd mean(0, 0, 0);
    for (auto& ppp: scene->placedFixtures)
        mean += ppp->location.shift;
    mean = mean / scene->placedFixtures.size();
    corecvs::Quaternion r(0, 0, 0, 1);
    corecvs::Affine3DQ tf(r, -mean);
    getErrorSummaryAll();
    scene->transform(tf);
    getErrorSummaryAll();

    for (size_t i = 0; i < speculativity() && i < scene->placingQueue.size(); ++i)
    {
        auto cf = scene->placingQueue[i];
        std::cout << "\tRunning with " << cf->name << " ";
        if (activeEstimates.count(cf))
        {
            std::cout << "already have estimate: " << activeEstimates[cf].shift << activeEstimates[cf].rotor;
            activeEstimates[cf].shift -= mean;
        }
        std::cout << std::endl;

        auto hypos = scene->getPossibleTracks(cf);
        std::cout << "PRE-CTR" << std::endl;
        corecvs::AbsoluteNonCentralRansacSolver solver = activeEstimates.count(cf) ? corecvs::AbsoluteNonCentralRansacSolver(cf, hypos, activeEstimates[cf]) : corecvs::AbsoluteNonCentralRansacSolver(cf, hypos);
        solver.maxIterations = maxP3PIterations();
        solver.reprojectionInlierThreshold = inlierP3PThreshold();
        solver.gamma = gammaP3P();
        std::cout << "POST-CTR" << std::endl;

        solver.forcePosition = scene->initializationData[cf].initializationType == FixtureInitializationType::GPS && scene->is3DAligned;
        solver.forcedPosition = scene->initializationData[cf].initData.shift - mean;
        if (!scene->is3DAligned && scene->initializationData[cf].initializationType == FixtureInitializationType::GPS)
        {
            solver.forceScale = true;
            solver.forcedScale = !(mean - scene->initializationData[cf].initData.shift);
        }

        solver.run();
        solver.runInliersRE();
        activeEstimates[cf] = solver.getBestHypothesis();
        activeEstimates[cf].shift += mean;
        if (solver.forcePosition)
            activeEstimates[cf].shift = scene->initializationData[cf].initData.shift;
        activeInlierCount[cf] = solver.getInliers();
        std::cout << "Final estimate: " << activeEstimates[cf].shift << activeEstimates[cf].rotor << std::endl;
    }
    corecvs::Affine3DQ tfi(r, mean);
    getErrorSummaryAll();
    scene->transform(tfi);
    getErrorSummaryAll();
}

bool corecvs::PhotostationPlacer::append2D()
{
    // 1. Center scene
    // 2. Get all 2d<->2d correspondeces with already aligned fixtures
    // 3. Run solver
    // 4. Return true if solution meets inlier threshold and gives enough confidence
    std::cout << "ENTERING P6P" << std::endl;
    corecvs::Affine3DQ tform;

    corecvs::Vector3dd mean(0, 0, 0);
    int cnt = 0;
    for (auto& fixture: scene->placedFixtures)
    {
        ++cnt;
        mean += fixture->location.shift;
    }
    mean = mean / cnt;

    tform.shift = -mean;
    scene->transform(tform);

    std::cout << "MEANSHIFTED" << std::endl;

    std::vector<std::pair<corecvs::CameraFixture*, decltype(scene->getFixtureMatches({}, 0))>> matches;
    for (size_t iii = 0; iii < speculativity() && iii < scene->placingQueue.size(); ++iii)
    {
        auto fixture = scene->placingQueue[iii];
        matches.emplace_back(fixture, scene->getFixtureMatches(scene->placedFixtures, fixture));
    }

    std::vector<std::tuple<double, double, corecvs::CameraFixture*, corecvs::Affine3DQ>> log;

    std::sort(matches.begin(), matches.end(), [](const decltype(matches)::value_type &a, const decltype(matches)::value_type &b) { return a.second.size() > b.second.size(); });

    for (auto& p: matches)
    {
        ReconstructionInitializerParams params;
        params.essentialFilterParams.b2bThreshold = b2bRansacP5RPThreshold();
        params.essentialFilterParams.inlierRadius = inlierP5RPThreshold();
        params.b2bThreshold = b2bRansacP6RPThreshold();
        params.runEssentialFiltering = runEssentialFiltering();
        params.essentialFilterParams.maxIterations = maxEssentialRansacIterations();
        params.essentialFilterParams.targetGamma = essentialTargetGamma();
        scene->filterEssentialRansac(scene->placedFixtures, {p.first}, params.essentialFilterParams);
        auto psB = p.first;
        auto B = psB;

        std::cout << "STARTING WITH " << B->name << std::endl;

        auto matches = p.second;
        RelativeNonCentralRansacSolver::MatchContainer rm, mm;
        for (auto&t : matches)
        {
            if (std::get<4>(t) < b2bRansacP6RPThreshold())
                rm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
            mm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
        }

        if (rm.size() < minimalInlierCount())
        {
            std::cout << "Too few matches (" << rm.size() << "), rejecting ";
            for (auto &A: scene->placedFixtures)
                std::cout << A->name;
            std::cout << "<>" << B->name << std::endl;
            scene->matches = scene->matchesCopy;
            continue;
        }

        psB->location.shift = corecvs::Vector3dd(0, 0, 0);
        psB->location.rotor = corecvs::Quaternion(0, 0, 0, 1);
        RelativeNonCentralRansacSolverSettings s(maxP6PIterations(), inlierP6PThreshold(), gammaP6P());
        RelativeNonCentralRansacSolver solver(
        //        psA,
                psB, rm, mm, s);
        if (scene->initializationData[psB].initializationType == FixtureInitializationType::GPS)
        {
            if (scene->is3DAligned)
            {
                solver.restrictions = decltype(solver.restrictions)::SHIFT;
                solver.shift = scene->initializationData[psB].initData.shift - mean;
            }
            else
            {
//                if (scene->placedFixtures.size() < 2)
                {
                    solver.restrictions = decltype(solver.restrictions)::SCALE;
                    solver.scale = !(scene->initializationData[psB].initData.shift - mean);
                }
            }
        }
        if (activeP6PEstimates.count(psB))
        {
            auto h = activeP6PEstimates[psB];
            h.shift -= mean;
            solver.makeTry(h);
        }
        solver.run();
        auto best = solver.getBestHypothesis();
        for (auto &A: scene->placedFixtures)
            std::cout << A->name;
        std::cout << "::" << psB->name << " " << best.shift << " " << best.rotor << std::endl;
        psB->location = best;
        std::cout << solver.getInliersCount() << " inliers" << std::endl;
        scene->matches = scene->matchesCopy;
        std::cout << "P6P: ";
        for (auto &A: scene->placedFixtures)
            std::cout << A->name;
        std::cout << "<>" << B->name << " in P6P sense: inliers: " << solver.getInliersCount() << " P: " << solver.getGamma() << std::endl;
        log.emplace_back(solver.getInliersCount(), 1.0 - solver.getGamma(), B, best);
        activeP6PEstimates[B].rotor = best.rotor;
        activeP6PEstimates[B].shift = best.shift + mean;
    }
    std::sort(log.begin(), log.end(), [](const decltype(log)::value_type &a, const decltype(log)::value_type &b) { return std::get<0>(a) == std::get<0>(b) ? std::get<1>(a) > std::get<1>(b) : std::get<0>(a) > std::get<0>(b); });

    if (std::get<0>(log[0]) < 20)
        return false;

    auto B = std::get<2>(log[0]);
    auto psB = B;
    auto best = std::get<3>(log[0]);
    for (auto &A: scene->placedFixtures)
        std::cout << A->name;
    std::cout << "::" << psB->name << " " << best.shift << " " << best.rotor << std::endl;
    psB->location = best;
    std::cout << std::get<0>(log[0]) << " inliers" << std::endl;

    scene->placedFixtures.push_back(psB);
    for (auto& cf: scene->placedFixtures)
        std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;

    std::remove(scene->placingQueue.begin(), scene->placingQueue.end(), psB);
    scene->placingQueue.resize(scene->placingQueue.size() - 1);
    tform.shift = mean;
    scene->transform(tform);

    for (auto& cf: scene->placedFixtures)
        std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
    return true;
}

void corecvs::PhotostationPlacer::appendTracks()
{
    // !!! This goes to reconstruction scene !!!
    // Get 2D->3D coreespondences from scene
    // Append all satisfying
    for (auto& ps: scene->placedFixtures)
        scene->appendTracks(ps, trackInlierThreshold(), distanceLimit());
}

void corecvs::PhotostationPlacer::createTracks()
{
    // !!! This goes to reconstruction scene !!!
    // Get 2D->2D correspondences from scene
    // Create new points for all satisfying
    for (auto& psA: scene->placedFixtures)
        for (auto& psB: scene->placedFixtures)
            if (psA < psB)
                scene->buildTracks(psA, psB, trackInlierThreshold(), distanceLimit());
}


void corecvs::PhotostationPlacer::initialize()
{
/*
 * This goes to initialize() section
 */
    std::unordered_map<std::pair<CameraFixture*, CameraFixture*>, int> cntr, cntrGood;
    std::unordered_set<corecvs::CameraFixture*> allowed(scene->placingQueue.begin(), scene->placingQueue.begin() + std::min(static_cast<size_t>(speculativity()), scene->placingQueue.size()));
    for (auto& first: scene->matches)
        for (auto& second: first.second)
        {
            auto A = first.first.u, B = second.first.u;
            if (A > B)
                std::swap(A, B);
            if (!allowed.count(A) || !allowed.count(B))
                continue;
            cntr[std::make_pair(A, B)] += second.second.size();
            for (auto& f: second.second)
                if (std::get<2>(f) < b2bRansacP6RPThreshold())
                    cntrGood[std::make_pair(A, B)]++;
        }
    std::cout << "Match stats: " << std::endl;
    for (auto& p: cntr)
        std::cout << p.first.first->name << "<>" << p.first.second->name << " " << p.second << " (" << cntrGood[p.first] << ")" << std::endl;

    std::vector<std::tuple<int, CameraFixture*, CameraFixture*>> matchCount;
    for (auto&p : cntr)
        if (p.first.first != p.first.second)
            matchCount.emplace_back(p.second, p.first.first, p.first.second);

    std::sort(matchCount.begin(), matchCount.end(), std::greater<decltype(matchCount[0])>());

    corecvs::Affine3DQ tform;
    bool initialized = false;
    for (auto& init: matchCount)
    {
        CameraFixture* A, *B;
        A = std::get<1>(init);
        B = std::get<2>(init);

        std::cout << "Selecting " << A->name << " and " << B->name << " for initialization" << std::endl;
        auto psA = A, psB = B;

        // 2. Detect relative orientation (till gamma < 0.001)
        ReconstructionInitializerParams params;
        params.essentialFilterParams.b2bThreshold = b2bRansacP5RPThreshold();
        params.essentialFilterParams.inlierRadius = inlierP5RPThreshold();
        params.b2bThreshold = b2bRansacP6RPThreshold();
        params.runEssentialFiltering = runEssentialFiltering();
        params.essentialFilterParams.maxIterations = maxEssentialRansacIterations();
        params.essentialFilterParams.targetGamma = essentialTargetGamma();
        scene->filterEssentialRansac(std::vector<CameraFixture*>{psA}, std::vector<CameraFixture*>{psB}, params.essentialFilterParams);

        auto matches = scene->getFixtureMatches({psA}, psB);
        RelativeNonCentralRansacSolver::MatchContainer rm, mm;
        for (auto&t : matches)
        {
            if (std::get<4>(t) < b2bRansacP6RPThreshold())
                rm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
            mm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
        }

        if (rm.size() < minimalInlierCount())
        {
            std::cout << "Too few matches (" << rm.size() << "), rejecting " << A->name << "<>" << B->name << std::endl;
            scene->matches = scene->matchesCopy;
            continue;
        }

        psA->location.shift = corecvs::Vector3dd(0, 0, 0);
        psB->location.shift = corecvs::Vector3dd(0, 0, 0);
        psA->location.rotor = corecvs::Quaternion(0, 0, 0, 1);
        psB->location.rotor = corecvs::Quaternion(0, 0, 0, 1);
        RelativeNonCentralRansacSolverSettings s(maxP6RPIterations(), inlierP6RPThreshold(), gammaP6RP());
        RelativeNonCentralRansacSolver solver(
                psB, rm, mm, s);
        if (scene->initializationData[psA].initializationType == FixtureInitializationType::GPS && scene->initializationData[psB].initializationType == FixtureInitializationType::GPS)
        {
            solver.restrictions = RelativeNonCentralRansacSolverSettings::Restrictions::SCALE;
            solver.scale = !(scene->initializationData[psA].initData.shift - scene->initializationData[psB].initData.shift);
        }
        solver.run();
        auto best = solver.getBestHypothesis();
        std::cout << psA->name << "::" << psB->name << " " << best.shift << " " << best.rotor << std::endl;
        psB->location = best;
        std::cout << solver.getInliersCount() << " inliers" << std::endl;
        if (solver.getInliersCount() < minimalInlierCount() || solver.getGamma() > maximalFailureProbability() )
        {
            scene->matches = scene->matchesCopy;
            std::cout << "Seems that " << A->name << "<>" << B->name << " is a bad initialization pair: inliers: " << solver.getInliersCount() << " P: " << solver.getGamma() << std::endl;
            continue;
        }
        initialized = true;

        scene->placedFixtures.push_back(psA);
        scene->placedFixtures.push_back(psB);
        for (auto& cf: scene->placedFixtures)
            std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;

        std::remove(scene->placingQueue.begin(), scene->placingQueue.end(), psA);
        std::remove(scene->placingQueue.begin(), scene->placingQueue.end(), psB);
        scene->placingQueue.resize(scene->placingQueue.size() - 2);
        break;
    }
    if (!initialized)
    {
        std::cout << "FAILFAILFAILFAILFAILFAIL" << std::endl;
        std::cout << "NO INIT PAIR!!!" << std::endl;
        std::cout << "FAILFAILFAILFAILFAILFAIL" << std::endl;
    }
    CORE_ASSERT_TRUE_S(initialized);
    postAppend();
}

void corecvs::PhotostationPlacer::postAppend()
{
#if 0
    scene->matches = scene->matchesCopy;
    create2PointCloud();
    std::cout << scene->trackedFeatures.size() << " reconstructed points" << std::endl;
    fit(optimizationParams & ~(ReconstructionFunctorOptimizationType::DEGENERATE_TRANSLATIONS | (ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::TUNE_GPS)), postAppendNonlinearIterations);
    for (auto& cf: scene->placedFixtures)
        std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
    SceneAligner::TryAlign(scene, tform, scale);
    for (auto& cf: scene->placedFixtures)
        std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
    fit(optimizationParams & ~(ReconstructionFunctorOptimizationType::DEGENERATE_TRANSLATIONS | (ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::TUNE_GPS)), postAppendNonlinearIterations);
    for (auto& cf: scene->placedFixtures)
        std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
    create2PointCloud();
    std::cout << scene->trackedFeatures.size() << " reconstructed points after fit and align" << std::endl;
#endif
    std::cout << "PRE-ALIGN: " << std::endl;
    getErrorSummaryAll();
    corecvs::Affine3DQ tform;
    double scale = -1.0;
    if (!scene->is3DAligned)
    {
        SceneAligner::TryAlign(scene, tform, scale);
        std::cout << tform << scale << "<<<< tform" << std::endl;
    }
    std::cout << "POST-ALIGN: " << std::endl;
    getErrorSummaryAll();
    auto params = optimizationParams;
    if (!scene->is3DAligned)
        params = params & ~(ReconstructionFunctorOptimizationType::DEGENERATE_TRANSLATIONS| ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS);

    for (int i = 0; i < maxPostAppend(); ++i)
    {
        std::cout << "PATA: " << i << " / " << maxPostAppend() << std::endl;
        auto pcnt = scene->trackedFeatures.size();
        auto ref = scene->placedFixtures[0]->location.rotor.conjugated();
        for (auto& cf: scene->placedFixtures)
            std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ ref) << std::endl;
        std::cout << "B:    " << pcnt << " reconstructed points" << std::endl;

        appendTracks();
        std::cout << "AA:   " << pcnt << " reconstructed points" << std::endl;
        createTracks();
        std::cout << "AA&C: " << pcnt << " reconstructed points" << std::endl;
        scene->pruneTracks(inlierThreshold() * rmsePruningScaler(), inlierThreshold() * maxPruningScaler(), distanceLimit());
        auto acnt = scene->trackedFeatures.size();
        if (acnt <= pcnt && i != 0)
            break;
        fit(params, postAppendNonlinearIterations / 2);
        std::cout << "Prune" << std::endl;
        scene->pruneTracks(inlierThreshold() * rmsePruningScaler() / 2.0, inlierThreshold() * maxPruningScaler() / 2.0, distanceLimit());
        fit(params, postAppendNonlinearIterations / 2);
    }
    postAppendHook();
}

void corecvs::PhotostationPlacer::fullRun()
{
    // 0. Detect features
    scene->ProcessState->reset("Detecting", 1);
    scene->ProcessState->incrementStarted();
    scene->detectAllFeatures(featureDetectionParams());
    scene->ProcessState->incrementCompleted();
    // 1. Select multicams with most matches
    // 3. Create twopointcloud
    scene->ProcessState->reset("Initialize", 1);
    scene->ProcessState->incrementStarted();
    initialize();
    scene->ProcessState->incrementCompleted();
    /*
     * 4. Append pss iteratively
     *       a) update all P3Ps
     *            + due to speculative updating this should be rather cheap
     *            + we do not have special P2P solver for orientation w. fixed position,
     *              so P3P is the least number of required points to get a hypothesis
     *              (hence, less iterations in order to get desired confidence level)
     *       b) if no good enough P3Ps - start P6P from the best fixture
     *       c) if no good enough P6Ps - fail?!
     *       d) Possibly we need some storage for reconstructed scenes (?!) - this will
     *          be adressend in future (when we add non-iterative reconstruction)
     */
    scene->state = ReconstructionState::APPENDABLE;
    scene->ProcessState->reset("Initialize", scene->placingQueue.size());
    while (scene->placingQueue.size())
    {
        scene->ProcessState->incrementStarted();
        std::cout << "PAINTING" << std::endl;
        paintTracksOnImages(true);
        std::cout << "PAINTED" << std::endl;
        if (!append3D() && !append2D())
        {
            std::cout << "RECONSTRUCTION FAILED!!!1111" << std::endl;
            return;
        }
        postAppend();
        for (auto& cf: scene->placedFixtures)
            std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
        scene->printTrackStats();
        scene->ProcessState->incrementCompleted();
    }
    scene->ProcessState->reset("Fit 1", 1);
    scene->ProcessState->incrementStarted();
    fit(optimizationParams, finalNonLinearIterations / 2);
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("Prunging", 1);
    scene->ProcessState->incrementStarted();
    scene->pruneTracks(inlierThreshold() * rmsePruningScaler() / 2.0, inlierThreshold() * maxPruningScaler() / 2.0, distanceLimit());
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("Fit 2", 1);
    scene->ProcessState->incrementStarted();
    fit(optimizationParams, finalNonLinearIterations / 2);
    scene->ProcessState->incrementCompleted();
}

/*
 * append3D() becomes append3D()
 * append2D() becomes append2D()
 */
bool corecvs::PhotostationPlacer::append3D()
{
    CORE_ASSERT_TRUE_S(speculativity() > 0);
    scene->validateAll();
    CORE_ASSERT_TRUE_S(scene->state == ReconstructionState::TWOPOINTCLOUD ||
            scene->state == ReconstructionState::APPENDABLE);
    // Here we first update speculatively selected CameraFixtures, and then
    // add one that has the biggest count of inliers
    updateTrackables();

    size_t maxInliers = 0;
    CameraFixture *psApp;
    for (auto&cfp: activeInlierCount)
    {
        std::cout << cfp.first->name << " : " << cfp.second.size() << std::endl;
        if (cfp.second.size() > maxInliers)
        {
            maxInliers = cfp.second.size();
            psApp = cfp.first;
        }
    }

    if (maxInliers < minimalInlierCount())
        return false;

    std::cout << "Choosing to append " << psApp->name << " because it had " << maxInliers << " inliers" << std::endl;
    for (auto ptr: scene->placedFixtures)
        std::cout << ptr->name << " " << ptr->location.shift << " " << ptr->location.rotor << std::endl;
    std::cout << "Placing #" << psApp->name << std::endl;
    L_ERROR << "Placing " << psApp->name ;
    L_ERROR << "Computing tracks" ;
    auto hypos = scene->getPossibleTracks(psApp);
    std::cout << "Total " << hypos.size() << " possible tracks" << std::endl;
    L_ERROR << "Computing P3P" ;

    switch(scene->initializationData[psApp].initializationType)
    {
        default:
        case FixtureInitializationType::GPS:
        {
            auto hypo = activeEstimates[psApp];
            psApp->location.rotor = hypo.rotor;
            std::cout << "!!!!" << hypo.rotor << "!!!!" << std::endl;
            psApp->location.shift = !scene->is3DAligned ? hypo.shift : scene->initializationData[psApp].initData.shift;
        }
        break;
        case FixtureInitializationType::STATIC:
        {
#if 0
            std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> foo;
            auto& initPts = scene->initializationData[psApp].staticPoints;
            for (auto& ptr: initPts)
            {
                for (auto& obs: ptr->observations__)
                {
                    if (obs.first.u != psApp)
                        continue;
                    auto cam = obs.first.v;
                    auto proj= obs.second.observation;
                    auto ptw = ptr->position;
                    foo.emplace_back(cam, proj, ptw, ptr, -1);
                }
            }
            solver.reprojectionInlierThreshold = 15.0;
            solver.cloudMatches = foo;
//;;            for (int ii = 0; ii < foo.size(); ++ii)
//                solver.inliers.push_back(ii);
//            solver.runInliersPNP();
            solver.run();
            solver.runInliersRE();
            auto hypo = solver.getBestHypothesis();
            psApp->location = hypo;
            solver.cloudMatches = hypos;
//            solver.inliers = solver.selectInliers(hypo);

            std::cout << "!!!!" << hypo << "!!!!" << std::endl;
            for (auto& ptr: initPts)
            {
                scene->staticPoints.push_back(ptr);
                ptr->reprojectedPosition = ptr->position;
            }
#endif
        }
        break;
    }
#if 0
    std::cout << "TRACKS BEFORE: " << scene->trackedFeatures.size() << std::endl;
    if (scene->state == ReconstructionState::APPENDABLE)
    {
        L_ERROR << "Appending tracks" ;
        appendTracks(activeInlierCount[psApp], psApp, hypos);
    }
    if (scene->state == ReconstructionState::TWOPOINTCLOUD)
    {
        scene->trackedFeatures.clear();
        scene->trackMap.clear();
        for (auto& ptr: scene->trackedFeatures)
            scene->deleteFeaturePoint(ptr);
        scene->state = ReconstructionState::APPENDABLE;
    }
#endif
#if 0
    tryAlign();
#endif
#if 0
    L_ERROR << "Building tracks" ;
    for (size_t aId = 0; aId < scene->placedFixtures.size(); ++aId)
    {
        for (size_t bId = aId + 1; bId < scene->placedFixtures.size(); ++bId)
        {
            scene->buildTracks(psApp, scene->placedFixtures[aId], scene->placedFixtures[bId], trackInlierThreshold, distanceLimit);
        }
    }
    std::cout << "TRACKS AFTER: " << scene->trackedFeatures.size() << std::endl;
#endif
    scene->placedFixtures.push_back(psApp);
    scene->placingQueue.resize(std::remove(scene->placingQueue.begin(), scene->placingQueue.end(), psApp) - scene->placingQueue.begin());
    scene->validateAll();
    return true;
}


void corecvs::PhotostationPlacer::detectAll()
{
    scene->validateAll();
    scene->detectAllFeatures(FeatureDetectionParams());
    switch(scene->state)
    {
        // It's Ok
        case ReconstructionState::NONE:
        case ReconstructionState::MATCHED:
            scene->state = ReconstructionState::MATCHED;
            break;
        // Someone is playing around, so left state untouched
        case ReconstructionState::INITALIZED:
        case ReconstructionState::TWOPOINTCLOUD:
        case ReconstructionState::APPENDABLE:
        case ReconstructionState::FINISHED:
            break;
    }
    scene->validateAll();
}

corecvs::Mesh3D corecvs::PhotostationPlacer::dumpMesh(const std::string &filename)
{
    corecvs::Mesh3D meshres;
    meshres.switchColor(true);
    CalibrationHelpers().drawScene(meshres, *scene);
    meshres.dumpPLY(filename);
    return meshres;
}
