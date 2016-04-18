#include "photostationPlacer.h"

#include <unordered_map>
#include <random>
#include <sstream>
#include <tuple>
#include <set>

#include "featureMatchingPipeline.h"
#include "essentialEstimator.h"
#include "essentialFeatureFilter.h"
#include "relativeNonCentralRansacSolver.h"
#include "absoluteNonCentralRansacSolver.h"
#include "multicameraTriangulator.h"
#include "pnpSolver.h"
#include "calibrationHelpers.h"
#include "calibrationLocation.h"
#include "reconstructionInitializer.h"
#include "statusTracker.h"
#include "sceneAligner.h"
#include "log.h"


#ifdef WITH_TBB
#include <tbb/task_group.h>
#endif

std::string toString(PhotostationPlacerOptimizationErrorType type)
{
    switch(type)
    {
        case PhotostationPlacerOptimizationErrorType::REPROJECTION:
            return "REPROJECTION";
        case PhotostationPlacerOptimizationErrorType::ANGULAR:
            return "ANGULAR";
        case PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT:
            return "CROSS-PRODUCT";
        case PhotostationPlacerOptimizationErrorType::RAY_DIFF:
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
        images.push_back(std::make_pair(p.first, p.second));
    }
    corecvs::parallelable_for(0, (int)images.size(), ParallelTrackPainter(images, scene, colorizer, pairs));
}

int corecvs::PhotostationPlacer::getMovablePointCount()
{
    // TODO: clarify which points are inmovable
    //       Immovable are scene->staticPoints
    return (int)scene->trackedFeatures.size();
}

void corecvs::PhotostationPlacer::tryAlign()
{
#if 0
    L_INFO << "Trying to align";
    if (scene->is3DAligned)
    {
        L_INFO << "Already aligned";
        return;
    }
    CameraFixture* gps[3], *idFixed = 0, *idStatic = 0;
    int cntGps = 0;

    for (auto& ptr: scene->placedFixtures)
    {
        switch(scene->initializationData[ptr].initializationType)
        {
            default:
            case PhotostationInitializationType::GPS:
                gps[cntGps++] = ptr;
                break;
            case PhotostationInitializationType::FIXED:
                idFixed = ptr;
                break;
            case PhotostationInitializationType::STATIC:
                idStatic = ptr;
                break;
        }
    }

    if (!idFixed && !idStatic && !cntGps)
    {
        L_INFO << "NO ALIGN DATA";
        return;
    }
//    fit(PhotostationPlacerOptimizationType::NON_DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS, 200);

    L_INFO << "ALIGNING:";
    getErrorSummaryAll();

    corecvs::Affine3DQ transform;
    if (idFixed || idStatic)
    {
        auto ptr = idFixed ? idFixed : idStatic;
        auto qe = scene->initializationData[ptr].initData;
        auto qo = ptr->location;
        transform.rotor = qe.rotor ^ qo.rotor.conjugated();
        transform.shift = -transform.rotor * qo.shift + qe.shift;
    }
    else
    {
        if (cntGps == 1)
        {
            L_INFO << "ALIGNING SINGLE GPS";
            Vector3dd shift = scene->initializationData[gps[0]].initData.shift - gps[0]->location.shift;
            gps[0]->location.shift += shift;
            for (auto tp: scene->trackedFeatures)
                tp->reprojectedPosition += shift;
            return;
        }
        if (cntGps == 2)
        {
            L_INFO << "ALIGNING 2xGPS";
            CORE_ASSERT_TRUE_S(!(gps[0]->location.shift - scene->initializationData[gps[0]].initData.shift) < 1e-6);
            auto o1 = gps[1]->location.shift - gps[0]->location.shift;
            auto o2 = Vector3dd(0, 0, 1);
            auto e1 = scene->initializationData[gps[1]].initData.shift - gps[0]->location.shift;
            auto e2 = Vector3dd(0, 0, 1);
            auto q = TransformFrom2RayCorrespondence(o1, o2, e1, e2);
            gps[0]->location.rotor = q.conjugated();
            gps[1]->location.rotor = gps[1]->location.rotor ^ q.conjugated();
            gps[1]->location.shift = scene->initializationData[gps[1]].initData.shift;
            return;
        }
        corecvs::Affine3DQ qe;
        qe.rotor = detectOrientationFirst(gps[0], gps[1], gps[2]);
        qe.shift = scene->initializationData[gps[0]].initData.shift;
        auto qo = gps[0]->location;
        transform.rotor = qe.rotor ^ qo.rotor.conjugated();
        transform.shift = -transform.rotor * qo.shift + qe.shift;
    }
    for (auto& ptr: scene->placedFixtures)
    {
        switch(scene->initializationData[ptr].initializationType)
        {
            case PhotostationInitializationType::NONE:
                {
                    auto qo = ptr->location;
                    ptr->location.rotor = transform.rotor ^ qo.rotor;
                    ptr->location.shift = (transform.rotor * qo.shift) + transform.shift;
                }
                break;
            case PhotostationInitializationType::GPS:
                {
                    auto qo = ptr->location;
                    ptr->location.rotor = transform.rotor ^ qo.rotor;
                    ptr->location.shift = scene->initializationData[ptr].initData.shift;
                }
                break;
            case PhotostationInitializationType::STATIC:
            case PhotostationInitializationType::FIXED:
                ptr->location = scene->initializationData[ptr].initData;
                break;
        }
    }
    for (auto& ptr: scene->trackedFeatures)
    {
        ptr->reprojectedPosition = (transform.rotor * ptr->reprojectedPosition) + transform.shift;
    }
    scene->is3DAligned = true;
//    fit();

    L_INFO << "POST-ALIGN:";
    getErrorSummaryAll();
#endif
}

void corecvs::PhotostationPlacer::addFirstPs()
{
    CORE_ASSERT_TRUE_S(scene->placedFixtures.size() == 0);
    auto ps = scene->placingQueue[0];
    auto init = scene->initializationData[ps];
    ps->location.shift = corecvs::Vector3dd(0.0, 0.0, 0.0);
    ps->location.rotor = corecvs::Quaternion(0.0, 0.0, 0.0, 1.0);
    switch (init.initializationType)
    {
        case PhotostationInitializationType::GPS:
//          ps->location.shift = init.initData.shift;
            break;
        case PhotostationInitializationType::NONE:
            break;
        case PhotostationInitializationType::STATIC:
            scene->initializationData[ps].initData = staticInit(ps, init.staticPoints);
            break;
        case PhotostationInitializationType::FIXED:
//            ps->location = init.initData;
            break;
    }
    scene->placedFixtures.push_back(ps);
    scene->placingQueue.erase(scene->placingQueue.begin());
    tryAlign();
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
        double inlierThreshold = 5.0;
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

                if ((!(ps->project(ptw, cam) - pt)) < inlierThreshold)
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

void corecvs::PhotostationPlacer::addSecondPs()
{
#if 0
    // Detect orientation
    // Align (2xGPS, GPS+STATIC, GPS+FIXED)
    // Create 2-point cloud
    // Try align
    CORE_ASSERT_TRUE_S(scene->placedFixtures.size() == 1);
    auto ps = scene->placingQueue[0];
    scene->placedFixtures.push_back(ps);
    scene->placingQueue.erase(scene->placingQueue.begin());
    std::vector<CameraFixture*> pps = scene->placedFixtures;
    EssentialFilterParams params;
    params.b2bThreshold = b2bRansacP5RPThreshold;
    params.inlierRadius = inlierP5RPThreshold;
    scene->filterEssentialRansac(pps, params);
    estimatePair(pps[0], pps[1]);
    scene->matches = scene->matchesCopy;
    create2PointCloud();
    tryAlign();
    scene->state = ReconstructionState::TWOPOINTCLOUD;
#endif
}

void corecvs::PhotostationPlacer::create2PointCloud()
{
    CORE_ASSERT_TRUE_S(scene->placedFixtures.size() == 2);
    auto psA = scene->placedFixtures[0];
    auto psB = scene->placedFixtures[1];

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
        if (fscore > trackInlierThreshold)
            continue;

        corecvs::MulticameraTriangulator mct;
        mct.addCamera(psA->getMMatrix(camA), kpA);
        mct.addCamera(psB->getMMatrix(camB), kpB);
        auto res = mct.triangulateLM(mct.triangulate());

        bool isVisibleInlierNotTooFar = true;
        isVisibleInlierNotTooFar &= psA->isVisible(res, camA);
        isVisibleInlierNotTooFar &= (!(kpA - psA->project(res, camA))) < trackInlierThreshold;
        isVisibleInlierNotTooFar &= (!(res - psA->getWorldCamera(camA).extrinsics.position)) < distanceLimit;
        isVisibleInlierNotTooFar &= psB->isVisible(res, camB);
        isVisibleInlierNotTooFar &= (!(kpB - psB->project(res, camB))) < trackInlierThreshold;
        isVisibleInlierNotTooFar &= (!(res - psB->getWorldCamera(camB).extrinsics.position)) < distanceLimit;

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

void corecvs::PhotostationPlacer::fit(bool tuneFocal)
{
    fit(tuneFocal ? PhotostationPlacerOptimizationType::FOCALS | optimizationParams : optimizationParams);
}

void corecvs::PhotostationPlacer::getErrorSummaryAll()
{
    getErrorSummary(PhotostationPlacerOptimizationErrorType::REPROJECTION);
    getErrorSummary(PhotostationPlacerOptimizationErrorType::ANGULAR);
    getErrorSummary(PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT);
    getErrorSummary(PhotostationPlacerOptimizationErrorType::RAY_DIFF);
}

void corecvs::PhotostationPlacer::getErrorSummary(PhotostationPlacerOptimizationErrorType errorType)
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
            switch (errorType)
            {
                case PhotostationPlacerOptimizationErrorType::REPROJECTION:
                    errP = p.cameraFixture->reprojectionError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += (!errP) * (!errP);
                    break;
                case PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT:
                    errV = p.cameraFixture->crossProductError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += !errV * !errV;
                    break;
                case PhotostationPlacerOptimizationErrorType::ANGULAR:
                    a = p.cameraFixture->angleError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += a * a;
                    break;
                case PhotostationPlacerOptimizationErrorType::RAY_DIFF:
                    errV = p.cameraFixture->rayDiffError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += !errV * !errV;
                    break;
            }
        }

    std::cout << toString(errorType) << "RMSE [features]: " << std::sqrt(totalsqr / totalcnt) << std::endl;
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
                case PhotostationPlacerOptimizationErrorType::REPROJECTION:
                    errP = p.cameraFixture->reprojectionError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += (!errP) * (!errP);
                    break;
                case PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT:
                    errV = p.cameraFixture->crossProductError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += !errV * !errV;
                    break;
                case PhotostationPlacerOptimizationErrorType::ANGULAR:
                    a = p.cameraFixture->angleError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += a * a;
                    break;
                case PhotostationPlacerOptimizationErrorType::RAY_DIFF:
                    errV = p.cameraFixture->rayDiffError(t->reprojectedPosition, p.observation, p.camera);
                    totalsqr += !errV * !errV;
                    break;
            }
        }

    std::cout << toString(errorType) << "RMSE [static points]: " << std::sqrt(totalsqr / totalcnt) << std::endl;
}

void corecvs::PhotostationPlacer::fit(const PhotostationPlacerOptimizationType &params, int num)
{
    if (scene->placedFixtures.size() < 2)
        return;
    optimizationParams = params;
    getErrorSummaryAll();
    corecvs::LevenbergMarquardtSparse lm;
    ReconstructionFunctor orient(scene, errorType, optimizationParams, 1.0);
    ReconstructionNormalizationFunctor orientNorm(&orient);
    lm.f = &orient;
    lm.normalisation = &orientNorm;
    lm.maxIterations = num;
    lm.trace = false;
    std::vector<double> input(orient.getInputNum());
    std::vector<double> out(orient.getOutputNum());
    if (orient.getOutputNum() <=  orient.getInputNum())
        return;
    lm.useConjugatedGradient = false;
    lm.conjugatedGradientIterations = std::max(100, (int)( 0.001 * input.size()));
    orient.writeParams(&input[0]);
    auto res = lm.fit(input, out);
    orient.readParams(&res[0]);

    getErrorSummaryAll();
    static int cnt = 0;
    cnt++;

    std::map<int, int> cntr;
    for (auto& f: scene->trackedFeatures)
        cntr[(int)f->observations__.size()]++;

    std::cout << "Track sizes: " << cntr.size() << std::endl;
    for (auto p : cntr) {
        std::cout << p.first << "\t" << p.second << std::endl;
    }
    scene->validateAll();
}

void corecvs::PhotostationPlacer::appendTracks(const std::vector<int> &inlierIds, CameraFixture* fixture, const std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> &possibleTracks)
{
    scene->validateAll();
    //int inlierIdx = 0;
    //int totalIdx = 0;
    //int appended = 0;

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
        if ((!fixture->reprojectionError(track->reprojectedPosition, proj, cam)) > trackInlierThreshold ||
            (!(track->reprojectedPosition - fixture->getWorldCamera(cam).extrinsics.position)) > distanceLimit)
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
    for (size_t i = 0; i < speculativity && i < scene->placingQueue.size(); ++i)
    {
        auto cf = scene->placingQueue[i];
        std::cout << "\tRunning with " << cf->name << " ";
        if (activeEstimates.count(cf))
            std::cout << "already have estimate: " << activeEstimates[cf].shift << activeEstimates[cf].rotor;
        std::cout << std::endl;

        auto hypos = scene->getPossibleTracks(cf);
        std::cout << "PRE-CTR" << std::endl;
        corecvs::AbsoluteNonCentralRansacSolver solver = activeEstimates.count(cf) ?
            corecvs::AbsoluteNonCentralRansacSolver(cf, hypos, activeEstimates[cf]) :
            corecvs::AbsoluteNonCentralRansacSolver(cf, hypos);
        std::cout << "POST-CTR" << std::endl;

        solver.forcePosition = scene->initializationData[cf].initializationType == PhotostationInitializationType::GPS && scene->is3DAligned;
        solver.forcedPosition = scene->initializationData[cf].initData.shift;

        solver.run();
        solver.runInliersRE();
        activeEstimates[cf] = solver.getBestHypothesis();
        if (solver.forcePosition)
            activeEstimates[cf].shift = scene->initializationData[cf].initData.shift;
        activeInlierCount[cf] = solver.getInliers();
        std::cout << "Final estimate: " << activeEstimates[cf].shift << activeEstimates[cf].rotor << std::endl;

        std::cout << "\tFinished with " << cf->name << std::endl;
    }
}

bool corecvs::PhotostationPlacer::appendP6P()
{
    // 1. Center scene
    // 2. Get all 2d<->2d correspondeces with already aligned fixtures
    // 3. Run solver
    // 4. Return true if solution meets inlier threshold and gives enough confidence
    corecvs::Affine3DQ tform;
    double scale = 1.0;

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

    std::vector<std::pair<corecvs::CameraFixture*, decltype(scene->getPhotostationMatches({}, 0))>> matches;
    for (auto& fixture: scene->placingQueue)
        matches.emplace_back(fixture, scene->getPhotostationMatches(scene->placedFixtures, fixture));

    std::sort(matches.begin(), matches.end(), [](const decltype(matches)::value_type &a, const decltype(matches)::value_type &b) { return a.second.size() > b.second.size(); });

    for (auto& p: matches)
    {
        ReconstructionInitializerParams params;
        params.essentialFilterParams.b2bThreshold = b2bRansacP5RPThreshold;
        params.essentialFilterParams.inlierRadius = inlierP5RPThreshold;
        params.b2bThreshold = b2bRansacP6RPThreshold;
        params.runEssentialFiltering = runEssentialFiltering;
        params.essentialFilterParams.maxIterations = maxEssentialRansacIterations;
        params.essentialFilterParams.targetGamma = essentialTargetGamma;
        scene->filterEssentialRansac(scene->placedFixtures, {p.first}, params.essentialFilterParams);
        auto psB = p.first;
        auto B = psB;

        auto matches = scene->getPhotostationMatches(scene->placedFixtures, psB);
        RelativeNonCentralRansacSolver::MatchContainer rm, mm;
        for (auto&t : matches)
        {
            if (std::get<4>(t) < b2bRansacP6RPThreshold)
                rm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
            mm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
        }

        if (rm.size() < minimalInlierCount)
        {
            std::cout << "Too few matches (" << rm.size() << "), rejecting ";
            for (auto &A: scene->placedFixtures)
                std::cout << A->name;
            std::cout << " <> " << B->name << std::endl;
            scene->matches = scene->matchesCopy;
            continue;
        }

        psB->location.shift = corecvs::Vector3dd(0, 0, 0);
        psB->location.rotor = corecvs::Quaternion(0, 0, 0, 1);
        RelativeNonCentralRansacSolver solver(
        //        psA,
                psB, rm, mm);
        if (scene->initializationData[psB].initializationType == PhotostationInitializationType::GPS)
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
        solver.run();
        auto best = solver.getBestHypothesis();
        for (auto &A: scene->placedFixtures)
            std::cout << A->name;
        std::cout << "::" << psB->name << " " << best.shift << " " << best.rotor << std::endl;
        psB->location = best;
        std::cout << solver.getInliersCount() << " inliers" << std::endl;
        if (solver.getInliersCount() < minimalInlierCount || solver.getGamma() > maximalFailureProbability )
        {
            scene->matches = scene->matchesCopy;
            std::cout << "Seems that ";
            for (auto &A: scene->placedFixtures)
                std::cout << A->name;
            std::cout << "<>" << B->name << " does not match in P6P sense: inliers: " << solver.getInliersCount() << " P: " << solver.getGamma() << std::endl;
            continue;
        }

        scene->placedFixtures.push_back(psB);
        for (auto& cf: scene->placedFixtures)
            std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;

        std::remove(scene->placingQueue.begin(), scene->placingQueue.end(), psB);
        scene->placingQueue.resize(scene->placingQueue.size() - 1);
        if (!scene->is3DAligned)
        {
           SceneAligner::TryAlign(scene, tform, scale);
           std::cout << tform << scale << "<<<< tform" << std::endl;
        }
        else
        {
            tform.shift = mean;
            scene->transform(tform);
        }
        for (auto& cf: scene->placedFixtures)
            std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
        scene->matches = scene->matchesCopy;
        return true;
    }
    return false;
}

bool corecvs::PhotostationPlacer::appendP3P(CameraFixture *f)
{
    // 1. Center scene
    // 2. Get all 3d<->2d correspondeces with already constructed pointcloud
    // 3. Run solver
    // 4. Return true if solution meets inlier threshold and gives enough confidence
    CORE_ASSERT_TRUE_S(false && f);
    return false;
}


void corecvs::PhotostationPlacer::testNewPipeline()
{
    // 0. Detect features
    scene->detectAllFeatures(FeatureDetectionParams());
    // 1. Select multicams with most matches
    std::unordered_map<std::pair<CameraFixture*, CameraFixture*>, int> cntr, cntrGood;
    for (auto& first: scene->matches)
        for (auto& second: first.second)
        {
            auto A = first.first.u, B = second.first.u;
            if (A > B)
                std::swap(A, B);
            cntr[std::make_pair(A, B)] += (int)second.second.size();
            for (auto& f: second.second)
                if (std::get<2>(f) < b2bRansacP6RPThreshold)
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
    double scale = 1.0;
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
        params.essentialFilterParams.b2bThreshold = b2bRansacP5RPThreshold;
        params.essentialFilterParams.inlierRadius = inlierP5RPThreshold;
        params.b2bThreshold = b2bRansacP6RPThreshold;
        params.runEssentialFiltering = runEssentialFiltering;
        params.essentialFilterParams.maxIterations = maxEssentialRansacIterations;
        params.essentialFilterParams.targetGamma = essentialTargetGamma;
        scene->filterEssentialRansac(std::vector<CameraFixture*>{psA}, std::vector<CameraFixture*>{psB}, params.essentialFilterParams);

        auto matches = scene->getPhotostationMatches({psA}, psB);
        RelativeNonCentralRansacSolver::MatchContainer rm, mm;
        for (auto&t : matches)
        {
            if (std::get<4>(t) < b2bRansacP6RPThreshold)
                rm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
            mm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
        }

        if (rm.size() < minimalInlierCount)
        {
            std::cout << "Too few matches (" << rm.size() << "), rejecting " << A->name << "<>" << B->name << std::endl;
            scene->matches = scene->matchesCopy;
            continue;
        }

        psA->location.shift = corecvs::Vector3dd(0, 0, 0);
        psB->location.shift = corecvs::Vector3dd(0, 0, 0);
        psA->location.rotor = corecvs::Quaternion(0, 0, 0, 1);
        psB->location.rotor = corecvs::Quaternion(0, 0, 0, 1);
        RelativeNonCentralRansacSolver solver(
        //        psA,
                psB, rm, mm);
        if (scene->initializationData[psA].initializationType == PhotostationInitializationType::GPS && scene->initializationData[psB].initializationType == PhotostationInitializationType::GPS)
        {
            solver.restrictions = RelativeNonCentralRansacSolverSettings::Restrictions::SCALE;
            solver.scale = !(scene->initializationData[psA].initData.shift - scene->initializationData[psB].initData.shift);
        }
        solver.run();
        auto best = solver.getBestHypothesis();
        std::cout << psA->name << "::" << psB->name << " " << best.shift << " " << best.rotor << std::endl;
        psB->location = best;
        std::cout << solver.getInliersCount() << " inliers" << std::endl;
        if (solver.getInliersCount() < minimalInlierCount || solver.getGamma() > maximalFailureProbability )
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
        SceneAligner::TryAlign(scene, tform, scale);
        std::cout << tform << scale << "<<<< tform" << std::endl;

        for (auto& cf: scene->placedFixtures)
            std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
        break;
    }
    if (!initialized)
    {
        std::cout << "FAILFAILFAILFAILFAILFAIL" << std::endl;
        std::cout << "NO INIT PAIR!!!" << std::endl;
        std::cout << "FAILFAILFAILFAILFAILFAIL" << std::endl;
    }
    CORE_ASSERT_TRUE_S(initialized);

    // 3. Create twopointcloud
    scene->matches = scene->matchesCopy;
    create2PointCloud();
    std::cout << scene->trackedFeatures.size() << " reconstructed points" << std::endl;
    fit(optimizationParams & ~(PhotostationPlacerOptimizationType::DEGENERATE_TRANSLATIONS | (PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::TUNE_GPS)), 100);
    for (auto& cf: scene->placedFixtures)
        std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
    SceneAligner::TryAlign(scene, tform, scale);
    for (auto& cf: scene->placedFixtures)
        std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
    fit(optimizationParams & ~(PhotostationPlacerOptimizationType::DEGENERATE_TRANSLATIONS | (PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::TUNE_GPS)), 100);
    for (auto& cf: scene->placedFixtures)
        std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
    create2PointCloud();
    std::cout << scene->trackedFeatures.size() << " reconstructed points after fit and align" << std::endl;
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
    while (scene->placingQueue.size())
    {
        if (!appendPs() && !appendP6P())
        {
            std::cout << "RECONSTRUCTION FAILED!!!1111" << std::endl;
            CORE_ASSERT_TRUE_S(false);
        }
        create2PointCloud();
        for (size_t aId = 0; aId + 1 < scene->placedFixtures.size(); ++aId)
        {
            for (size_t bId = aId + 1; bId + 1 < scene->placedFixtures.size(); ++bId)
            {
                scene->buildTracks(*scene->placedFixtures.rbegin(), scene->placedFixtures[aId], scene->placedFixtures[bId], trackInlierThreshold, distanceLimit);
            }
        }
        if (!scene->is3DAligned)
        {
            corecvs::Affine3DQ transform;
            double scale;
            SceneAligner::TryAlign(scene, transform, scale);
            fit(optimizationParams & ~(PhotostationPlacerOptimizationType::DEGENERATE_TRANSLATIONS | (PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::TUNE_GPS)), 100);
        }
        else
        {
            fit(optimizationParams, 100);
        }
        for (auto& cf: scene->placedFixtures)
            std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
    }
}

bool corecvs::PhotostationPlacer::appendPs()
{
    CORE_ASSERT_TRUE_S(speculativity > 0);
    scene->validateAll();
    if (scene->state == ReconstructionState::MATCHED)
    {
        CORE_ASSERT_TRUE_S(scene->placedFixtures.size() < 2);
        if (scene->placedFixtures.size())
            addSecondPs();
        else
            addFirstPs();
        return true;
    }
    CORE_ASSERT_TRUE_S(scene->state == ReconstructionState::TWOPOINTCLOUD ||
                       scene->state == ReconstructionState::APPENDABLE);
    // Here we first update speculatively selected CameraFixtures, and then
    // add one that has the biggest count of inliers
    updateTrackables();

    size_t maxInliers = 0;
    CameraFixture *psApp = NULL;
    for (auto & cfp : activeInlierCount)
    {
        std::cout << cfp.first->name << " : " << cfp.second.size() << std::endl;
        if (cfp.second.size() > maxInliers)
        {
            maxInliers = cfp.second.size();
            psApp = cfp.first;
        }
    }

    if (maxInliers < minimalInlierCount)
        return false;

    if (psApp == NULL) {
        L_ERROR << "Nothing to append!";
        return false;
    }

    std::cout << "Choosing to append " << psApp->name << " because it had " << maxInliers << " inliers" << std::endl;

    for (auto ptr : scene->placedFixtures)
        std::cout << ptr->name << " " << ptr->location.shift << " " << ptr->location.rotor << std::endl;
//    CameraFixture* psApp = scene->placingQueue[0];
    std::cout << "Placing #" << psApp->name << std::endl;
    L_INFO << "Placing " << psApp->name;
    L_INFO << "Computing tracks";
    auto hypos = scene->getPossibleTracks(psApp);
    std::cout << "Total " << hypos.size() << " possible tracks" << std::endl;
    L_INFO << "Computing P3P";

 // corecvs::AbsoluteNonCentralRansacSolverParams params;
//  AbsoluteNonCentralRansacSolver solver(psApp, hypos, params);
    switch (scene->initializationData[psApp].initializationType)
    {
        default:
        case PhotostationInitializationType::GPS:
        {
            auto hypo = activeEstimates[psApp];
            psApp->location.rotor = hypo.rotor;
            std::cout << "!!!!" << hypo.rotor << "!!!!" << std::endl;
            psApp->location.shift = !scene->is3DAligned ? hypo.shift : scene->initializationData[psApp].initData.shift;
        }
        break;
        case PhotostationInitializationType::STATIC:
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
    std::cout << "TRACKS BEFORE: " << scene->trackedFeatures.size() << std::endl;
    if (scene->state == ReconstructionState::APPENDABLE)
    {
        L_INFO << "Appending tracks";
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

    tryAlign();

    L_INFO << "Building tracks";
    for (size_t aId = 0; aId < scene->placedFixtures.size(); ++aId)
    {
        for (size_t bId = aId + 1; bId < scene->placedFixtures.size(); ++bId)
        {
            scene->buildTracks(psApp, scene->placedFixtures[aId], scene->placedFixtures[bId], trackInlierThreshold, distanceLimit);
        }
    }
    std::cout << "TRACKS AFTER: " << scene->trackedFeatures.size() << std::endl;
    scene->placedFixtures.push_back(psApp);
    scene->placingQueue.resize(std::remove(scene->placingQueue.begin(), scene->placingQueue.end(), psApp) - scene->placingQueue.begin());
    scene->validateAll();
    return true;
}

bool corecvs::PhotostationPlacer::initialize()
{
    ReconstructionInitializerParams params;
    params.essentialFilterParams.b2bThreshold = b2bRansacP5RPThreshold;
    params.essentialFilterParams.inlierRadius = inlierP5RPThreshold;
    params.b2bThreshold = b2bRansacP6RPThreshold;
    params.runEssentialFiltering = runEssentialFiltering;
    params.essentialFilterParams.maxIterations = maxEssentialRansacIterations;
    params.essentialFilterParams.targetGamma = essentialTargetGamma;

    ReconstructionInitializer initializer;
    (ReconstructionInitializerParams&)initializer = params;
    initializer.scene = scene;
    bool initOk = initializer.initialize();
    if (!initOk)
        return false;

    L_INFO << "Building tracks";
    scene->buildTracks(scene->placingQueue[0], scene->placingQueue[1], scene->placingQueue[2], trackInlierThreshold, distanceLimit);
    for (int i = 0; i < 3; ++i)
    {
        scene->placedFixtures.push_back(scene->placingQueue[0]);
        scene->placingQueue.erase(scene->placingQueue.begin());
    }
    scene->is3DAligned = true;
    scene->state = ReconstructionState::APPENDABLE;
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

void corecvs::PhotostationPlacer::fullRun()
{
    L_INFO << "Starting full run";

    scene->ProcessState->reset("Detecting", 1);
    scene->ProcessState->incrementStarted();
    {
        L_INFO << "Detecting features";
        detectAll();
    }
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("Initializing", 1);
    scene->ProcessState->incrementStarted();
    {
        L_INFO << "Initalizing";
        initialize();
    }
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("First fitting", 1);
    scene->ProcessState->incrementStarted();
    {
        L_INFO << "Fitting";
        fit(optimizationParams, 100);
    }
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("Appending", 1);
    scene->ProcessState->incrementStarted();
    {
        L_INFO << "Appending";

        while (scene->placingQueue.size())
        {
            L_INFO << "Appending" << (*scene->placingQueue.begin())->name;
            if (!appendPs())
                break;

            L_INFO << "Fitting";
            //      if (scene->is3DAligned)
            //          fit(PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS, 100);
            //    else
            //   fit(PhotostationPlacerOptimizationType::NON_DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS, 200);
            fit(optimizationParams, 100);

            std::stringstream ss;
            ss << (*scene->placedFixtures.rbegin())->name << "_app.ply";
            dumpMesh(ss.str());
        }
    }
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("Final fitting", 1);
    scene->ProcessState->incrementStarted();
    {
        L_INFO << "Fitting final";
        fit(optimizationParams, 10000);
    }
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("Dumping mesh", 1);
    scene->ProcessState->incrementStarted();
    {
        dumpMesh("final.ply");
    }
    scene->ProcessState->incrementCompleted();
}

corecvs::Mesh3D corecvs::PhotostationPlacer::dumpMesh(const std::string &filename)
{
    corecvs::Mesh3D meshres;
    meshres.switchColor(true);
    CalibrationHelpers().drawScene(meshres, *scene);
    meshres.dumpPLY(filename);
    return meshres;
}
