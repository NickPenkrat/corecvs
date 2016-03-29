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
#include "sceneAligner.h"
#include "log.h"


#ifdef WITH_TBB
#include <tbb/task_group.h>
#endif
#define IFNOT(cond, expr) \
    if (!(optimizationParams & PhotostationPlacerOptimizationType::cond)) \
    { \
        expr; \
    }
#define IF(cond, expr) \
    if (!!(optimizationParams & PhotostationPlacerOptimizationType::cond)) \
    { \
        expr; \
    }
#define GETPARAM(ref) \
    ref = in[argin++];
#define IF_GETPARAM(cond, ref) \
    if (!!(optimizationParams & PhotostationPlacerOptimizationType::cond)) ref = in[argin++];
#define IFNOT_GETPARAM(cond, ref) \
    if ( !(optimizationParams & PhotostationPlacerOptimizationType::cond)) ref = in[argin++];
#define SETPARAM(ref) \
    out[argout++] = ref;
#define IF_SETPARAM(cond, ref) \
    if (!!(optimizationParams & PhotostationPlacerOptimizationType::cond)) out[argout++] = ref;
#define IFNOT_SETPARAM(cond, ref) \
    if ( !(optimizationParams & PhotostationPlacerOptimizationType::cond)) out[argout++] = ref;

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

int corecvs::PhotostationPlacer::getErrorComponentsPerPoint()
{
    switch(errorType)
    {
        case PhotostationPlacerOptimizationErrorType::ANGULAR:
            return 1;
        case PhotostationPlacerOptimizationErrorType::REPROJECTION:
            return 2;
        case PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT:
            return 3;
        case PhotostationPlacerOptimizationErrorType::RAY_DIFF:
            return 3;
    }
    CORE_ASSERT_TRUE_S(false);
}

int corecvs::PhotostationPlacer::getReprojectionCnt()
{
    int total = 0;
    for (auto& o: scene->trackedFeatures)
        total += (int)o->observations__.size();
    for (auto& p: scene->staticPoints)
    {
        int totalPlaced = 0;
        for (auto& o: p->observations__)
        {
            bool isPlaced = false;
            for (auto& f: scene->placedFixtures)
                if (f == o.first.u)
                {
                    isPlaced = true;
                    break;
                }
            if (!isPlaced)
                continue;
            totalPlaced++;
        }
        total += totalPlaced;
    }
    int tot = total * getErrorComponentsPerPoint();
    std::cout << "REPCNT: " << tot << std::endl;

    return tot;
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
    L_ERROR << "Trying to align";
    if (scene->is3DAligned)
    {
        L_ERROR << "Already aligned";
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
        L_ERROR << "NO ALIGN DATA";
        return;
    }
//    fit(PhotostationPlacerOptimizationType::NON_DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS, 200);

    L_ERROR << "ALIGNING:";
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
            L_ERROR << "ALIGNING SINGLE GPS";
            Vector3dd shift = scene->initializationData[gps[0]].initData.shift - gps[0]->location.shift;
            gps[0]->location.shift += shift;
            for (auto tp: scene->trackedFeatures)
                tp->reprojectedPosition += shift;
            return;
        }
        if (cntGps == 2)
        {
            L_ERROR << "ALIGNING 2xGPS";
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

    L_ERROR << "POST-ALIGN:";
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
            directions.push_back(ps->getWorldCamera(cam).rayFromPixel(pt).a);
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

void corecvs::PhotostationPlacer::prepareNonLinearOptimizationData()
{
    sparsity.clear();
    activeCameras.clear();
    revDependency.clear();
    gpsConstrainedCameras.clear();
    inputNum = outputNum = 0;
    gpsConstraintNum = psNum = camNum = 0;
    ptNum = projNum = 0;
    scalerPoints = 1.0;

    psNum = (int)scene->placedFixtures.size();

    std::set<FixtureCamera*> unique;
    for (auto& f: scene->placedFixtures)
    {
        if (scene->initializationData[f].enforcePosition)
            gpsConstrainedCameras.push_back(f);

        for (auto& c: f->cameras)
            unique.insert(c);
    }
    gpsConstraintNum = (int)gpsConstrainedCameras.size();
    camNum = (int)unique.size();
    activeCameras = std::vector<FixtureCamera*>(unique.begin(), unique.end());

    projNum = getReprojectionCnt();
    ptNum   = getMovablePointCount();

    inputNum = getInputNum();
    outputNum = getOutputNum();

    buildDependencyList();
    CORE_ASSERT_TRUE_S((int)sparsity.size() == inputNum);

    scalerGps = projNum / psNum * 3e-4  / 3.0;
    std::cout << "PT/PRJ: " << ((double)ptNum) / projNum << std::endl;
    std::cout << "GPS scaler: " << scalerGps << std::endl;

    std::cout << "Finally: " << inputNum << ">" << outputNum << " problem, " << ptNum << " points, " << psNum << " fixtures," << camNum << " cameras" << projNum << " projections, " << gpsConstraintNum << " gps constraints" <<  std::endl;
}


int corecvs::PhotostationPlacer::getInputNum()
{
    int input = 0;
    IF(DEGENERATE_ORIENTATIONS,
        input += 4);
    IF(NON_DEGENERATE_ORIENTATIONS,
        input += (psNum - 1) * 4);
    IF(DEGENERATE_TRANSLATIONS,
        input += 3);
    IF(NON_DEGENERATE_TRANSLATIONS,
        input += (psNum - 1) * 3);
    IF(FOCALS,
        input += camNum);
    IF(PRINCIPALS,
        input += camNum * 2);
    IF(POINTS,
        input += ptNum * 3);
    return input;
}

int corecvs::PhotostationPlacer::getOutputNum()
{
    int output = 0;
    output += projNum;
    IF(TUNE_GPS,
        output += gpsConstraintNum * 3);
    return output;
}

void corecvs::PhotostationPlacer::buildDependencyList()
{
    int errSize = getErrorComponentsPerPoint();
    int id = 0, argin = 0;
    auto& placedFixtures = scene->placedFixtures;

    sparsity.clear();
    sparsity.resize(inputNum);
    revDependency.resize(projNum);

    // First step - build projections list for feature
    // CORE_ASSERT_TRUE_S(argin == getInputNum() - getErrorComponentsPerPoint() * ptNum);s
    for (auto& t: scene->trackedFeatures)
    {
        for (auto& p: t->observations__)
        {
            for (int k = 0; k < errSize; ++k)
                revDependency[id++] = &p.second;
        }
    }
    for (auto& t: scene->staticPoints)
    {
        for (auto& p: t->observations__)
        {
            bool isPlaced = false;
            for (auto& f: scene->placedFixtures)
                if (f == p.first.u)
                {
                    isPlaced = true;
                    break;
                }
            if (!isPlaced)
                continue;
            for (int k = 0; k < errSize; ++k)
                revDependency[id++] = &p.second;
        }
    }
    CORE_ASSERT_TRUE_S(id == projNum);
    // Now we add all dependencies

    // Orientation of first camera
    IF(DEGENERATE_ORIENTATIONS,
        auto firstFixture = placedFixtures[0];
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < projNum; ++j)
            {
                auto observation = revDependency[j];
                if (observation->cameraFixture == firstFixture)
                    sparsity[argin].push_back(j);
            }
            for (auto& id: sparsity[argin])
                CORE_ASSERT_TRUE_S(id < outputNum);
            argin++;
        }
    );
    // Orientation for other cameras
    IF(NON_DEGENERATE_ORIENTATIONS,
        for (int i = 1; i < psNum; ++i)
        {
            auto fixture = placedFixtures[i];
            for (int jj = 0; jj < 4; ++jj)
            {
                for (int j = 0; j < projNum; ++j)
                {
                    auto observation = revDependency[j];
                    if (observation->cameraFixture == fixture)
                        sparsity[argin].push_back(j);
                }
                for (auto& id: sparsity[argin])
                    CORE_ASSERT_TRUE_S(id < outputNum);
                ++argin;
            }
        });
    // Translation for first camera
    IF(DEGENERATE_TRANSLATIONS,
        auto firstFixture = placedFixtures[0];
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < projNum; ++j)
            {
                auto p = revDependency[j];
                if (p->cameraFixture == firstFixture)
                    sparsity[argin].push_back(j);
            }
            if (!!(optimizationParams & PhotostationPlacerOptimizationType::TUNE_GPS))
            {
                for (int j = 0; j < gpsConstraintNum * 3; ++j)
                {
                    auto ps = gpsConstrainedCameras[j / 3];
                    if (ps == firstFixture)
                        sparsity[argin].push_back(j + projNum);
                }
            }
            for (auto& id: sparsity[argin])
                CORE_ASSERT_TRUE_S(id < outputNum);
            ++argin;
        });
    // Translation for other cameras
    IF(NON_DEGENERATE_TRANSLATIONS,
        for (size_t i = 1; i < scene->placedFixtures.size(); ++i)
        {
            auto fixture = placedFixtures[i];
            for (int jj = 0; jj < 3; ++jj)
            {
                for (int j = 0; j < projNum; ++j)
                {
                    auto p = revDependency[j];
                    if (p->cameraFixture == fixture)
                        sparsity[argin].push_back(j);
                }
                if (!!(optimizationParams & PhotostationPlacerOptimizationType::TUNE_GPS))
                {
                    for (int j = 0; j < gpsConstraintNum * 3; ++j)
                    {
                        auto ps = gpsConstrainedCameras[j / 3];
                        if (ps == fixture)
                            sparsity[argin].push_back(j + projNum);
                    }
                }
                for (auto& id: sparsity[argin])
                    CORE_ASSERT_TRUE_S(id < outputNum);
                ++argin;
            }
        });
    // Camera focals
    IF(FOCALS,
        for (int i = 0; i < camNum; ++i)
        {
            auto camera = activeCameras[i];
            for (int j = 0; j < projNum; ++j)
            {
                auto p = revDependency[j];
                if (p->camera == camera)
                    sparsity[argin].push_back(j);
            }
            for (auto& id: sparsity[argin])
                CORE_ASSERT_TRUE_S(id < outputNum);
            ++argin;
        });
    // Camera principals
    IF(PRINCIPALS,
        for (int i = 0; i < camNum; ++i)
        {
            auto camera = activeCameras[i];
            for (int jj = 0; jj < 2; ++jj)
            {
                for (int j = 0; j < projNum; ++j)
                {
                    auto p = revDependency[j];
                    if (p->camera == camera)
                        sparsity[argin].push_back(j);
                }
                for (auto& id: sparsity[argin])
                    CORE_ASSERT_TRUE_S(id < outputNum);
                ++argin;
            }
        });
    CORE_ASSERT_TRUE_S(argin == getInputNum() - getErrorComponentsPerPoint() * ptNum);
    // 3d points
    IF(POINTS,
        for (int i = 0; i < ptNum; ++i)
        {
            auto feature = scene->trackedFeatures[i];
            for (int jj = 0; jj < 3; ++jj)
            {
                for (int j = 0; j < projNum; ++j)
                {
                    auto p = revDependency[j];
                    if (p->featurePoint == feature)
                        sparsity[argin].push_back(j);
                }
                for (auto& id: sparsity[argin])
                    CORE_ASSERT_TRUE_S(id < outputNum);
                ++argin;
            }
        });
    CORE_ASSERT_TRUE_S(argin == getInputNum());
}

std::vector<std::vector<int>> corecvs::PhotostationPlacer::getDependencyList()
{
    return sparsity;
}

void corecvs::PhotostationPlacer::readOrientationParams(const double in[])
{
    int argin = 0;
    auto& placedFixtures = scene->placedFixtures;
    //int errSize = getErrorComponentsPerPoint();
    int psNum   = (int)placedFixtures.size();
    size_t camCnt  = activeCameras.size();

    IF(DEGENERATE_ORIENTATIONS,
        auto firstFixture = placedFixtures[0];
        for (int i = 0; i < 4; ++i)
            GETPARAM(firstFixture->location.rotor[i])
        firstFixture->location.rotor.normalise();
    );
    IF(NON_DEGENERATE_ORIENTATIONS,
        for (int i = 1; i < psNum; ++i)
        {
            auto fixture = placedFixtures[i];
            for (int j = 0; j < 4; ++j)
                GETPARAM(fixture->location.rotor[j]);
            fixture->location.rotor.normalise();
        });
    IF(DEGENERATE_TRANSLATIONS,
        auto firstFixture = placedFixtures[0];
        for (int i = 0; i < 3; ++i)
            GETPARAM(firstFixture->location.shift[i]));
    IF(NON_DEGENERATE_TRANSLATIONS,
        for (int i = 1; i < psNum; ++i)
        {
            auto fixture = placedFixtures[i];
            for (int j = 0; j < 3; ++j)
                GETPARAM(fixture->location.shift[j]);
        });
    IF(FOCALS,
        for (size_t i = 0; i < camCnt; ++i)
        {
            double f;
            GETPARAM(f);
            activeCameras[i]->intrinsics.focal = Vector2dd(f, f);
        });
    IF(PRINCIPALS,
        for (size_t i = 0; i < camCnt; ++i)
        {
            double cx;
            double cy;
            GETPARAM(cx);
            GETPARAM(cy);
            activeCameras[i]->intrinsics.principal = Vector2dd(cx, cy);
        });
    IF(POINTS,
        for (size_t j = 0; j < scene->trackedFeatures.size(); ++j)
        {
            Vector3dd& foo = scene->trackedFeatures[j]->reprojectedPosition;
            for (int i = 0; i < 3; ++i)
                GETPARAM(foo[i]);
        });
    CORE_ASSERT_TRUE_S(getInputNum() == argin);
}

void corecvs::PhotostationPlacer::writeOrientationParams(double out[])
{
    int argout = 0;
    auto& placedFixtures = scene->placedFixtures;
    //int errSize = getErrorComponentsPerPoint();
    int psNum   = (int)scene->placedFixtures.size();
    size_t camCnt  = activeCameras.size();

    IF(DEGENERATE_ORIENTATIONS,
        auto firstFixture = placedFixtures[0];
        for (int i = 0; i < 4; ++i)
            SETPARAM(firstFixture->location.rotor[i])
    );
    IF(NON_DEGENERATE_ORIENTATIONS,
        for (int i = 1; i < psNum; ++i)
        {
            auto fixture = placedFixtures[i];
            for (int j = 0; j < 4; ++j)
                SETPARAM(fixture->location.rotor[j]);
        });
    IF(DEGENERATE_TRANSLATIONS,
        auto firstFixture = placedFixtures[0];
        for (int i = 0; i < 3; ++i)
            SETPARAM(firstFixture->location.shift[i]));
    IF(NON_DEGENERATE_TRANSLATIONS,
        for (int i = 1; i < psNum; ++i)
        {
            auto fixture = placedFixtures[i];
            for (int j = 0; j < 3; ++j)
                SETPARAM(fixture->location.shift[j]);
        });
    IF(FOCALS,
        for (size_t i = 0; i < camCnt; ++i)
        {
            SETPARAM(activeCameras[i]->intrinsics.focal[0]);
        });
    IF(PRINCIPALS,
        for (size_t i = 0; i < camCnt; ++i)
        {
            SETPARAM(activeCameras[i]->intrinsics.principal[0]);
            SETPARAM(activeCameras[i]->intrinsics.principal[1]);
        });
    IF(POINTS,
        for (size_t j = 0; j < scene->trackedFeatures.size(); ++j)
        {
            for (int i = 0; i < 3; ++i)
                SETPARAM(scene->trackedFeatures[j]->reprojectedPosition[i]);
        });
    CORE_ASSERT_TRUE_S(getInputNum() == argout);
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
    IF(FOCALS,
            std::cout << "focals" << std::endl;);
    IF(PRINCIPALS,
            std::cout << "principals" << std::endl;);

    if (scene->placedFixtures.size() < 2)
        return;
    optimizationParams = params;
    prepareNonLinearOptimizationData();
    scene->validateAll();
    getDependencyList();
    getErrorSummaryAll();
    corecvs::LevenbergMarquardtSparse lm;
    OrientationFunctor orient(this);
    OrientationNormalizationFunctor orientNorm(this);
    lm.f = &orient;
    lm.normalisation = &orientNorm;
    lm.maxIterations = num;
    lm.trace = false;
    std::vector<double> input(getInputNum());
    std::vector<double> out(getOutputNum());
    lm.useConjugatedGradient = false;
    lm.conjugatedGradientIterations = std::max(100, (int)( 0.001 * input.size()));
    writeOrientationParams(&input[0]);
    auto res = lm.fit(input, out);
    readOrientationParams(&res[0]);

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
}

void corecvs::PhotostationPlacer::ParallelErrorComputator::operator() (const corecvs::BlockedRange<int> &r) const
{
    auto& revDependency = placer->revDependency;
    switch(placer->errorType)
    {
        case PhotostationPlacerOptimizationErrorType::REPROJECTION:
            for (int ii = r.begin(); ii < r.end(); ii++)
            {
                int i = idxs[ii * 2];
                CORE_ASSERT_TRUE_S(idxs[ii * 2] + 1 == idxs[ii * 2 + 1]);
                auto& o = *revDependency[i];
                auto error = o.cameraFixture->reprojectionError(o.featurePoint->reprojectedPosition, o.observation, o.camera);
                output[ii * 2]     = error[0];
                output[ii * 2 + 1] = error[1];
                for (int jjj = 0; jjj < 2; ++jjj)
                CORE_ASSERT_TRUE_S(!std::isnan(output[ii * 2 + jjj]));
            }
            break;
        case PhotostationPlacerOptimizationErrorType::ANGULAR:
            for (int ii = r.begin(); ii < r.end(); ++ii)
            {
                int i = idxs[ii];
                auto& o = *revDependency[i];
                auto error = o.cameraFixture->angleError(o.featurePoint->reprojectedPosition, o.observation, o.camera);
                output[ii] = error;
                for (int jjj = 0; jjj < 1; ++jjj)
                CORE_ASSERT_TRUE_S(!std::isnan(output[ii * 1 + jjj]));
            }
            break;
        case PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT:
            for (int ii = r.begin(); ii < r.end(); ++ii)
            {
                int i = idxs[ii * 3];
                CORE_ASSERT_TRUE_S(idxs[ii * 3] + 1 == idxs[ii * 3 + 1] && idxs[ii * 3] + 2 == idxs[ii * 3 + 2]);
                auto& o = *revDependency[i];
                auto error = o.cameraFixture->crossProductError(o.featurePoint->reprojectedPosition, o.observation, o.camera);
                for (int j = 0; j < 3; ++j)
                    output[ii * 3 + j] = error[j];
                for (int jjj = 0; jjj < 3; ++jjj)
                CORE_ASSERT_TRUE_S(!std::isnan(output[ii * 3 + jjj]));
            }
            break;
        case PhotostationPlacerOptimizationErrorType::RAY_DIFF:
            for (int ii = r.begin(); ii < r.end(); ++ii)
            {
                int i = idxs[ii * 3];
                CORE_ASSERT_TRUE_S(idxs[ii * 3] + 1 == idxs[ii * 3 + 1] && idxs[ii * 3] + 2 == idxs[ii * 3 + 2]);
                auto& o = *revDependency[i];
                auto error = o.cameraFixture->rayDiffError(o.featurePoint->reprojectedPosition, o.observation, o.camera);
                for (int j = 0; j < 3; ++j)
                    output[ii * 3 + j] = error[j];
            }
            break;
    }
}

void corecvs::PhotostationPlacer::computeErrors(double out[], const std::vector<int> &idxs)
{
    int lastProj = 0;
    int errSize = getErrorComponentsPerPoint();
    std::vector<int> reprojectionIdx, gpsIdx;
    for (auto& idx: idxs)
        if (idx < projNum)
            reprojectionIdx.push_back(idx);
        else
            gpsIdx.push_back(idx);
    lastProj = (int)reprojectionIdx.size();

    ParallelErrorComputator computator(this, reprojectionIdx, out);
    corecvs::parallelable_for(0, lastProj / errSize, 16, computator, true);

    int idx = lastProj;
    for (size_t i = 0; i < gpsIdx.size(); i += 3)
    {
        CORE_ASSERT_TRUE_S(gpsIdx[i] == gpsIdx[i + 1] - 1);
        CORE_ASSERT_TRUE_S(gpsIdx[i + 2] == gpsIdx[i + 1] + 1);
        int id = gpsIdx[i];
        int psId = (id - projNum) / 3;
        auto ps = scene->placedFixtures[psId];
        CORE_ASSERT_TRUE_S(scene->initializationData[ps].enforcePosition);
        auto diff = ps->location.shift - scene->initializationData[ps].initData.shift;
        auto foo = scene->initializationData[ps].positioningAccuracy * diff * scalerGps;
        for (int j = 0; j < 3; ++j)
            out[idx++] = foo[i];
    }
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
        if ((!fixture->reprojectionError(track->reprojectedPosition, proj, cam)) > trackInlierThreshold || (!(track->reprojectedPosition - fixture->getWorldCamera(cam).extrinsics.position)) > distanceLimit)
            continue;

        SceneObservation observation;
        observation.featurePoint = track;
        observation.camera = cam;
        observation.cameraFixture = fixture;
        bool found = false;
        for (auto& f: scene->fixtures)
            if (f == fixture)
                found = true;
        CORE_ASSERT_TRUE_S(found);
        observation.observation = proj;
        track->observations[cam] = observation;
        track->observations__[wpp] = observation;
        scene->trackMap[wpp][feature] = track;
        found = false;
        for (auto& fp: scene->points)
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
        corecvs::AbsoluteNonCentralRansacSolver solver = activeEstimates.count(cf) ? corecvs::AbsoluteNonCentralRansacSolver(cf, hypos, activeEstimates[cf]) : corecvs::AbsoluteNonCentralRansacSolver(cf, hypos);
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

        if (rm.size() < 100)
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
        RelativeNonCentralRansacSolver solver(
        //        psA,
                psB, rm, mm);
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
        scene->placingQueue.resize(scene->placingQueue.size() - 2);
        SceneAligner::TryAlign(scene, tform, scale);
        std::cout << tform << scale << "<<<< tform" << std::endl;

        for (auto& cf: scene->placedFixtures)
            std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
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
            cntr[std::make_pair(A, B)] += second.second.size();
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

        if (rm.size() < 100)
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
//    fit(optimizationParams & ~(PhotostationPlacerOptimizationType::DEGENERATE_TRANSLATIONS | (PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::TUNE_GPS)), 100);
    for (auto& cf: scene->placedFixtures)
        std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
    SceneAligner::TryAlign(scene, tform, scale);
    for (auto& cf: scene->placedFixtures)
        std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
//    fit(optimizationParams & ~(PhotostationPlacerOptimizationType::DEGENERATE_TRANSLATIONS | (PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::TUNE_GPS)), 100);
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
        if (!scene->is3DAligned)
        {
            corecvs::Affine3DQ transform;
            double scale;
            SceneAligner::TryAlign(scene, transform, scale);
        }
    }
    for (auto& cf: scene->placedFixtures)
        std::cout << cf->name << " " << cf->location.shift << " " << (cf->location.rotor ^ scene->placedFixtures[0]->location.rotor.conjugated()) << std::endl;
}

bool corecvs::PhotostationPlacer::appendPs()
{
    CORE_ASSERT_TRUE_S(speculativity > 0);
    scene->validateAll();
    if (scene->state == ReconstructionState::MATCHED)
    {
        CORE_ASSERT_TRUE_S(scene->placedFixtures.size() < 2);
        if (scene->placedFixtures.size())
        {
            addSecondPs();
            return true;
        }
        else
        {
            addFirstPs();
            return true;
        }
        return false;
    }
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

    if (maxInliers < minimalInlierCount)
        return false;

    std::cout << "Choosing to append " << psApp->name << " because it had " << maxInliers << " inliers" << std::endl;
    for (auto ptr: scene->placedFixtures)
        std::cout << ptr->name << " " << ptr->location.shift << " " << ptr->location.rotor << std::endl;
//    CameraFixture* psApp = scene->placingQueue[0];
    std::cout << "Placing #" << psApp->name << std::endl;
    L_ERROR << "Placing " << psApp->name ;
    L_ERROR << "Computing tracks" ;
    auto hypos = scene->getPossibleTracks(psApp);
    std::cout << "Total " << hypos.size() << " possible tracks" << std::endl;
    L_ERROR << "Computing P3P" ;

 // corecvs::AbsoluteNonCentralRansacSolverParams params;
//  AbsoluteNonCentralRansacSolver solver(psApp, hypos, params);
    switch(scene->initializationData[psApp].initializationType)
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
    tryAlign();
    L_ERROR << "Building tracks" ;
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
    L_ERROR << "Building tracks" ;
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
    L_ERROR << "Starting full run" ;
    L_ERROR << "Detecting features";
    detectAll();
    L_ERROR << "Initalizing";
    initialize();
    L_ERROR << "Fitting";
    fit(optimizationParams, 100);
    L_ERROR << "Appending";

    while(scene->placingQueue.size())
    {
        L_ERROR << "Appending" << (*scene->placingQueue.begin())->name ;
        appendPs();
        L_ERROR << "Fitting";
//      if (scene->is3DAligned)
//          fit(PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS, 100);
  //    else
    //   fit(PhotostationPlacerOptimizationType::NON_DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS, 200);
        fit(optimizationParams, 100);

        std::stringstream ss;
        ss << (*scene->placedFixtures.rbegin())->name << "_app.ply";
        dumpMesh(ss.str());
    }
    fit(optimizationParams, 10000);
    dumpMesh("final.ply");
}

corecvs::Mesh3D corecvs::PhotostationPlacer::dumpMesh(const std::string &filename)
{
    corecvs::Mesh3D meshres;
    meshres.switchColor(true);
    CalibrationHelpers().drawScene(meshres, *scene);
    meshres.dumpPLY(filename);
    return meshres;
}
