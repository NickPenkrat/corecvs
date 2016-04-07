#include "photostationPlacer.h"

#include <unordered_set>
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
#include "bufferReaderProvider.h"
#include "multicameraTriangulator.h"
#include "pnpSolver.h"
#include "abstractPainter.h"
#include "calibrationHelpers.h"
#include "calibrationLocation.h"
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

struct ParallelTrackPainter
{
    ParallelTrackPainter(
            std::vector<std::pair<WPP, std::string>> &images,
            ReconstructionFixtureScene* scene,
            std::unordered_map<SceneFeaturePoint*, RGBColor> colorizer) :
            colorizer(&colorizer)
          , images(images)
          , scene(scene)
    {
    }

    void operator() (const corecvs::BlockedRange<int> &r) const
    {
        for (int i = r.begin(); i < r.end(); ++i)
        {
            auto& p = images[i];
            auto key = p.first;
            auto name= p.second;
            std::stringstream ss;
            ss << name << "_tracks.png";

            auto nameNew = ss.str();
            corecvs::RGB24Buffer src = BufferReaderProvider::readRgb(name);

            AbstractPainter<RGB24Buffer> painter(&src);
            for (auto& tf: scene->trackedFeatures)
            {
                for (auto& obs: tf->observations__)
                    if (obs.first == key)
                    {
                        painter.drawFormat(obs.second.observation[0] + 5, obs.second.observation[1], colorizer[0][tf], 1,  tf->name.c_str());
                        painter.drawCircle(obs.second.observation[0], obs.second.observation[1], 3, colorizer[0][tf]);
                    }
            }
            BufferReaderProvider::writeRgb(src, nameNew);
            std::cout << "Writing tracks image into " << nameNew << std::endl;
        }
    }

    std::unordered_map<SceneFeaturePoint*, RGBColor> *colorizer;
    std::vector<std::pair<WPP, std::string>> images;
    ReconstructionFixtureScene* scene;
};

void corecvs::PhotostationPlacer::paintTracksOnImages()
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
    corecvs::parallelable_for(0, (int)images.size(), ParallelTrackPainter(images, scene, colorizer));
}

int corecvs::PhotostationPlacer::getMovablePointCount()
{
    // TODO: clarify which points are inmovable
    //       Immovable are scene->staticPoints
    return (int)scene->trackedFeatures.size();
}

void corecvs::PhotostationPlacer::tryAlign()
{
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

                if (!(ps->project(ptw, cam) - pt) < inlierThreshold)
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

corecvs::Quaternion corecvs::PhotostationPlacer::TransformFrom2RayCorrespondence(corecvs::Vector3dd o1, corecvs::Vector3dd o2, corecvs::Vector3dd e1, corecvs::Vector3dd e2)
{

    e1.normalise();
    e2.normalise();
    o1.normalise();
    o2.normalise();

    corecvs::Vector3dd e3 = e1 ^ e2;
    corecvs::Vector3dd o3 = o1 ^ o2;

    corecvs::Matrix A(9, 9);
    corecvs::Vector B(9);
    corecvs::Matrix33 RC;

    for (int i = 0; i < 3; ++i)
    {
        A.a(0, i) = A.a(1, i + 3) = A.a(2, i + 6) = e1[i];
        A.a(3, i) = A.a(4, i + 3) = A.a(5, i + 6) = e2[i];
        A.a(6, i) = A.a(7, i + 3) = A.a(8, i + 6) = e3[i];
        B[i] = o1[i];
        B[i + 3] = o2[i];
        B[i + 6] = o3[i];
    }
	corecvs::Vector Rv;
    corecvs::Matrix::LinSolve(A, B, Rv);
    int id = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            RC.a(i, j) = Rv[id++];
        }
    }

    corecvs::Vector3dd v;
    corecvs::Matrix33 Vt;
    corecvs::Matrix::svd(&RC, &v, &Vt);

    corecvs::Matrix33 R = RC * Vt.transposed();
    return corecvs::Quaternion::FromMatrix(R);

}

void corecvs::PhotostationPlacer::addSecondPs()
{
    // Detect orientation
    // Align (2xGPS, GPS+STATIC, GPS+FIXED)
    // Create 2-point cloud
    // Try align
    CORE_ASSERT_TRUE_S(scene->placedFixtures.size() == 1);
    auto ps = scene->placingQueue[0];
    scene->placedFixtures.push_back(ps);
    scene->placingQueue.erase(scene->placingQueue.begin());
    std::vector<CameraFixture*> pps = scene->placedFixtures;
    filterEssentialRansac(pps);
    estimatePair(pps[0], pps[1]);
    scene->matches = scene->matchesCopy;
    create2PointCloud();
    tryAlign();
    scene->state = ReconstructionState::TWOPOINTCLOUD;
}

void corecvs::PhotostationPlacer::create2PointCloud()
{
    CORE_ASSERT_TRUE_S(scene->placedFixtures.size() == 2);
    auto psA = scene->placedFixtures[0];
    auto psB = scene->placedFixtures[1];

    auto freeFeatures = getUnusedFeatures(psA, psB);

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

        double fscore = scoreFundamental(psA, camA, kpA, psB, camB, kpB);
        if (fscore > trackInlierThreshold)
            continue;

        corecvs::MulticameraTriangulator mct;
        mct.addCamera(psA->getMMatrix(camA), kpA);
        mct.addCamera(psB->getMMatrix(camB), kpB);
        auto res = mct.triangulateLM(mct.triangulate());

        bool isVisibleInlierNotTooFar = true;
        isVisibleInlierNotTooFar &= psA->isVisible(res, camA);
        isVisibleInlierNotTooFar &= !(kpA - psA->project(res, camA)) < trackInlierThreshold;
        isVisibleInlierNotTooFar &= !(res - psA->getWorldCamera(camA).extrinsics.position) < distanceLimit;
        isVisibleInlierNotTooFar &= psB->isVisible(res, camB);
        isVisibleInlierNotTooFar &= !(kpB - psB->project(res, camB)) < trackInlierThreshold;
        isVisibleInlierNotTooFar &= !(res - psB->getWorldCamera(camB).extrinsics.position) < distanceLimit;

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
    for (int i = 0; i < gpsIdx.size(); i += 3)
    {
        CORE_ASSERT_TRUE_S(gpsIdx[i] == gpsIdx[i + 1] - 1);
        CORE_ASSERT_TRUE_S(gpsIdx[i + 2] == gpsIdx[i + 1] + 1);
        int id = gpsIdx[i];
        int psId = (id - projNum) / 3;
        auto ps = scene->placedFixtures[psId];
        CORE_ASSERT_TRUE_S(scene->initializationData[ps].enforcePosition);
        auto diff = ps->location.shift - scene->initializationData[ps].initData.shift;
        auto foo = scene->initializationData[ps].positioningAccuracy * diff * scalerGps;
        for (int i = 0; i < 3; ++i)
            out[idx++] = foo[i];
    }
}

std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> corecvs::PhotostationPlacer::getPossibleTracks(CameraFixture *psA)
{
    CORE_ASSERT_TRUE_S(scene->state == ReconstructionState::APPENDABLE || scene->state == ReconstructionState::TWOPOINTCLOUD || scene->state == ReconstructionState::FINISHED);

    std::unordered_set<SceneFeaturePoint*> selectedTracks;
    std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> res;

    auto& trackMap = scene->trackMap;

    for (auto& psB: scene->placedFixtures)
    {
        for (auto& camB: psB->cameras)
        {
            for (auto& camA: psA->cameras)
            {
                WPP idB(psB, camB), idA(psA, camA);
                auto& keyPoints = scene->keyPoints[idA];
                bool swap = idB < idA;
                auto id1 = swap ? idB : idA;
                auto id2 = swap ? idA : idB;

                if (!scene->matches.count(id1))
                    continue;
                auto& mmv = scene->matches[id1];
                if (!mmv.count(id2))
                    continue;

                auto& mm = mmv[id2];
                for (auto& m: mm)
                {
                    int pt1 = std::get<0>(m),
                        pt2 = std::get<1>(m);
                    int ptA = swap ? pt2 : pt1;
                    int ptB = swap ? pt1 : pt2;
                    if (!trackMap.count(idB) ||
                        !trackMap[idB].count(ptB))
                        continue;
                    auto track = trackMap[idB][ptB];
                    if (selectedTracks.count(track))
                        continue;
                    res.emplace_back(
                        camA,
                        keyPoints[ptA].first,
                        track->reprojectedPosition,
                        track,
                        ptA);

                    selectedTracks.insert(track);
                }
            }
        }
    }
    return res;
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
        if (!fixture->reprojectionError(track->reprojectedPosition, proj, cam) > trackInlierThreshold || !(track->reprojectedPosition - fixture->getWorldCamera(cam).extrinsics.position) > distanceLimit)
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

void corecvs::PhotostationPlacer::appendPs()
{
    scene->validateAll();
    if (scene->state == ReconstructionState::MATCHED)
    {
        CORE_ASSERT_TRUE_S(scene->placedFixtures.size() < 2);
        if (scene->placedFixtures.size())
            addSecondPs();
        else
            addFirstPs();
        return;
    }
    CORE_ASSERT_TRUE_S(scene->state == ReconstructionState::TWOPOINTCLOUD ||
            scene->state == ReconstructionState::APPENDABLE);

    for (auto ptr: scene->placedFixtures)
        std::cout << ptr->name << " " << ptr->location.shift << " " << ptr->location.rotor << std::endl;
    CameraFixture* psApp = scene->placingQueue[0];
    std::cout << "Placing #" << psApp->name << std::endl;
    L_ERROR << "Placing " << psApp->name ;
    L_ERROR << "Computing tracks" ;
    auto hypos = getPossibleTracks(psApp);
    std::cout << "Total " << hypos.size() << " possible tracks" << std::endl;
    L_ERROR << "Computing P3P" ;

    corecvs::AbsoluteNonCentralRansacSolverParams params;
    AbsoluteNonCentralRansacSolver solver(psApp, hypos, params);
    switch(scene->initializationData[psApp].initializationType)
    {
        default:
        case PhotostationInitializationType::GPS:
        {
            if (scene->is3DAligned)
                solver.forcePosition = true;
            solver.forcedPosition = scene->initializationData[psApp].initData.shift;

            solver.run();
            solver.runInliersRE();
            auto hypo = solver.getBestHypothesis();
            psApp->location.rotor = hypo.rotor;
            std::cout << "!!!!" << hypo.rotor << "!!!!" << std::endl;
            psApp->location.shift = !scene->is3DAligned ? hypo.shift : scene->initializationData[psApp].initData.shift;
        }
        break;
        case PhotostationInitializationType::STATIC:
        {

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
        }
        break;
    }
    std::cout << "TRACKS BEFORE: " << scene->trackedFeatures.size() << std::endl;
    if (scene->state == ReconstructionState::APPENDABLE)
    {
        L_ERROR << "Appending tracks" ;
        appendTracks(solver.getInliers(), psApp, hypos);
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
            buildTracks(psApp, scene->placedFixtures[aId], scene->placedFixtures[bId]);
        }
    }
    std::cout << "TRACKS AFTER: " << scene->trackedFeatures.size() << std::endl;
    scene->placedFixtures.push_back(psApp);
    scene->placingQueue.erase(scene->placingQueue.begin());
    scene->validateAll();
    return;
}

std::unordered_map<std::tuple<FixtureCamera*, FixtureCamera*, int>, int> corecvs::PhotostationPlacer::getUnusedFeatures(CameraFixture *psA, CameraFixture *psB)
{
    std::unordered_map<std::tuple<FixtureCamera*, FixtureCamera*, int>, int> res;
    for (auto camA: psA->cameras)
    {
        for (auto camB: psB->cameras)
        {
            WPP idA(psA, camA), idB(psB, camB);
            bool swap = !(idA < idB);
            auto id1 = swap ? idB : idA, id2 = swap ? idA : idB;
            if (!scene->matches.count(id1))
                continue;
            if (!scene->matches[id1].count(id2))
                continue;
            auto& mv = scene->matches[id1][id2];
            for (auto &m: mv)
            {
                int f1 = std::get<0>(m),
                    f2 = std::get<1>(m);
                if (scene->trackMap[id1].count(f1))
                    continue;
                if (scene->trackMap[id2].count(f2))
                    continue;
                int fA = swap ? f2 : f1,
                    fB = swap ? f1 : f2;
                res[std::make_tuple(camA, camB, fA)] = fB;
            }
        }
    }
    return res;
}


void corecvs::PhotostationPlacer::buildTracks(CameraFixture *psA, CameraFixture *psB, CameraFixture *psC)
{
    const int NPS = 3;
    const int NPAIRS = 3;
    int pairIdx[NPAIRS][2] = {{0, 1}, {1, 2}, {0, 2}};

    CameraFixture*        ps[NPS] = {psA, psB, psC};
    FixtureCamera*       cam[NPS] = {  0,   0,   0};
    int                   pt[NPS] = {  0,   0,   0};
    FixtureCamera* &camA = cam[0], *&camB = cam[1], *&camC = cam[2];
    int &ptA = pt[0], &ptB = pt[1], &ptC = pt[2];
    WPP                  wpp[NPS];
    corecvs::Vector2dd    kp[NPS];
  //corecvs::Vector2dd &kpA = kp[0], &kpB = kp[1], &kpC = kp[2];

    std::unordered_map<std::tuple<FixtureCamera*, FixtureCamera*, int>, int> free[NPAIRS];
    auto &freeAB = free[0], &freeBC = free[1]/*, &freeAC = free[2]*/;

    for (int i = 0; i < NPAIRS; ++i)
        free[i] = getUnusedFeatures(ps[pairIdx[i][0]], ps[pairIdx[i][1]]);

    std::vector<std::tuple<FixtureCamera*, int, FixtureCamera*, int, FixtureCamera*, int>> trackCandidates;

    for (auto& mAC: free[2])
    {
        camA = std::get<0>(mAC.first);
        camC = std::get<1>(mAC.first);
        ptA = std::get<2>(mAC.first);
        ptC = mAC.second;

        for (auto& camB: psB->cameras)
        {
            auto idAB = std::make_tuple(camA, camB, ptA);
            if (!free[0].count(idAB))
                continue;

            ptB = freeAB[idAB];
            auto idBC = std::make_tuple(camB, camC, ptB);
            if (!freeBC.count(idBC))
                continue;
            int ptC2 = freeBC[idBC];
            if (ptC == ptC2)
                trackCandidates.emplace_back(camA, ptA, camB, ptB, camC, ptC);
        }
    }

    for (auto& c: trackCandidates)
    {
        camA = std::get<0>(c);
        camB = std::get<2>(c);
        camC = std::get<4>(c);
        ptA = std::get<1>(c);
        ptB = std::get<3>(c);
        ptC = std::get<5>(c);

        bool alreadyIn = false;
        for(int i = 0; i < NPS; ++i)
        {
            wpp[i] = WPP(ps[i], cam[i]);
            kp[i] = scene->keyPoints[wpp[i]][pt[i]].first;
            if (scene->trackMap[wpp[i]].count(pt[i]))
                alreadyIn = true;
        }
        if (alreadyIn)
            continue;

        double fscore = 0.0;
        for (int i = 0; i < NPAIRS; ++i)
        {
            int id1 = pairIdx[i][0];
            int id2 = pairIdx[i][1];
            fscore = std::max(fscore, scoreFundamental(ps[id1], cam[id1], kp[id1], ps[id2], cam[id2], kp[id2]));
        }
        if (fscore > trackInlierThreshold)
            continue;

        corecvs::MulticameraTriangulator mct;
        for (int i = 0; i < NPS; ++i)
            mct.addCamera(ps[i]->getMMatrix(cam[i]), kp[i]);
        auto res = mct.triangulateLM(mct.triangulate());

        bool isVisibleInlierNotTooFar = true;
        for (int i = 0; i < NPS; ++i)
        {
            isVisibleInlierNotTooFar &= ps[i]->isVisible(res, cam[i]);
            isVisibleInlierNotTooFar &= !(kp[i] - ps[i]->project(res, cam[i])) < trackInlierThreshold;
            isVisibleInlierNotTooFar &= !(res - ps[i]->getWorldCamera(cam[i]).extrinsics.position) < distanceLimit;
        }
        if (!isVisibleInlierNotTooFar)
            continue;

        auto track = scene->createFeaturePoint();
        track->reprojectedPosition = res;
        track->hasKnownPosition = false;
        track->type = SceneFeaturePoint::POINT_RECONSTRUCTED;

        for (int i = 0; i < NPS; ++i)
        {
            SceneObservation so;
            so.camera = cam[i];
            so.cameraFixture = ps[i];
            so.featurePoint = track;
            so.observation = kp[i];
            track->observations[cam[i]] = so;
            track->observations__[wpp[i]] = so;
            track->color = scene->keyPoints[wpp[i]][pt[i]].second;
            scene->trackMap[wpp[i]][pt[i]] = track;
        }
        scene->trackedFeatures.push_back(track);
    }
}

double corecvs::PhotostationPlacer::scoreFundamental(CameraFixture* psA, FixtureCamera *camA, corecvs::Vector2dd ptA,
                                 CameraFixture* psB, FixtureCamera* camB, corecvs::Vector2dd ptB)
{
    auto FAB = psA->getWorldCamera(camA).fundamentalTo(
               psB->getWorldCamera(camB));
    corecvs::Line2d left = FAB.mulBy2dRight(ptB);
    corecvs::Line2d right= FAB.mulBy2dLeft (ptA);
    return std::max(left.distanceTo(ptA), right.distanceTo(ptB));

}

void corecvs::PhotostationPlacer::estimateFirstPair()
{
#ifdef WITH_TBB
    tbb::task_group g;
    g.run([=]() { estimatePair(scene->placingQueue[0], scene->placingQueue[1]); });
    g.run([=]() { estimatePair(scene->placingQueue[0], scene->placingQueue[2]); });
    g.wait();
#else
    estimatePair(scene->placingQueue[0], scene->placingQueue[1]);
    estimatePair(scene->placingQueue[0], scene->placingQueue[2]);
#endif

    auto q = detectOrientationFirst(scene->placingQueue[0], scene->placingQueue[1], scene->placingQueue[2]);
    scene->placingQueue[0]->location.rotor = q.conjugated();
    scene->placingQueue[0]->location.shift = scene->initializationData[scene->placingQueue[0]].initData.shift;

    scene->placingQueue[1]->location.rotor = q.conjugated() ^ scene->placingQueue[1]->location.rotor;
    scene->placingQueue[1]->location.shift = scene->initializationData[scene->placingQueue[1]].initData.shift;

    scene->placingQueue[2]->location.rotor = q.conjugated() ^ scene->placingQueue[2]->location.rotor;
    scene->placingQueue[2]->location.shift = scene->initializationData[scene->placingQueue[2]].initData.shift;

    scene->matches = scene->matchesCopy;
}

corecvs::Quaternion corecvs::PhotostationPlacer::detectOrientationFirst(CameraFixture* psA, CameraFixture* psB, CameraFixture* psC)
{
    auto init = scene->initializationData;
    corecvs::Vector3dd e1 = init[psB].initData.shift - init[psA].initData.shift;
    corecvs::Vector3dd e2 = init[psC].initData.shift - init[psA].initData.shift;

    corecvs::Vector3dd o1 = psB->location.shift - psA->location.shift;
    corecvs::Vector3dd o2 = psC->location.shift - psA->location.shift;
    return TransformFrom2RayCorrespondence(o1, o2, e1, e2);
}

void corecvs::PhotostationPlacer::estimatePair(CameraFixture *psA, CameraFixture *psB)
{
    auto matches = getPhotostationMatches(psA, psB);
    RelativeNonCentralRansacSolver::MatchContainer rm, mm;
    for (auto&t : matches)
    {
        if (std::get<4>(t) < b2bRansacP6RPThreshold)
            rm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
        mm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
    }

    RelativeNonCentralRansacSolver solver(
            psA,
            psB, rm, mm);
    solver.run();
    auto best = solver.getBestHypothesis();
    std::cout << psA->name << "::" << psB->name << " " << best.shift << " " << best.rotor << std::endl;
    best = solver.getBestHypothesis();
    std::cout << psA->name << "::" << psB->name << " " << best.shift << " " << best.rotor << std::endl;
    psB->location = best;
}

void corecvs::PhotostationPlacer::filterEssentialRansac(WPP a, WPP b)
{
    bool swap = !(a < b);
    WPP idA = swap ? b : a;
    WPP idB = swap ? a : b;

    std::cout << "Starting: " << idA.u->name << idA.v->nameId << "<>" << idB.u->name << idB.v->nameId << std::endl;

    std::vector<std::array<corecvs::Vector2dd, 2>> features, featuresInlier;
    auto K1 = idA.v->intrinsics.getKMatrix33();
    auto K2 = idB.v->intrinsics.getKMatrix33();

    if (!scene->matches.count(idA) || !scene->matches[idA].count(idB))
        return;

    auto& mm = scene->matches[idA][idB];
    size_t szBefore = mm.size();
    auto& kpA= scene->keyPoints[idA];
    auto& kpB= scene->keyPoints[idB];
    features.reserve(mm.size());
    featuresInlier.resize(mm.size());
    //int idf = 0;
    for (auto& m: mm)
    {
        int idA = std::get<0>(m);
        int idB = std::get<1>(m);
        auto fA = kpA[idA].first;
        auto fB = kpB[idB].first;
        int id = &m - &mm[0];
        featuresInlier[id][0] = fA;
        featuresInlier[id][1] = fB;
        if (std::get<2>(m) < b2bRansacP5RPThreshold)
           features.push_back(featuresInlier[id]);
    }
    size_t szAfter1= mm.size();
    CORE_ASSERT_TRUE_S(&mm == &scene->matches[idA][idB]);
    CORE_ASSERT_TRUE_S(szBefore == szAfter1);

    EssentialFeatureFilter filter(K1, K2, features, featuresInlier, inlierP5RPThreshold, 0.001, maxEssentialRansacIterations);
    filter.estimate();
    auto bestInliers = filter.inlierIdx;

    size_t szAfter = mm.size();
    CORE_ASSERT_TRUE_S(&mm == &scene->matches[idA][idB]);
    CORE_ASSERT_TRUE_S(szBefore == szAfter);
    CORE_ASSERT_TRUE_S(bestInliers.size() <= mm.size());
    std::sort(bestInliers.begin(), bestInliers.end());
    std::cout << "Total: " << featuresInlier.size() << " good: " << features.size() << " del: " << (featuresInlier.size() - bestInliers.size()) << " rem: " << bestInliers.size() << " (" << ((double)bestInliers.size()) / featuresInlier.size() * 100.0 << "%)" << idA.u->name << idA.v->nameId << "<>" << idB.u->name << idB.v->nameId << std::endl;
    remove(a, b, bestInliers);
}

void corecvs::PhotostationPlacer::filterEssentialRansac(std::vector<CameraFixture*> &pss)
{
    scene->matchesCopy = scene->matches;
    std::vector<std::pair<WPP, WPP>> work;
    for (size_t psA = 0; psA < pss.size(); ++psA)
    {
        for (size_t psB = psA; psB < pss.size(); ++psB)
        {
            auto psA_ = pss[psA];
            auto psB_ = pss[psB];
            for (size_t camA = 0; camA < psA_->cameras.size(); ++camA)
            {
                for (size_t camB = 0; camB < psB_->cameras.size(); ++camB)
                {
                    WPP idFirst(psA_, psA_->cameras[camA]), idSecond(psB_, psB_->cameras[camB]);
                    bool alreadyIn = false;
                    for (auto& pp: work)
                        if ((pp.first == idFirst && pp.second == idSecond) || (pp.second == idFirst && pp.first == idSecond))
                        {
                            alreadyIn = true;
                            break;
                        }
                    if (!alreadyIn)
                        work.emplace_back(WPP(psA_, psA_->cameras[camA]), WPP(psB_, psB_->cameras[camB]));
                }
            }
        }
    }
    corecvs::parallelable_for(0, (int)work.size(), ParallelEssentialFilter(this, work));
}

bool corecvs::PhotostationPlacer::initialize()
{
    scene->ProcessState->reset("Initialize", 1);
    scene->ProcessState->incrementStarted();

    if (scene->state != ReconstructionState::MATCHED)
        return false;
    CORE_ASSERT_TRUE_S(scene->placingQueue.size() >= 2);
    std::unordered_map<PhotostationInitializationType, int> cnt;
    for (size_t i = 0; i < std::min((size_t)3, scene->placingQueue.size()); ++i)
        cnt[scene->initializationData[scene->placingQueue[i]].initializationType]++;

    // Gives 6-DoF initialization + 3-view cloud (1)
    if (cnt[PhotostationInitializationType::GPS] == 3)
        return initGPS();
    // Gives 6-DoF initialization + 2-view cloud (16)
    if (cnt[PhotostationInitializationType::FIXED] >= 1 || cnt[PhotostationInitializationType::STATIC] >= 1)
    {
        return cnt[PhotostationInitializationType::FIXED] > cnt[PhotostationInitializationType::STATIC] ? initFIXED() : initSTATIC();
    }
    if (cnt[PhotostationInitializationType::GPS] > 0)
    {
        // requires DoF estimation on the fly, NIY
        CORE_ASSERT_TRUE_S(false);
    }
    // Gives 0-DoF initialization + 2-view cloud
    scene->ProcessState->incrementCompleted();
    return initNONE();
}

bool corecvs::PhotostationPlacer::initGPS()
{
    L_ERROR << "Starting feature filtering" ;
    std::vector<CameraFixture*> pss = {scene->placingQueue[0], scene->placingQueue[1], scene->placingQueue[2]};
    if (runEssentialFiltering)
        filterEssentialRansac(pss);
    else
        scene->matchesCopy = scene->matches;
    L_ERROR << "Estimating first pair orientation" ;
    estimateFirstPair();
    L_ERROR << "Building tracks" ;
    buildTracks(scene->placingQueue[0], scene->placingQueue[1], scene->placingQueue[2]);
    for (int i = 0; i < 3; ++i)
    {
        scene->placedFixtures.push_back(scene->placingQueue[0]);
        scene->placingQueue.erase(scene->placingQueue.begin());
    }
    scene->is3DAligned = true;
    scene->state = ReconstructionState::APPENDABLE;
    dumpMesh("gpsinit.ply");
    return true;
}

bool corecvs::PhotostationPlacer::initNONE()
{
    CORE_ASSERT_TRUE_S(false);
    return true;
}

bool corecvs::PhotostationPlacer::initSTATIC()
{
    L_ERROR << "Initializing 3 pss" ;
    for (int i = 0; i < 3; ++i)
    {
        auto psApp = scene->placingQueue[i];
        corecvs::AbsoluteNonCentralRansacSolverParams params;
        params.reprojectionInlierThreshold = 16.0;
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
            bool alreadyIn = false;
            for (auto& ptrS: scene->staticPoints)
                if (ptrS == ptr)
                {
                    alreadyIn = true;
                    break;
                }
            if (alreadyIn)
                continue;
            scene->staticPoints.push_back(ptr);
            ptr->reprojectedPosition = ptr->position;
        }
        AbsoluteNonCentralRansacSolver solver(psApp, foo, params);
        solver.cloudMatches = foo;
        solver.run();
        solver.runInliersRE();
        auto hypo = solver.getBestHypothesis();
        psApp->location = hypo;
        std::cout << "!!!!"  << psApp->name << hypo << "!!!!" << std::endl;
    }
    L_ERROR << "Building tracks" ;
    buildTracks(scene->placingQueue[0], scene->placingQueue[1], scene->placingQueue[2]);
    for (int i = 0; i < 3; ++i)
    {
        scene->placedFixtures.push_back(scene->placingQueue[0]);
        scene->placingQueue.erase(scene->placingQueue.begin());
    }
    scene->is3DAligned = true;
    scene->state = ReconstructionState::APPENDABLE;
    dumpMesh("staticinit.ply");
    return true;
}

bool corecvs::PhotostationPlacer::initFIXED()
{
    CORE_ASSERT_TRUE_S(false);
    return true;
}

void corecvs::PhotostationPlacer::remove(WPP a, WPP b, std::vector<int> idx)
{
    bool swap = !(a < b);
    CORE_ASSERT_TRUE_S(scene->matches.count(swap ? b : a));
    CORE_ASSERT_TRUE_S(scene->matches[swap ? b : a].count(swap ? a : b));
    auto& ref = scene->matches[swap ? b : a][swap ? a : b];
    CORE_ASSERT_TRUE_S(idx.size() <= ref.size());
    int ok = 0;
    for (auto& i: idx)
    {
        ref[ok++] = ref[i];
    }
    ref.resize(ok);
}

std::vector<std::tuple<WPP, corecvs::Vector2dd, WPP, corecvs::Vector2dd, double>>
corecvs::PhotostationPlacer::getPhotostationMatches(CameraFixture *psA, CameraFixture *psB)
{
    WPP wcA = WPP(psA, WPP::VWILDCARD), wcB = WPP(psB, WPP::VWILDCARD);
    bool swap = psA > psB;
    std::vector<std::tuple<WPP, corecvs::Vector2dd, WPP, corecvs::Vector2dd, double>> res;
    auto id1 = swap ? wcB : wcA;
    auto id2 = swap ? wcA : wcB;

    for (auto& ref1: scene->matches)
    {
        if (!(ref1.first == id1))
            continue;
        for (auto& ref2: ref1.second)
        {
            if (!(ref2.first == id2))
                continue;
            auto  idA  = swap ? ref2.first : ref1.first;
            auto  idB  = swap ? ref1.first : ref2.first;
            CORE_ASSERT_TRUE_S(idA.u != WPP::UWILDCARD && idA.v != WPP::VWILDCARD);
            CORE_ASSERT_TRUE_S(idB.u != WPP::UWILDCARD && idB.v != WPP::VWILDCARD);
            auto& kpsA = scene->keyPoints[idA];
            auto& kpsB = scene->keyPoints[idB];
            for (auto& m: ref2.second)
            {
                int kpA = std::get<0>(m);
                int kpB = std::get<1>(m);
                if (swap)
                    std::swap(kpA, kpB);
                auto pA = kpsA[kpA].first;
                auto pB = kpsB[kpB].first;
                res.emplace_back(idA, pA, idB, pB, std::get<2>(m));
            }
        }
    }
    return res;
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
}

void corecvs::PhotostationPlacer::fullRun()
{
    scene->ProcessState->reset("Detecting", 1);
    scene->ProcessState->incrementStarted();

    L_ERROR << "Starting full run" ;
    L_ERROR << "Detecting features";
    detectAll();
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("Initializing", 1);
    scene->ProcessState->incrementStarted();
    L_ERROR << "Initalizing";
    initialize();
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("First fitting", 1);
    scene->ProcessState->incrementStarted();
    L_ERROR << "Fitting";
    fit(optimizationParams, 100);
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("Appending", 1);
    scene->ProcessState->incrementStarted();
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
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("Finale fitting", 1);
    scene->ProcessState->incrementStarted();

    fit(optimizationParams, 10000);
    scene->ProcessState->incrementCompleted();

    scene->ProcessState->reset("Dumping", 1);
    scene->ProcessState->incrementStarted();
    dumpMesh("final.ply");
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
