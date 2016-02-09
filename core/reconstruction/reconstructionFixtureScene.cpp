#include "reconstructionFixtureScene.h"

#include <set>

#include "featureMatchingPipeline.h"
#include "log.h"

using namespace corecvs;

ReconstructionFixtureScene::ReconstructionFixtureScene()
{
}

void ReconstructionFixtureScene::deleteCamera(FixtureCamera *camera)
{
    FixtureScene::deleteCamera(camera);
    deleteFixtureCameraUMWPP(images,    camera);
    deleteFixtureCameraUMWPP(keyPoints, camera);
    deleteFixtureCameraUMWPP(matches,   camera);
    deleteFixtureCameraUMWPP(trackMap,  camera);
}

void ReconstructionFixtureScene::deleteCameraFixture(CameraFixture *fixture, bool recursive)
{
    std::cout << "deleting from parent" << std::endl;
    FixtureScene::deleteCameraFixture(fixture, recursive);
    std::cout << "deleting from images" << std::endl;
    deleteCameraFixtureUMWPP(images,    fixture);
    std::cout << "deleting from keypoints" << std::endl;
    deleteCameraFixtureUMWPP(keyPoints, fixture);
    std::cout << "deleting from matches" << std::endl;
    deleteCameraFixtureUMWPP(matches,   fixture);
    std::cout << "deleting from trackmap" << std::endl;
    deleteCameraFixtureUMWPP(trackMap,  fixture);
    std::cout << "deleting from init" << std::endl;
    initializationData.erase(fixture);
    std::cout << "deleting from placed" << std::endl;
    vectorErase(placedFixtures, fixture);
    std::cout << "deleting from queue" << std::endl;
    vectorErase(placingQueue, fixture);
}

void ReconstructionFixtureScene::deleteFixturePair(CameraFixture *fixture, FixtureCamera *camera)
{
    FixtureScene::deleteFixturePair(fixture, camera);
    deletePairUMWPP(images,    fixture, camera);
    deletePairUMWPP(keyPoints, fixture, camera);
    deletePairUMWPP(matches,   fixture, camera);
    deletePairUMWPP(trackMap,  fixture, camera);
}

void ReconstructionFixtureScene::deleteFeaturePoint(SceneFeaturePoint *point)
{
    FixtureScene::deleteFeaturePoint(point);
    vectorErase(trackedFeatures, point);
    vectorErase(staticPoints, point);
    for (auto& foo: trackMap)
        for (auto boo = foo.second.begin(); boo != foo.second.end(); boo = boo->second == point ? foo.second.erase(boo) : boo++);
}

void ReconstructionFixtureScene::detectAllFeatures(const FeatureDetectionParams &params)
{
    // Mapping from indexes to fixture,camera pairs
    std::unordered_map<int, WPP> map;
    std::vector<std::string> filenames;
    for (auto& f: placingQueue)
    {
        for (auto& c: f->cameras)
        {
            WPP idx(f, c);
            if (!images.count(idx))
                continue;
            filenames.push_back(images[idx]);
            map[(int)filenames.size() - 1] = idx;
            CORE_ASSERT_TRUE_S(idx.u != WPP::UWILDCARD && idx.v != WPP::VWILDCARD);
        }
    }
    size_t N = filenames.size();
    // Cleanup
    for (auto& pp: map)
    {
        keyPoints[pp.second].clear();
        matches[pp.second].clear();
        for (auto& id: matches)
        {
            if (id.second.count(pp.second))
            {
                id.second[pp.second].clear();
            }
        }
        trackMap[pp.second].clear();
    }

    // Feature detection and matching
    FeatureMatchingPipeline pipeline(filenames);
    pipeline.add(new KeyPointDetectionStage(params.detector), true);
    pipeline.add(new DescriptorExtractionStage(params.descriptor), true);
    pipeline.add(new MatchingPlanComputationStage(), true);
    pipeline.add(new MatchAndRefineStage(params.descriptor, params.matcher, params.b2bThreshold), true);

    pipeline.run();

    // Here we discard some data and store only part (w/o descriptors and etc)
    for (size_t i = 0; i < N; ++i)
    {
        auto& kps = pipeline.images[i].keyPoints.keyPoints;
        CORE_ASSERT_TRUE_S(map.count((int)i));
        auto id = map[(int)i];

        auto& kpp = keyPoints[id];
        for (auto& kp: kps)
        {
            kpp.emplace_back(Vector2dd(kp.x, kp.y), kp.color);
        }
    }
    // Since our matches are reflective, we can store them only once
    auto& ref = pipeline.refinedMatches.matchSets;
    for (auto& ms: ref)
    {
        CORE_ASSERT_TRUE_S(map.count((int)ms.imgA));
        CORE_ASSERT_TRUE_S(map.count((int)ms.imgB));
        auto id1 = map[(int)ms.imgA];
        auto id2 = map[(int)ms.imgB];
        bool swap = !(id1 < id2);

        auto idA = swap ? id2 : id1;
        auto idB = swap ? id1 : id2;
        CORE_ASSERT_TRUE_S(idA.u != WPP::UWILDCARD && idA.v != WPP::VWILDCARD);
        CORE_ASSERT_TRUE_S(idB.u != WPP::UWILDCARD && idB.v != WPP::VWILDCARD);

        auto& mset = matches[idA][idB];
        for (auto& m: ms.matches)
        {
            int fA = swap ? m.featureB : m.featureA;
            int fB = swap ? m.featureA : m.featureB;
            mset.emplace_back(fA, fB, m.best2ndBest);
        }
    }
}

std::vector<FixtureCamera*> ReconstructionFixtureScene::getDistinctCameras() const
{
    std::cout << "GDC" << std::endl;
    std::set<FixtureCamera*> cameras;
    for (auto& f: placedFixtures)
    {
        for (auto& c: f->cameras)
        {
            cameras.insert(c);
        }
    }
    std::cout << cameras.size() << std::endl;
    return std::vector<FixtureCamera*>(cameras.begin(), cameras.end());
}

int ReconstructionFixtureScene::getDistinctCameraCount() const
{
    return (int)getDistinctCameras().size();
}

bool ReconstructionFixtureScene::validateMatches()
{
    for (auto& mv: matches)
    {
        auto fixture = mv.first.u;
        auto camera  = mv.first.v;
        CORE_ASSERT_TRUE_S(std::find(fixtures.begin(), fixtures.end(), fixture) != fixtures.end());
//        CORE_ASSERT_TRUE_S(std::find(cameras.begin(), cameras.end(), cameras) != cameras.end());

        for (auto& mvv: mv.second)
        {
            auto fixtureB = mvv.first.u;
            auto cameraB  = mvv.first.v;
            CORE_ASSERT_TRUE_S(std::find(fixtures.begin(), fixtures.end(), fixtureB) != fixtures.end());
  //          CORE_ASSERT_TRUE_S(std::find(cameras.begin(), cameras.end(), cameraB) != cameras.end());
            for (auto& t: mvv.second)
            {
                int mA = std::get<0>(t);
                int mB = std::get<1>(t);
                auto itA = keyPoints.count(WPP(fixture, camera));
                CORE_ASSERT_TRUE_S(itA > 0);
                CORE_ASSERT_TRUE_S(keyPoints[WPP(fixture, camera)].size() > mA);
                auto itB = keyPoints.count(WPP(fixtureB, cameraB));
                CORE_ASSERT_TRUE_S(itB > 0);
                CORE_ASSERT_TRUE_S(keyPoints[WPP(fixtureB, cameraB)].size() > mB);
            }
        }
    }
    return true;
}

bool ReconstructionFixtureScene::haveCamera(FixtureCamera * camera)
{
    for (auto& p: mOwnedObjects)
    {
        if (dynamic_cast<FixtureCamera*>(p) == camera)
            return true;
    }
    return false;
}

bool ReconstructionFixtureScene::haveFixture(CameraFixture * camera)
{
    for (auto& p: mOwnedObjects)
    {
        if (dynamic_cast<CameraFixture*>(p) == camera)
            return true;
    }
    return false;
}

bool ReconstructionFixtureScene::havePoint(SceneFeaturePoint *point)
{
    for (auto& p: mOwnedObjects)
    {
        if (dynamic_cast<SceneFeaturePoint*>(p) == point)
            return true;
    }
    return false;
}

bool ReconstructionFixtureScene::validateTracks()
{
    for (auto pt: trackedFeatures)
    {
        CORE_ASSERT_TRUE_S(pt);
        CORE_ASSERT_TRUE_S(std::find(points.begin(), points.end(), pt) != points.end());
        for (auto wpp: pt->observations__)
        {
            auto fixture = wpp.first.u;
            auto camera  = wpp.first.v;
            CORE_ASSERT_TRUE_S(std::find(fixtures.begin(), fixtures.end(), fixture) != fixtures.end());
            auto o       = wpp.second;
            CORE_ASSERT_TRUE_S(std::find(fixture->cameras.begin(), fixture->cameras.end(), camera) != fixture->cameras.end());
            CORE_ASSERT_TRUE_S(fixture == o.cameraFixture);
            CORE_ASSERT_TRUE_S(camera  == o.camera);
            CORE_ASSERT_TRUE_S(pt      == o.featurePoint);
            for (int i = 0; i < 2; ++i)
                CORE_ASSERT_TRUE_S(!std::isnan(o.observation[i]));
        }
        for (int i = 0; i < 3; ++i)
            CORE_ASSERT_TRUE_S(!std::isnan(pt->reprojectedPosition[i]));
    }
    return true;
}

bool ReconstructionFixtureScene::validateAll()
{
    L_ERROR << "Validating..." ;
    if (validateMatches() && validateTracks())
    {
        std::cout << "VALID!!!" << std::endl;
        return true;
    }
    return false;
}
