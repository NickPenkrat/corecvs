#include "reconstructionFixtureScene.h"

#include <set>

#include "featureMatchingPipeline.h"

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
    FixtureScene::deleteCameraFixture(fixture, recursive);
    deleteCameraFixtureUMWPP(images,    fixture);
    deleteCameraFixtureUMWPP(keyPoints, fixture);
    deleteCameraFixtureUMWPP(matches,   fixture);
    deleteCameraFixtureUMWPP(trackMap,  fixture);
    initializationData.erase(fixture);
    vectorErase(placedFixtures, fixture);
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
            map[filenames.size()] = idx;
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
        auto id = map[i];

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
        auto id1 = map[ms.imgA];
        auto id2 = map[ms.imgB];
        bool swap = !(id1 < id2);

        auto idA = swap ? id2 : id1;
        auto idB = swap ? id1 : id2;

        auto& mset = matches[idA][idB];
        for (auto& m: ms.matches)
        {
            int fA = swap ? m.featureB : m.featureA;
            int fB = swap ? m.featureA : m.featureB;
            mset.emplace_back(fA, fB, m.best2ndBest);
        }
    }
}

int ReconstructionFixtureScene::getDistinctCameraCount() const
{
    std::set<FixtureCamera*> cameras;
    for (auto& f: placedFixtures)
    {
        for (auto& c: f->cameras)
        {
            cameras.insert(c);
        }
    }
    return cameras.size();
}
