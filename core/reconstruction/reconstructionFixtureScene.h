#ifndef RECONSTRUCTIONFIXTURESCENE
#define RECONSTRUCTIONFIXTURESCENE

#include "fixtureScene.h"

#include "rgbColor.h"


namespace corecvs
{
struct FeatureDetectionParams
{
};
class ReconstructionFixtureScene : public FixtureScene
{
public:
    ReconstructionFixtureScene();
    
    virtual void deleteCamera        (FixtureCamera *camera);
    virtual void deleteCameraFixture (CameraFixture *fixture, bool recursive = true);
    virtual void deleteFixturePair   (CameraFixture *fixture, FixtureCamera *camera);
    virtual void deleteFeaturePoint  (SceneFeaturePoint *camera);

    //\brief Detect and match features between all images
    void detectAllFeatures(const FeatureDetectionParams &params);

    //\brief Returns number of FixtureCamera's in placedFixtures fixtures
    int getDistinctCameraCount() const;


    std::vector<SceneFeaturePoint*> trackedFeatures;
    umwpp<std::string> images;
    umwppv<std::pair<corecvs::Vector2dd, corecvs::RGBColor>> keyPoints;
    umwpp<umwppv<std::tuple<int, int, double>>> matches;
    umwpp<std::unordered_map<int, int>> trackMap;
    std::unordered_map<CameraFixture*, int> initializationData;
    std::vector<CameraFixture*> placedFixtures;
    std::vector<CameraFixture*> placingQueue;

};
}

#endif

