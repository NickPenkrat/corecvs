#ifndef RECONSTRUCTIONFIXTURESCENE
#define RECONSTRUCTIONFIXTURESCENE

#include "fixtureScene.h"

#include "rgbColor.h"


namespace corecvs
{
struct FeatureDetectionParams
{
    std::string detector   = "SURF";
    std::string descriptor = "SURF";
    std::string matcher    = "ANN";
    double b2bThreshold = 0.9;
};

enum class ReconstructionState
{
    NONE,          // Before any manipulations
    MATCHED,       // Image matches are computed
    INITALIZED,    // Either inital positions of two fixtures are calculated (static point/none initialization), or 3 fixtures are positioned and oriented (gps initialization)
    TWOPOINTCLOUD, // Pointcloud created by 2-view matches. It will be pruned after appending next fixture
    APPENDABLE,     // >= 3 fixtures are positioned, pointcloud consists from 3+ view matches
    FINISHED
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

    // ==================================================================================
    // SERIALIZEABLE STATE
    std::vector<SceneFeaturePoint*> trackedFeatures;
    umwpp<std::string> images;
    umwppv<std::pair<corecvs::Vector2dd, corecvs::RGBColor>> keyPoints;
    umwpp<umwppv<std::tuple<int, int, double>>> matches;
    umwpp<std::unordered_map<int, int>> trackMap;
    std::unordered_map<CameraFixture*, int> initializationData;
    std::vector<CameraFixture*> placedFixtures;
    std::vector<CameraFixture*> placingQueue;

    bool is3DAligned = false;
    ReconstructionState state = ReconstructionState::NONE;
    // ==================================================================================
};
}

#endif

