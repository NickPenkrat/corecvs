#ifndef RECONSTRUCTIONFIXTURESCENE
#define RECONSTRUCTIONFIXTURESCENE

#include "fixtureScene.h"

#include "rgbColor.h"
#include "essentialFeatureFilter.h"


namespace corecvs {

struct FeatureDetectionParams
{
    std::string detector   = "ORB";
    std::string descriptor = "ORB";
    std::string matcher    = "BF";
    double b2bThreshold = 0.9;
    bool matchF2F = false;
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

enum class PhotostationInitializationType
{
    NONE,
    GPS,
    STATIC,
    FIXED
};

struct PhotostationInitialization
{
    PhotostationInitializationType initializationType;
    // NOTE: Static points should be appended to scene before
    //       supplying them as initialization data
    std::vector<SceneFeaturePoint*> staticPoints;
    Affine3DQ initData;
    /*
     * NOTE: This values seems to be accurate only if we are dealing
     *       with measured fixtures.
     *       If we perform picture collection and measurements in different
     *       moments (and do it crudely), then we should use less modest
     *       error estimates.
     *       This matrix represents "invers square root" from covariance estimate
     *       for position accuracy (if your covariance matrix decomposes into
     *       V'DV, then we are looking for (1/sqrt(D))*V
     */
    Matrix33  positioningAccuracy = corecvs::Matrix33(0.005, 0, 0, 0, 0.005, 0, 0, 0, 0.005).inv();
    bool enforcePosition = true;
    double    rotationalAccuracy;
};


class ReconstructionFixtureScene : public FixtureScene
{
public:
    ReconstructionFixtureScene();

    virtual void deleteCamera        (FixtureCamera *camera);
    virtual void deleteCameraFixture (CameraFixture *fixture, bool recursive = true);
    virtual void deleteFixturePair   (CameraFixture *fixture, FixtureCamera *camera);
    virtual void deleteFeaturePoint  (SceneFeaturePoint *camera);

    FixtureScene* dumbify();

    //\brief Detect and match features between all images
    void detectAllFeatures(const FeatureDetectionParams &params);
    std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> getPossibleTracks(CameraFixture* ps);
    void buildTracks(CameraFixture *psA, CameraFixture *psB, CameraFixture *psC, double trackInlierThreshold, double distanceLimit);
    std::unordered_map<std::tuple<FixtureCamera*, FixtureCamera*, int>, int> getUnusedFeatures(CameraFixture *psA, CameraFixture *psB);
    std::vector<std::tuple<WPP, corecvs::Vector2dd, WPP, corecvs::Vector2dd, double>> getPhotostationMatches(const std::vector<CameraFixture*> &train, CameraFixture *query);
    void filterEssentialRansac(WPP a, WPP b, EssentialFilterParams params);
    void filterEssentialRansac(const std::vector<CameraFixture*> &lhs, const std::vector<CameraFixture*> &rhs, EssentialFilterParams params);
    void remove(WPP a, WPP b, std::vector<int> idx);
    void pruneTracks(double threshold);


    //\brief Returns number of FixtureCamera's in placedFixtures fixtures
    int getDistinctCameraCount() const;
    std::vector<FixtureCamera*> getDistinctCameras() const;

    void printMatchStats() const;
    void printMatchStats(const umwpp<umwppv<std::tuple<int, int, double>>> &m);

    // ==================================================================================
    // SERIALIZEABLE STATE
    std::vector<SceneFeaturePoint*> trackedFeatures, staticPoints;
    umwpp<std::string> images;
    umwppv<std::pair<corecvs::Vector2dd, corecvs::RGBColor>> keyPoints;
    umwpp<umwppv<std::tuple<int, int, double>>> matches;
    umwpp<std::unordered_map<int, SceneFeaturePoint*>> trackMap;
    std::unordered_map<CameraFixture*, PhotostationInitialization> initializationData;
    std::vector<CameraFixture*> placedFixtures;
    std::vector<CameraFixture*> placingQueue;

    bool is3DAligned = false;
    ReconstructionState state = ReconstructionState::NONE;
    // ==================================================================================
    umwpp<umwppv<std::tuple<int, int, double>>> matchesCopy;

    bool validateMatches();
    bool validateTracks();
    bool validateAll();

    bool haveCamera(FixtureCamera* camera);
    bool haveFixture(CameraFixture* camera);
    bool havePoint(SceneFeaturePoint* point);

    void printMatchStats();
    void printTrackStats();
    void printPosStats();

    void transform(const corecvs::Affine3DQ &transform, const double scale = 1.0);

    friend std::ostream& operator<< (std::ostream& os, ReconstructionFixtureScene &rfs)
    {
        std::vector<CameraFixture*> fSorted;
        for (auto ptr: rfs.fixtures())
            fSorted.push_back(ptr);
        std::sort(fSorted.begin(), fSorted.end(), [](const CameraFixture* a, const CameraFixture* b) { return a->name < b->name; });

        for (auto ptr: fSorted)
        {
            std::vector<FixtureCamera*> cSorted;
            for (auto ptC: ptr->cameras)
                cSorted.push_back(ptC);

            std::sort(cSorted.begin(), cSorted.end(), [](const FixtureCamera* a, const FixtureCamera* b) { return a->nameId < b->nameId; });

            os << ptr->name << " " << rfs.initializationData[ptr].initData.shift << std::endl;
            for (auto c: cSorted)
                os << "\t" << c->nameId << " " << rfs.images[WPP(ptr, c)] << std::endl;
        }
        return os;
    }

private:
    struct ParallelEssentialFilter
    {
        ReconstructionFixtureScene* scene;
        std::vector<std::pair<WPP, WPP>> work;
        EssentialFilterParams params;
        ParallelEssentialFilter(ReconstructionFixtureScene* scene, std::vector<std::pair<WPP, WPP>> &work, EssentialFilterParams params) : scene(scene), work(work), params(params)
        {
        }
        void operator() (const corecvs::BlockedRange<int> &r) const
        {
            for (int i = r.begin(); i < r.end(); ++i)
            {
                auto w = work[i];
                scene->filterEssentialRansac(w.first, w.second, params);
            }
        }
    };

};

struct ParallelTrackPainter
{
    ParallelTrackPainter(
            std::vector<std::pair<WPP, std::string>> &images,
            ReconstructionFixtureScene* scene,
            std::unordered_map<SceneFeaturePoint*, RGBColor> colorizer, bool pairs = false) :
            colorizer(&colorizer)
          , images(images)
          , scene(scene), pairs(pairs)
    {
    }

    void operator() (const corecvs::BlockedRange<int> &r) const;

    std::unordered_map<SceneFeaturePoint*, RGBColor> *colorizer;
    std::vector<std::pair<WPP, std::string>> images;
    ReconstructionFixtureScene* scene;
    bool pairs;
};

} // namespace corecvs

namespace std
{
template<>
struct hash<PhotostationInitializationType>
{
    size_t operator() (const PhotostationInitializationType &t) const
    {
        using foo = std::underlying_type<PhotostationInitializationType>::type;
        return hash<foo>()(static_cast<const foo>(t));
    }
};

}

#endif // RECONSTRUCTIONFIXTURESCENE
