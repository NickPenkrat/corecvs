#ifndef RECONSTRUCTIONFIXTURESCENE
#define RECONSTRUCTIONFIXTURESCENE

#include "fixtureScene.h"

#include "rgbColor.h"
#include "essentialFeatureFilter.h"

#include "generated/featureDetectionParams.h"

namespace corecvs {

struct Combination
{
    Combination(int M = 0, int N = 0) : M(M > N / 2 ? N - M : M), N(N), inverse(M > N / 2)
    {
        CORE_ASSERT_TRUE_S((M <= N && M >= 0 && N > 0) || (M == 0 && N == 0));
        for (int i = 0; i < this->M; ++i)
            ids.push_back(i);
    }
    Combination& operator++()
    {
        for (int i = 1; i <= M; ++i)
            if (ids[M - i] < N - i)
            {
                ++ids[M - i];
                for (int j = M - i + 1; j < M; ++j)
                    ids[j] = ids[j - 1] + 1;
                return *this;
            }
        *this = Combination();
        return *this;
    }
    bool operator!=(const Combination &that)
    {
        if (that.M != M || that.N != N)
            return true;
        if (that.inverse == inverse)
        {
            for (int i = 0; i < M; ++i)
                if (ids[i] != that.ids[i])
                    return true;
            return false;
        }
        int ptrL = 0, ptrR = 0;
        while (ptrL < M && ptrR < that.M)
        {
            if (ids[ptrL] == ids[ptrR])
                return true;
            if (ids[ptrL] < ids[ptrR])
                ++ptrL;
            else
                ++ptrR;
        }
        return false;
    }
    Combination  operator++(int)
    {
        Combination ret = *this;
        ++(*this);
        return ret;
    }
    const std::vector<int>& operator*() const
    {
        if (!inverse)
            return ids;
        scratchpad.resize(N - M);
        int last = -1;
        int id = 0;
        for (int i = 0; i < M; ++i)
        {
            for (int j = last + 1;  j < ids[i]; ++j)
                scratchpad[id++] = j;
            last = ids[i];
        }
        for (int j = last + 1; j < N; ++j)
            scratchpad[id++] = j;
        CORE_ASSERT_TRUE_S(id == N - M);
        return scratchpad;
    }
    int M, N;
    bool inverse;
    std::vector<int> ids;
    mutable std::vector<int> scratchpad;
};

struct CombinationRange
{
    CombinationRange(int M, int N) : M(M), N(N)
    {
    }
    Combination begin() const
    {
        return Combination(M, N);
    }
    Combination end() const
    {
        return Combination();
    }
    int N, M;
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

enum class FixtureInitializationType
{
    NONE,
    GPS,
    STATIC,
    FIXED
};

struct FixtureInitialization
{
    FixtureInitializationType initializationType;
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
    bool      enforcePosition = true;
    double    rotationalAccuracy;
};


class ReconstructionFixtureScene : public FixtureScene
{
public:
    ReconstructionFixtureScene();
    virtual ~ReconstructionFixtureScene() {}

    virtual void deleteCamera        (FixtureCamera *camera);
    virtual void deleteCameraFixture (CameraFixture *fixture, bool recursive = true);
    virtual void deleteFixturePair   (CameraFixture *fixture, FixtureCamera *camera);
    virtual void deleteFeaturePoint  (SceneFeaturePoint *camera);

    FixtureScene* dumbify();

    //\brief Detect and match features between all images
    void detectAllFeatures(const FeatureDetectionParams &params, bool detectOnly = false);
    std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> getPossibleTracks(CameraFixture* ps);
    // This routine builds tracks from 2D<->2D correspondences
    void buildTracks(CameraFixture *psA, CameraFixture *psB, double trackInlierThreshold, double distanceLimit);
    // This routine appends existing tracks with inlying correspondences
    void appendTracks(CameraFixture *ps, double trackInlierThreshold, double distanceLimit);
    std::unordered_map<std::tuple<FixtureCamera*, FixtureCamera*, int>, int> getUnusedFeatures(CameraFixture *psA, CameraFixture *psB);
    std::vector<std::tuple<WPP, int, WPP, int, double>> getFixtureMatchesIdx(const std::vector<CameraFixture*> &train, CameraFixture *query);
    std::vector<std::tuple<WPP, corecvs::Vector2dd, WPP, corecvs::Vector2dd, double>> getFixtureMatches(const std::vector<CameraFixture*> &train, CameraFixture *query);
    void filterEssentialRansac(WPP a, WPP b, EssentialFilterParams params);
    void filterEssentialRansac(const std::vector<CameraFixture*> &lhs, const std::vector<CameraFixture*> &rhs, EssentialFilterParams params);
    void remove(WPP a, WPP b, std::vector<int> idx);
    void pruneTracks(double rmse, double maxe, double distanceThreshold);
    bool checkTrack(SceneFeaturePoint* track, std::vector<int> *mask = nullptr, double rmse = 1.0, double maxe = 3.0, double distanceThreshold = 1000.0);
//    static std::vector<uint64_t> GenerateBitmasks(int N, int M);
    void pruneSmallTracks();

    void printRAWDetectedPoint(std::string path, std::string fileNameMask, RGBColor color);

    //\brief Returns number of FixtureCamera's in placedFixtures fixtures
    int getDistinctCameraCount() const;
    std::vector<FixtureCamera*> getDistinctCameras() const;

    //void printMatchStats() const;
    //void printMatchStats(const umwpp<umwppv<std::tuple<int, int, double>>> &m);

    // ==================================================================================
    // SERIALIZEABLE STATE
    std::vector<SceneFeaturePoint*> trackedFeatures, staticPoints;
    umwpp<std::string> images;
    umwppv<std::pair<corecvs::Vector2dd, corecvs::RGBColor>> keyPoints;
    umwpp<umwppv<std::tuple<int, int, double>>> matches;
    umwpp<std::unordered_map<int, SceneFeaturePoint*>> trackMap;
    std::unordered_map<CameraFixture*, FixtureInitialization> initializationData;
    std::vector<CameraFixture*> placedFixtures;
    std::vector<CameraFixture*> placingQueue;

    bool is3DAligned = false;
    ReconstructionState state = ReconstructionState::NONE;
    // ==================================================================================
    umwpp<umwppv<std::tuple<int, int, double>>>                     matchesCopy;
    std::unordered_map<std::pair<WPP, WPP>, std::tuple<corecvs::EssentialDecomposition, double, bool>> essentialCache;
    std::unordered_map<corecvs::CameraFixture*, corecvs::Affine3DQ> activeEstimates;
    std::unordered_map<corecvs::CameraFixture*, int>                activeInlierCount;
    std::unordered_map<corecvs::CameraFixture*, corecvs::Affine3DQ> activeP6PEstimates;

    bool validateMatches();
    bool validateTracks();
    bool validateAll();

    bool haveCamera(FixtureCamera* camera);
    bool haveFixture(CameraFixture* camera);
    bool havePoint(SceneFeaturePoint* point);

    void printMatchStats();
    void printTrackStats();
    void printPosStats();

    class Detransformer
    {
    public:
        ~Detransformer();
        Detransformer(ReconstructionFixtureScene *scene = nullptr, const Affine3DQ &transform = Affine3DQ(), const double scale = 1.0, bool transformGt = false);
        Detransformer(Detransformer &&rhs);
        Detransformer& operator=(Detransformer &&rhs);
    private:
        Detransformer(const Detransformer&) = delete;
        Detransformer& operator=(const Detransformer& that) = delete;
        ReconstructionFixtureScene *scene;
        Affine3DQ transform;
        double scale;
        bool transformGt;
    };
    Detransformer transform(const corecvs::Affine3DQ &transform, bool transformGt = true, bool provideDetransformer = false, const double scale = 1.0);
    Detransformer center(Vector3dd &shift, bool transformGt = true, bool provideDetransformer = false);

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
        ReconstructionFixtureScene      *scene;
        std::vector<std::pair<WPP, WPP>> work;
        EssentialFilterParams            params;

        ParallelEssentialFilter(ReconstructionFixtureScene* scene_
                , std::vector<std::pair<WPP, WPP>> &work_
                , EssentialFilterParams params_)
            : scene(scene_), work(work_), params(params_)
        {}

        void operator() (const corecvs::BlockedRange<int> &r) const
        {
            for (int i = r.begin(); i < r.end(); ++i)
            {
                auto& w = work[i];
                scene->filterEssentialRansac(w.first, w.second, params);
            }
        }
    };

    /* Register myself in the fabric */
private:
    static bool dummy;
public:
    static bool staticInit();

};

struct ParallelTrackPainter
{
    ParallelTrackPainter(
            std::vector<std::pair<WPP, std::string>> &images_,
            ReconstructionFixtureScene* scene_,
            std::unordered_map<SceneFeaturePoint*, RGBColor> &colorizer_,
            bool pairs_ = false)
        : colorizer(colorizer_), images(images_), scene(scene_), pairs(pairs_)
    {}

    void operator() (const corecvs::BlockedRange<int> &r) const;

    std::unordered_map<SceneFeaturePoint*, RGBColor> &colorizer;
    std::vector<std::pair<WPP, std::string>>         &images;
    ReconstructionFixtureScene*                       scene;
    bool                                              pairs;
};

} // namespace corecvs

namespace std
{
template<>
struct hash<FixtureInitializationType>
{
    size_t operator() (const FixtureInitializationType &t) const
    {
        using foo = std::underlying_type<FixtureInitializationType>::type;
        return hash<foo>()(static_cast<const foo>(t));
    }
};

}

#endif // RECONSTRUCTIONFIXTURESCENE
