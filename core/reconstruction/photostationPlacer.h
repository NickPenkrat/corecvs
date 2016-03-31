#ifndef PHOTOSTATIONPLACER
#define PHOTOSTATIONPLACER

#include <string>
#include <vector>
#include <atomic>

#include "vector3d.h"
#include "calibrationPhotostation.h"
#include "reconstructionStructs.h"
#include "reconstructionFixtureScene.h"
#include "levenmarq.h"
#include "mesh3d.h"
#include "reconstructionFunctor.h"

#ifdef WITH_TBB
#include "tbb/mutex.h"
#endif

namespace corecvs
{

struct PhotostationPlacerEssentialFilterParams
{
    double b2bRansacP5RPThreshold = 0.8;
    double inlierP5RPThreshold = 5.0;
    int maxEssentialRansacIterations = 100000;
    double b2bRansacP6RPThreshold = 0.8;
    bool runEssentialFiltering = true;
    double essentialTargetGamma = 0.001;

    template<typename V>
    void accept(V &v)
    {
        v.visit(b2bRansacP5RPThreshold, 0.8, "Best-2nd best essential estimator threshold");
        v.visit(inlierP5RPThreshold, 5.0, "Inlier threshold");
        v.visit(maxEssentialRansacIterations, 1000, "Maximal essential estimator rounds");
        v.visit(b2bRansacP6RPThreshold, 0.8, "Best-2nd best relative pose estimator threshold");
        v.visit(runEssentialFiltering, false, "Run essential filtering prior relative pose estimation");
    }
};

struct PhotostationPlacerFeatureSelectionParams
{
    double inlierThreshold = 1.0;
    double trackInlierThreshold = 3;
    double pairCorrespondenceThreshold = 0.25;
    double distanceLimit =1000.0;
//    int nonLinearAfterAppend = 40;
//    int nonLinearAfterAdd    = 40;
//    int nonLinearFinal       =100;
//    bool tuneFocal = false;

    template<typename V>
    void accept(V &v)
    {
        v.visit(inlierThreshold, 1.0, "Inlier threshold");
        v.visit(trackInlierThreshold, 3.0, "Track append threshold");
        v.visit(pairCorrespondenceThreshold, 0.25, "Correspondence threshold");
        v.visit(distanceLimit, 1000.0, "Track distance limit");
    }
};

struct PhotostationPlacerParams
{
    bool forceGps = true;
    PhotostationPlacerOptimizationType optimizationParams =
        PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS |
        PhotostationPlacerOptimizationType::POINTS;// | PhotostationPlacerOptimizationType::FOCALS;
    PhotostationPlacerOptimizationErrorType errorType = PhotostationPlacerOptimizationErrorType::RAY_DIFF;
    // This defines how many multicameras are subject for P3P evaluation at each iteration
    size_t speculativity = 1000;
    size_t minimalInlierCount = 20;
    double maximalFailureProbability = 0.15;

    template<typename V>
    void accept(V &v)
    {
        v.visit(forceGps, true, "Enforce gps locations");
        auto vv = static_cast<typename std::underlying_type<PhotostationPlacerOptimizationType>::type>(optimizationParams);
        v.visit(vv, 0, "Optimization type");
        optimizationParams = static_cast<PhotostationPlacerOptimizationType>(vv);
        auto vvv = static_cast<typename std::underlying_type<PhotostationPlacerOptimizationErrorType>::type>(errorType);
        v.visit(vvv, 0, "Error type");
        errorType = static_cast<PhotostationPlacerOptimizationErrorType>(vvv);
    }
};

class PhotostationPlacer :    public PhotostationPlacerEssentialFilterParams, public PhotostationPlacerFeatureSelectionParams, public PhotostationPlacerParams
{
public:
    ReconstructionFixtureScene* scene;
    void testNewPipeline();
    void fullRun();
    corecvs::Mesh3D dumpMesh(const std::string &filename);
    void paintTracksOnImages(bool pairs = false);
    void detectAll();
    bool initialize();
    void create2PointCloud();
    corecvs::Affine3DQ staticInit(CameraFixture* fixture, std::vector<SceneFeaturePoint*> &staticPoints);
    void pruneTracks();
    bool appendPs();
    void fit(const PhotostationPlacerOptimizationType& optimizationSet = PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS, int num = 100);
    void fit(bool tuneFocal);
    void appendTracks(const std::vector<int> &inlierIds, CameraFixture* fixture, const std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> &possibleTracks);
    int getMovablePointCount();
    int getReprojectionCnt();
    int getInputNum();
    int getOutputNum();
    int getErrorComponentsPerPoint();
    void getErrorSummary(PhotostationPlacerOptimizationErrorType errorType);
    void getErrorSummaryAll();

    // Tries to append f using P6P (with 2d<->2d correspondences)
    bool appendP6P();
    // Tries to append f using P3P (with 3d<->2d correspondences
    bool appendP3P(CameraFixture* f);

    void addFirstPs();
    void addSecondPs();
    void tryAlign();
protected:
    void readOrientationParams(const double in[]);
    void writeOrientationParams(double out[]);
    void computeErrors(double out[], const std::vector<int> &idxs);
    std::vector<std::vector<int>> getDependencyList();

    /*
     * These are valid only in non-linear fit time
     */
    std::vector<FixtureCamera*> activeCameras;
    std::vector<SceneObservation*> revDependency; // track, projection
    std::vector<CameraFixture*> gpsConstrainedCameras;
    std::vector<std::vector<int>> sparsity;
    void prepareNonLinearOptimizationData();
    void buildDependencyList();
    double scalerPoints, scalerGps;
    int inputNum, outputNum, psNum, camNum, ptNum, projNum, gpsConstraintNum, staticNum;

    std::unordered_map<corecvs::CameraFixture*, corecvs::Affine3DQ> activeEstimates;
    std::unordered_map<corecvs::CameraFixture*, std::vector<int>> activeInlierCount;
    void updateTrackables();

    struct ParallelErrorComputator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const;
        ParallelErrorComputator(PhotostationPlacer* placer, const std::vector<int> &idxs, double* output) : placer(placer), idxs(idxs), output(output)
        {
        }
        PhotostationPlacer* placer;
        std::vector<int> idxs;
        double* output;
    };
    struct OrientationFunctor : corecvs::SparseFunctionArgs
    {
        void operator() (const double in[], double out[], const std::vector<int> &idxs)
        {
            placer->readOrientationParams(in);
            placer->computeErrors(out, idxs);
        }
        OrientationFunctor(PhotostationPlacer *placer) : SparseFunctionArgs(placer->getInputNum(), placer->getOutputNum(), placer->sparsity), placer(placer)
        {
        }
        PhotostationPlacer* placer;
    };
    struct OrientationNormalizationFunctor : corecvs::FunctionArgs
    {
        OrientationNormalizationFunctor(PhotostationPlacer *placer) : FunctionArgs(placer->getInputNum(), placer->getInputNum()), placer(placer)
        {
        }
        void operator() (const double in[], double out[])
        {
            placer->readOrientationParams(in);
            placer->writeOrientationParams(out);
        }
        PhotostationPlacer* placer;
    };
#ifdef WITH_TBB
    tbb::mutex mutex;
#endif
};
}

#endif
