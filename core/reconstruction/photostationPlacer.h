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

#include "typesafeBitmaskEnums.h"

enum class PhotostationPlacerOptimizationType
{
    NON_DEGENERATE_ORIENTATIONS = 1, // Orientations of all cameras except first
    DEGENERATE_ORIENTATIONS = 2,     // Orientation of first camera
    NON_DEGENERATE_TRANSLATIONS = 4, // Translations of all cameras except first (TODO: Clarify if for noncentral camera we would like to fix "scale")
    DEGENERATE_TRANSLATIONS = 8,     // Translation of first camera
    FOCALS = 16,                     // Camera focals in multicamera
    PRINCIPALS = 32,                 // Camera principals in multicamera
    POINTS = 64,                     // 3D points
    TUNE_GPS = 128                   // Allow shifting of GPS-initialized cameras
};
enum class PhotostationPlacerOptimizationErrorType
{
    REPROJECTION,                    // Reprojection error
    ANGULAR,                         // Angular error
    CROSS_PRODUCT,                    // Cross product error
    RAY_DIFF
};

/*
 * Parameters are stored in the following way (section is omitted depending on PhotostationPlacerOptimizationType)
 * N - number of "preplaced" photostations
 * M - number of points
 * M'- number of projections
 * K - number of cameras in photostation
 *
 * 4            Orientation of first multicamera       DEGENERATE_ORIENTATIONS
 * 4 x (N - 1)  Orientations of multicameras           NON_DEGENERATE_ORIENTATIONS
 * 3            Translation of first multicamera       DEGENERATE_TRANSLATIONS
 * 3 x (N - 1)  Translations of multicameras           NON_DEGENERATE_TRANSLATIONS
 * 1 x K        Focal lengths of cameras               FOCALS
 * 2 x K        Principal point projections of cameras PRINCIPALS
 * 3 x M        3D points                              POINTS
 *
 * Output parameters are stored sequentially as all projections of all 3d points
 * 2 x M'       Reprojections                          ANGULAR
 * 1 x M'       Angles between rays                    REPROJECTION
 * 3 x M'       Ray cross products                     CROSS_PRODUCT
 * 2 x M'       Ray differences                        RAY_DIFF
 * If TUNE_GPS is present then position differences are also being calculated and stored as
 * 3 x N        Normalised (using covariance 'square root') difference
 */

template<>
struct is_bitmask<PhotostationPlacerOptimizationType> : std::true_type {};


namespace corecvs
{

struct PhotostationPlacerEssentialFilterParams
{
    double b2bRansacP5RPThreshold = 0.8;
    double inlierP5RPThreshold = 5.0;
    int maxEssentialRansacIterations = 1000000;
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
    int speculativity = 1;

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
    void fullRun();
    corecvs::Mesh3D dumpMesh(const std::string &filename);
    void paintTracksOnImages(bool pairs = false);
    void detectAll();
    bool initialize();
    void create2PointCloud();
    corecvs::Affine3DQ staticInit(CameraFixture* fixture, std::vector<SceneFeaturePoint*> &staticPoints);
    void pruneTracks();
    void appendPs();
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
};
}

#endif
