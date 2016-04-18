#ifndef RECONSTRUCTIONFUNCTOR
#define RECONSTRUCTIONFUNCTOR

#include "typesafeBitmaskEnums.h"
#include "tbbWrapper.h"
#include "function.h"
#include "reconstructionFixtureScene.h"

namespace corecvs
{
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

struct ReconstructionFunctor;
struct ParallelErrorComputator
{
    void operator() (const corecvs::BlockedRange<int> &r) const;
    ParallelErrorComputator(ReconstructionFunctor *functor, const std::vector<int> &idxs, double* output) : functor(functor), idxs(idxs), output(output)
    {
    }
    ReconstructionFunctor* functor;
    std::vector<int> idxs;
    double* output;
};
struct ReconstructionFunctor : corecvs::SparseFunctionArgs
{
    void operator() (const double in[], double out[], const std::vector<int> &idxs)
    {
        readParams(in);
        computeErrors(out, idxs);
    }
    ReconstructionFunctor(ReconstructionFixtureScene *scene, const PhotostationPlacerOptimizationErrorType &error, const PhotostationPlacerOptimizationType &optimization, const double pointErrorEstimate);

    ReconstructionFixtureScene *scene;
    PhotostationPlacerOptimizationErrorType error;
    PhotostationPlacerOptimizationType optimization;

    void computePointCounts();
    void computeInputs();
    int getInputNum();
    void computeOutputs();
    int getOutputNum();
    void computeDependency();

    void readParams(const double *params);
    void writeParams(double *params);
    void computeErrors(double *out, const std::vector<int> &idxs);

    std::unordered_map<FixtureScenePart*, int> counter;

    std::vector<SceneObservation*> revDependency; // track, projection
    std::vector<CameraFixture*> positionConstrainedCameras;
    std::vector<std::vector<int>> sparsity;

    double scalerPoints, scalerPosition;
    int lastProjection;
    int getErrorComponentsPerPoint();

    std::vector<CameraFixture*> orientableFixtures, translateableFixtures, translationConstrainedFixtures;
    std::vector<FixtureCamera*> focalTunableCameras, principalTunableCameras;
    /*
     * These are DoF limits for cameras/fixtures
     * TODO: Check if this stuff is valid
     *
     * A) orientable fixtures       (>= 3 3d-points)
     * B) translatable fixtures     (>= 3 3d-points)
     * C) focal-tunable cameras     (>= 4 3d-points)
     * D) principal-tunable cameras (>= 6 3d-points)
     */
    const int    MINIMAL_TRACKED_FOR_ORIENTATION = 3,
                 MINIMAL_TRACKED_FOR_TRANSLATION = 3,
                 MINIMAL_TRACKED_FOR_FOCALS      = 4,
                 MINIMAL_TRACKED_FOR_PRINCIPALS  = 6,
    // Inputs/outputs per item
                 INPUTS_PER_ORIENTATION          = 4,
                   INPUTS_PER_TRANSLATION          = 3,
                   INPUTS_PER_3D_POINT             = 3,
                   INPUTS_PER_FOCAL                = 1,
                   INPUTS_PER_PRINCIPAL            = 2,
                   OUTPUTS_PER_POSITION_CONSTRAINT = 3;

};
struct ReconstructionNormalizationFunctor : corecvs::FunctionArgs
{
    ReconstructionNormalizationFunctor(ReconstructionFunctor *functor) : FunctionArgs(functor->getInputNum(), functor->getInputNum()), functor(functor)
    {
    }
    void operator() (const double in[], double out[])
    {
        functor->readParams(in);
        functor->writeParams(out);
    }
    ReconstructionFunctor* functor;
};

}


#endif