#ifndef PHOTOSTATIONPLACER_H_
#define PHOTOSTATIONPLACER_H_

#include <string>
#include <vector>
#include <atomic>
#include <functional>

#include "vector3d.h"
#include "calibrationPhotostation.h"
#include "reconstructionStructs.h"
#include "reconstructionFixtureScene.h"
#include "levenmarq.h"
#include "mesh3d.h"
#include "reconstructionFunctor.h"
#include "iterativeReconstructionSettings.h"

#ifdef WITH_TBB
#include "tbb/mutex.h"
#endif

namespace corecvs
{

class PhotostationPlacer :    public IterativeReconstructionInitializationParams, public IterativeReconstructionFeatureSelectionParams, public IterativeReconstructionNonlinearOptimizationParams, public IterativeReconstructionAppendParams
{
public:
    ReconstructionFixtureScene* scene;
    void fullRun();
    corecvs::Mesh3D dumpMesh(const std::string &filename);
    void paintTracksOnImages(bool pairs = false);
    void detectAll();
    void create2PointCloud();
    void create2PointCloud(CameraFixture* A, CameraFixture* B);
    corecvs::Affine3DQ staticInit(CameraFixture* fixture, std::vector<SceneFeaturePoint*> &staticPoints);
    void pruneTracks();
    bool append3D();

    void fit(int num);
    void fit(const ReconstructionFunctorOptimizationType& optimizationSet = ReconstructionFunctorOptimizationType::NON_DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::POINTS | ReconstructionFunctorOptimizationType::FOCALS | ReconstructionFunctorOptimizationType::PRINCIPALS, int num = 100);
    void appendTracks(const std::vector<int> &inlierIds, CameraFixture* fixture, const std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> &possibleTracks);
    int getInputNum();
    int getOutputNum();
    void getErrorSummary(ReconstructionFunctorOptimizationErrorType errorType);
    void getErrorSummaryAll();

    // Tries to select 2 fixtures and align them
    void initialize();
    // Typical post-append actions
    void postAppend();
    // Appends 3D-tracks with new correspondences
    void appendTracks();
    // Creates new 3D-tracks
    void createTracks();

    // Tries to append f using P6P (with 2d<->2d correspondences)
    bool append2D();
    // Tries to append f using P3P (with 3d<->2d correspondences
    bool appendP3P(CameraFixture* f);

    std::function<void()> postAppendHook = [](){};
protected:
    std::unordered_map<corecvs::CameraFixture*, corecvs::Affine3DQ> activeEstimates;
    std::unordered_map<corecvs::CameraFixture*, std::vector<int>> activeInlierCount;
    std::unordered_map<corecvs::CameraFixture*, corecvs::Affine3DQ> activeP6PEstimates;
    void updateTrackables();

#ifdef WITH_TBB
    tbb::mutex mutex;
#endif
};
}

#endif //PHOTOSTATIONPLACER_H_
