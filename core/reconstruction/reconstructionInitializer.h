#ifndef RECONSTRUCTIONINITIALIZER_H
#define RECONSTRUCTIONINITIALIZER_H

#include "reconstructionFixtureScene.h"
#include "essentialFeatureFilter.h"

namespace corecvs
{
struct ReconstructionInitializerParams
{
    EssentialFilterParams essentialFilterParams;
    double b2bThreshold = 0.9;
    bool runEssentialFiltering = true;
};
/*
 * This class is responsible for reconstruction initialization.
 *
 * When there is no 3D pointcloud and we cannot use P3P+RANSAC
 * in order to robustly determine pose and position of multicamera,
 * we are trying to detect relative orientation/position using
 * more correspondences.
 */
struct ReconstructionInitializer : ReconstructionInitializerParams
{
    bool initialize();
    bool initGPS();
    bool initNONE();
    bool initSTATIC();
    bool initFIXED();
    void estimateFirstPair();
    void estimatePair(CameraFixture *psA, CameraFixture *psB);
    corecvs::Quaternion detectOrientationFirst(CameraFixture *psA, CameraFixture* psB, CameraFixture *psC);
    ReconstructionFixtureScene *scene;
    static corecvs::Quaternion TransformFrom2RayCorrespondence(corecvs::Vector3dd o1, corecvs::Vector3dd o2, corecvs::Vector3dd e1, corecvs::Vector3dd e2);
};
}

#endif
