#include "rgb24Buffer.h"
#include "fixtureScene.h"
#include "affine.h"
#include "sceneStereoAlignerBlock.h"
#include "stereoAligner.h"

namespace corecvs {

SceneStereoAlignerBlock::SceneStereoAlignerBlock()
{

}

int SceneStereoAlignerBlock::operator ()()
{
    if (inScene() == NULL)
    {
        SYNC_PRINT(("Fail. No input"));
        return 1;
    }

    if (camera1 == NULL) {
        camera1 = inScene()->getCameraByNumber(inFixture1(), inCamera1());
    }
    if (camera2 == NULL) {
        camera2 = inScene()->getCameraByNumber(inFixture2(), inCamera2());
    }

    if (camera1 == NULL || camera2 == NULL)
    {
        SYNC_PRINT(("Fail. No camera input"));
        return 2;
    }

    Affine3DQ pos1 = camera1->getWorldLocation();
    Affine3DQ pos2 = camera2->getWorldLocation();

    Affine3DQ relativeTransform = pos1.inverted() * pos2;

    EssentialDecomposition decomposition = EssentialDecomposition::FromAffine(relativeTransform);
    EssentialMatrix F = decomposition.operator Matrix33();

    /* We would use existing code, though it is an inverse abstraction.
     * We already know all the geometry, but will be using code based on the Essential Matrix */

    Vector2dd lSize = camera1->intrinsics.size;
    Vector2dd rSize = camera2->intrinsics.size;

    Vector2dd size  = (lSize + rSize) / 2;

    Vector3dd direction = Vector3dd(mParameters.zdirX(), mParameters.zdirY(), mParameters.zdirZ());
    if (mParameters.autoZ())
    {
        printf("Autoguessing best Z\n");
        direction = StereoAligner::getBestZ(F, size /*, GRANULARITY, distL, distR*/);
    }
    cout << "Using Z value: " << direction << endl;

    /**/
    ProjectiveTransform  leftTransform(1.0);
    ProjectiveTransform rightTransform(1.0);

    StereoAligner::getAlignmentTransformation(F, &leftTransform, &rightTransform, direction);


    /*Forming new Fixture*/
    CameraFixture *fixture = inScene()->createCameraFixture();
    fixture->name = "Stereo Fixture";
    fixture->setLocation(pos1);

    FixtureCamera *cam1 = inScene()->createCamera();
    cam1->nameId = "Stereo Camera 1";
    FixtureCamera *cam2 = inScene()->createCamera();
    cam2->nameId = "Stereo Camera 2";

    inScene()->addCameraToFixture(cam1, fixture);
    inScene()->addCameraToFixture(cam2, fixture);
    cam2->extrinsics = CameraLocationData(Affine3DQ::Identity());
    cam2->extrinsics = CameraLocationData(relativeTransform);

    cam1->intrinsics = camera1->intrinsics;
    cam2->intrinsics = camera2->intrinsics;

    /* Forming new images*/

    RGB24Buffer *resImage1 = inImage1()->doReverseDeformationBlTyped<ProjectiveTransform>(&leftTransform);
    RGB24Buffer *resImage2 = inImage2()->doReverseDeformationBlTyped<ProjectiveTransform>(&rightTransform);

    setOutImage1(resImage1);
    setOutImage2(resImage2);


    setOutCamera1(cam1);
    setOutCamera2(cam2);

    return 0;

}

} //namespace corecvs

