/**
 * \file mergerThread.cpp
 * \brief Implements a frame recording calculation thread based on BaseCalculationThread
 *
 * \date Sep 17, 2010
 * \author Sergey Levi
 */

#include "mergerThread.h"

#include <stdio.h>
#include <QMetaType>
#include <QMessageBox>

#include "g12Image.h"
#include "imageResultLayer.h"

#include "fixtureScene.h"
#include "calibrationDrawHelpers.h"

// TEST
// #include "viFlowStatisticsDescriptor.h"

MergerThread::MergerThread() :
   BaseCalculationThread(Frames::MAX_INPUTS_NUMBER)
  , mMergerParameters(NULL)
{
    qRegisterMetaType<MergerThread::RecordingState>("MergerThread::RecordingState");
    mIdleTimer = PreciseTimer::currentTime();
}


AbstractOutputData* MergerThread::processNewData()
{
    Statistics stats;

   qDebug("MergerThread::processNewData(): called for %d inputs", mActiveInputsNumber);

#if 0
    stats.setTime(ViFlowStatisticsDescriptor::IDLE_TIME, mIdleTimer.usecsToNow());
#endif

    PreciseTimer start = PreciseTimer::currentTime();
//    PreciseTimer startEl = PreciseTimer::currentTime();

    bool have_params = !(mMergerParameters.isNull());
    bool two_frames = have_params && (CamerasConfigParameters::TwoCapDev == mActiveInputsNumber); // FIXME: additional params needed here

    // We are missing data, so pause calculation
    if ((!mFrames.getCurrentFrame(Frames::LEFT_FRAME) ) ||
       ((!mFrames.getCurrentFrame(Frames::RIGHT_FRAME)) && (CamerasConfigParameters::TwoCapDev == mActiveInputsNumber)))
    {
        emit errorMessage("Capture error.");
        pauseCalculation();
    }

    recalculateCache();

    G12Buffer *result[Frames::MAX_INPUTS_NUMBER];
    for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; ++i) {
        result[i] = NULL;
    }

    /*TODO: Logic here should be changed according to the host base change*/
    for (int id = 0; id < mActiveInputsNumber; id++)
    {
        G12Buffer   *buf    = mFrames.getCurrentFrame   ((Frames::FrameSourceId)id);
        RGB24Buffer *bufrgb = mFrames.getCurrentRgbFrame((Frames::FrameSourceId)id);
        if (bufrgb != NULL) {
            buf = bufrgb->toG12Buffer();
        }

        //result[id] = mTransformationCache[id] ? mTransformationCache[id]->doDeformation(mBaseParams->interpolationType(), buf) : buf;
        result[id] = buf;
    }
#if 0
    stats.setTime(ViFlowStatisticsDescriptor::CORRECTON_TIME, startEl.usecsToNow());
#endif

    MergerOutputData* outputData = new MergerOutputData();

    SYNC_PRINT(("MergerThread::processNewData(): G12: ["));
    for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; i++)
    {
         SYNC_PRINT(("%s", result[i] == NULL ? "-" : "+"));
    }
    SYNC_PRINT(("]\n"));

    outputData->mMainImage.addLayer(
            new ImageResultLayer(
                    mPresentationParams->output(),
                    result
            )
    );

    outputData->mMainImage.setHeight(mBaseParams->h());
    outputData->mMainImage.setWidth (mBaseParams->w());

#if 1
    stats.setTime("Total time", start.usecsToNow());
#endif
    mIdleTimer = PreciseTimer::currentTime();

    for (int id = 0; id < mActiveInputsNumber; id++)
    {
        if (result[id] != mFrames.getCurrentFrame((Frames::FrameSourceId)id)) {
             delete_safe(result[id]);
        }
    }

    outputData->mainOutput = new RGB24Buffer(1000,1000);
    outputData->mainOutput->checkerBoard(20, RGBColor::Black(), RGBColor::White());

    /* Scene */
    FixtureScene *autoScene = new FixtureScene;
    CameraFixture *body = autoScene->createCameraFixture();
    body->setLocation(Affine3DQ::Identity());
    body->name = "Car Body";

    PinholeCameraIntrinsics pinhole(Vector2dd(640, 480), degToRad(60));



    FixtureCamera *frontCam = autoScene->createCamera();
    FixtureCamera *rightCam = autoScene->createCamera();
    FixtureCamera *backCam  = autoScene->createCamera();
    FixtureCamera *leftCam  = autoScene->createCamera();

    frontCam->intrinsics = pinhole;
    rightCam->intrinsics = pinhole;
    backCam ->intrinsics = pinhole;
    leftCam ->intrinsics = pinhole;

    double groundZ = 200;

    autoScene->positionCameraInFixture(body, frontCam, Affine3DQ::Shift(-8.57274,     0    , 37.30)                                          * Affine3DQ::RotationY(degToRad(25)));
    autoScene->positionCameraInFixture(body, backCam , Affine3DQ::Shift(28.00267,  -3.00   , 91.0712)  * Affine3DQ::RotationZ(degToRad(180)) * Affine3DQ::RotationY(degToRad(45)));
    autoScene->positionCameraInFixture(body, leftCam , Affine3DQ::Shift(18.88999, -10.37   , 81.20)    * Affine3DQ::RotationZ(degToRad( 90)));
    autoScene->positionCameraInFixture(body, rightCam, Affine3DQ::Shift(18.88999,  10.37   , 81.20)    * Affine3DQ::RotationZ(degToRad(270)));

    outputData->visualisation = new Mesh3DDecorated;
    outputData->visualisation->switchColor(true);

    CalibrationDrawHelpers drawer;
    drawer.drawScene(*outputData->visualisation, *autoScene, 3);

    outputData->frameCount = this->mFrameCount;
    outputData->stats = stats;

    delete_safe(autoScene);
    return outputData;
}

/*

  switch (id) {
    case OC_E_FRONT_VIEW_CAMERA:
    {
      inputName = "front";

      translate.tx = -3900.0;
      translate.ty = 0.0;
      translate.tz = groundZ;

      cam.extrinsic.pos.tx = -857.274;
      cam.extrinsic.pos.tz = 373.0;
      cam.extrinsic.rot.alpha = (tFloat)-25.0 / 180.0 * c_D_PI_f32;
    } break;
    case OC_E_REAR_VIEW_CAMERA:
    {
      inputName = "rear";
      translate.tx = 7000.0;
      translate.ty = 0.0;
      translate.tz = groundZ;

      cam.extrinsic.pos.tx =  2800.267;
      cam.extrinsic.pos.ty = -300.194;
      cam.extrinsic.pos.tz =  910.712;
      cam.extrinsic.rot.alpha = (tFloat)-43.0 / 180.0 * c_D_PI_f32;
    } break;
    case OC_E_LEFT_SIDE_VIEW_CAMERA:
    {
      inputName = "left";

      translate.tx = 888.0;
      translate.ty = -2300.0;
      translate.tz = groundZ;

      cam.extrinsic.pos.tx = 1888.999;
      cam.extrinsic.pos.ty = -1037;
      cam.extrinsic.pos.tz = 812.0;
      cam.extrinsic.rot.alpha = (tFloat)-77.3 / 180.0 * c_D_PI_f32;
      cam.extrinsic.rot.beta = (tFloat) -4.0 / 180.0 * c_D_PI_f32;
    } break;
    case OC_E_RIGHT_SIDE_VIEW_CAMERA:
    {
      inputName = "right";
      translate.tx = 888.0;
      translate.ty = 2300.0;
      translate.tz = groundZ;

      cam.extrinsic.pos.tx = 1888.999;
      cam.extrinsic.pos.ty = 1037;
      cam.extrinsic.pos.tz = 812.0;
      cam.extrinsic.rot.alpha = (tFloat)-81.5 / 180.0 * c_D_PI_f32;

    } break;

*/


void MergerThread::mergerControlParametersChanged(QSharedPointer<Merger> mergerParameters)
{
    if (!mergerParameters)
        return;

    mMergerParameters = mergerParameters;
    mPath = QString(mergerParameters->path().c_str()) + "/" + QString(mergerParameters->fileTemplate().c_str());
}

void MergerThread::baseControlParametersChanged(QSharedPointer<BaseParameters> params)
{
    BaseCalculationThread::baseControlParametersChanged(params);
}

void MergerThread::camerasParametersChanged(QSharedPointer<CamerasConfigParameters> parameters)
{
    BaseCalculationThread::camerasParametersChanged(parameters);
}
