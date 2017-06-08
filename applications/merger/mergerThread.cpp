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

#include "legacy/spherical_lut.h"
#include "sphericalCorrectionLUT.h"

// TEST
// #include "viFlowStatisticsDescriptor.h"

MergerThread::MergerThread() :
   BaseCalculationThread(Frames::MAX_INPUTS_NUMBER)
  , mMergerParameters(NULL)
{
    qRegisterMetaType<MergerThread::RecordingState>("MergerThread::RecordingState");
    mIdleTimer = PreciseTimer::currentTime();
}


Affine3DQ getTransform (const EuclidianMoveParameters &input)
{
    return Affine3DQ::Shift(input.x(),input.y(), input.z())
            * Affine3DQ::RotationZ(degToRad(input.gamma()))
            * Affine3DQ::RotationY(degToRad(input.beta()))
            * Affine3DQ::RotationX(degToRad(input.alpha()));

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

    outputData->mainOutput = new RGB24Buffer(mMergerParameters->outSize(), mMergerParameters->outSize());
    outputData->mainOutput->checkerBoard(20, RGBColor::Black(), RGBColor::White());

    /* Unwrap */
    //outputData->unwarpOutput = new RGB24Buffer(fstBuf->getSize());

    vector<Vector2dd> lut;
    for (unsigned i = 0; i < LUT_LEN; i++) {
        lut.push_back(Vector2dd(UnwarpToWarpLUT[i][0], UnwarpToWarpLUT[i][1]));
    }
    RadiusCorrectionLUT radiusLUT(&lut);
    RGB24Buffer *input = mFrames.getCurrentRgbFrame((Frames::FrameSourceId)mMergerParameters->frameToUndist());
    Vector2dd center(input->w / 2.0, input->h / 2.0);
    SphericalCorrectionLUT corrector(center, &radiusLUT);
    //outputData->unwarpOutput = input->doReverseDeformationBl<RGB24Buffer, SphericalCorrectionLUT>(&corrector, input->h, input->w);
    outputData->unwarpOutput = input->doReverseDeformationBlTyped<SphericalCorrectionLUT>(&corrector, input->h, input->w);


    /* Scene */
    SYNC_PRINT(("Forming scene\n"));
    FixtureScene *autoScene = new FixtureScene;
    CameraFixture *body = autoScene->createCameraFixture();
    body->setLocation(Affine3DQ::Identity());
    body->name = "Car Body";

    G12Buffer *fstBuf = mFrames.getCurrentFrame(Frames::LEFT_FRAME);
    PinholeCameraIntrinsics pinhole(Vector2dd(fstBuf->w, fstBuf->h), degToRad(mMergerParameters->fOV()));

    FixtureCamera *frontCam = autoScene->createCamera(); autoScene->addCameraToFixture(frontCam, body);
    FixtureCamera *rightCam = autoScene->createCamera(); autoScene->addCameraToFixture(rightCam, body);
    FixtureCamera *backCam  = autoScene->createCamera(); autoScene->addCameraToFixture(backCam , body);
    FixtureCamera *leftCam  = autoScene->createCamera(); autoScene->addCameraToFixture(leftCam , body);

    FixtureCamera *cams[] = {frontCam, rightCam, backCam, leftCam};

    frontCam->intrinsics = pinhole;
    rightCam->intrinsics = pinhole;
    backCam ->intrinsics = pinhole;
    leftCam ->intrinsics = pinhole;

    frontCam->nameId = "Front";
    rightCam->nameId = "Right";
    backCam ->nameId = "Back";
    leftCam ->nameId = "Left";

    double groundZ = mMergerParameters->groundZ();

#if 0
    autoScene->positionCameraInFixture(body, frontCam, Affine3DQ::Shift(-8.57274,     0    , 37.30)                                        * Affine3DQ::RotationY(degToRad(25)));
    autoScene->positionCameraInFixture(body, backCam , Affine3DQ::Shift(28.00267,  -3.00   , 91.07)  * Affine3DQ::RotationZ(degToRad(180)) * Affine3DQ::RotationY(degToRad(45)));
    autoScene->positionCameraInFixture(body, leftCam , Affine3DQ::Shift(18.88999, -10.37   , 81.20)  * Affine3DQ::RotationZ(degToRad( 90)));
    autoScene->positionCameraInFixture(body, rightCam, Affine3DQ::Shift(18.88999,  10.37   , 81.20)  * Affine3DQ::RotationZ(degToRad(270)));
#endif

#if 0
    autoScene->positionCameraInFixture(body, frontCam, Affine3DQ::Shift(  8.57274,     0    , 3.730)                                         * Affine3DQ::RotationY(degToRad(25)) );
    autoScene->positionCameraInFixture(body, backCam , Affine3DQ::Shift(-28.00267,  -3.00   , 9.107)  * Affine3DQ::RotationZ(degToRad(180))  * Affine3DQ::RotationY(degToRad(43)) );
    autoScene->positionCameraInFixture(body, leftCam , Affine3DQ::Shift(-18.88999,  10.37   , 8.120)  * Affine3DQ::RotationZ(degToRad( 90))  * Affine3DQ::RotationY(degToRad(80)) );
    autoScene->positionCameraInFixture(body, rightCam, Affine3DQ::Shift(-18.88999, -10.37   , 8.120)  * Affine3DQ::RotationZ(degToRad(270))  * Affine3DQ::RotationY(degToRad(80)) * Affine3DQ::RotationX(degToRad(180)));
#endif

    autoScene->positionCameraInFixture(body, frontCam, getTransform(mMergerParameters->pos1()));
    autoScene->positionCameraInFixture(body, rightCam, getTransform(mMergerParameters->pos2()));
    autoScene->positionCameraInFixture(body, backCam , getTransform(mMergerParameters->pos3()));
    autoScene->positionCameraInFixture(body, leftCam , getTransform(mMergerParameters->pos4()));

    PlaneFrame frame;
    double l = mMergerParameters->outPhySize();
    frame.p1 = Vector3dd( l/2, -l/2, groundZ);
    frame.e1 = Vector3dd(  -l,    0,   0    );
    frame.e2 = Vector3dd(   0,    l,   0    );

    RGB24Buffer *out = outputData->mainOutput;
    bool flags[4] = {
        mMergerParameters->switch1(),
        mMergerParameters->switch2(),
        mMergerParameters->switch3(),
        mMergerParameters->switch4(),
    };

    for (int i = 0; i < out->h; i++)
        for (int j = 0; j < out->w; j++)
        {
            Vector2dd p = Vector2dd( (double)j / out->w, (double)i / out->h);
            Vector3dd pos = frame.getPoint(p);

            Vector3dd color(0.0);
            int count = 0;

            for(int c = 0; c < 4; c++ )
            {
                if (!flags[c])
                    continue;
                RGB24Buffer *buffer = mFrames.getCurrentRgbFrame((Frames::FrameSourceId)c);
                Vector2dd prj;

                if (!mMergerParameters->undist())
                {
                    bool visible = cams[c]->projectPointFromWorld(pos, &prj);
                    if (!visible)
                        continue;
                } else {
                    CameraModel m = cams[c]->getWorldCameraModel();
                    Vector3dd relative   = m.extrinsics.project(pos);
                    if (relative.z() < 0)
                        continue;
                    Vector2dd projection = m.intrinsics.project(relative);
                    prj = corrector.map(projection);
                }

                if (buffer->isValidCoord(prj.y(), prj.x())) {
                    color += buffer->element(prj.y(), prj.x()).toDouble();
                    count++;
                }
            }

            if (count != 0) {
                out->element(i,j) = RGBColor::FromDouble(color / count);
            }
        }


    outputData->visualisation = new Mesh3DDecorated;
    outputData->visualisation->switchColor(true);


    CalibrationDrawHelpers drawer;
    drawer.setPrintNames(true);
    drawer.drawScene(*outputData->visualisation, *autoScene, 3);

    outputData->visualisation->addPlaneFrame(frame);

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
}

void MergerThread::baseControlParametersChanged(QSharedPointer<BaseParameters> params)
{
    BaseCalculationThread::baseControlParametersChanged(params);
}

void MergerThread::camerasParametersChanged(QSharedPointer<CamerasConfigParameters> parameters)
{
    BaseCalculationThread::camerasParametersChanged(parameters);
}
