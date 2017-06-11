/**
 * \file mergerThread.cpp
 * \brief Implements a frame recording calculation thread based on BaseCalculationThread
 *
 * \date Sep 17, 2010
 * \author Sergey Levi
 */

#include <memory>
#include "mergerThread.h"

#include <stdio.h>
#include <QMetaType>
#include <QMessageBox>


#include "g12Image.h"
#include "imageResultLayer.h"

#include "bufferFactory.h"
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

MergerThread::~MergerThread()
{
    for (int c = 0; c < 4; c++) {
        delete_safe(mMasks[c]);
    }
    delete_safe(mCarScene);
}

Affine3DQ getTransform (const EuclidianMoveParameters &input)
{
    return    Affine3DQ::Shift(input.x(),input.y(), input.z())
            * Affine3DQ::RotationZ(degToRad(input.gamma()))
            * Affine3DQ::RotationY(degToRad(input.beta()))
            * Affine3DQ::RotationX(degToRad(input.alpha()));
}


/**
    This method processes changes when parameters have changed.
    And updates data structures that do not depend on the frame itself
 **/
void MergerThread::prepareMapping()
{
    if (!recomputeMergerState)
        return;

    SYNC_PRINT(("MergerThread::prepareMapping(): Updating internal data structures\n"));
    recomputeMergerState = false;

    /* Loading masks */
    for (int c = 0; c < 4; c++) {
        delete_safe(mMasks[c]);
    }
    mMasks[0] = BufferFactory::getInstance()->loadRGB24Bitmap("front-mask.bmp");
    mMasks[1] = BufferFactory::getInstance()->loadRGB24Bitmap("right-mask.bmp");
    mMasks[2] = BufferFactory::getInstance()->loadRGB24Bitmap("back-mask.bmp");
    mMasks[3] = BufferFactory::getInstance()->loadRGB24Bitmap("left-mask.bmp");

    for (int c = 0; c < 4; c++)
    {
        if (mMasks[c] == NULL) {
            SYNC_PRINT(("Mask %d is zero. Setting blank\n", c));
            mMasks[c] = new RGB24Buffer(mFrames.getCurrentFrame(Frames::DEFAULT_FRAME)->getSize(), RGBColor::White());
        }
    }




    /* Forming scene */
    delete_safe(mCarScene);
    mCarScene = new FixtureScene;

    CameraFixture *body = mCarScene->createCameraFixture();
    body->setLocation(Affine3DQ::Identity());
    body->name = "Car Body";

    G12Buffer *fstBuf = mFrames.getCurrentFrame(Frames::LEFT_FRAME);
    PinholeCameraIntrinsics pinhole(Vector2dd(fstBuf->w, fstBuf->h), degToRad(mMergerParameters->fOV()));

    FixtureCamera *frontCam = mCarScene->createCamera(); mCarScene->addCameraToFixture(frontCam, body);
    FixtureCamera *rightCam = mCarScene->createCamera(); mCarScene->addCameraToFixture(rightCam, body);
    FixtureCamera *backCam  = mCarScene->createCamera(); mCarScene->addCameraToFixture(backCam , body);
    FixtureCamera *leftCam  = mCarScene->createCamera(); mCarScene->addCameraToFixture(leftCam , body);

    frontCam->intrinsics = pinhole;
    rightCam->intrinsics = pinhole;
    backCam ->intrinsics = pinhole;
    leftCam ->intrinsics = pinhole;

    frontCam->nameId = "Front";
    rightCam->nameId = "Right";
    backCam ->nameId = "Back";
    leftCam ->nameId = "Left";

    mCarScene->positionCameraInFixture(body, frontCam, getTransform(mMergerParameters->pos1()));
    mCarScene->positionCameraInFixture(body, rightCam, getTransform(mMergerParameters->pos2()));
    mCarScene->positionCameraInFixture(body, backCam , getTransform(mMergerParameters->pos3()));
    mCarScene->positionCameraInFixture(body, leftCam , getTransform(mMergerParameters->pos4()));



}


void MergerThread::drawMaskOver(RGB24Buffer *inputRaw, RGB24Buffer *mask)
{
    if (inputRaw == NULL || mask == NULL) {
        return;
    }
    for (int i = 0; i < inputRaw->h; i++)
        for (int j = 0; j < inputRaw->w; j++)
        {
            if (mask->isValidCoord(i,j))
            {
                inputRaw->element(i,j).r() = 255 - mask->element(i,j).r();
            }
        }
}




AbstractOutputData* MergerThread::processNewData()
{
    Statistics stats;

    qDebug("MergerThread::processNewData(): called for %d inputs", mActiveInputsNumber);

    stats.setTime("Idle", mIdleTimer.usecsToNow());
    stats.startInterval();

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
    prepareMapping();

    RGB24Buffer *inputRaw[Frames::MAX_INPUTS_NUMBER];
    for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; ++i) {
        inputRaw[i] = NULL;
    }

    /*TODO: Logic here should be changed according to the host base change*/
    for (int id = 0; id < mActiveInputsNumber; id++)
    {
        G12Buffer   *buf    = mFrames.getCurrentFrame   ((Frames::FrameSourceId)id);
        RGB24Buffer *bufrgb = mFrames.getCurrentRgbFrame((Frames::FrameSourceId)id);
        inputRaw[id] = new RGB24Buffer(bufrgb);
    }

    stats.resetInterval("Legacy Correction");
    MergerOutputData* outputData = new MergerOutputData();

    SYNC_PRINT(("MergerThread::processNewData(): G12: ["));
    for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; i++)
    {
         SYNC_PRINT(("%s", inputRaw[i] == NULL ? "-" : "+"));
    }
    SYNC_PRINT(("]\n"));

    if (mMergerParameters->showMask())
    {
        for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; i++)
        {
            drawMaskOver(inputRaw[i], mMasks[i]);
        }
    }


    outputData->mMainImage.addLayer(
            new ImageResultLayer(
                    mPresentationParams->output(),
                    inputRaw
            )
    );

    outputData->mMainImage.setHeight(mBaseParams->h());
    outputData->mMainImage.setWidth (mBaseParams->w());

    stats.resetInterval("Main Layout");

    for (int id = 0; id < mActiveInputsNumber; id++)
    {       
         delete_safe(inputRaw[id]);
    }


    int hSize = mMergerParameters->outSizeH();
    int wSize = mMergerParameters->outSizeH() / mMergerParameters->outPhySizeW() * mMergerParameters->outPhySizeL();
    outputData->mainOutput = new RGB24Buffer(hSize, wSize);
    outputData->mainOutput->checkerBoard(20, RGBColor::Gray(), RGBColor::White());

    stats.resetInterval("Canvas cleanup");

    /* Unwrap */
    RGB24Buffer *input = mFrames.getCurrentRgbFrame((Frames::FrameSourceId)mMergerParameters->frameToUndist());
    Vector2dd center(input->w / 2.0, input->h / 2.0);

    vector<Vector2dd> lut;
    for (unsigned i = 0; i < LUT_LEN; i++) {
        lut.push_back(Vector2dd(UnwarpToWarpLUT[i][0], UnwarpToWarpLUT[i][1]));
    }
    RadiusCorrectionLUTSq radiusLUTSq(&lut);
    SphericalCorrectionLUTSq correctorSq(center, &radiusLUTSq);

    RadiusCorrectionLUT radiusLUT(lut);
    SphericalCorrectionLUT corrector(center, &radiusLUT);


    //outputData->unwarpOutput = input->doReverseDeformationBl<RGB24Buffer, SphericalCorrectionLUT>(&corrector, input->h, input->w);
    outputData->unwarpOutput = input->doReverseDeformationBlTyped<SphericalCorrectionLUT>(&corrector, input->h, input->w);

    stats.resetInterval("Example unwrap");


    /* Scene */
    CameraFixture *fixture= mCarScene->fixtures().front();
    FixtureCamera *cams[] = {fixture->cameras[0], fixture->cameras[1], fixture->cameras[2], fixture->cameras[3]};

    bool flags[4] = {
        mMergerParameters->switch1(),
        mMergerParameters->switch2(),
        mMergerParameters->switch3(),
        mMergerParameters->switch4(),
    };

    stats.resetInterval("Forming scene");
    double groundZ = mMergerParameters->groundZ();
    PlaneFrame frame;
    double l = mMergerParameters->outPhySizeL();
    double w = mMergerParameters->outPhySizeW();
    frame.p1 = Vector3dd( l/2, -w/2, groundZ);
    frame.e1 = Vector3dd(  -l,    0,   0    );
    frame.e2 = Vector3dd(   0,    w,   0    );

    CameraModel models[4];
    for (int c = 0; c < 4; c++)
    {
        models[c] = cams[c]->getWorldCameraModel();
    }

    RGB24Buffer *out = outputData->mainOutput;

    parallelable_for(0, out->h, [&](const BlockedRange<int>& r)
    {
    for(int i = r.begin(); i < r.end(); i++)
        for (int j = 0; j < out->w; j++)
        {
            Vector2dd p = Vector2dd( (double)j / out->w, (double)i / out->h);
            Vector3dd pos = frame.getPoint(p);

            Vector3dd color(0.0);
            double sum = 0;

            for(int c = 0; c < 4; c++ )
            {
                if (!flags[c])
                    continue;
                RGB24Buffer *buffer = mFrames.getCurrentRgbFrame((Frames::FrameSourceId)c);

                Vector2dd prj;               
                switch (mMergerParameters->undistMethod())
                {
                    case 0:
                    {
                        bool visible = cams[c]->projectPointFromWorld(pos, &prj);
                        if (!visible)
                            continue;
                        break;
                    }
                    case 1:
                    {
                         Vector3dd relative   = models[c].extrinsics.project(pos);
                        if (relative.z() < 0)
                            continue;
                        Vector2dd projection = models[c].intrinsics.project(relative);
                        prj = correctorSq.map(projection);
                        break;
                    }
                    case 2:
                    {
                        Vector3dd relative   = models[c].extrinsics.project(pos);
                        if (relative.z() < 0)
                            continue;
                        Vector2dd projection = models[c].intrinsics.project(relative);
                        prj = corrector.map(projection);
                        break;
                    }
                }

                if (buffer->isValidCoord(prj.y(), prj.x()) && mMasks[c]->isValidCoord(prj.y(), prj.x())) {
                    double weight = mMasks[c]->element(prj.y(), prj.x()).r() / 255.0;
                    color += weight * buffer->element(prj.y(), prj.x()).toDouble();
                    sum += weight;
                }
            }

            if (sum != 0) {
                out->element(i,j) = RGBColor::FromDouble(color / sum);
            }
        }

    });

    stats.resetInterval("Reprojecting");

    for (int c = 0; c < 4; c++)
    {
        Vector2dd proj = frame.projectTo(cams[c]->getWorldLocation().shift);
        cout << "PROJECTED:" << proj << endl;
        proj = proj * Vector2dd(out->w, out->h);
        out->drawCrosshare2(proj.x(), proj.y(), RGBColor::Red());
        

    }

    Vector2d32 sizeRect = { 400, 100 };
    Vector2d32 corner   = { 200, 500 };
    Rectangle32 rect = Rectangle32(corner, sizeRect);
    out->drawRectangle(rect, { 0, 0, 0 }, 0);

    outputData->visualisation = new Mesh3DDecorated;
    outputData->visualisation->switchColor(true);


    CalibrationDrawHelpers drawer;
    drawer.setPrintNames(true);
    drawer.drawScene(*outputData->visualisation, *mCarScene, 3);


    outputData->visualisation->addPlaneFrame(frame);

    outputData->frameCount = this->mFrameCount;

    stats.endInterval("Debug preparation");

    outputData->stats = stats;
    mIdleTimer = PreciseTimer::currentTime();
    return outputData;
}

#if 0
double x_center = corrector.center.x();
double y_center = corrector.center.y();
double x = x_center - projection.x();
double y = y_center - projection.y();
if (x_center != 0 || y_center != 0)
{
    double xx = (sqrt(x*x + y*y) / sqrt(x_center*x_center + y_center*y_center));
    //front
    if (c == 0)
        weigth = 1 - 0.0625*xx*xx;
    //right
    if (c == 1)
        weigth = 1 - 0.125*xx*xx;
    //rear
    if (c == 2)
        weigth = 1 - 0.0625*xx*xx;
    //left
    if (c == 3)
        weigth = 1 - 0.125*xx*xx;
    // c = 0 - front
    // c = 1 - right
    //c = 2 - rear
    //c = 3 - left
}
/*   printf(" /n!!!!!!!!!!! x_center = %f y_center = %f projection.x() =%f projection.y() = %f sqrt(x*x + y*y) = %f sqrt(x_center*x_center + y_center*y_center) = %fweight = %f", x_center,
    y_center, projection.x(), projection.y(), sqrt(x*x + y*y), sqrt(x_center*x_center + y_center*y_center), weigth);*/


#endif

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
    recomputeMergerState = true;
}

void MergerThread::baseControlParametersChanged(QSharedPointer<BaseParameters> params)
{
    BaseCalculationThread::baseControlParametersChanged(params);
    recomputeMergerState = true;
}

void MergerThread::camerasParametersChanged(QSharedPointer<CamerasConfigParameters> parameters)
{
    BaseCalculationThread::camerasParametersChanged(parameters);
    recomputeMergerState = true;
}
