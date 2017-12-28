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

#include "core/buffers/bufferFactory.h"
#include "core/camerafixture/fixtureScene.h"
#include "core/camerafixture/cameraFixture.h"
#include "core/cameracalibration/calibrationDrawHelpers.h"

#include "legacy/spherical_lut.h"
#include "core/cammodel/sphericalCorrectionLUT.h"

// TEST
// #include "viFlowStatisticsDescriptor.h"

MergerThread::MergerThread() :
   BaseCalculationThread(Frames::MAX_INPUTS_NUMBER)
  , mMergerParameters(NULL)
{
    for (int i = 0; i < 4;++i)
      mMasks[i] = NULL;

    qRegisterMetaType<MergerThread::RecordingState>("MergerThread::RecordingState");
    mIdleTimer = PreciseTimer::currentTime();
}

MergerThread::~MergerThread()
{
    for (int c = 0; c < 4; c++) {
        delete_safe(mMasks[c]);
    }
    delete_safe(mCarScene);
    delete_safe(mUndistort);
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
    PinholeCameraIntrinsics pinhole1(Vector2dd(fstBuf->w, fstBuf->h), degToRad(mMergerParameters->fOV1()));
    PinholeCameraIntrinsics pinhole2(Vector2dd(fstBuf->w, fstBuf->h), degToRad(mMergerParameters->fOV2()));
    PinholeCameraIntrinsics pinhole3(Vector2dd(fstBuf->w, fstBuf->h), degToRad(mMergerParameters->fOV3()));
    PinholeCameraIntrinsics pinhole4(Vector2dd(fstBuf->w, fstBuf->h), degToRad(mMergerParameters->fOV4()));


    /** Undistortion would be load only once **/

    if (mUndistort == NULL)
    {
        LensDistortionModelParameters disortion;
        JSONGetter loader("lens.json");
        loader.visit(disortion, "intrinsic");
        cout << disortion << endl;

        //disortion.setShift(Vector2dd(400,400));
        RadialCorrection corr(disortion);
        delete_safe(mUndistort);
        //mUndistort = new DisplacementBuffer(&corr, fstBuf->h, fstBuf->w, false);

        int overshoot = mMergerParameters->distortionOvershoot();

        mUndistort = TableInverseCache::CacheInverse(
                      /*  - fstBuf->w,   - fstBuf->h,
                      2 * fstBuf->w, 2 * fstBuf->h,*/
                      -overshoot, -overshoot,
                      fstBuf->w + overshoot, fstBuf->h + overshoot,
                      /*0, 0,
                      fstBuf->w, fstBuf->h,*/
                      &corr,
                      0.0, 0.0,
                      (double)fstBuf->w, (double)fstBuf->h,
                      0.1, false);
    }



    FixtureCamera *frontCam = mCarScene->createCamera(); mCarScene->addCameraToFixture(frontCam, body);
    FixtureCamera *rightCam = mCarScene->createCamera(); mCarScene->addCameraToFixture(rightCam, body);
    FixtureCamera *backCam  = mCarScene->createCamera(); mCarScene->addCameraToFixture(backCam , body);
    FixtureCamera *leftCam  = mCarScene->createCamera(); mCarScene->addCameraToFixture(leftCam , body);

    frontCam->intrinsics = pinhole1; // frontCam->distortion = inverted.mParams;
    rightCam->intrinsics = pinhole2; // rightCam->distortion = inverted.mParams;
    backCam ->intrinsics = pinhole3; // backCam ->distortion = inverted.mParams;
    leftCam ->intrinsics = pinhole4; // leftCam ->distortion = inverted.mParams;

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
 
    /* Unwrap */
    RGB24Buffer *input = mFrames.getCurrentRgbFrame((Frames::FrameSourceId)mMergerParameters->frameToUndist());
    Vector2dd center(input->w / 2.0, input->h / 2.0);

    vector<Vector2dd> lut;
    for (unsigned i = 0; i < LUT_LEN; i++) {
        lut.push_back(Vector2dd(UnwarpToWarpLUT[i][0], UnwarpToWarpLUT[i][1]));
    }
    RadiusCorrectionLUTSq radiusLUTSq(&lut);
    SphericalCorrectionLUTSq correctorSq(center, &radiusLUTSq);

    RadiusCorrectionLUT radiusLUT = RadiusCorrectionLUT::FromSquareToSquare(lut);
    SphericalCorrectionLUT corrector(center, &radiusLUT);

    vector<Vector2dd> luthd;
    for (unsigned i = 0; i < LUT_LEN_HD; i++) {
        luthd.push_back(Vector2dd(AngleToShiftLUT_HD[i][0], AngleToShiftLUT_HD[i][1]));
    }

    double mmToPixel = (double)input->w / mMergerParameters->sensorWidth();
    RadiusCorrectionLUT radiusLUTHd = RadiusCorrectionLUT::FromAngleAndProjection(luthd, mmToPixel, mMergerParameters->undistFocal());
    SphericalCorrectionLUT correctorHD(center, &radiusLUTHd);

#if 0
    for (unsigned i = 0; i < radiusLUTHd.LUT.size(); i++) {
        cout << "Table for HD " << i  << " - " << radiusLUTHd.LUT[i]  << endl;
    }
#endif


    //outputData->unwarpOutput = input->doReverseDeformationBl<RGB24Buffer, SphericalCorrectionLUT>(&corrector, input->h, input->w);

    switch (mMergerParameters->undistMethod()) {
      case MergerUndistMethod::HD_TABLE:
           outputData->unwarpOutput = input->doReverseDeformationBlTyped<SphericalCorrectionLUT>(&correctorHD, input->h, input->w);
           break;
      case MergerUndistMethod::LOADED_CAMERA:
           outputData->unwarpOutput = input->doReverseDeformationBlTyped<TableInverseCache>(mUndistort, input->h, input->w);
           break;
      default:
           outputData->unwarpOutput = input->doReverseDeformationBlTyped<SphericalCorrectionLUT>(&corrector, input->h, input->w);
           break;
    }

    stats.resetInterval("Example unwrap");

    //precalculations
    //car simulation
    //{
    //Get x
    Vector2dd projX0 = frame.projectTo(cams[0]->getWorldLocation().shift);
    projX0 = projX0 * Vector2dd(out->w, out->h);
    Vector2dd projX2 = frame.projectTo(cams[2]->getWorldLocation().shift);
    projX2 = projX2 * Vector2dd(out->w, out->h);
    int32_t sizeX = projX2.x() - projX0.x();
    //Get y
    Vector2dd projY3 = frame.projectTo(cams[3]->getWorldLocation().shift);
    projY3 = projY3 * Vector2dd(out->w, out->h);
    Vector2dd projY1 = frame.projectTo(cams[1]->getWorldLocation().shift);
    projY1 = projY1 * Vector2dd(out->w, out->h);
    int32_t sizeY = projY3.y() - projY1.y();

    //size of car
    int X_car_picture = 80;
    corecvs::Vector2d<int32_t> sizeRect = { sizeX + X_car_picture, sizeY };

    //draw car
    int shift_car_picture = 30;
    corecvs::Vector2d<int32_t> corner = { (int32_t)projX0.x() - shift_car_picture, (int32_t)projY1.y() };
    corecvs::Rectangled rect = corecvs::Rectangled(corner.x(), corner.y(), sizeRect.x(), sizeRect.y());
    //corecvs::Rectangle32 rect = corecvs::Rectangle32(corner, sizeRect);
    Vector2dd v1 = { 0, 0 };
    Vector2dd v1_end = rect.ulCorner();

    Vector2dd v2 = { 0, (double)out->h };
    Vector2dd v2_end = rect.llCorner();

    Vector2dd v3 = { (double)out->w, 0 };
    Vector2dd v3_end = rect.urCorner();

    Vector2dd v4 = { (double)out->w, (double)out->h };
    Vector2dd v4_end = rect.lrCorner();


    //}

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

                Vector2dd prj (-1, -1);
                switch (mMergerParameters->undistMethod())
                {
                    case MergerUndistMethod::NONE:
                    {
                        bool visible = cams[c]->projectPointFromWorld(pos, &prj);
                        if (!visible)
                            continue;
                        break;
                    }
                    case MergerUndistMethod::SQUARE_TABLE:
                    {
                         Vector3dd relative   = models[c].extrinsics.project(pos);
                        if (relative.z() < 0)
                            continue;
                        Vector2dd projection = models[c].intrinsics.project(relative);
                        prj = correctorSq.map(projection);
                        break;
                    }
                    case MergerUndistMethod::RADIAL_TABLE:
                    {
                        Vector3dd relative   = models[c].extrinsics.project(pos);
                        if (relative.z() < 0)
                            continue;
                        Vector2dd projection = models[c].intrinsics.project(relative);
                        prj = corrector.map(projection);
                        break;
                    }
                    case MergerUndistMethod::HD_TABLE:
                    {
                        Vector3dd relative   = models[c].extrinsics.project(pos);
                        if (relative.z() < 0)
                            continue;
                        Vector2dd projection = models[c].intrinsics.project(relative);
                        prj = correctorHD.map(projection);
                        break;
                    }
                    case MergerUndistMethod::LOADED_CAMERA:
                    {
                        Vector3dd relative   = models[c].extrinsics.project(pos);
                        if (relative.z() < 0)
                            continue;
                        Vector2dd projection = models[c].intrinsics.project(relative);
                        if (mUndistort->isValidCoord((int32_t)projection.y(), (int32_t)projection.x())) {
                            prj = mUndistort->map((int32_t)projection.y(), (int32_t)projection.x());
                        }
                        break;
                    }

                }
                double weigth_separated_view = 1;

                if (mMergerParameters->mSeparateView)
                {
                    //front  // swith 1
                    if (c == 0) 
                    {
                        Vector2dd v_out = { (double)j, (double)i };
                        if (isUnderLine(v_out, v1, v1_end) || !isUnderLine(v_out, v2, v2_end))
                            weigth_separated_view = 0;
                    } 
                    //rigth // swith 2
                    else if (c == 1)
                    {
                        Vector2dd v_out = { (double)j, (double)i };
                        if (!isUnderLine(v_out, v1, v1_end) || !isUnderLine(v_out, v3, v3_end))
                            weigth_separated_view = 0;
                    }
                    //rear // swith 3
                    else if (c == 2)
                    {
                        Vector2dd v_out = { (double)j, (double)i };
                        if (isUnderLine(v_out, v3, v3_end) || !isUnderLine(v_out, v4, v4_end))
                            weigth_separated_view = 0;
                    }
                    //left // swith 4
                    else if (c == 3)
                    {
                        Vector2dd v_out = { (double)j, (double)i };
                        if (isUnderLine(v_out, v2, v2_end) || isUnderLine(v_out, v4, v4_end))
                            weigth_separated_view = 0;
                    }
                }

                if (!mMergerParameters->bilinear()) {
                    if ( buffer   ->isValidCoord(prj.y(), prj.x()) &&
                         mMasks[c]->isValidCoord(prj.y(), prj.x())) {
                        double weight = weigth_separated_view * mMasks[c]->element(prj.y(), prj.x()).r() / 255.0;
                        color += weight * buffer->element(prj.y(), prj.x()).toDouble();
                        sum += weight;
                    }
                } else {
                    if ( buffer   ->isValidCoordBl(prj.y(), prj.x()) &&
                         mMasks[c]->isValidCoord(prj.y(), prj.x())) {
                        double weight = weigth_separated_view * mMasks[c]->element(prj.y(), prj.x()).r() / 255.0;
                        color += weight * buffer->elementBl(prj.y(), prj.x()).toDouble();
                        sum += weight;
                    }
                }
            }

            if (sum != 0) {
                out->element(i,j) = RGBColor::FromDouble(color / sum);
            }
        }

    });

    stats.resetInterval("Reprojecting");

    //car simulation
    if (mMergerParameters->drawCar())
    {
        out->drawRectangle(rect, RGBColor::Black(), 2);

        //draw lines
        if (mMergerParameters->mSeparateView)
        {
            out->drawLine(v1, v1_end, RGBColor::Black());
            out->drawLine(v2, v2_end, RGBColor::Black());
            out->drawLine(v3, v3_end, RGBColor::Black());
            out->drawLine(v4, v4_end, RGBColor::Black()); 
        }
    }

    //camera position
    for (int c = 0; c < 4; c++)
    {
        Vector2dd proj = frame.projectTo(cams[c]->getWorldLocation().shift);
        proj = proj * Vector2dd(out->w, out->h);
        out->drawCrosshare2(proj.x(), proj.y(), RGBColor::Red());
    }

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

bool MergerThread::isUnderLine(Vector2dd point, Vector2dd point1, Vector2dd point2)
{
    double y = point.y();
    if (point2.x() != point1.x())
    {
        double k = ((point2.y() - point1.y()) / (point2.x() - point1.x()));
        double b = point1.y() - k * point1.x();
        double value_y = k * point.x() + b;
        if (y < value_y)
            return true;
    }
    return false;
}

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

vector<cv::Mat> MergerThread::calculateRemap()
{
  qDebug("MergerThread::calculateRemap(): called for %d inputs", mActiveInputsNumber);

  bool have_params = !(mMergerParameters.isNull());
  bool two_frames = have_params && (CamerasConfigParameters::TwoCapDev == mActiveInputsNumber); // FIXME: additional params needed here

                                                                                                // We are missing data, so pause calculation
  if ((!mFrames.getCurrentFrame(Frames::LEFT_FRAME)) ||
    ((!mFrames.getCurrentFrame(Frames::RIGHT_FRAME)) && (CamerasConfigParameters::TwoCapDev == mActiveInputsNumber)))
  {
    emit errorMessage("Capture error.");
    pauseCalculation();
  }

  recalculateCache();
  prepareMapping();

  /* Scene */
  CameraFixture *fixture = mCarScene->fixtures().front();
  FixtureCamera *cams[] = { fixture->cameras[0], fixture->cameras[1], fixture->cameras[2], fixture->cameras[3] };

  double groundZ = mMergerParameters->groundZ();
  PlaneFrame frame;
  double l = mMergerParameters->outPhySizeL();
  double w = mMergerParameters->outPhySizeW();
  frame.p1 = Vector3dd(l / 2, -w / 2, groundZ);
  frame.e1 = Vector3dd(-l, 0, 0);
  frame.e2 = Vector3dd(0, w, 0);

  CameraModel models[4];
  for (int c = 0; c < 4; c++)
  {
    models[c] = cams[c]->getWorldCameraModel();
  }

  /* Unwrap */
  RGB24Buffer *input = mFrames.getCurrentRgbFrame((Frames::FrameSourceId)mMergerParameters->frameToUndist());
  Vector2dd center(input->w / 2.0, input->h / 2.0);

  vector<Vector2dd> lut;
  for (unsigned i = 0; i < LUT_LEN; i++) {
    lut.push_back(Vector2dd(UnwarpToWarpLUT[i][0], UnwarpToWarpLUT[i][1]));
  }
  RadiusCorrectionLUTSq radiusLUTSq(&lut);
  SphericalCorrectionLUTSq correctorSq(center, &radiusLUTSq);

  RadiusCorrectionLUT radiusLUT = RadiusCorrectionLUT::FromSquareToSquare(lut);
  SphericalCorrectionLUT corrector(center, &radiusLUT);

  vector<Vector2dd> luthd;
  for (unsigned i = 0; i < LUT_LEN_HD; i++) {
    luthd.push_back(Vector2dd(AngleToShiftLUT_HD[i][0], AngleToShiftLUT_HD[i][1]));
  }

  double mmToPixel = (double)input->w / mMergerParameters->sensorWidth();
  RadiusCorrectionLUT radiusLUTHd = RadiusCorrectionLUT::FromAngleAndProjection(luthd, mmToPixel, mMergerParameters->undistFocal());
  SphericalCorrectionLUT correctorHD(center, &radiusLUTHd);

#if 0
  for (unsigned i = 0; i < radiusLUTHd.LUT.size(); i++) {
    cout << "Table for HD " << i << " - " << radiusLUTHd.LUT[i] << endl;
  }
#endif


  //outputData->unwarpOutput = input->doReverseDeformationBl<RGB24Buffer, SphericalCorrectionLUT>(&corrector, input->h, input->w);

  //precalculations
  //car simulation
  //{
  //Get x
  int height = mMergerParameters->outSizeH();
  int width = mMergerParameters->outSizeH() / mMergerParameters->outPhySizeW() * mMergerParameters->outPhySizeL();

  Vector2dd projX0 = frame.projectTo(cams[0]->getWorldLocation().shift);
  projX0 = projX0 * Vector2dd(width, height);
  Vector2dd projX2 = frame.projectTo(cams[2]->getWorldLocation().shift);
  projX2 = projX2 * Vector2dd(width, height);
  int32_t sizeX = projX2.x() - projX0.x();
  //Get y
  Vector2dd projY3 = frame.projectTo(cams[3]->getWorldLocation().shift);
  projY3 = projY3 * Vector2dd(width, height);
  Vector2dd projY1 = frame.projectTo(cams[1]->getWorldLocation().shift);
  projY1 = projY1 * Vector2dd(width, height);
  int32_t sizeY = projY3.y() - projY1.y();

  //size of car
  int X_car_picture = 80;
  corecvs::Vector2d<int32_t> sizeRect = { sizeX + X_car_picture, sizeY };

  //draw car
  int shift_car_picture = 30;
  corecvs::Vector2d<int32_t> corner = { (int32_t)projX0.x() - shift_car_picture, (int32_t)projY1.y() };
  corecvs::Rectangled rect = corecvs::Rectangled(corner.x(), corner.y(), sizeRect.x(), sizeRect.y());
  //corecvs::Rectangle32 rect = corecvs::Rectangle32(corner, sizeRect);
  Vector2dd v1 = { 0, 0 };
  Vector2dd v1_end = rect.ulCorner();

  Vector2dd v2 = { 0, (double)height };
  Vector2dd v2_end = rect.llCorner();

  Vector2dd v3 = { (double)width, 0 };
  Vector2dd v3_end = rect.urCorner();

  Vector2dd v4 = { (double)width, (double)height };
  Vector2dd v4_end = rect.lrCorner();


  //}
  vector<cv::Mat> result;
  for (int c = 0; c < 8; c++)
  {
    cv::Mat mat = cv::Mat(height, width, CV_32S);
    mat.setTo(cv::Scalar(-1));
    result.push_back(mat);
  }

    for (int i = 0; i < height; i++)
      for (int j = 0; j < width; j++)
      {
        Vector2dd p = Vector2dd((double)j / width, (double)i / height);
        Vector3dd pos = frame.getPoint(p);

        Vector3dd color(0.0);
        double sum = 0;

        for (int c = 0; c < 4; c++)
        {
          RGB24Buffer *buffer = mFrames.getCurrentRgbFrame((Frames::FrameSourceId)c);

          Vector2dd prj(-1, -1);
          switch (mMergerParameters->undistMethod())
          {
          case MergerUndistMethod::NONE:
          {
            bool visible = cams[c]->projectPointFromWorld(pos, &prj);
            if (!visible)
              continue;
            break;
          }
          case MergerUndistMethod::SQUARE_TABLE:
          {
            Vector3dd relative = models[c].extrinsics.project(pos);
            if (relative.z() < 0)
              continue;
            Vector2dd projection = models[c].intrinsics.project(relative);
            prj = correctorSq.map(projection);
            break;
          }
          case MergerUndistMethod::RADIAL_TABLE:
          {
            Vector3dd relative = models[c].extrinsics.project(pos);
            if (relative.z() < 0)
              continue;
            Vector2dd projection = models[c].intrinsics.project(relative);
            prj = corrector.map(projection);
            break;
          }
          case MergerUndistMethod::HD_TABLE:
          {
            Vector3dd relative = models[c].extrinsics.project(pos);
            if (relative.z() < 0)
              continue;
            Vector2dd projection = models[c].intrinsics.project(relative);
            prj = correctorHD.map(projection);
            break;
          }
          case MergerUndistMethod::LOADED_CAMERA:
          {
            Vector3dd relative = models[c].extrinsics.project(pos);
            if (relative.z() < 0)
              continue;
            Vector2dd projection = models[c].intrinsics.project(relative);
            if (mUndistort->isValidCoord((int32_t)projection.y(), (int32_t)projection.x())) {
              prj = mUndistort->map((int32_t)projection.y(), (int32_t)projection.x());
            }
            break;
          }
          }

          double weigth_separated_view = 1;

          //front  // swith 1
          if (c == 0)
          {
            Vector2dd v_out = { (double)j, (double)i };
            if (isUnderLine(v_out, v1, v1_end) || !isUnderLine(v_out, v2, v2_end))
              weigth_separated_view = 0;
          }
          //rigth // swith 2
          else if (c == 1)
          {
            Vector2dd v_out = { (double)j, (double)i };
            if (!isUnderLine(v_out, v1, v1_end) || !isUnderLine(v_out, v3, v3_end))
              weigth_separated_view = 0;
          }
          //rear // swith 3
          else if (c == 2)
          {
            Vector2dd v_out = { (double)j, (double)i };
            if (isUnderLine(v_out, v3, v3_end) || !isUnderLine(v_out, v4, v4_end))
              weigth_separated_view = 0;
          }
          //left // swith 4
          else if (c == 3)
          {
            Vector2dd v_out = { (double)j, (double)i };
            if (isUnderLine(v_out, v2, v2_end) || isUnderLine(v_out, v4, v4_end))
              weigth_separated_view = 0;
          }

          double weight = 0;
          if (!mMergerParameters->bilinear()) {
            if (buffer->isValidCoord(prj.y(), prj.x()) &&
              mMasks[c]->isValidCoord(prj.y(), prj.x())) {
              weight = weigth_separated_view * mMasks[c]->element(prj.y(), prj.x()).r() / 255.0;
            }
          }
          else {
            if (buffer->isValidCoordBl(prj.y(), prj.x()) &&
              mMasks[c]->isValidCoord(prj.y(), prj.x())) {
              weight = weigth_separated_view * mMasks[c]->element(prj.y(), prj.x()).r() / 255.0;
            }
          }

          if (weight > 0)
          {
            // xmap
            result[c * 2].at<int>(i, j) = prj.x();
            // ymap
            result[c * 2 + 1].at<int>(i, j) = prj.y();
          }
        }
      }

  return result;
}

void MergerThread::saveRemap(QString directory)
{
  QString d = directory;
  auto maps = calculateRemap();

  for (int c = 0; c < 4; c++)
  {
    QString filename = directory;
    switch (c)
    {
      // front
      case 0:
        filename += "/front.yml";
        break;
      case 1:
        filename += "/right.yml";
        break;
      case 2:
        filename += "/rear.yml";
        break;
      case 3:
        filename += "/left.yml";
        break;
    }
    cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
      fs << "XMAP" << maps[c * 2];
      fs << "YMAP" << maps[c * 2 + 1];
      fs.release();
    }
  }
}
