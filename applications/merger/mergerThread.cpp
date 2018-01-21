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

#include "core/xml/generated/euclidianMoveParameters.h"


using namespace corecvs;

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

    if (mCarScene.isNull())
        return;

    /* Loading masks */
    const char *maskFiles[] = {"front-mask.bmp", "right-mask.bmp", "back-mask.bmp", "left-mask.bmp"};

    for (int c = 0; c < 4; c++) {
        delete_safe(mMasks[c]);
        mMasks[c] = BufferFactory::getInstance()->loadRGB24Bitmap(maskFiles[c]);
        if (mMasks[c] != NULL)
            continue;

        SYNC_PRINT(("Mask %d is zero. Setting blank\n", c));
        mMasks[c] = new RGB24Buffer(mFrames.getCurrentFrame(Frames::DEFAULT_FRAME)->getSize(), RGBColor::White());
    }

    /** Undistortion would be load only once **/

    if (mUndistort == NULL)
    {
        LensDistortionModelParameters disortion;
        JSONGetter loader("lens.json");
        if (!loader.hasError()) {
            loader.visit(disortion, "intrinsic");
        }
        cout << disortion << endl;

        RadialCorrection corr(disortion);
        delete_safe(mUndistort);
             int overshoot = mMergerParameters->distortionOvershoot();

        G12Buffer *fstBuf = mFrames.getCurrentFrame(Frames::LEFT_FRAME);

        mUndistort = TableInverseCache::CacheInverse(
                      -overshoot, -overshoot,
                      fstBuf->w + overshoot, fstBuf->h + overshoot,
                      &corr,
                      0.0, 0.0,
                      (double)fstBuf->w, (double)fstBuf->h,
                      0.1, false);
    }

    /* We would fill mapper with caching data*/
    double groundZ = mMergerParameters->groundZ();

    double l = mMergerParameters->outPhySizeL();
    double w = mMergerParameters->outPhySizeW();
    mFrame.p1 = Vector3dd( l/2, -w/2, groundZ);
    mFrame.e1 = Vector3dd(  -l,    0,   0    );
    mFrame.e2 = Vector3dd(   0,    w,   0    );

    CameraModel models[4];
    for (int c = 0; c < 4; c++)
    {
        models[c] = mCarScene->fixtures()[0]->cameras[c]->getWorldCameraModel();
    }

    delete_safe(mMapper);
    mMapper = new MultiewMapping();

    for (int i = 0; i < mMapper->h; i++)
    {
        for (int j = 0; j < mMapper->w; j++)
        {
            Vector2dd p = Vector2dd( (double)j / mMapper->w, (double)i / mMapper->h);
            Vector3dd pos = mFrame.getPoint(p);

            for (int c = 0; c < 4; c++)
            {
                Vector3dd posCam = models[c].extrinsics.worldToCam(pos);
                if (!models[c].intrinsics->isVisible(posCam))
                    continue;

                Vector2dd prj = models[c].intrinsics->project(posCam);
                mMapper->element(i, j).sourcePos[c] = prj;
                if (mMasks[c]->isValidCoord(prj.y(), prj.x()))
                mMapper->element(i, j).weight[c] = mMasks[c]->element(prj.y(), prj.x()).r() / 255.0;
            }
        }

    }






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



    if (mCarScene.isNull())
    {
        SYNC_PRINT(("MergerThread::processNewData(): mCarScene.isNull()\n"));
        return NULL;
    }
    if (mMapper == NULL)
    {
        SYNC_PRINT(("MergerThread::processNewData(): mMapper == NULL\n"));
        return NULL;
    }

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


    parallelable_for(0, out->h, [&](const BlockedRange<int>& r)
    {
    for(int i = r.begin(); i < r.end(); i++)
        for (int j = 0; j < out->w; j++)
        {
            Vector2dd p = Vector2dd( (double)j / out->w, (double)i / out->h);
            Vector3dd pos = mFrame.getPoint(p);

            if (!mMapper->isValidCoord(i,j))
                continue;

            Vector3dd color(0.0);
            double sum = 0;

            for(int c = 0; c < 4; c++ )
            {
                if (!flags[c])
                    continue;
                RGB24Buffer *buffer = mFrames.getCurrentRgbFrame((Frames::FrameSourceId)c);
                RequestEntry &entry = mMapper->element(i,j);

                if (entry.weight[c] != 0)
                {
                    color += entry.weight[c] * buffer->elementBl(entry.sourcePos[c]).toDouble();
                    sum   += entry.weight[c];
                }
            }

            if (sum != 0) {
                out->element(i,j) = RGBColor::FromDouble(color / sum);
            }
        }

    });

    stats.resetInterval("Reprojecting");

    Vector2dd sizeScaler = Vector2dd(out->w, out->h);

    /* car simulation */
    if (mMergerParameters->drawCar())
    {
        Vector2dd projX0 = mFrame.projectTo(cams[0]->getWorldLocation().shift) * sizeScaler;
        Vector2dd projX2 = mFrame.projectTo(cams[2]->getWorldLocation().shift) * sizeScaler;
        int32_t sizeX = projX2.x() - projX0.x();

        Vector2dd projY3 = mFrame.projectTo(cams[3]->getWorldLocation().shift) * sizeScaler;
        Vector2dd projY1 = mFrame.projectTo(cams[1]->getWorldLocation().shift) * sizeScaler;
        int32_t sizeY = projY3.y() - projY1.y();

        int carPictureX = 80;
        corecvs::Vector2d<int32_t> sizeRect = { sizeX + carPictureX, sizeY };

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
        Vector2dd proj = mFrame.projectTo(cams[c]->getWorldLocation().shift) * sizeScaler;
        out->drawCrosshare2(proj.x(), proj.y(), RGBColor::Red());
    }

    outputData->visualisation = new Mesh3DDecorated;
    outputData->visualisation->switchColor(true);


    CalibrationDrawHelpers drawer;
    drawer.setPrintNames(true);
    drawer.drawScene(*outputData->visualisation, *mCarScene, 3);


    outputData->visualisation->addPlaneFrame(mFrame);

    outputData->frameCount = this->mFrameCount;

    stats.endInterval("Debug preparation");

    outputData->stats = stats;
    mIdleTimer = PreciseTimer::currentTime();
    return outputData;
}


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

void MergerThread::sceneParametersChanged(QSharedPointer<FixtureScene> mCarScene)
{
    SYNC_PRINT(("MergerThread::sceneParametersChanged(%s):called\n", mCarScene.isNull() ? "NULL" : "nonull"));
    mCarScene->dumpInfo();
    this->mCarScene = mCarScene;
    recomputeMergerState = true;
}
