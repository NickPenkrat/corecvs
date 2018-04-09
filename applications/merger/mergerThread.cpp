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

void MergerThread::cacheIntrinsics()
{
    SYNC_PRINT(("MergerThread::cacheIntrinsics()\n"));
    for (int c=0; c < 4; c++)
    {
        CameraModel slowModel = mCarScene->fixtures()[0]->cameras[c]->getWorldCameraModel();
        if (slowModel.getOmnidirectional() == NULL )
        {
             SYNC_PRINT(("Cam %d doesn't have catadioptric projection %d\n", c, slowModel.intrinsics->projection));
             continue;
        }

        bool same = true;
        if      (  mRemapCached.cached[c].getOmnidirectional() == NULL) {
            same = false;
            SYNC_PRINT(("Cache not catadioptric\n"));
        } else if (!(mRemapCached.cached[c].distortion == slowModel.distortion)) {
            //same = false;
            SYNC_PRINT(("Distortion differ\n"));
        }
        else if (!(*slowModel.getOmnidirectional() == *mRemapCached.cached[c].getOmnidirectional()))
        {
            //same = false;
            SYNC_PRINT(("Omnidirectional model parameters differ\n"));
        }

        if (mRemapCached.cached[c].getOmnidirectional() != 0)
        {
            cout << "Cached" <<  *mRemapCached.cached[c].getOmnidirectional() << std::endl;
            cout << "New"    <<  *slowModel.getOmnidirectional() << std::endl;
        }

        if (same)
        {
            SYNC_PRINT(("Cam %d cache is valid\n", c));
            continue;
        }

        SYNC_PRINT(("Cam %d recalculating\n", c));


        OmnidirectionalProjection slowProjection = *static_cast<OmnidirectionalProjection *>(slowModel.intrinsics.get());

        EquidistantProjection target;
        target.setPrincipalX(slowProjection.principalX());
        target.setPrincipalY(slowProjection.principalY());
        target.setDistortedSizeX(slowProjection.distortedSizeX());
        target.setDistortedSizeY(slowProjection.distortedSizeY());
        target.setSizeX(slowProjection.sizeX());
        target.setSizeY(slowProjection.sizeY());

        delete_safe(mRemapCached.displacement[c]);
        mRemapCached.displacement[c] = new AbstractBuffer<Vector2dd>(slowProjection.sizeY(), slowProjection.sizeX());


        PreciseTimer timer = PreciseTimer::currentTime();
        /**
         *
         *    2D    ->          catadioptric           -> 3D
         *    2D    <-  remap  <-  2D  <- equiditstant -> 3D
         *
         *
         **/
        int count = 0;
        parallelable_for(0, (int)slowProjection.sizeY(), [&](const BlockedRange<int>& r)
        {
            for(int i = r.begin(); i < r.end(); i++)
            {
                for (int j = 0; j < slowProjection.sizeX(); j++)
                {
                    Vector2dd pixel(j, i); /* This is a remap pixel */

                    Vector3dd dir = target.reverse(pixel);
                    Vector2dd map = slowProjection.project(dir);

                    /* We need to invent something */

                    if (mRemapCached.displacement[c]->isValidCoord(i, j)) {
                        mRemapCached.displacement[c]->element(i, j) = Vector2dd(map.x(), map.y());
                    }
                }
                count++;
                if ((count % 100) == 0)
                {
                    SYNC_PRINT(("...%lf%%", 100.0 * count / slowProjection.sizeY()));
                }
            }
        });
        SYNC_PRINT(("\n"));
        int inversions = slowProjection.sizeY() *  slowProjection.sizeX();
        uint64_t time = timer.nsecsToNow();
        SYNC_PRINT(("%d inversions done in %d ms (%lf ns per inversion)\n", inversions, time / 1000000, (double)time / inversions));


        mRemapCached.simplified[c] = target;
        mRemapCached.cached[c].copyModelFrom(slowModel);
    }
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

    cacheIntrinsics();

    /* Loading masks */
    const char *maskFiles[] = {"front-mask.bmp", "right-mask.bmp", "back-mask.bmp", "left-mask.bmp"};

    SYNC_PRINT(("MergerThread::prepareMapping(): re-loading masks\n"));
    for (int c = 0; c < 4; c++) {
        delete_safe(mMasks[c]);
        mMasks[c] = BufferFactory::getInstance()->loadRGB24Bitmap(maskFiles[c]);
        if (mMasks[c] != NULL) {
            SYNC_PRINT(("  Mask %d Loaded\n", c));
            continue;
        }

        SYNC_PRINT(("  Mask %d is zero. Setting blank\n", c));
        mMasks[c] = new RGB24Buffer(mFrames.getCurrentFrame(Frames::DEFAULT_FRAME)->getSize(), RGBColor::White());
    }

    /** Undistortion would be load only once **/
    SYNC_PRINT(("MergerThread::prepareMapping(): caching undistortion\n"));

    /* We would fill mapper with caching data*/
    double groundZ = mMergerParameters->groundZ();

    double l = mMergerParameters->outPhySizeL();
    double w = mMergerParameters->outPhySizeW();
    mFrame.p1 = Vector3dd( l/2,  w/2, groundZ);
    mFrame.e2 = Vector3dd(  -w,    0,   0    );
    mFrame.e1 = Vector3dd(   0,   -l,   0    );

    CameraModel models[4];
    for (int c = 0; c < 4; c++)
    {
        models[c] = mCarScene->fixtures()[0]->cameras[c]->getWorldCameraModel();
    }

    SYNC_PRINT(("MergerThread::prepareMapping(): caching mapping\n"));
    delete_safe(mMapper);

    int hSize = mMergerParameters->outSizeH();
    int wSize = mMergerParameters->outSizeH() / mMergerParameters->outPhySizeW() * mMergerParameters->outPhySizeL();

    mMapper = new MultiewMapping(hSize, wSize);

    SYNC_PRINT(("MergerThread::prepareMapping(): processing: "));
    int count = 0;
    int done = 0;
    parallelable_for(0, mMapper->h, [&](const BlockedRange<int>& r)
    {
        for(int i = r.begin(); i < r.end(); i++)
        {
            for (int j = 0; j < mMapper->w; j++)
            {
                Vector2dd p = Vector2dd( (double)j / mMapper->w, (double)i / mMapper->h);
                Vector3dd pos = mFrame.getPoint(p);

                for (int c = 0; c < 4; c++)
                {
                    Vector3dd posCam = models[c].extrinsics.worldToCam(pos);
#if 0
                    if (!models[c].intrinsics->isVisible(posCam))
                        continue;
                    Vector2dd prj = models[c].intrinsics->project(posCam);
#else
                    Vector2dd prj = mRemapCached.simplified[c].project(posCam);
                    if (!mRemapCached.displacement[c]->isValidCoord(prj.y(), prj.x()))
                        continue;
                    prj = mRemapCached.displacement[c]->element(prj.y(), prj.x());
#endif
                    mMapper->element(i, j).sourcePos[c] = prj;

                    if (mMasks[c]->isValidCoordBl(prj.y(), prj.x()))
                        mMapper->element(i, j).weight[c] = mMasks[c]->element(prj.y(), prj.x()).r() / 255.0;
                    count++;
                }
            }
            done++;
            if (done % 100 == 0)
                SYNC_PRINT(("#"));
        }
    });
    SYNC_PRINT(("\n"));
    SYNC_PRINT(("MergerThread::prepareMapping(): %d non zero", count));


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

                if (entry.weight[c] != 0 && buffer->isValidCoordBl(entry.sourcePos[c]))
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
    drawer.setGridStepForCameras(50);
    drawer.drawScene(*outputData->visualisation, *mCarScene, 3);


    outputData->visualisation->addPlaneFrame(mFrame);

    outputData->frameCount = this->mFrameCount;

    stats.endInterval("Debug preparation");

    outputData->stats = stats;
    mIdleTimer = PreciseTimer::currentTime();
    return outputData;
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


  //}

  vector<cv::Mat> result;

  if (mMapper == NULL) {
      SYNC_PRINT(("You should step input to form remap cache\n"));
      return result;
  }

  for (int c = 0; c < 8; c++)
  {
    cv::Mat mat = cv::Mat(mMapper->h, mMapper->w, CV_32FC1);
    mat.setTo(cv::Scalar(-1.0));
    result.push_back(mat);
  }

  int done = 0;
  for(int i = 0; i < mMapper->h; i++)
  {
      for (int j = 0; j < mMapper->w; j++)
      {
          Vector2dd p = Vector2dd( (double)j / mMapper->w, (double)i / mMapper->h);
          Vector3dd pos = mFrame.getPoint(p);

          for (int c = 0; c < 4; c++)
          {
              Vector3dd posCam = models[c].extrinsics.worldToCam(pos);

              Vector2dd prj = mRemapCached.simplified[c].project(posCam);
              if (!mRemapCached.displacement[c]->isValidCoord(prj.y(), prj.x()))
                  continue;
              prj = mRemapCached.displacement[c]->element(prj.y(), prj.x());

              if (mMasks[c]->isValidCoordBl(prj.y(), prj.x()))
              {
                 result[2 * c    ].at<int>(i, j) = prj.x();
                 result[2 * c + 1].at<int>(i, j) = prj.y();
              }
          }
       }
       done++;
       if (done % 100 == 0) {
           SYNC_PRINT(("#"));
       }
   }






  return result;
}



void MergerThread::saveRemap(QString directory)
{

  QString d = directory;
  vector<cv::Mat> maps = calculateRemap();

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
