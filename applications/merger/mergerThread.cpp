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
// TEST
// #include "viFlowStatisticsDescriptor.h"

MergerThread::MergerThread() :
   BaseCalculationThread()
  , mRecordingStarted(false)
  , mIsRecording(false)
  , mFrameCount(0)
  , mPath("")
  , mMergerParameters(NULL)
{
    qRegisterMetaType<MergerThread::RecordingState>("MergerThread::RecordingState");
    mIdleTimer = PreciseTimer::currentTime();
}

void MergerThread::toggleRecording()
{
    if (!mIsRecording)
    {
        if (mMergerParameters.isNull())
        {
            cout << "MergerThread: Internal error. Recording toggled but no parameters provided." << endl;
        }

        if (mMergerParameters->path().empty())
        {
            cout << "MergerThread: Path is empty" << endl;
        }

        if (mMergerParameters->fileTemplate().empty())
        {
            cout << "MergerThread: File template is empty\n";
        }

        if (!mRecordingStarted)
        {
            mRecordingStarted = true;
            printf("Recording started.\n");
        }
        else
        {
            printf("Recording resumed.\n");
        }
        mIsRecording = true;
        emit recordingStateChanged(StateRecordingActive);
    }
    else
    {
        mIsRecording = false;
        printf("Recording paused.\n");
        emit recordingStateChanged(StateRecordingPaused);
    }
}

AbstractOutputData* MergerThread::processNewData()
{
    Statistics stats;

//    qDebug("MergerThread::processNewData(): called");

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

    G12Buffer *result[Frames::MAX_INPUTS_NUMBER] = {NULL, NULL};

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

        if (mIsRecording)
        {
            char currentPath[256];
            if (two_frames)
                snprintf2buf(currentPath, mPath.toStdString().c_str(), mFrameCount, id);
            else
                snprintf2buf(currentPath, mPath.toStdString().c_str(), mFrameCount);

            if (mSaver.save(currentPath, result[id]))
            {
                resetRecording();
                emit errorMessage(QString("Error writing frame to file") + currentPath);
                emit recordingStateChanged(StateRecordingFailure);
            }
        }

    }
#if 0
    stats.setTime(ViFlowStatisticsDescriptor::CORRECTON_TIME, startEl.usecsToNow());
#endif
    if (mIsRecording) {
        mFrameCount++;
    }

    MergerOutputData* outputData = new MergerOutputData();

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

    outputData->frameCount = this->mFrameCount;
    outputData->stats = stats;

    return outputData;
}

void MergerThread::resetRecording()
{
    mIsRecording = false;
    mRecordingStarted = false;
    mPath = "";
    mFrameCount = 0;
    emit recordingStateChanged(StateRecordingReset);
}

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
