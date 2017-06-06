/**
 * \file mergerThread.h
 * \see mergerThread.cpp
 *
 * \date Sep 17, 2010
 * \author Sergey Levi
 */

#ifndef RECORDERTHREAD_H_
#define RECORDERTHREAD_H_

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <QString>

#include "global.h"

#include "baseCalculationThread.h"
#include "imageCaptureInterface.h"
#include "ppmLoader.h"
#include "preciseTimer.h"
#include "generatedParameters/merger.h"
#include "calculationStats.h"

class MergerOutputData : public BaseOutputData
{
public:
    Statistics stats;
    unsigned frameCount;
};

class MergerThread : public BaseCalculationThread
{
    Q_OBJECT

public:
    enum RecordingState
    {
        StateRecordingActive = 0,
        StateRecordingPaused,
        StateRecordingReset,
        StateRecordingFailure
    };

    MergerThread();

public slots:
    void toggleRecording();
    void resetRecording();

    void mergerControlParametersChanged(QSharedPointer<Merger> params);
    void baseControlParametersChanged(QSharedPointer<BaseParameters> params);
    void camerasParametersChanged(QSharedPointer<CamerasConfigParameters> parameters);


signals:
    void recordingStateChanged(MergerThread::RecordingState state);
    void errorMessage(QString string);

protected:
    virtual AbstractOutputData *processNewData();

private:
    bool mRecordingStarted;
    bool mIsRecording;
    PreciseTimer mIdleTimer;

    /* Might be misleading, but PPMLoader handles saving as well */
    PPMLoader mSaver;

    uint32_t mFrameCount;
    QString mPath;
    QSharedPointer<Merger> mMergerParameters;
};

#endif /* RECORDERTHREAD_H_ */
