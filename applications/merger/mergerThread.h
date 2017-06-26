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

#include "mesh3DDecorated.h"
typedef RGB24Buffer * PtrRGB24Buffer;
class MergerOutputData : public BaseOutputData
{
public:
    Statistics stats;
    unsigned frameCount;

    RGB24Buffer     *unwarpOutput  = NULL;
    RGB24Buffer     *mainOutput    = NULL;
    Mesh3DDecorated *visualisation = NULL;


    virtual ~MergerOutputData()
    {
        delete_safe(unwarpOutput);
        delete_safe(mainOutput);
        delete_safe(visualisation);
    }


};

struct RequestEntry {
    int sourceBuffer;
    BilinearMapPoint sourcePos;
    double weight;
};

class MultiewMapping : public AbstractBuffer<RequestEntry, int>
{
public:
    Merger mMergerParameters; /* This is to check if update is needed*/


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
    virtual ~MergerThread();

    bool recomputeMergerState = true;

    PtrRGB24Buffer  mMasks[4];// = { NULL, NULL, NULL, NULL };

    FixtureScene  *mCarScene = NULL;
    TableInverseCache *mUndistort = NULL;

    MultiewMapping mMapper;
    void prepareMapping();


    void drawMaskOver(RGB24Buffer *inputRaw, RGB24Buffer *mMasks);

public slots:
    void mergerControlParametersChanged(QSharedPointer<Merger> params);
    void baseControlParametersChanged(QSharedPointer<BaseParameters> params);
    void camerasParametersChanged(QSharedPointer<CamerasConfigParameters> parameters);


signals:
    void errorMessage(QString string);

protected:
    virtual AbstractOutputData *processNewData();

private: 
    PreciseTimer mIdleTimer;

    /* Might be misleading, but PPMLoader handles saving as well */
    PPMLoader mSaver;

    uint32_t mFrameCount;
    QString mPath;
    QSharedPointer<Merger> mMergerParameters;

    bool isUnderLine(Vector2dd point, Vector2dd point1, Vector2dd point2);
    //FixtureScene *scene = NULL;

};

#endif /* RECORDERTHREAD_H_ */
