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


#include "core/utils/global.h"
#include "core/fileformats/ppmLoader.h"
#include "core/utils/preciseTimer.h"
#include "core/stats/calculationStats.h"

#include "core/geometry/mesh3DDecorated.h"
#include "core/camerafixture/fixtureScene.h"

#include "baseCalculationThread.h"
#include "core/framesources/imageCaptureInterface.h"

#include "generatedParameters/merger.h"

#ifdef WITH_OPENCV
# include "opencv2/imgproc/imgproc.hpp"
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/core/core_c.h"
#endif


using namespace corecvs;

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

struct IntrisicsRemapCache
{
    AbstractBuffer<Vector2dd> *displacement[4] = {NULL, NULL, NULL, NULL};
    EquidistantProjection simplified[4];

    CameraModel cached[4];

};

struct RequestEntry {
    RequestEntry() {
        for (int k = 0; k < 4; k++)
            weight[k] = 0.0;
    }

    Vector2dd sourcePos[4];
    double weight[4];
};

typedef AbstractBuffer<RequestEntry, int> MultiewMapping;

#if 0
class MultiewMapping : public AbstractBuffer<RequestEntry, int>
{
public:
    Merger mMergerParameters; /* This is to check if update is needed*/


};
#endif

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


    /* Cached data */
    bool recomputeMergerState = true;
    IntrisicsRemapCache mRemapCached;
    MultiewMapping *mMapper = NULL;
    PlaneFrame mFrame;


    PtrRGB24Buffer  mMasks[4];// = { NULL, NULL, NULL, NULL };

    //FixtureScene  *mCarScene = NULL;
    QSharedPointer<FixtureScene> mCarScene;
    QSharedPointer<FixtureScene> mCachedScene;

    TableInverseCache *mUndistort = NULL;


    void cacheIntrinsics();
    void prepareMapping();


    void drawMaskOver(RGB24Buffer *inputRaw, RGB24Buffer *mMasks);

public slots:
    void sceneParametersChanged(QSharedPointer<FixtureScene> mCarScene);


    void mergerControlParametersChanged(QSharedPointer<Merger> params);
    void baseControlParametersChanged(QSharedPointer<BaseParameters> params);
    void camerasParametersChanged(QSharedPointer<CamerasConfigParameters> parameters);
    void saveRemap(QString directory);

signals:
    void errorMessage(QString string);

protected:
    virtual AbstractOutputData *processNewData();

private: 
#ifdef WITH_OPENCV
    vector<cv::Mat> calculateRemap();
#endif

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
