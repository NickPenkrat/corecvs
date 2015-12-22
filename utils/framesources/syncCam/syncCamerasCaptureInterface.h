/**
 * \file syncCamerasCaptureInterface.h
 * \brief Add Comment Here
 *
 * \date Apr 9, 2010
 * \author osapershteyn
 */

#pragma once

#include <QtCore/QThread>
#include <QtCore/QMutex>
#include <string>
#include <errno.h>

#include <usbsynccam.h>

#include "global.h"

#include "cameraControlParameters.h"
#include "imageCaptureInterface.h"
#include "preciseTimer.h"
#include "../../frames.h"

#ifdef interface
#undef interface
#endif

using namespace std;
using namespace syncCam;

class SyncCamerasCaptureInterface : public ImageCaptureInterface
{
public:
    SyncCamerasCaptureInterface(string _devname);
    virtual ~SyncCamerasCaptureInterface();

    virtual int setConfigurationString(string _devname);

    virtual FramePair getFrame() override;
    virtual FramePair getFrameRGB24() override;

    virtual CapErrorCode initCapture() override;
    virtual CapErrorCode startCapture() override;

    virtual CapErrorCode setCaptureProperty(int id, int value) override;
    virtual CapErrorCode getCaptureProperty(int id, int *value) override;
    /**
     * Check if a specific property can be set/read and what values it can take
     **/

    virtual CapErrorCode queryCameraParameters(CameraParameters &parameter) override;

private:

    SensParam ConvertParamId(int id);

    class SpinThread : public QThread
    {
//        Q_OBJECT
    public:
        SyncCamerasCaptureInterface *interface;

        SpinThread(SyncCamerasCaptureInterface *_interface);

        virtual void run (void);
    };

    string devname;  /**< Stores the device name*/

    string           deviceName;
    USBSyncCam       *camera;
    FramePair        currentPair;

    int formatH;
    int formatW;

    SpinThread spin;
    QMutex protectFrame;  /**< This mutex protects both buffers from concurrent reading/writing
                            *  If somebody is reading of modifying the buffer it should lock the mutex
                            **/

    QMutex spinRunning;
    volatile bool shouldStopSpinThread;

    /* Statistics fields */
    PreciseTimer lastFrameTime;
    uint64_t frameDelay;

    uint64_t skippedCount;

    uint64_t maxDesync; /**< Maximum de-synchronization that we will not attempt to correct */

};
