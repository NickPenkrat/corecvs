#pragma once

#include <QThread>

#include <QDebug>
#include <QMutex>

#include "global.h"

#include "topViCameraDescriptor.h"
#include "topViGrillInterface.h"

using namespace std;

class TopViCaptureInterface;

//IMPORTANT: this class is used as static
class TopViDeviceDescriptor
{
public:
    QAtomicInt inited = 0;
    int deviceId = -1;

    QImage                  *result = NULL;

    TopViDeviceDescriptor(){}

    //IMPORTANT: we assume that all objects with the type TopViCapture creation in the single thread
    int init(int deviceId, TopViCaptureInterface *parent);

    //This is a wrappers for GRILL API
    int getCamerasSysId(vector<string> &camDesc);

    void grab(TopViCaptureInterface *parent);
    void grabAll(TopViCaptureInterface *parent);

private:

    int mCamerasNumber = 0;
    vector<TopViCameraDescriptor *>  mCameras;

    class CmdSpinThread : public QThread
    {
    public:

        TopViDeviceDescriptor *device;

        CmdSpinThread() {
            this->setObjectName("TopViDeviceDescriptor:CmdSpinThread");
        }

        void run (void);
    };

    CmdSpinThread cmdSpin;

    QMutex cmdSpinRunning;
    volatile bool shouldStopCmdSpinThread = false;

    TopViGrillInterface grillInterface;
    QMutex protectGrillRequest;

    void replyCallback(TopViGrillCommand *cmd);
    //This is a methods for GRILL API
    void executeCommand(enum TopViCmd cmdType, int camId = 0, int value = 0, int add_value = 0, TopViCaptureInterface *parent = NULL);

};

