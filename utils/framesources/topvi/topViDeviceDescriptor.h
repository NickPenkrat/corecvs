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

    void setCommonExposure(TopViCaptureInterface *parent, int value);
    void setCommonGain(TopViCaptureInterface *parent,int value);

    void getExposure(TopViCaptureInterface *parent);
    void setExposure(TopViCaptureInterface *parent, int value);
    void setGain(TopViCaptureInterface *parent, enum TopViGain gainType, int value);

    int mCamerasNumber = 0;
    vector<TopViCameraDescriptor *>  mCameras;

private:

    struct TopVi_SU *deviceParams;

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
    QMutex protectGrillReply;

    void replyCallback(TopViGrillCommand *cmd);
    //This is a methods for GRILL API
    void executeCommand(enum TopViCmdType cmdType, enum TopViCmdName cmdName, int camId = 0, string value = "", string add_value = "", TopViCaptureInterface *parent = NULL);

};

