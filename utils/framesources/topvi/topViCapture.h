#pragma once

#include <errno.h>
#include <QImage>
#include <QDebug>

#include "global.h"

#include "cameraControlParameters.h"
#include "topViDeviceDescriptor.h"
#include "preciseTimer.h"

#include "ftpLoader.h"

using namespace std;

class TopViCaptureInterface : public virtual ImageCaptureInterface
{
public:

    int fAuto = 0; //set auto exposure mode
    int fClean = 0; //set auto clean mode

    //OK
    TopViCaptureInterface(string _devname);
    //OK
    TopViCaptureInterface(string _devname, int h, int w, int fps, bool isRgb = false);

    virtual int setConfigurationString(string _devname, bool isRgb = false);

    //TODO:
    virtual ~TopViCaptureInterface();

    virtual CapErrorCode setCaptureProperty(int id, int value) override;
    virtual CapErrorCode getCaptureProperty(int id, int *value) override;

    virtual CapErrorCode queryCameraParameters(CameraParameters &parameter) override;

    //TODO:
    virtual CapErrorCode initCapture() override;
    virtual CapErrorCode startCapture() override;

    virtual CapErrorCode getFormats(int *num, CameraFormat *& formats) override;

    virtual FramePair    getFrame() override;

    //TODO:
    virtual bool getCurrentFormat(CameraFormat &format);

    //TODO:
    static void getAllCameras(vector<std::string> &cameras);

    virtual std::string  getDeviceSerial(int num = LEFT_FRAME) override;

    virtual QString getInterfaceName()  override;

    //OK
    static int topViTrace(int res, const char *prefix = NULL);

    int activateFtpLoader(string linkReply);
    int replyCallback(TopViGrillCommand *cmd);

    QString getActiveFileName() {
        return QString(this->ftpSpin.ftpLoader.activeFile.c_str());
    }

    void updateCamera(int camId);

    static const double EXPOSURE_SCALER;

private:

    string interfaceName;  /**< Stores the camera name >**/

    /* This is a private helper class */
    class FtpSpinThread : public QThread
    {
    public:
        TopViCaptureInterface *capInterface;
        FtpLoader ftpLoader;

        FtpSpinThread(TopViCaptureInterface *_capInterface) :
            capInterface(_capInterface)
        {
            this->setObjectName("TopViCaptureInterface:FtpSpinThread");
        }

        void run (void);
    };

    string devname;  /**< Stores the device name*/

    enum SyncType {
        NO_SYNC,
        SOFT_SYNC
    };

    SyncType sync;

    static TopViDeviceDescriptor device;

    BufferDescriptorType *currentFrame;

    //void decodeData  (TopViCameraDescriptor *camera, BufferDescriptorType *buffer, G12Buffer **output);
    //void decodeData24(TopViCameraDescriptor *camera, BufferDescriptorType *buffer, RGB24Buffer **output);

    FtpSpinThread ftpSpin;         /**< ftp spin thread that blocks waiting get file >**/
    QMutex protectFrame;
    QMutex protectActivate;
    QMutex ftpSpinRunning;
    volatile bool shouldStopFtpSpinThread = false;
    volatile bool shouldActivateFtpSpinThread = false;

};

