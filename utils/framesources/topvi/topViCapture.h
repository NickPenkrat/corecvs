#ifndef TopViCapture_H
#define TopViCapture_H

#include <errno.h>
#include <QImage>
#include <QDebug>

#include "global.h"

#include "topViDeviceDescriptor.h"
#include "cameraControlParameters.h"
#include "imageCaptureInterface.h"
#include "preciseTimer.h"

#include "ftpLoader.h"

using namespace std;

class TopViCaptureInterface : public virtual ImageCaptureInterface
{
public:
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

private:
     string interfaceName;  /**< Stores the device name*/

    /* This is a private helper class */
    class SpinThread : public QThread
    {
    public:
        TopViCaptureInterface *capInterface;
        FtpLoader ftp;

        SpinThread(TopViCaptureInterface *_capInterface) :
            capInterface(_capInterface)
        {
            this->setObjectName("TopViCaptureInterface:SpinThread");
        }

        void run (void);
    };

    class CmdThread : public QThread
    {
    public:
        TopViCaptureInterface *capInterface;

        CmdThread(TopViCaptureInterface *_capInterface) :
            capInterface(_capInterface)
        {
            this->setObjectName("TopViCaptureInterface:CmdThread");
        }

        void run (void);
    };

    string devname;  /**< Stores the device name*/

    enum SyncType {
        NO_SYNC,
        SOFT_SYNC
    };

    SyncType sync;

    //void decodeData  (TopViCameraDescriptor *camera, BufferDescriptorType *buffer, G12Buffer **output);
    //void decodeData24(TopViCameraDescriptor *camera, BufferDescriptorType *buffer, RGB24Buffer **output);

    TopViDeviceDescriptor device;

    SpinThread spin;         /**< Spin thread that blocks waiting for the frames */
    QMutex protectFrame;     /**< This mutex protects both buffers from concurrent reading/writind
                              *  If somebody is reading of modifying the buffer it should look the mutex
                              **/
    QMutex spinRunning;
    volatile bool shouldStopSpinThread;
    volatile bool shouldActivateSpinThread;

    CmdThread cmd;            /**< Spin thread that blocks waiting for the cmd queues */
    QMutex protectSocket;  /**< This mutex protects queue from concurrent reading/writind
                               *  If somebody is reading of modifying the buffer it should look the mutex
                               **/
    QMutex cmdRunning;
    volatile bool shouldStopCmdThread;

};

#endif // TopViCapture_H
