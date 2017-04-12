/**
 * \file TopViCapture.cpp
 * \brief Add Comment Here
 *
 * \date March 15, 2017
 * \author kat
 */

#define __STDC_FORMAT_MACROS
//#include <inttypes.h>
#undef __STDC_FORMAT_MACROS

#include <fcntl.h>
//#include <unistd.h>
#include <stdlib.h>
#include <QtCore/QRegExp>
#include <QtCore/QString>
#include <QtCore/QtCore>

#include "global.h"

#include "topViCapture.h"
#include "topViDeviceDescriptor.h"

#ifdef PROFILE_DEQUEUE
#define TRACE_DEQUEUE(X) printf X
#else
#define TRACE_DEQUEUE(X)
#endif

#ifdef DEEP_TRACE_ON
#define DEEP_TRACE(X) SYNC_PRINT (X)
#else
#define DEEP_TRACE(X)
#endif

TopViDeviceDescriptor TopViCaptureInterface::device;

int TopViCaptureInterface::topViTrace(int result, const char *prefix)
{
    if (result != 0)
    {
        char * str = 0;
        if (prefix == NULL)
            SYNC_PRINT(("TopViCaptureInterface: function failed with error code %d. Error string: %s\n", result, str));
        else
            SYNC_PRINT(("%s: error code %d. Error string: %s\n", prefix, result, str));

    }

    return result;
}

TopViCaptureInterface::TopViCaptureInterface(string _devname) :
    sync(NO_SYNC),
    ftpSpin(this)
{
    SYNC_PRINT(("TopViCaptureInterface: TopVi capture is using now\n"));
    setConfigurationString(_devname);
    device.init(0, this);
}

TopViCaptureInterface::TopViCaptureInterface(string _devname, int /*h*/, int /*w*/, int /*fps*/, bool isRgb) :
    sync(NO_SYNC),
    ftpSpin(this)
{
    SYNC_PRINT(("TopViCaptureInterface: TopVi capture is using now (constructor with parameters)\n"));
    setConfigurationString(_devname, isRgb);
    device.init(0, this);
}

void TopViCaptureInterface::replyCallback(string reply){
    SYNC_PRINT(("TopViCaptureInterface: return answer for GUI: %s\n", reply.c_str()));
    if (!shouldActivateFtpSpinThread) {
        ftpSpin.ftpLoader.init(reply);
        shouldActivateFtpSpinThread = true;
    }
}

int TopViCaptureInterface::setConfigurationString(string _devname, bool isRgb)
{
    interfaceName = _devname;

    SYNC_PRINT(("TopViCaptureInterface: configuration string is %s\n", _devname.c_str()));

    this->devname = _devname;

    return 0;
}

TopViCaptureInterface::~TopViCaptureInterface()
{
    bool result = 0;
    int timeout = 100000;
    int waitTime = 200;

    SYNC_PRINT(("TopViCaptureInterface(%s): request for killing the threads\n", QSTR_DATA_PTR(getInterfaceName())));
#if 1
    /* Stopping ftp loader */
    while (shouldActivateFtpSpinThread) {
        usleep(waitTime);
        timeout -= waitTime;
        if (timeout <= 0) break;
    }

    if (shouldActivateFtpSpinThread) {
        SYNC_PRINT(("FtpLoader is active too long. Stop it."));
        shouldActivateFtpSpinThread = false;
    }

    shouldStopFtpSpinThread = true;
    result = ftpSpinRunning.tryLock(2000);

    //TODO: use stop camera command

    /* Now no events would be generated, and it is safe to unlock mutex */

    if (result) {
        printf("TopViCaptureInterface: Camera thread killed\n");
        ftpSpinRunning.unlock();
    } else {
        printf("TopViCaptureInterface: Unable to exit Camera thread\n");
    }

    printf("TopViCaptureInterface: Deleting image buffers\n");

    printf("TopViCaptureInterface: Closing ftp getter...\n");
#endif

    printf("TopViCaptureInterface: Closing TopVi capture...\n");
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::startCapture()
{
    SYNC_PRINT((" TopViCaptureInterface::startCapture(): called\n"));
    if (!ftpSpin.isRunning()) {
        ftpSpinRunning.lock();
        shouldStopFtpSpinThread = false;
        ftpSpinRunning.unlock();
        SYNC_PRINT(("TopVi is starting ftp loader\n"));
        ftpSpin.start();
    }

    SYNC_PRINT(("Ftp loader should be started\n"));

    device.grab(this);

    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::getFormats(int *num, ImageCaptureInterface::CameraFormat *&formats)
{
    SYNC_PRINT(("TopViCaptureInterface::getFormats()\n"));
    vector<ImageCaptureInterface::CameraFormat> cameraFormats;

    cameraFormats.push_back(ImageCaptureInterface::CameraFormat(3840, 2748, 6));
    //TODO: add format for preview

    *num = cameraFormats.size();
    formats = new ImageCaptureInterface::CameraFormat[cameraFormats.size()];
    for (unsigned i = 0; i < cameraFormats.size(); i ++)
    {
        formats[i] = cameraFormats[i];
    }
    return ImageCaptureInterface::SUCCESS;
}

bool TopViCaptureInterface::getCurrentFormat(ImageCaptureInterface::CameraFormat &format)
{
    SYNC_PRINT(("TopViCaptureInterface::getFormat() is not supported\n"));
    return ImageCaptureInterface::SUCCESS;
}


void TopViCaptureInterface::getAllCameras(vector<string> &cameras)
{
    device.getCamerasSysId(cameras);
}

QString TopViCaptureInterface::getInterfaceName()
{
    return QString("tpv://") + QString(interfaceName.c_str());
}

string TopViCaptureInterface::getDeviceSerial(int /*num*/)
{
    //SYNC_PRINT(("TopViCaptureInterface::getDeviceSerial():called\n"));
    //TODO: doesn't better solution !!!
    string camSysId = interfaceName.substr(interfaceName.find_first_of('_') + 1, 1);
    int camId = QString(camSysId.c_str()).toInt();
    std::string toReturn = to_string(camId);

    SYNC_PRINT(("TopviCaptureInterface::getDeviceSerial():returning <%s>\n", toReturn.c_str()));
    return toReturn;
}

void TopViCaptureInterface::FtpSpinThread::run()
{
    SYNC_PRINT(("TopViCaptureInterface::ftpSpinThread(): ftp loader thread running\n"));

    while (capInterface->ftpSpinRunning.tryLock()) {

        if (ftpLoader.inited && capInterface->shouldActivateFtpSpinThread) {
            SYNC_PRINT(("FtpSpinThread::run(): ftp get starting\n"));
            //ftpLoader.makeTest();
            ftpLoader.getFile();
            capInterface->shouldActivateFtpSpinThread = false;
            SYNC_PRINT(("FtpSpinThread::run(): ftp get finished\n"));
        }
        else {
            usleep(500);
        }

        /* Now exchange the buffer that is visible from */

        //capInterface->protectFrame.lock();
        //frame_data_t frameData;
        //frameData.timestamp = capInterface->currentLeft->usecsTimeStamp();
        //capInterface->notifyAboutNewFrame(frameData);

        capInterface->ftpSpinRunning.unlock();
        if (capInterface->shouldStopFtpSpinThread)
        {
            SYNC_PRINT(("Break command to ftp spin thread received"));
            break;
        }
    }
    SYNC_PRINT(("TopViCaptureInterface: ftpLoader thread finished\n"));
}


ImageCaptureInterface::CapErrorCode TopViCaptureInterface::initCapture()
{
    SYNC_PRINT(("TopViCaptureInterface::initCapture(): called\n"));

    int res = ImageCaptureInterface::SUCCESS_1CAM;

    CapErrorCode result = (CapErrorCode)((bool) res);

    SYNC_PRINT(("TopViCaptureInterface::initCapture(): returning %d\n", result));
    return result;
}

TopViCaptureInterface::FramePair   TopViCaptureInterface::getFrame(){
    FramePair result( NULL, NULL);
    if (!shouldActivateFtpSpinThread)
    {
        shouldActivateFtpSpinThread = true;
    }
    else {
        SYNC_PRINT(("FtpLoader is busy. Please, try some later."));
    }
    return result;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::queryCameraParameters(CameraParameters &params)
{
    SYNC_PRINT(("TopViCaptureInterface: queryCameraParameters is not supported for TopVi interface yet.\n"));
    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::setCaptureProperty(int id, int value)
{
    SYNC_PRINT(("TopViCaptureInterface:: setCaptureProperty is not supported for TopVi interface yet.\n"));
    //TODO: may be good place for get output dir path
    return ImageCaptureInterface::FAILURE;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::getCaptureProperty(int id, int *value)
{
    SYNC_PRINT(("TopViCaptureInterface:: getCaptureProperty is not supported for TopVi interface yet.\n"));
    return ImageCaptureInterface::FAILURE;
}

