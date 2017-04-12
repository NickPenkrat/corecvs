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

int TopViCaptureInterface::topViTrace(int result, const char *prefix)
{
    if (result != 0)
    {
        char * str = 0;
        if (prefix == NULL)
            printf("   TPV: Function failed with error code %d. Error string: %s\n", result, str);
        else
            printf("%s: error code %d. Error string: %s\n", prefix, result, str);

    }

    return result;
}

TopViCaptureInterface::TopViCaptureInterface(string _devname) :
    sync(NO_SYNC),
    device(),
    spin(this),
    cmd(this)
{
    setConfigurationString(_devname);
}

TopViCaptureInterface::TopViCaptureInterface(string _devname, int /*h*/, int /*w*/, int /*fps*/, bool isRgb) :
    sync(NO_SYNC),
    device(),
    spin(this),
    cmd(this)
{
    setConfigurationString(_devname, isRgb);
}

int TopViCaptureInterface::setConfigurationString(string _devname, bool isRgb)
{
    interfaceName = _devname;
    printf("######################################\n");
    printf("This is a help with the TopVi cameras.\n");
    printf("######################################\n");

    //TODO: use all cameras get info

    printf ("TPV: input string %s\n", _devname.c_str());

    this->devname = _devname;

    return 0;
}

TopViCaptureInterface::~TopViCaptureInterface()
{
    bool result = 0;
    int timeout = 100000;
    int waitTime = 200;

    cout << "Request for killing the threads" << endl;

    /* Stopping ftp loader */
    while (shouldActivateSpinThread) {
        usleep(waitTime);
        timeout -= waitTime;
        if (timeout <= 0) break;
    }

    if (shouldActivateSpinThread) {
        SYNC_PRINT(("FtpLoader is active too long. Stop it."));
        shouldActivateSpinThread = false;
    }

    shouldStopSpinThread = true;
    result = spinRunning.tryLock(2000);

    //TODO: use stop camera command

    /* Now no events would be generated, and it is safe to unlock mutex */

    if (result) {
        printf("TPV: Camera thread killed\n");
        spinRunning.unlock();
    } else {
        printf("TPV: Unable to exit Camera thread\n");
    }

    printf("TPV: Deleting image buffers\n");

    printf("TPV: Closing ftp getter...\n");

    shouldStopCmdThread = true;
    result = cmdRunning.tryLock(2000);

    if (result) {
        printf("TPV: command thread killed\n");
        cmdRunning.unlock();
    } else {
        printf("TPV: Unable to exit command thread\n");
    }

    printf("TPV: Closing cmd client...\n");

    printf("TPV: Closing TopVi capture...\n");
\

}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::startCapture()
{
    SYNC_PRINT((" TopViCaptureInterface::startCapture(): called\n"));

    //TODO:

    SYNC_PRINT(("Enabling events...\n"));

    SYNC_PRINT(("Starting capture...\n"));


    SYNC_PRINT(("Starting ftp-get thread...\n"));
    shouldStopSpinThread = false;
    shouldActivateSpinThread = false;

    spin.start();

    SYNC_PRINT(("Starting cmd thread...\n"));
    shouldStopCmdThread = false;

    cmd.start();

    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::getFormats(int *num, ImageCaptureInterface::CameraFormat *&formats)
{
    SYNC_PRINT(("TopViCaptureInterface::getFormats()\n"));
    vector<ImageCaptureInterface::CameraFormat> cameraFormats;

    cameraFormats.push_back(ImageCaptureInterface::CameraFormat( 3840, 2748, 6));

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
    //TODO:
    return false;
}


void TopViCaptureInterface::getAllCameras(vector<string> &cameras)
{
 //TODO:
}

QString TopViCaptureInterface::getInterfaceName()
{
    return QString("TPV: ") + QString(interfaceName.c_str());
}


string TopViCaptureInterface::getDeviceSerial(int /*num*/)
{
    SYNC_PRINT(("TopViCaptureInterface::getDeviceSerial():called\n"));

    std::string toReturn = "unknown";

    //TODO:

    SYNC_PRINT(("TopviCaptureInterface::getDeviceSerial():returning <%s>\n", toReturn.c_str()));
    return toReturn;
}

void TopViCaptureInterface::SpinThread::run()
{
    qDebug("TopViCaptureInterface::SpinThread(): New frame thread running");

    while (capInterface->spinRunning.tryLock()) {
        int result = 0;
        int timeout = 4000;

        if (capInterface->shouldActivateSpinThread) {
            DEEP_TRACE(("SpinThread::run(): ftp get starting"));
            ftp.ftpGetTest();
            capInterface->shouldActivateSpinThread = false;
            DEEP_TRACE(("SpinThread::run(): ftp get finished"));
        }

        /* Now exchange the buffer that is visible from */
        capInterface->protectFrame.lock();

        frame_data_t frameData;

        //frameData.timestamp = capInterface->currentLeft->usecsTimeStamp();

        capInterface->notifyAboutNewFrame(frameData);

        capInterface->spinRunning.unlock();
        if (capInterface->shouldStopSpinThread)
        {
            qDebug("Break command to spin thread received");

            break;
        }
    }
    qDebug("new frame thread finished");
}

void TopViCaptureInterface::CmdThread::run()
{
    qDebug("TopViCaptureInterface::CmdThread(): New command thread running");

    while (capInterface->cmdRunning.tryLock()) {
        int result = 0;
        int timeout = 4000;

        //TODO:
        printf("This is place for waitting event or ftp get");

        DEEP_TRACE(("CmdThread::run():\n"));

        /* If we are here seems like both new cameras produced frames*/

        //TODO:

        DEEP_TRACE(("CmdThread::run():We have locked buffers [%d and %d]\n", bufIDL, bufIDR));

        /* Now exchange the buffer that is visible from */
        capInterface->protectSocket.lock();

        frame_data_t frameData;

        //frameData.timestamp = capInterface->currentLeft->usecsTimeStamp();

        //capInterface->notifyAboutNewCommand(frameData);

        capInterface->cmdRunning.unlock();
        if (capInterface->shouldStopCmdThread)
        {
            qDebug("Break command to cmd thread received\n");

            break;
        }
    }
    qDebug("cmd thread finished\n");
}


ImageCaptureInterface::CapErrorCode TopViCaptureInterface::initCapture()
{
    SYNC_PRINT(("TopViCaptureInterface::initCapture(): called\n"));

    int res = 1;

    CapErrorCode result = (CapErrorCode)((bool) res);

    SYNC_PRINT(("TopViCaptureInterface::initCapture(): returning %d\n", result));
    return result;
}

TopViCaptureInterface::FramePair   TopViCaptureInterface::getFrame(){
    FramePair result( NULL, NULL);
    if (!shouldActivateSpinThread)
    {
        shouldActivateSpinThread = true;
    }
    else {
        SYNC_PRINT(("FtlLoader is busy. Please, try some later."));
    }
    return result;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::queryCameraParameters(CameraParameters &params)
{
    printf("TPV: queryCameraParameters is not supported for TopVi interface yet.\n");
    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::setCaptureProperty(int id, int value)
{
    printf("TPV: setCaptureProperty is not supported for TopVi interface yet.\n");
    return ImageCaptureInterface::FAILURE;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::getCaptureProperty(int id, int *value)
{
    printf("TPV: getCaptureProperty is not supported for TopVi interface yet.\n");
    return ImageCaptureInterface::FAILURE;
}

